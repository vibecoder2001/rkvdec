/* driver/rkmpp_ccu/driver.c — KMDF driver for rkmpp_ccu.sys.
 *
 * Phase 2: maps the CRU MMIO for RKCP3503 (rkv-decoder-v2 CCU / RDCC) and
 * exposes RaiseCluster / DropCluster / AssertCoreReset / DeassertCoreReset
 * via the RKMPP_CCU_INTERFACE query-interface mechanism.
 *
 * RKCP3501 / RKCP3502 are still matched (for ACPI) but the MMIO pointer is
 * only set when HID == 0x3503, so their no-op semantics fall out naturally:
 * g_rdcc_mmio remains NULL and all CCU operations return STATUS_DEVICE_NOT_READY
 * or STATUS_SUCCESS (for DropCluster, which is safe to call on an ungated CCU).
 */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ccu_ifc.h"

/* Globals consumed by ccu.c. */
volatile UCHAR *g_rdcc_mmio     = NULL;
LONG            g_raise_refcount = 0;

/* MMIO mapping bookkeeping — kept here so ReleaseHardware can unmap. */
static SIZE_T   g_rdcc_mmio_len = 0;

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        RkMppCcuEvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  RkMppCcuEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  RkMppCcuEvtReleaseHardware;

NTSTATUS RkMppCcuRegisterIfc(_In_ WDFDEVICE Device);  /* in ifc.c */

/* ---------------------------------------------------------------------------
 * ACPI HID parse — reused from rkmpp/device.c pattern.
 * Returns the last 4 hex digits of the first "ACPI\RKCP35xx" entry as a
 * UINT32 (e.g. RKCP3503 → 0x3503), or 0 on failure.
 * --------------------------------------------------------------------------- */
static UINT32 RkMppCcuReadHid(_In_ WDFDEVICE Device)
{
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(Device);
    WCHAR buf[1024] = {0};
    ULONG size = 0;

    NTSTATUS status = IoGetDeviceProperty(pdo, DevicePropertyHardwareID,
                                          sizeof(buf), buf, &size);
    if (!NT_SUCCESS(status)) return 0;

    PCWSTR cursor = buf;
    while (*cursor) {
        size_t len = wcslen(cursor);
        if (len >= 13 &&
            cursor[0] == L'A' && cursor[1] == L'C' && cursor[2] == L'P' &&
            cursor[3] == L'I' && cursor[4] == L'\\' &&
            cursor[5] == L'R' && cursor[6] == L'K' && cursor[7] == L'C' &&
            cursor[8] == L'P' && cursor[9] == L'3' && cursor[10] == L'5')
        {
            UINT32 hid = 0;
            for (int i = 9; i < 13; i++) {
                WCHAR  c = cursor[i];
                UINT32 d;
                if      (c >= L'0' && c <= L'9') d = (UINT32)(c - L'0');
                else if (c >= L'a' && c <= L'f') d = 10u + (UINT32)(c - L'a');
                else if (c >= L'A' && c <= L'F') d = 10u + (UINT32)(c - L'A');
                else { hid = 0; break; }
                hid = (hid << 4) | d;
            }
            if (hid) return hid;
        }
        cursor += len + 1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * EvtPrepareHardware — map the first memory resource.
 * Only store the base in g_rdcc_mmio when HID == 0x3503 (RDCC / rkvdec-v2).
 * --------------------------------------------------------------------------- */
NTSTATUS
RkMppCcuEvtPrepareHardware(_In_ WDFDEVICE Device,
                            _In_ WDFCMRESLIST ResourcesRaw,
                            _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);

    UINT32 hid = RkMppCcuReadHid(Device);
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkmpp_ccu: PrepareHardware HID=RKCP%04x\n", hid);

    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            PVOID base = MmMapIoSpaceEx(d->u.Memory.Start,
                                        d->u.Memory.Length,
                                        PAGE_READWRITE | PAGE_NOCACHE);
            if (!base) return STATUS_INSUFFICIENT_RESOURCES;

            if (hid == 0x3503) {
                /* RKCP3503 — this is the RDCC CRU window we operate on. */
                g_rdcc_mmio     = (volatile UCHAR*)base;
                g_rdcc_mmio_len = d->u.Memory.Length;
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                           "rkmpp_ccu: RDCC CRU mapped @ %p len 0x%Ix\n",
                           base, g_rdcc_mmio_len);
            } else {
                /* RKCP3501/3502 — map succeeds but we don't use it for RDCC.
                 * Unmap immediately; CCU functions will return gracefully via
                 * the g_rdcc_mmio == NULL guards in ccu.c.
                 */
                MmUnmapIoSpace(base, d->u.Memory.Length);
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                           "rkmpp_ccu: HID %04x — RDCC not managed, skipped\n",
                           hid);
            }
            break;
        }
    }
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * EvtReleaseHardware — unmap the RDCC CRU window if we mapped it.
 * --------------------------------------------------------------------------- */
NTSTATUS
RkMppCcuEvtReleaseHardware(_In_ WDFDEVICE Device,
                            _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    if (g_rdcc_mmio) {
        MmUnmapIoSpace((PVOID)g_rdcc_mmio, g_rdcc_mmio_len);
        g_rdcc_mmio     = NULL;
        g_rdcc_mmio_len = 0;
    }
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * DriverEntry / EvtDeviceAdd
 * --------------------------------------------------------------------------- */
NTSTATUS
DriverEntry(_In_ PDRIVER_OBJECT DriverObject, _In_ PUNICODE_STRING RegistryPath)
{
    WDF_DRIVER_CONFIG cfg;
    WDF_DRIVER_CONFIG_INIT(&cfg, RkMppCcuEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}

NTSTATUS
RkMppCcuEvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
{
    UNREFERENCED_PARAMETER(Driver);

    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = RkMppCcuEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = RkMppCcuEvtReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, WDF_NO_OBJECT_ATTRIBUTES,
                                      &device);
    if (!NT_SUCCESS(status)) return status;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkmpp_ccu: device added\n");

    return RkMppCcuRegisterIfc(device);
}
