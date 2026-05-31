/* driver/rkmpp_ccu/driver.c — KMDF driver for rkmpp_ccu.sys.
 *
 * Phase 2: maps the CRU MMIO for RKCP3503 (rkv-decoder-v2 CCU / RDCC) and
 * exposes RaiseCluster / DropCluster / AssertRvdec{0,1}CoreReset /
 * DeassertRvdec{0,1}CoreReset / GateRvdec{0,1}LeafClocks / etc.
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
#include "../shared/rkmpp_log.h"
#include "../shared/acpi_uid.h"
#include "pmu.h"

/* Globals consumed by ccu.c.
 *
 * g_rdcc_mmio  — ACPI _CRS for RKCP3503 (RDCC cluster coordinator at
 *                0xfdc30000, len 0x100).  Reserved for Phase 3 task
 *                arbitration; not used in Phase 2.
 * g_cru_mmio   — direct physical-address map of the system CRU at
 *                0xfd7c0000.  Owns the clock-gate and soft-reset
 *                registers for the rkvdec cluster.  Mapped here because
 *                the firmware does not expose the CRU as an ACPI device.
 */
volatile UCHAR *g_rdcc_mmio     = NULL;
volatile UCHAR *g_cru_mmio      = NULL;
volatile UCHAR *g_pmu_mmio      = NULL;
/* g_raise_refcount moved to ccu.c as static — it's purely a
 * RaiseCluster/DropCluster state field, never referenced outside ccu.c. */

/* MMIO mapping bookkeeping — kept here so ReleaseHardware can unmap. */
static SIZE_T   g_rdcc_mmio_len = 0;
static SIZE_T   g_cru_mmio_len  = 0;
static SIZE_T   g_pmu_mmio_len  = 0;

/* System CRU physical base + length we map.  The full CRU is ~0x5c000;
 * we map just one page covering the rkvdec gate (0x8A0) and reset
 * (0xAA0) registers.  See ccu.c for the offsets and rationale.
 */
#define RKMPP_CCU_CRU_PHYS_BASE   0xFD7C0000ULL
#define RKMPP_CCU_CRU_MAP_LENGTH  0x1000u

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        RkMppCcuEvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  RkMppCcuEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  RkMppCcuEvtReleaseHardware;

NTSTATUS RkMppCcuRegisterIfc(_In_ WDFDEVICE Device);  /* in ifc.c */
extern VOID RkMppCcuInitMutex(VOID);                   /* in ccu.c */

/* ---------------------------------------------------------------------------
 * ACPI HID parse — reused from rkmpp/device.c pattern.
 * Returns the last 4 hex digits of the first "ACPI\RKCP35xx" entry as a
 * UINT32 (e.g. RKCP3503 → 0x3503), or 0 on failure.
 * --------------------------------------------------------------------------- */
static UINT32 RkMppCcuReadHid(_In_ WDFDEVICE Device)
{
    UINT32 hid = 0, uid = 0;
    if (!NT_SUCCESS(RkSharedReadAcpiHidUid(Device, &hid, &uid))) return 0;
    return hid;
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
    RKMPP_LOG_INFO("rkmpp_ccu: PrepareHardware HID=RKCP%04x\n", hid);

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
                /* RKCP3503 — RDCC cluster-coordinator MMIO (Phase 3 task
                 * arbitration).  We don't touch it in Phase 2 but keep the
                 * mapping live so the address space is reserved. */
                g_rdcc_mmio     = (volatile UCHAR*)base;
                g_rdcc_mmio_len = d->u.Memory.Length;
                RKMPP_LOG_INFO("rkmpp_ccu: RDCC coord mapped @ %p len 0x%Ix\n",
                               base, g_rdcc_mmio_len);

                /* Also map the system CRU (not exposed via ACPI) so we can
                 * gate clocks and assert resets for the rkvdec cluster. */
                PHYSICAL_ADDRESS cruPhys;
                cruPhys.QuadPart = RKMPP_CCU_CRU_PHYS_BASE;
                PVOID cruVa = MmMapIoSpaceEx(cruPhys,
                                             RKMPP_CCU_CRU_MAP_LENGTH,
                                             PAGE_READWRITE | PAGE_NOCACHE);
                if (!cruVa) {
                    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                               "rkmpp_ccu: CRU map failed @ 0x%llx\n",
                               cruPhys.QuadPart);
                    return STATUS_INSUFFICIENT_RESOURCES;
                }
                g_cru_mmio     = (volatile UCHAR*)cruVa;
                g_cru_mmio_len = RKMPP_CCU_CRU_MAP_LENGTH;
                RKMPP_LOG_INFO("rkmpp_ccu: system CRU mapped @ %p (phys 0x%llx)\n",
                               cruVa, (unsigned long long)cruPhys.QuadPart);

                /* Map the PMU (not exposed via ACPI) for power-domain control. */
                PHYSICAL_ADDRESS pmuPhys;
                pmuPhys.QuadPart = RKMPP_PMU_PHYS_BASE;
                PVOID pmuVa = MmMapIoSpaceEx(pmuPhys,
                                             RKMPP_PMU_MAP_LENGTH,
                                             PAGE_READWRITE | PAGE_NOCACHE);
                if (!pmuVa) {
                    /* Unwind the CRU map we already took. */
                    MmUnmapIoSpace((PVOID)g_cru_mmio, g_cru_mmio_len);
                    g_cru_mmio = NULL; g_cru_mmio_len = 0;
                    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                               "rkmpp_ccu: PMU map failed @ 0x%llx\n",
                               (unsigned long long)pmuPhys.QuadPart);
                    return STATUS_INSUFFICIENT_RESOURCES;
                }
                g_pmu_mmio     = (volatile UCHAR*)pmuVa;
                g_pmu_mmio_len = RKMPP_PMU_MAP_LENGTH;
                RKMPP_LOG_INFO("rkmpp_ccu: PMU mapped @ %p (phys 0x%llx)\n",
                               pmuVa, (unsigned long long)pmuPhys.QuadPart);
            } else {
                /* RKCP3501/3502 — map succeeds but we don't use it for RDCC.
                 * Unmap immediately; CCU functions will return gracefully via
                 * the g_cru_mmio == NULL guards in ccu.c.
                 */
                MmUnmapIoSpace(base, d->u.Memory.Length);
                RKMPP_LOG_INFO("rkmpp_ccu: HID %04x — RDCC not managed, skipped\n",
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

    if (g_pmu_mmio) {
        MmUnmapIoSpace((PVOID)g_pmu_mmio, g_pmu_mmio_len);
        g_pmu_mmio     = NULL;
        g_pmu_mmio_len = 0;
    }
    if (g_cru_mmio) {
        MmUnmapIoSpace((PVOID)g_cru_mmio, g_cru_mmio_len);
        g_cru_mmio     = NULL;
        g_cru_mmio_len = 0;
    }
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
    RkMppCcuInitMutex();
    WDF_DRIVER_CONFIG cfg;
    WDF_DRIVER_CONFIG_INIT(&cfg, RkMppCcuEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}

NTSTATUS
RkMppCcuEvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
{
    UNREFERENCED_PARAMETER(Driver);

    /* Explicit device ACL — admin/system only.  rkmpp_ccu has no
     * user-mode IOCTL surface (interfaces are kernel-side via
     * WdfDeviceAddQueryInterface), so non-admin access has no
     * legitimate purpose. */
    DECLARE_CONST_UNICODE_STRING(sddl, L"D:P(A;;GA;;;SY)(A;;GA;;;BA)");
    NTSTATUS sddlStatus = WdfDeviceInitAssignSDDLString(DeviceInit, &sddl);
    if (!NT_SUCCESS(sddlStatus)) return sddlStatus;
    WdfDeviceInitSetCharacteristics(DeviceInit, FILE_DEVICE_SECURE_OPEN, FALSE);

    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = RkMppCcuEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = RkMppCcuEvtReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, WDF_NO_OBJECT_ATTRIBUTES,
                                      &device);
    if (!NT_SUCCESS(status)) return status;

    RKMPP_LOG_INFO("rkmpp_ccu: device added\n");

    return RkMppCcuRegisterIfc(device);
}
