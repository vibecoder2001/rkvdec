/* driver/rkiommu_av1d/device.c — per-instance RKIOMMU_DEVICE for rkiommu_av1d.sys.
 * RKCP3571 (rockchip,iommu-av1d) only.  AHB_CONTROL flat enable/disable model.
 * No STALL/COMMAND/ENABLE_PAGING — completely different register layout from v2.
 */
#include <ntddk.h>
#include <wdf.h>
#include "device.h"
#include "../shared/iommu/pgtable.h"
#include "../shared/iommu/fault.h"
#include "../shared/acpi_uid.h"

LIST_ENTRY  g_deviceList;
KSPIN_LOCK  g_deviceListLock;
static BOOLEAN g_listInitialized = FALSE;

EVT_WDF_DEVICE_PREPARE_HARDWARE  RkIommuEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  RkIommuEvtReleaseHardware;
extern NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device);

static NTSTATUS
RkIommuReadAcpiId(_In_ WDFDEVICE Device, _Out_ PUINT32 Hid, _Out_ PUINT32 Uid)
{
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(Device);
    WCHAR buf[1024] = {0};
    ULONG size = 0;

    NTSTATUS status = IoGetDeviceProperty(pdo, DevicePropertyHardwareID,
                                          sizeof(buf), buf, &size);
    if (!NT_SUCCESS(status)) return status;

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
            if (hid) {
                *Hid = hid;
                *Uid = RkSharedQueryAcpiUid(pdo);
                return STATUS_SUCCESS;
            }
        }
        cursor += len + 1;
    }
    return STATUS_INVALID_DEVICE_REQUEST;
}

_Use_decl_annotations_
NTSTATUS RkIommuDisableHw(PRKIOMMU_DEVICE Dev)
{
    if (!Dev || !Dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    if (!Dev->PagingEnabled)    return STATUS_SUCCESS;

    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + AV1_MMU_AHB_CONTROL), 0u);
    Dev->PagingEnabled = FALSE;
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu_av1d: disabled (HID=RKCP%04x UID=%u)\n",
               Dev->Hid, Dev->Uid);
    return STATUS_SUCCESS;
}

_Use_decl_annotations_
NTSTATUS RkIommuEnableHw(PRKIOMMU_DEVICE Dev)
{
    if (!Dev->MmioBase || !Dev->Domain) return STATUS_DEVICE_NOT_READY;
    if (Dev->PagingEnabled)             return STATUS_SUCCESS;

    volatile UCHAR *base = Dev->MmioBase;
    ULONG ctrl = READ_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_AHB_CONTROL));
    if (ctrl & AV1_MMU_AHB_CONTROL_ENABLE) {
        Dev->PagingEnabled = TRUE;
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkiommu_av1d: already enabled (ctrl=0x%08x)\n", ctrl);
        return STATUS_SUCCESS;
    }

    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_AHB_TBL_ARRAY_BASE_L),
        Dev->Domain->PtaPhys);
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_AHB_TBL_ARRAY_BASE_H), 0u);
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_CONFIG1),
        AV1_MMU_CONFIG1_OUT_OF_BOUND);
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_AHB_EXCEPTION),
        AV1_MMU_AHB_CONTROL_ENABLE);
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_AHB_CONTROL),
        AV1_MMU_AHB_CONTROL_ENABLE);

    ctrl = READ_REGISTER_ULONG(
        (volatile ULONG*)(base + AV1_MMU_AHB_CONTROL));
    if (!(ctrl & AV1_MMU_AHB_CONTROL_ENABLE)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: enable readback ctrl=0x%08x — failed\n", ctrl);
        return STATUS_DEVICE_HARDWARE_ERROR;
    }
    Dev->PagingEnabled = TRUE;
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu_av1d: paging enabled (HID=RKCP%04x UID=%u PTA=0x%08x)\n",
               Dev->Hid, Dev->Uid, Dev->Domain->PtaPhys);
    return STATUS_SUCCESS;
}

NTSTATUS
RkIommuEvtPrepareHardware(_In_ WDFDEVICE Device,
                           _In_ WDFCMRESLIST ResourcesRaw,
                           _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);
    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(Device);

    NTSTATUS status = RkIommuReadAcpiId(Device, &ctx->Hid, &ctx->Uid);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: failed to read ACPI ID (0x%08x)\n", status);
        return status;
    }

    /* RKCP3571 always carries the three BSP _DSD flags. */
    ctx->FlagDisableMmuReset = TRUE;
    ctx->FlagEnableCmdRetry  = TRUE;
    ctx->FlagShootdownEntire = TRUE;

    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    PHYSICAL_ADDRESS basePhys = {0};
    ULONG totalLen = 0;
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type != CmResourceTypeMemory) continue;
        if (totalLen == 0) {
            basePhys = d->u.Memory.Start;
            totalLen = d->u.Memory.Length;
        } else if (d->u.Memory.Start.QuadPart == basePhys.QuadPart + totalLen) {
            totalLen += d->u.Memory.Length;
        }
    }
    if (!totalLen) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: no MMIO resource\n");
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    ctx->MmioBase = (volatile UCHAR*)MmMapIoSpaceEx(
        basePhys, totalLen, PAGE_READWRITE | PAGE_NOCACHE);
    ctx->MmioLength = totalLen;
    if (!ctx->MmioBase) return STATUS_INSUFFICIENT_RESOURCES;

    status = RkIommuDomainCreateAv1d(&ctx->Domain);
    if (!NT_SUCCESS(status)) {
        MmUnmapIoSpace((PVOID)ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase = NULL;
        return status;
    }

    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    InsertTailList(&g_deviceList, &ctx->ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu_av1d: RKCP%04x UID=%u ready, MMIO=%p PtaPhys=0x%08x\n",
               ctx->Hid, ctx->Uid, ctx->MmioBase, ctx->Domain->PtaPhys);
    return STATUS_SUCCESS;
}

NTSTATUS
RkIommuEvtReleaseHardware(_In_ WDFDEVICE Device,
                           _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(Device);

    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    RemoveEntryList(&ctx->ListEntry);
    InitializeListHead(&ctx->ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    ctx->PagingEnabled = FALSE;
    if (ctx->Domain) { RkIommuDomainDestroy(ctx->Domain); ctx->Domain = NULL; }
    if (ctx->MmioBase) {
        MmUnmapIoSpace((PVOID)ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase = NULL; ctx->MmioLength = 0;
    }
    return STATUS_SUCCESS;
}

NTSTATUS
RkIommuDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit)
{
    if (!g_listInitialized) {
        InitializeListHead(&g_deviceList);
        KeInitializeSpinLock(&g_deviceListLock);
        g_listInitialized = TRUE;
    }

    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = RkIommuEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = RkIommuEvtReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    WDF_OBJECT_ATTRIBUTES attr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr, RKIOMMU_DEVICE);

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, &attr, &device);
    if (!NT_SUCCESS(status)) return status;

    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(device);
    RtlZeroMemory(ctx, sizeof(*ctx));
    InitializeListHead(&ctx->ListEntry);

    {
        NTSTATUS sId = RkIommuReadAcpiId(device, &ctx->Hid, &ctx->Uid);
        if (!NT_SUCCESS(sId)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu_av1d: early ACPI ID read failed (0x%08x)\n", sId);
            return sId;
        }
    }

    WDF_INTERRUPT_CONFIG intCfg;
    WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkIommuEvtIsr, RkIommuEvtDpc);
    status = WdfInterruptCreate(device, &intCfg,
                                WDF_NO_OBJECT_ATTRIBUTES,
                                &ctx->Interrupt);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: WdfInterruptCreate failed (0x%08x) — non-fatal\n",
                   status);
    }

    return RkIommuRegisterIfc(device);
}
