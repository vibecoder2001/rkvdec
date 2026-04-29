/* driver/rkiommu/device.c — per-instance RKIOMMU_DEVICE lifecycle.
 *
 * Responsibilities:
 *   - parse HID + _UID from the ACPI hardware-ID multi-sz list
 *   - allocate an RKIOMMU_DOMAIN (page directory + IOVA bitmap)
 *   - map the MMIO window from _CRS resource 0
 *   - connect the IRQ (WdfInterruptCreate)
 *   - register itself in the global g_deviceList so MapMdl can find it
 *   - expose RkIommuEnable() for lazy paging activation
 *
 * Register offsets used here are defined in pgtable.h and sourced from
 *   torvalds/linux drivers/iommu/rockchip-iommu.c (master, 2024).
 */
#include <ntddk.h>
#include <wdf.h>
#include "device.h"
#include "pgtable.h"
#include "fault.h"

/* ---------------------------------------------------------------------------
 * Global instance list — all RKIOMMU_DEVICE instances live here.
 * Protected by g_deviceListLock (DISPATCH_LEVEL spin lock).
 * --------------------------------------------------------------------------- */
LIST_ENTRY  g_deviceList;
KSPIN_LOCK  g_deviceListLock;

static BOOLEAN g_listInitialized = FALSE;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
EVT_WDF_DEVICE_PREPARE_HARDWARE  RkIommuEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  RkIommuEvtReleaseHardware;

extern NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device);  /* in ifc.c */

/* ---------------------------------------------------------------------------
 * ACPI HID/UID parse
 *
 * Walks the multi-sz hardware-ID list looking for "ACPI\RKCP35xx".
 * Extracts the last 4 hex digits as the HID (e.g. RKCP3570 → 0x3570).
 * Reads _UID via DevicePropertyUINumber (defaults to 0 if absent).
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuReadAcpiId(_In_ WDFDEVICE Device,
                  _Out_ PUINT32 Hid,
                  _Out_ PUINT32 Uid)
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
                ULONG uid = 0;
                IoGetDeviceProperty(pdo, DevicePropertyUINumber,
                                    sizeof(uid), &uid, &size);
                *Uid = uid;
                return STATUS_SUCCESS;
            }
        }
        cursor += len + 1;
    }
    return STATUS_INVALID_DEVICE_REQUEST;
}

/* ---------------------------------------------------------------------------
 * RkIommuEnable — activate IOMMU paging on the hardware.
 *
 * Writes DTE_ADDR, then issues ENABLE_PAGING.  Called lazily by MapMdl on
 * the first successful map so we don't touch hardware before a client is
 * ready.
 *
 * Phase 2: leave disabled until first client maps.
 * Uncomment the body or call this function from MapMdl to activate.
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
VOID RkIommuEnable(PRKIOMMU_DEVICE Dev)
{
    if (Dev->PagingEnabled) return;
    if (!Dev->MmioBase)     return;
    if (!Dev->Domain)       return;

    /* Write the page-directory physical address */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_DTE_ADDR),
        Dev->Domain->PdPhys);

    /* Enable auto-gating */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_AUTO_GATING),
        1u);

    /* Enable IRQs */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_INT_MASK),
        RK_MMU_IRQ_MASK);

    /* Issue ENABLE_PAGING command */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_COMMAND),
        RK_MMU_CMD_ENABLE_PAGING);

    Dev->PagingEnabled = TRUE;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu: RKCP%04x UID=%u paging enabled, DTE_ADDR=0x%08x\n",
               Dev->Hid, Dev->Uid, Dev->Domain->PdPhys);
}

/* ---------------------------------------------------------------------------
 * EvtPrepareHardware
 * --------------------------------------------------------------------------- */
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
                   "rkiommu: failed to read ACPI ID (0x%08x)\n", status);
        return status;
    }

    /* Map the first MMIO resource */
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            ctx->MmioBase = (volatile UCHAR*)MmMapIoSpaceEx(
                d->u.Memory.Start,
                d->u.Memory.Length,
                PAGE_READWRITE | PAGE_NOCACHE);
            ctx->MmioLength = d->u.Memory.Length;
            break;
        }
    }
    if (!ctx->MmioBase) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu: no MMIO resource\n");
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    /* Allocate the domain (page directory + IOVA bitmap) */
    status = RkIommuDomainCreate(&ctx->Domain);
    if (!NT_SUCCESS(status)) {
        MmUnmapIoSpace((PVOID)ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase = NULL;
        return status;
    }

    /* Register in the global list */
    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    InsertTailList(&g_deviceList, &ctx->ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu: RKCP%04x UID=%u ready, MMIO=%p PdPhys=0x%08x\n",
               ctx->Hid, ctx->Uid, ctx->MmioBase, ctx->Domain->PdPhys);

    /* Phase 2: leave IOMMU paging disabled until first client maps.
     * RkIommuEnable(ctx) will be called lazily from the MapMdl path. */

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * EvtReleaseHardware
 * --------------------------------------------------------------------------- */
NTSTATUS
RkIommuEvtReleaseHardware(_In_ WDFDEVICE Device,
                           _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(Device);

    /* Remove from global list */
    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    RemoveEntryList(&ctx->ListEntry);
    InitializeListHead(&ctx->ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    /* Disable IOMMU if it was enabled */
    if (ctx->PagingEnabled && ctx->MmioBase) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(ctx->MmioBase + RK_MMU_INT_MASK), 0u);
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(ctx->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_DISABLE_PAGING);
        ctx->PagingEnabled = FALSE;
    }

    if (ctx->Domain) {
        RkIommuDomainDestroy(ctx->Domain);
        ctx->Domain = NULL;
    }

    if (ctx->MmioBase) {
        MmUnmapIoSpace((PVOID)ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase   = NULL;
        ctx->MmioLength = 0;
    }

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuDeviceCreate — called from EvtDeviceAdd
 * --------------------------------------------------------------------------- */
NTSTATUS
RkIommuDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit)
{
    /* Ensure the global list is initialized (idempotent, called once) */
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

    /* Create the interrupt object (ISR + DPC wired in fault.c) */
    WDF_INTERRUPT_CONFIG intCfg;
    WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkIommuEvtIsr, RkIommuEvtDpc);

    status = WdfInterruptCreate(device, &intCfg,
                                WDF_NO_OBJECT_ATTRIBUTES,
                                &ctx->Interrupt);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu: WdfInterruptCreate failed (0x%08x)\n", status);
        return status;
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu: device created\n");

    return RkIommuRegisterIfc(device);
}
