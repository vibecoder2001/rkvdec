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
UINT32 RkIommuQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo);  /* below */

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
                /* IRP_MN_QUERY_ID(BusQueryInstanceID) returns the _UID for
                 * ACPI devices.  DevicePropertyUINumber is not reliably
                 * populated by acpi.sys, so we use the IRP path. */
                *Uid = RkIommuQueryAcpiUid(pdo);
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
 * Phase 3a: real MMIO programming, with three BSP-mandated flags:
 *   FlagDisableMmuReset   — skip the MMU reset command (RK3588 rkvdec/enc/AV1)
 *   FlagEnableCmdRetry    — retry ENABLE_PAGING up to 3 times with status poll
 *   FlagShootdownEntire   — ZAP_CACHE used for TLB flush (set in MapMdl/UnmapMdl)
 *
 * Called lazily by MapMdl on the first successful map.
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuEnable(PRKIOMMU_DEVICE Dev)
{
    if (!Dev->MmioBase || !Dev->Domain) return STATUS_DEVICE_NOT_READY;
    if (Dev->PagingEnabled)             return STATUS_SUCCESS;

    /* 1. Optionally reset the MMU.  BSP flag says skip on RK3588 rkvdec/rkvenc/AV1. */
    if (!Dev->FlagDisableMmuReset) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_RESET);
        KeStallExecutionProcessor(50);
    }

    /* 2. Program the page-directory base. */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_DTE_ADDR),
        Dev->Domain->PdPhys);

    /* 3. Enable auto-gating. */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_AUTO_GATING),
        1u);

    /* 4. Enable IRQs. */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_INT_MASK),
        RK_MMU_IRQ_MASK);

    /* Diagnostic: read back DTE_ADDR + STATUS + INT_MASK to verify the
     * MMIO writes above actually landed.  If the IOMMU is mis-clocked, the
     * read returns 0xFFFFFFFF or stale; if DTE_ADDR readback != PdPhys,
     * the IOMMU isn't in our page table. */
    {
        ULONG dteRb  = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_DTE_ADDR));
        ULONG statRb = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_STATUS));
        ULONG maskRb = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_INT_MASK));
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu: pre-enable readback HID=RKCP%04x UID=%u "
                   "DTE_ADDR=0x%08x (want 0x%08x) STATUS=0x%08x INT_MASK=0x%08x\n",
                   Dev->Hid, Dev->Uid, dteRb, Dev->Domain->PdPhys, statRb, maskRb);
    }

    /* 5. Enable paging.  With cmd-retry: write up to 3 times, polling status. */
    ULONG attempts = Dev->FlagEnableCmdRetry ? 3u : 1u;
    for (ULONG i = 0; i < attempts; i++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_ENABLE_PAGING);
        ULONG st = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_STATUS));
        if ((st & RK_MMU_STATUS_PAGING_ENABLED) != 0) {
            Dev->PagingEnabled = TRUE;
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu: paging enabled (HID=RKCP%04x UID=%u attempt=%u)\n",
                       Dev->Hid, Dev->Uid, i + 1u);
            return STATUS_SUCCESS;
        }
        KeStallExecutionProcessor(20);
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu: paging-enable timeout (HID=RKCP%04x UID=%u)\n",
               Dev->Hid, Dev->Uid);
    return STATUS_DEVICE_HARDWARE_ERROR;
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

    /* BSP _DSD flags applied by the firmware.  RKCP3570 UID 7..10 (the
     * encoder/decoder IOMMUs RE0M, RE1M, RD0M, RD1M) and RKCP3571 (A1MU)
     * all carry the three flags per the ACPI source; the older block
     * IOMMUs (VPMU, JDMU, J0..J3MU, IEMU) do not. */
    if ((ctx->Hid == 0x3570 && ctx->Uid >= 7 && ctx->Uid <= 10) ||
        ctx->Hid == 0x3571) {
        ctx->FlagDisableMmuReset = TRUE;
        ctx->FlagEnableCmdRetry  = TRUE;
        ctx->FlagShootdownEntire = TRUE;
    } else {
        ctx->FlagDisableMmuReset = FALSE;
        ctx->FlagEnableCmdRetry  = FALSE;
        ctx->FlagShootdownEntire = FALSE;
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

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
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

    /* Do NOT touch MMIO here.  By the time EvtReleaseHardware runs (driver
     * uninstall, surprise-removal, sleep), the codec power-domain that gates
     * this IOMMU's MMIO may already have been torn down by the rkmpp / ccu
     * stack — touching the registers in that window trips a synchronous
     * external abort and bugcheck 0x124 (WHEA_UNCORRECTABLE_ERROR, sub 0x12).
     *
     * The page-table RAM is freed by RkIommuDomainDestroy below, and the
     * IOMMU hardware state is reset by RkIommuEnable on the next load, so
     * skipping the disable-paging / int-mask writes here costs nothing. */
    ctx->PagingEnabled = FALSE;

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

    /* Read ACPI _HID/_UID up front so RkIommuRegisterIfc below publishes
     * the correct (Hid, Uid) in the in-kernel interface struct.  WDF
     * snapshots the interface contents at AddQueryInterface time, so
     * filling these in later (e.g. from EvtPrepareHardware) is too late —
     * consumers would see Hid=0/Uid=0 and the topology match in
     * rkmpp/ifc_client.c would never succeed. */
    {
        NTSTATUS sId = RkIommuReadAcpiId(device, &ctx->Hid, &ctx->Uid);
        if (!NT_SUCCESS(sId)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu: early ACPI ID read failed (0x%08x)\n", sId);
            return sId;
        }
    }

    /* Wire the IOMMU fault interrupt.  Phase 3a saw spurious WHEA-ing on
     * IRQ entry, but that was a symptom of the broader codec MMIO gating
     * issue (the ISR touched not-yet-clocked MMIO).  With Phase 3b's
     * cluster bring-up landed (REVISION reads cleanly), the path is safe.
     *
     * Note: per the BSP DSD `rockchip,master-handle-irq = 1`, RD0M/RD1M's
     * fault interrupts are routed to the master rkvdec core's IRQ rather
     * than the IOMMU's own line.  The vector here may therefore never
     * fire for those instances; that's expected, not a bug.  Other IOMMU
     * UIDs (VPMU, JDMU, ...) follow the standard path. */
    WDF_INTERRUPT_CONFIG intCfg;
    WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkIommuEvtIsr, RkIommuEvtDpc);

    status = WdfInterruptCreate(device, &intCfg,
                                WDF_NO_OBJECT_ATTRIBUTES,
                                &ctx->Interrupt);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu: WdfInterruptCreate failed (0x%08x)\n", status);
        /* Non-fatal: many of our IOMMU instances will never fire an IRQ
         * (master-handle-irq), so refusing to load on this would deny
         * service for the working ones.  Continue without an ISR. */
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu: device created\n");

    return RkIommuRegisterIfc(device);
}

/* IRP_MN_QUERY_ID(BusQueryInstanceID) helper.  Sends a synchronous IRP up
 * the device's PnP stack to retrieve the per-enumerator instance ID and
 * parses it as a decimal integer.  For ACPI devices this returns _UID. */
typedef struct _RKIOMMU_QID_CTX { KEVENT Done; NTSTATUS Status; } RKIOMMU_QID_CTX;

_Use_decl_annotations_
static IO_COMPLETION_ROUTINE RkIommuQidCompletion;
static NTSTATUS RkIommuQidCompletion(PDEVICE_OBJECT DeviceObject, PIRP Irp, PVOID Ctx)
{
    UNREFERENCED_PARAMETER(DeviceObject);
    RKIOMMU_QID_CTX *c = (RKIOMMU_QID_CTX *)Ctx;
    NT_ASSERT(c != NULL);
    _Analysis_assume_(c != NULL);
    c->Status = Irp->IoStatus.Status;
    KeSetEvent(&c->Done, IO_NO_INCREMENT, FALSE);
    return STATUS_MORE_PROCESSING_REQUIRED;
}

UINT32 RkIommuQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo)
{
    PDEVICE_OBJECT topDev = IoGetAttachedDeviceReference(Pdo);
    if (!topDev) return 0;

    PIRP irp = IoAllocateIrp(topDev->StackSize, FALSE);
    if (!irp) {
        ObDereferenceObject(topDev);
        return 0;
    }

    RKIOMMU_QID_CTX ctx;
    KeInitializeEvent(&ctx.Done, NotificationEvent, FALSE);
    ctx.Status = STATUS_NOT_SUPPORTED;

    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    IoSetCompletionRoutine(irp, RkIommuQidCompletion, &ctx, TRUE, TRUE, TRUE);

    PIO_STACK_LOCATION sl = IoGetNextIrpStackLocation(irp);
    sl->MajorFunction = IRP_MJ_PNP;
    sl->MinorFunction = IRP_MN_QUERY_ID;
    sl->Parameters.QueryId.IdType = BusQueryInstanceID;

    NTSTATUS status = IoCallDriver(topDev, irp);
    if (status == STATUS_PENDING) {
        KeWaitForSingleObject(&ctx.Done, Executive, KernelMode, FALSE, NULL);
        status = ctx.Status;
    }

    UINT32 uid = 0;
    if (NT_SUCCESS(status)) {
        PCWSTR instId = (PCWSTR)irp->IoStatus.Information;
        if (instId) {
            UNICODE_STRING us;
            RtlInitUnicodeString(&us, instId);
            /* acpi.sys formats numeric _UID values as hex without "0x"
             * prefix.  _UID=10 returns "A", _UID=11 returns "B", etc.
             * Parse base 16 — single-digit UIDs (0-9) match both bases. */
            ULONG val = 0;
            if (NT_SUCCESS(RtlUnicodeStringToInteger(&us, 16, &val))) {
                uid = val;
            }
            ExFreePool((PVOID)instId);
        }
    }

    IoFreeIrp(irp);
    ObDereferenceObject(topDev);
    return uid;
}
