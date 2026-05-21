/* driver/rkiommu_av1d/fault.c — IOMMU fault IRQ ISR and DPC (AV1D only).
 *
 * Adapted from driver/rkiommu_vdec/fault.c — v2 registers replaced with
 * AV1D equivalents:
 *   RK_MMU_INT_STATUS       → AV1_MMU_STATUS_AV1      (0x384)
 *   RK_MMU_PAGE_FAULT_ADDR  → AV1_MMU_PAGE_FAULT_ADDR_AV1 (0x380)
 *
 * AV1D has no separate INT_CLEAR register.  The fault status is cleared
 * implicitly when the codec resumes and the IOMMU services the outstanding
 * AXI transaction.  There is no ZAP_CACHE or PAGE_FAULT_DONE command on
 * AV1D — the hardware clears the IRQ on next successful table walk.
 * We pulse AV1_MMU_FLUSH to invalidate the walk cache after any fault.
 */
#include <ntddk.h>
#include <wdf.h>
#include "device.h"
#include "../shared/iommu/pgtable.h"
#include "../shared/iommu/fault.h"

/* ---------------------------------------------------------------------------
 * RkIommuEvtIsr — interrupt service routine (DIRQL)
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
BOOLEAN RkIommuEvtIsr(WDFINTERRUPT Interrupt, ULONG MessageID)
{
    UNREFERENCED_PARAMETER(MessageID);

    WDFDEVICE       device = WdfInterruptGetDevice(Interrupt);
    PRKIOMMU_DEVICE ctx    = RkIommuDeviceGet(device);

    if (!ctx->MmioBase) return FALSE;

    /* AV1D has a single MMU instance with STATUS at 0x384. */
    ULONG status = READ_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + AV1_MMU_STATUS_AV1));
    if (!(status & AV1_MMU_STATUS_AV1_IRQ_MASK)) return FALSE;

    ULONG faultIova = READ_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + AV1_MMU_PAGE_FAULT_ADDR_AV1));

    /* Pulse FLUSH to invalidate the walk cache — closest equivalent to
     * ZAP_CACHE on v2.  AV1D has no PAGE_FAULT_DONE or INT_CLEAR. */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + AV1_MMU_FLUSH), AV1_MMU_FLUSH_BIT);
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + AV1_MMU_FLUSH), 0u);

    /* Save context for the DPC. */
    ctx->FaultCtx.IntStatus = status;
    ctx->FaultCtx.FaultIova = faultIova;

    /* Queue the DPC to call the client callback at DISPATCH_LEVEL */
    WdfInterruptQueueDpcForIsr(Interrupt);
    return TRUE;
}

/* ---------------------------------------------------------------------------
 * RkIommuEvtDpc — deferred procedure call (DISPATCH_LEVEL)
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
VOID RkIommuEvtDpc(WDFINTERRUPT Interrupt, WDFOBJECT AssociatedObject)
{
    UNREFERENCED_PARAMETER(AssociatedObject);

    WDFDEVICE       device = WdfInterruptGetDevice(Interrupt);
    PRKIOMMU_DEVICE ctx    = RkIommuDeviceGet(device);

    /* Pair-atomic read under FaultLock — see rkiommu_vdec/fault.c. */
    KIRQL irql;
    KeAcquireSpinLock(&ctx->FaultLock, &irql);
    RKIOMMU_FAULT_CALLBACK cb     = ctx->FaultCb;
    PVOID                  cookie = ctx->FaultCbCookie;
    KeReleaseSpinLock(&ctx->FaultLock, irql);

    if (cb) {
        cb(cookie,
           (ULONG64)ctx->FaultCtx.FaultIova,
           ctx->FaultCtx.IntStatus);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: RKCP%04x UID=%u fault IOVA=0x%08x status=0x%08x"
                   " (no client callback registered)\n",
                   ctx->Hid, ctx->Uid,
                   ctx->FaultCtx.FaultIova, ctx->FaultCtx.IntStatus);
    }
}
