/* driver/rkiommu/fault.c — IOMMU fault IRQ ISR and DPC.
 *
 * The ISR fires at DIRQL when the IOMMU raises an IRQ for a page fault or
 * bus error.  It:
 *   1. Reads INT_STATUS to confirm this interrupt belongs to us.
 *   2. Reads PAGE_FAULT_ADDR to capture the faulting IOVA.
 *   3. Acknowledges the IRQ via INT_CLEAR.
 *   4. Issues PAGE_FAULT_DONE so the IOMMU stops stalling.
 *   5. Saves state into the per-device RKIOMMU_FAULT_CTX.
 *   6. Queues the DPC.
 *
 * The DPC fires at DISPATCH_LEVEL and calls the client-registered fault
 * callback (if any) with the saved context.
 *
 * Register offsets from pgtable.h (sourced from
 *   torvalds/linux drivers/iommu/rockchip-iommu.c, master 2024).
 */
#include <ntddk.h>
#include <wdf.h>
#include "device.h"
#include "pgtable.h"
#include "fault.h"

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

    /* Check masked interrupt status — if zero, not our interrupt */
    ULONG intStatus = READ_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + RK_MMU_INT_STATUS));
    if (!(intStatus & RK_MMU_IRQ_MASK)) return FALSE;

    /* Capture the faulting IOVA */
    ULONG faultIova = READ_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + RK_MMU_PAGE_FAULT_ADDR));

    /* Save context for the DPC */
    ctx->FaultCtx.IntStatus  = intStatus;
    ctx->FaultCtx.FaultIova  = faultIova;

    /* Acknowledge (clear) the interrupt */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(ctx->MmioBase + RK_MMU_INT_CLEAR),
        intStatus & RK_MMU_IRQ_MASK);

    /* If it was a page fault, issue PAGE_FAULT_DONE so the IOMMU unstalls */
    if (intStatus & RK_MMU_IRQ_PAGE_FAULT) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(ctx->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_PAGE_FAULT_DONE);
    }

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

    RKIOMMU_FAULT_CALLBACK cb     = ctx->FaultCb;
    PVOID                  cookie = ctx->FaultCbCookie;

    if (cb) {
        cb(cookie,
           (ULONG64)ctx->FaultCtx.FaultIova,
           ctx->FaultCtx.IntStatus);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkiommu: RKCP%04x UID=%u fault IOVA=0x%08x status=0x%08x"
                   " (no client callback registered)\n",
                   ctx->Hid, ctx->Uid,
                   ctx->FaultCtx.FaultIova, ctx->FaultCtx.IntStatus);
    }
}
