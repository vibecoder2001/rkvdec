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

    /* RK3588 codec MMUs come in pairs (read-port + write-port) sharing
     * one GIC IRQ.  Walk all instances; ack faults on every one with
     * a non-zero INT_STATUS.  If we only check MMU#0 and #1 has a
     * fault, the IRQ stays asserted forever → SoC freezes. */
    int n_mmu = (ctx->MmioLength >= 0x80) ? 2 : 1;
    BOOLEAN handled = FALSE;
    ULONG mergedStatus = 0;
    ULONG faultIova    = 0;

    for (int mi = 0; mi < n_mmu; mi++) {
        volatile UCHAR *base = ctx->MmioBase + (mi * 0x40);
        ULONG intStatus = READ_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_INT_STATUS));
        if (!(intStatus & RK_MMU_IRQ_MASK)) continue;

        handled = TRUE;
        mergedStatus |= intStatus;
        if (mi == 0 || faultIova == 0) {
            faultIova = READ_REGISTER_ULONG(
                (volatile ULONG*)(base + RK_MMU_PAGE_FAULT_ADDR));
        }

        if (intStatus & RK_MMU_IRQ_PAGE_FAULT) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG*)(base + RK_MMU_COMMAND),
                RK_MMU_CMD_ZAP_CACHE);
            WRITE_REGISTER_ULONG(
                (volatile ULONG*)(base + RK_MMU_COMMAND),
                RK_MMU_CMD_PAGE_FAULT_DONE);
        }
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_INT_CLEAR),
            intStatus & RK_MMU_IRQ_MASK);
    }
    if (!handled) return FALSE;

    /* Save merged context for the DPC. */
    ctx->FaultCtx.IntStatus = mergedStatus;
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

    RKIOMMU_FAULT_CALLBACK cb     = ctx->FaultCb;
    PVOID                  cookie = ctx->FaultCbCookie;

    if (cb) {
        cb(cookie,
           (ULONG64)ctx->FaultCtx.FaultIova,
           ctx->FaultCtx.IntStatus);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu: RKCP%04x UID=%u fault IOVA=0x%08x status=0x%08x"
                   " (no client callback registered)\n",
                   ctx->Hid, ctx->Uid,
                   ctx->FaultCtx.FaultIova, ctx->FaultCtx.IntStatus);
    }
}
