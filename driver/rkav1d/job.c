/* driver/rkav1d/job.c — register-list job submission queue for rkav1d.sys.
 *
 * AV1D (RKCP3560) only — no rkvdec2 paths.
 *
 * Phase 2: software-completion stub. Phase 3 (real decode) replaces
 * RkMppJobStart() with a hardware-kick implementation.
 *
 * Decision: no-op kick mechanism.
 * SUBMIT_JOB queues a job and immediately schedules a software DPC that
 * signals completion.  The real WDFINTERRUPT (ISR + DPC) is wired at
 * PrepareHardware so Phase 3 only needs to change RkMppJobStart().
 * In Phase 2 the ISR will never fire because the hardware is never kicked.
 */

#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "job.h"
#include "profile.h"      /* RKMPP_CODEC_PERSONALITY for per-codec dispatch */
#include "../shared/rkmpp/bufpool.h"      /* for RkMppBufLookupIova (iova substitution) */
#include "../shared/rkmpp/ifc_client.h"   /* for RKMPP_CCU_INTERFACE via RkMppGetCcuIfc */

extern PRKIOMMU_INTERFACE RkMppGetIommuIfc(_In_ WDFDEVICE Device);

/* Defined in device.c — accessor pair for RKMPP_DEVICE.NeedsCoreReset. */
extern LONG RkMppQueryAndClearNeedsCoreReset(_In_ WDFDEVICE Device);
extern VOID RkMppSetNeedsCoreReset(_In_ WDFDEVICE Device);

/* Tracing wrapper for codec MMIO writes — matches BSP's
 * mpp_dev_debug=DEBUG_SET_REG output format so the two traces can be
 * diffed directly.  Set to 0 to disable. */
#define RKMPP_TRACE_WRITES 0
#if RKMPP_TRACE_WRITES
#define TRACED_WRITE_ULONG(addr, val)                                     \
    do {                                                                  \
        ULONG _v = (val);                                                 \
        ULONG_PTR _a = (ULONG_PTR)(addr);                                 \
        ULONG _swreg_off = (ULONG)(_a - (ULONG_PTR)mmio);                \
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,               \
                   "rkav1d: write reg[%03d]: %04x: 0x%08x\n",             \
                   _swreg_off / 4, _swreg_off, _v);                       \
        WRITE_REGISTER_ULONG((volatile ULONG *)(_a), _v);                 \
    } while (0)
#else
#define TRACED_WRITE_ULONG(addr, val) WRITE_REGISTER_ULONG((volatile ULONG*)(addr), val)
#endif

/* -----------------------------------------------------------------------
 * AV1 (vdpu / hal_av1d_vdpu — not vdpu383) MMIO offsets, sourced from
 * BSP mpp_av1dec.c, hal_av1d_vdpu_reg.h, and av1_bringup_table memory:
 *   - VCD window starts at MmioBase (no SWREG prefix; offset = idx * 4).
 *   - REG0 (offset 0x000) is the IP version/build register — read for
 *     AXI-drain barrier (probe returned 0x80019000).
 *   - Kick + IRQ status share offset 0x004 (swreg1 = idx 1).
 *   - err_mask 0x7e000 (bits 13..18: BUS_ERROR | BUF_EMPTY | ASO_ERROR |
 *     STRM_ERROR | SLICE | TIMEOUT).
 *   - PIC_INF (frame done) at BIT(24).
 *   - PERF_WORKING_CNT lives at swreg263 = byte offset 0x41C (matches
 *     rkvdec2 family convention; vdpu_av1d shares the IP-family stat
 *     register layout with vdpu38x). The earlier value 0x518 (swreg326)
 *     pointed at sw_pp_out_ybase_msb — a static PP base address that
 *     never changes between reads, so the polled drain saw "stable" on
 *     its first sample and exited after the 200 µs minimum instead of
 *     waiting for true codec idle.
 * --------------------------------------------------------------------- */
#define AV1D_REVISION_OFFSET          0x000u
#define AV1D_REG_KICK_OFFSET          0x004u
#define AV1D_REG_KICK_BIT             0x1u
#define AV1D_INT_STATUS_OFFSET        0x004u
#define AV1D_INT_DEC_IRQ              (1u << 8)
#define AV1D_INT_DEC_RDY_INT          (1u << 12)
#define AV1D_INT_BUS_ERROR            (1u << 13)
#define AV1D_INT_BUF_EMPTY            (1u << 14)
#define AV1D_INT_ASO_ERROR            (1u << 15)
#define AV1D_INT_STRM_ERROR           (1u << 16)
#define AV1D_INT_SLICE                (1u << 17)
#define AV1D_INT_TIMEOUT              (1u << 18)
/* Done = clean RDY (bit 12) OR any error/timeout bit (13..18).  Earlier
 * mask used bit 24 (PIC_INF) but that's a vdpu383 (newer-IP) signal;
 * vdpu (RK3588 av1d) reserves bits 24+.  With the old mask, kicks that
 * completed cleanly with hwstatus=0x1100 (dec_irq + dec_rdy_int) made
 * the poller time out instead of completing — we observed this in the
 * 2026-05-05 test on av1_720p.ivf. */
#define AV1D_INT_DONE_MASK            (AV1D_INT_DEC_RDY_INT | 0x7e000u)
#define AV1D_INT_ERROR_MASK           (0x7e000u)
#define AV1D_PERF_WORKING_CNT_OFFSET  0x41Cu

/* Per-codec MMIO geometry table.  Job.c's hot path reads from this
 * struct so the kick / poll / drain code stays consistent. */
typedef struct _RKMPP_CODEC_OPS {
    /* Byte offset added to a regbuilder swreg index (idx*4) to land on
     * the actual MMIO register.  AV1 has no prefix. */
    ULONG SwregBase;
    /* INT_STATUS register offset (relative to MmioBase). */
    ULONG IntStatusOffset;
    /* Bits in INT_STATUS that signal "kick complete" (any). */
    ULONG IntDoneMask;
    /* Bits in INT_STATUS that classify a complete kick as error. */
    ULONG IntErrorMask;
    /* PERF_WORKING_CNT register offset for polled AXI drain. */
    ULONG PerfWorkingCntOffset;
    /* REVISION/version register — side-effect-free read used to force
     * AXI drain after DEC_RDY. For AV1 the IP version lives at
     * REG0 (offset 0x000). */
    ULONG RevisionOffset;
    /* Kick (dec_e) register offset and bit. */
    ULONG KickRegOffset;
    ULONG KickRegBit;
} RKMPP_CODEC_OPS;

static const RKMPP_CODEC_OPS g_ops = {
    .SwregBase             = 0,                /* offset = idx*4 directly */
    .IntStatusOffset       = AV1D_INT_STATUS_OFFSET,
    .IntDoneMask           = AV1D_INT_DONE_MASK,
    .IntErrorMask          = AV1D_INT_ERROR_MASK,
    .PerfWorkingCntOffset  = AV1D_PERF_WORKING_CNT_OFFSET,
    .RevisionOffset        = AV1D_REVISION_OFFSET,
    .KickRegOffset         = AV1D_REG_KICK_OFFSET,
    .KickRegBit            = AV1D_REG_KICK_BIT,
};

/* Forward declarations */
static VOID RkMppJobStart(_In_ WDFDEVICE Device, _In_ RKMPP_JOB *Job);
static VOID RkMppJobComplete(_In_ WDFDEVICE Device,
                             _In_ NTSTATUS Result,
                             _In_ UINT32 HardwareStatus);
static KSTART_ROUTINE RkMppPollerThread;

/* -----------------------------------------------------------------------
 * Accessors for device context — implemented in device.c.
 * --------------------------------------------------------------------- */
extern PRKMPP_JOB_QUEUE    RkMppGetJobQueue(_In_ WDFDEVICE Device);
extern PRKMPP_CCU_INTERFACE RkMppGetCcuIfc(_In_ WDFDEVICE Device);
extern PVOID               RkMppGetMmioBase(_In_ WDFDEVICE Device);
extern ULONG               RkMppGetMmioLength(_In_ WDFDEVICE Device);
extern PVOID               RkMppGetMmioWindow(_In_ WDFDEVICE Device,
                                              _In_ UINT32 Index,
                                              _Out_opt_ PULONG Length);

/* -----------------------------------------------------------------------
 * Queue initialisation
 * --------------------------------------------------------------------- */

VOID
RkMppJobQueueInit(_In_ WDFDEVICE Device, _Inout_ RKMPP_JOB_QUEUE *Queue)
{
    KeInitializeSpinLock(&Queue->Lock);
    InitializeListHead(&Queue->Pending);
    InitializeListHead(&Queue->Completed);
    Queue->InFlight = NULL;
    Queue->NextId   = 0;
    Queue->Device   = Device;
    Queue->Interrupt = NULL;
    RtlZeroMemory(Queue->PrevNonzeroMask, sizeof(Queue->PrevNonzeroMask));
    Queue->Draining = 0;

    /* Auto-reset kick: each Set wakes the poller exactly once. */
    KeInitializeEvent(&Queue->KickEvent, SynchronizationEvent, FALSE);
    /* Notification exit: stays signalled so the poller's wait returns
     * even if KickEvent was just set racing with teardown. */
    KeInitializeEvent(&Queue->ExitEvent, NotificationEvent, FALSE);
    Queue->PollerThread = NULL;

    HANDLE threadHandle = NULL;
    OBJECT_ATTRIBUTES oa;
    InitializeObjectAttributes(&oa, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
    NTSTATUS status = PsCreateSystemThread(&threadHandle, THREAD_ALL_ACCESS,
                                           &oa, NULL, NULL,
                                           RkMppPollerThread, Queue);
    if (NT_SUCCESS(status)) {
        ObReferenceObjectByHandle(threadHandle, THREAD_ALL_ACCESS,
                                  *PsThreadType, KernelMode,
                                  (PVOID*)&Queue->PollerThread, NULL);
        ZwClose(threadHandle);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkav1d: poller thread create failed 0x%08x\n", status);
    }
}

VOID
RkMppJobQueueTeardown(_Inout_ RKMPP_JOB_QUEUE *Queue)
{
    if (Queue->PollerThread) {
        KeSetEvent(&Queue->ExitEvent, IO_NO_INCREMENT, FALSE);
        KeWaitForSingleObject(Queue->PollerThread, Executive, KernelMode,
                              FALSE, NULL);
        ObDereferenceObject(Queue->PollerThread);
        Queue->PollerThread = NULL;
    }
}

/* -----------------------------------------------------------------------
 * RkMppJobsDrainOwner — drain the queue of jobs belonging to a closing
 * file-object (process exit / handle close).
 *
 * Without this, a process can submit a job and exit before the codec
 * finishes the kick.  The next IOCTL caller's EvtFileCleanup frees the
 * file's buffer pool — which includes the bitstream / output / colmv
 * buffers the in-flight job is still DMA'ing to/from.  IOMMU then
 * tears down those mappings and the codec writes to garbage iovas
 * (or page-faults, or wedges the box).
 *
 * Strategy:
 *   1. Under the queue lock, splice the Pending list, removing the
 *      owner's jobs.  These haven't been kicked yet — just free them.
 *   2. Splice the Completed list the same way.
 *   3. If InFlight->Owner == File, wait up to TimeoutMs for completion.
 *      The poller thread keeps running and will move the job from
 *      InFlight to Completed once INT_RDY (or timeout) hits.  We then
 *      remove + free our matched entry from Completed.
 *
 * Returns STATUS_SUCCESS regardless; *InFlightTimedOut is TRUE when the
 * in-flight wait ran out, so the caller (EvtFileCleanup) can decide
 * whether to force-reset before freeing buffers.
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobsDrainOwner(_In_ WDFDEVICE Device,
                    _In_ WDFFILEOBJECT File,
                    _In_ ULONG TimeoutMs,
                    _Out_ BOOLEAN *InFlightTimedOut)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    KIRQL old;
    LIST_ENTRY toFree;
    InitializeListHead(&toFree);
    *InFlightTimedOut = FALSE;

    /* Phase 1: under-lock walk of Pending + Completed lists.  Also
     * capture a pointer to the in-flight job (if it's ours) so we know
     * whether to wait afterwards.  We must NOT touch InFlight under
     * the lock here — the poller fills/clears it. */
    KeAcquireSpinLock(&q->Lock, &old);

    PLIST_ENTRY entry = q->Pending.Flink;
    while (entry != &q->Pending) {
        RKMPP_JOB *cand = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
        PLIST_ENTRY next = entry->Flink;
        if (cand->Owner == File) {
            RemoveEntryList(entry);
            InsertTailList(&toFree, entry);
        }
        entry = next;
    }

    entry = q->Completed.Flink;
    while (entry != &q->Completed) {
        RKMPP_JOB *cand = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
        PLIST_ENTRY next = entry->Flink;
        if (cand->Owner == File) {
            RemoveEntryList(entry);
            InsertTailList(&toFree, entry);
        }
        entry = next;
    }

    /* Snapshot in-flight ownership while still under the lock. */
    RKMPP_JOB *inFlight = q->InFlight;
    BOOLEAN inFlightIsOurs = (inFlight != NULL && inFlight->Owner == File);

    /* Capture the Done event pointer for the wait below — it's safe to
     * dereference once we have the spinlock-stable inFlight pointer
     * because the poller never frees an in-flight job while it's still
     * marked in-flight; it only moves to Completed (then we'd own it
     * via the next under-lock walk). */
    KEVENT *doneEvt = inFlightIsOurs ? &inFlight->Done : NULL;

    KeReleaseSpinLock(&q->Lock, old);

    /* Phase 2: free the pending+completed-of-ours list (no lock). */
    while (!IsListEmpty(&toFree)) {
        PLIST_ENTRY e = RemoveHeadList(&toFree);
        RKMPP_JOB *j = CONTAINING_RECORD(e, RKMPP_JOB, Link);
        ExFreePoolWithTag(j, 'JppM');
    }

    /* Phase 3: wait for the in-flight job (if it was ours) to complete
     * naturally.  Most decode kicks finish in <30ms; a 500ms cap is
     * generous and keeps process-exit latency bounded. */
    if (inFlightIsOurs && doneEvt) {
        LARGE_INTEGER timeout;
        timeout.QuadPart = -((LONGLONG)TimeoutMs * 10000);
        NTSTATUS w = KeWaitForSingleObject(doneEvt, Executive, KernelMode,
                                           FALSE, &timeout);
        if (w == STATUS_TIMEOUT) {
            *InFlightTimedOut = TRUE;
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                       "rkav1d: file-cleanup in-flight wait timed out\n");
            /* Don't free the in-flight job here — the poller still owns
             * it and will eventually move it to Completed.  Leaking the
             * struct is acceptable; the alternative (freeing while the
             * poller holds a pointer) is a use-after-free.  In practice
             * the next decode session triggers a core reset which
             * unblocks the wedged poll. */
        } else {
            /* In-flight completed cleanly.  It's now on the Completed
             * list — pull it off so it doesn't leak waiting for a
             * WaitJob caller that's never coming. */
            KeAcquireSpinLock(&q->Lock, &old);
            entry = q->Completed.Flink;
            while (entry != &q->Completed) {
                RKMPP_JOB *cand = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
                PLIST_ENTRY next = entry->Flink;
                if (cand->Owner == File) {
                    RemoveEntryList(entry);
                    InsertTailList(&toFree, entry);
                }
                entry = next;
            }
            KeReleaseSpinLock(&q->Lock, old);
            while (!IsListEmpty(&toFree)) {
                PLIST_ENTRY e = RemoveHeadList(&toFree);
                RKMPP_JOB *j = CONTAINING_RECORD(e, RKMPP_JOB, Link);
                ExFreePoolWithTag(j, 'JppM');
            }
        }
    }

    return STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------
 * RkMppJobQueueQuiesce — block tail-chain new-kick starts and wait for
 * any in-flight job to drain naturally.  Intended for EvtDeviceD0Exit
 * (PASSIVE_LEVEL).  Paired 1:1 with RkMppJobQueueResume on D0Entry.
 *
 * The locking pattern mirrors RkMppJobsDrainOwner: snapshot the in-flight
 * job's Done event under the spinlock (so the poller can't free the job
 * between snapshot and wait — it only moves InFlight to Completed, never
 * frees it from under us while the poller is alive), then wait outside
 * the lock with a bounded timeout.
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobQueueQuiesce(_Inout_ RKMPP_JOB_QUEUE *Queue,
                     _In_ ULONG TimeoutMs)
{
    NT_ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);

    /* Set Draining first so any RkMppJobComplete that fires between here
     * and the wait below will NOT promote a new InFlight from Pending. */
    InterlockedExchange(&Queue->Draining, 1);

    KIRQL old;
    KeAcquireSpinLock(&Queue->Lock, &old);
    RKMPP_JOB *inFlight = Queue->InFlight;
    KEVENT *doneEvt = inFlight ? &inFlight->Done : NULL;
    KeReleaseSpinLock(&Queue->Lock, old);

    if (doneEvt == NULL) {
        return STATUS_SUCCESS;
    }

    LARGE_INTEGER timeout;
    timeout.QuadPart = -((LONGLONG)TimeoutMs * 10000);
    NTSTATUS w = KeWaitForSingleObject(doneEvt, Executive, KernelMode,
                                       FALSE, &timeout);
    if (w == STATUS_TIMEOUT) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkav1d: D0Exit Quiesce in-flight wait timed out\n");
        return STATUS_TIMEOUT;
    }
    return STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------
 * RkMppJobQueueResume — clear Draining and restart the chain if any jobs
 * are pending.  Intended for EvtDeviceD0Entry (PASSIVE_LEVEL).  Mirrors
 * the Pending → InFlight promotion in RkMppJobComplete.
 * --------------------------------------------------------------------- */

VOID
RkMppJobQueueResume(_In_ WDFDEVICE Device,
                    _Inout_ RKMPP_JOB_QUEUE *Queue)
{
    NT_ASSERT(KeGetCurrentIrql() == PASSIVE_LEVEL);

    InterlockedExchange(&Queue->Draining, 0);

    KIRQL old;
    RKMPP_JOB *next = NULL;
    KeAcquireSpinLock(&Queue->Lock, &old);
    if (Queue->InFlight == NULL && !IsListEmpty(&Queue->Pending)) {
        PLIST_ENTRY entry = RemoveHeadList(&Queue->Pending);
        next = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
        Queue->InFlight = next;
    }
    KeReleaseSpinLock(&Queue->Lock, old);

    if (next) {
        RkMppJobStart(Device, next);
    }
}

/* -----------------------------------------------------------------------
 * RkMppPollerThread — per-device kernel thread that polls INT_STATUS for
 * job completion.
 *
 * Runs at PASSIVE_LEVEL; waits on (KickEvent | ExitEvent).  A kick means
 * RkMppJobStart just wrote the register list and asserted dec_e.  We
 * poll INT_STATUS up to a deadline; on done bit set we ack the status,
 * call RkMppJobComplete, and loop.  Exit unblocks teardown.
 * --------------------------------------------------------------------- */

static VOID
RkMppPollerThread(_In_ PVOID Context)
{
    RKMPP_JOB_QUEUE *q = (RKMPP_JOB_QUEUE *)Context;
    PVOID waitObjects[2] = { &q->ExitEvent, &q->KickEvent };

    for (;;) {
        NTSTATUS w = KeWaitForMultipleObjects(2, waitObjects, WaitAny,
                                              Executive, KernelMode,
                                              FALSE, NULL, NULL);
        if (w == STATUS_WAIT_0) break;     /* exit */
        if (w != STATUS_WAIT_0 + 1) continue;

        /* KickEvent — poll INT_STATUS until done or timeout. */
        PVOID mmio = RkMppGetMmioBase(q->Device);
        if (!mmio) {
            RkMppJobComplete(q->Device, STATUS_DEVICE_NOT_READY, 0);
            continue;
        }

        const RKMPP_CODEC_OPS *ops = &g_ops;
        UINT32 hwStatus = 0;
        NTSTATUS result = STATUS_DEVICE_HUNG;
        /* Cooperative wait: KeDelayExecutionThread yields the CPU to
         * other threads between polls.  An earlier busy-wait via
         * KeStallExecutionProcessor pinned the CPU for the entire poll
         * window which made the system feel hung when decode never
         * completes.  Use a tight burst of stalls to catch a quick
         * completion, then fall back to yieldable sleeps so the system
         * stays responsive while the codec works (or hangs).
         *
         * Phase A: 32 × 50us busy-wait — covers the typical
         *          single-millisecond decode without context-switch cost.
         * Phase B: ~200 × 1ms KeDelayExecutionThread — yields each tick;
         *          200ms total budget; system stays responsive. */
        const ULONG kBusyIters = 32;
        for (ULONG i = 0; i < kBusyIters; i++) {
            hwStatus = READ_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset));
            if (hwStatus & ops->IntDoneMask) goto have_status;
            KeStallExecutionProcessor(50);
        }
        {
            const ULONG kSleepIters = 200;
            LARGE_INTEGER iv;
            iv.QuadPart = -10000;   /* 1 ms relative, 100ns units */
            for (ULONG i = 0; i < kSleepIters; i++) {
                hwStatus = READ_REGISTER_ULONG(
                    (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset));
                if (hwStatus & ops->IntDoneMask) goto have_status;
                KeDelayExecutionThread(KernelMode, FALSE, &iv);
            }
        }
have_status:
        if (hwStatus & ops->IntDoneMask) {
            result = (hwStatus & ops->IntErrorMask)
                   ? STATUS_DEVICE_HARDWARE_ERROR
                   : STATUS_SUCCESS;

            /* AXI write-buffer drain.  RK3588 vdpu's IRQ asserts on the
             * codec's internal "core done" event, NOT after the last
             * outbound write reaches DRAM.  When buffers are mapped
             * uncached, the slow ~12 ms user-mode read provides plenty
             * of drain time; when cached + invalidate, we read fast
             * enough to race the codec's tail-end writes — manifests
             * as deterministic top-fresh / bottom-stale tearing.
             *
             * Force a drain before signalling completion: a dummy MMIO
             * read of the codec's REVISION register (offset 0x000, REG0)
             * goes through the same AXI bus the codec used for its DMA
             * writes; the read can't return until prior writes from the
             * same master domain have committed, so it forces drain.
             * Plus a short KeStallExecutionProcessor as a belt-and-
             * suspenders upper bound for any NoC-side buffering not
             * covered by the AXI ordering rule. */
            volatile ULONG drain = READ_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + ops->RevisionOffset));
            UNREFERENCED_PARAMETER(drain);
            /* AXI write-tail drain via polled idle indicator.  The
             * codec's PERF_WORKING_CNT (offset 0x41c) increments every
             * cycle the codec is processing.  Once DEC_RDY is asserted
             * AND the counter stops changing for several consecutive
             * reads, the codec's processing engine is idle — but the
             * AXI write channel may still be retiring buffered pixel
             * writes through the NoC to DRAM.
             *
             * Bounds: minimum 200 µs (covers basic AXI BVALID round-
             * trip), maximum 10000 µs (4K worst case with chains of
             * alt-refs).  Step 100 µs per poll.  Require 5 consecutive
             * equal reads (= 500 µs of unchanged counter) before
             * declaring the engine idle.
             *
             * After PERF_WORKING_CNT settles, an unconditional 1500 µs
             * settle gives the AXI write channel time to retire the
             * codec's last pixel writes through the NoC to DRAM. */
            {
                volatile ULONG *perf_cnt =
                    (volatile ULONG *)((PUCHAR)mmio + ops->PerfWorkingCntOffset);
                KeStallExecutionProcessor(200);  /* min */
                ULONG prev = READ_REGISTER_ULONG(perf_cnt);
                ULONG stable = 0;
                ULONG total_us = 200;
                while (total_us < 10000) {
                    KeStallExecutionProcessor(100);
                    total_us += 100;
                    ULONG cur = READ_REGISTER_ULONG(perf_cnt);
                    if (cur == prev) {
                        if (++stable >= 5) break;
                    } else {
                        stable = 0;
                        prev = cur;
                    }
                }
                /* Unconditional final settle for the AXI/NoC write
                 * channel.  Cost: 1.5 ms × 30 fps = 45 ms/sec extra
                 * wait, well within frame budget. */
                KeStallExecutionProcessor(1500);
            }
        }

        /* Ack pending status bits (write-1-to-clear). */
        if (hwStatus) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset),
                hwStatus);
        }

        /* AV1 error path: dump INT status + IOMMU snapshot when any
         * error/timeout bit is set.  Only fires on real failures, not per-kick. */
        if (hwStatus & AV1D_INT_ERROR_MASK) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkav1d: poller INT=0x%08x result=0x%08x\n",
                       hwStatus, result);
            PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(q->Device);
            if (iommu && iommu->Snapshot) {
                RKIOMMU_FAULT_SNAPSHOT snap = {0};
                if (NT_SUCCESS(iommu->Snapshot(iommu->Header.Context, &snap))) {
                    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                               "  av1d-iommu STATUS=0x%08x INT_RAWSTAT=0x%08x "
                               "INT_STATUS=0x%08x FAULT_ADDR=0x%08x PTA=0x%08x\n",
                               snap.Status, snap.IntRawStat, snap.IntStatus,
                               snap.PageFaultAddr, snap.DteAddr);
                }
            }
        }

        RkMppJobComplete(q->Device, result, hwStatus);
    }
    PsTerminateSystemThread(STATUS_SUCCESS);
}

/* -----------------------------------------------------------------------
 * ISR — wired via WdfInterruptCreate; fires on the codec core's interrupt.
 *
 * Phase 2: reads + acks the INT_STATUS register and queues the WDF DPC.
 * The ISR will not fire in Phase 2 because we never kick the hardware;
 * this is structural for Phase 3.
 * --------------------------------------------------------------------- */

_Use_decl_annotations_
BOOLEAN
RkMppEvtIsr(WDFINTERRUPT Interrupt, ULONG MessageId)
{
    UNREFERENCED_PARAMETER(MessageId);

    WDFDEVICE device = WdfInterruptGetDevice(Interrupt);
    PVOID mmioBase   = RkMppGetMmioBase(device);
    const RKMPP_CODEC_OPS *ops = &g_ops;

    /* Read and acknowledge the interrupt-status register. */
    UINT32 hwStatus = 0;
    if (mmioBase) {
        hwStatus = READ_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + ops->IntStatusOffset));
        /* Ack by writing back the status bits (write-1-to-clear). */
        WRITE_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + ops->IntStatusOffset),
            hwStatus);
    }

    if (hwStatus == 0) {
        /* Not our interrupt (spurious). */
        return FALSE;
    }

    /* Stash status for the DPC to consume.  We repurpose SystemArgument1
     * on the WDF interrupt DPC via WdfInterruptQueueDpcForIsr. */
    WdfInterruptQueueDpcForIsr(Interrupt);
    return TRUE;
}

/* -----------------------------------------------------------------------
 * WDF Interrupt DPC — runs at DISPATCH_LEVEL after ISR signals completion.
 *
 * Phase 2: not reachable (ISR never fires).  Phase 3: calls
 * RkMppJobComplete with the real hardware status.
 * --------------------------------------------------------------------- */

_Use_decl_annotations_
VOID
RkMppEvtDpc(WDFINTERRUPT Interrupt, WDFOBJECT AssociatedObject)
{
    UNREFERENCED_PARAMETER(AssociatedObject);

    WDFDEVICE device = WdfInterruptGetDevice(Interrupt);
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(device);
    if (!q) return;

    /* Grab InFlight under lock to safely read the hardware status captured
     * during RkMppJobStart.  In Phase 3 the ISR will pass hwStatus via a
     * per-device field; placeholder 0 is fine for Phase 2. */
    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);
    RKMPP_JOB *job = q->InFlight;
    KeReleaseSpinLock(&q->Lock, oldIrql);

    if (job) {
        RkMppJobComplete(device, STATUS_SUCCESS, 0);
    }

    /* Mirror Linux `mpp_iommu_dev_deactivate` semantics: on every IRQ
     * completion clear stale TLB state so the next kick — possibly from
     * a different session — never inherits address-space residue.
     * Linux's deactivate is pure software (clears `info->dev_active`);
     * the closest hardware analog we have is `FlushTlb` (ZAP_CACHE).
     * This pairs with the per-kick FlushTlb already done in JobStart;
     * doing it on both edges of the kick → completion arc gives us
     * defense-in-depth without any measurable cost (one MMIO write per
     * MMU instance, on the order of 100 ns). */
    {
        PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(device);
        if (iommu && iommu->FlushTlb) {
            iommu->FlushTlb(iommu->Header.Context);
        }
    }
}

/* -----------------------------------------------------------------------
 * RkMppJobStart — kick the job (AV1D only).
 *
 * Phase 2 (software-completion stub): capture start timestamp and schedule
 * the software DPC for immediate completion.  Phase 3 replaces this with
 * writing the register list to MMIO and asserting the hardware kick bit.
 * --------------------------------------------------------------------- */

static VOID
RkMppJobStart(_In_ WDFDEVICE Device, _In_ RKMPP_JOB *Job)
{
    PRKMPP_JOB_QUEUE q       = RkMppGetJobQueue(Device);
    PVOID            mmio    = RkMppGetMmioBase(Device);
    ULONG            mmioLen = RkMppGetMmioLength(Device);
    const RKMPP_CODEC_OPS *ops = &g_ops;

    Job->StartQpc = KeQueryPerformanceCounter(NULL);

    if (!mmio || mmioLen == 0) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkav1d: JobStart with no MmioBase — failing job %llu\n",
                   (unsigned long long)Job->Id);
        RkMppJobComplete(Device, STATUS_DEVICE_NOT_READY, 0);
        return;
    }

    /* Conditional core reset.  BSP only resets after error/timeout — see
     * mpp_common.c:2026 (mpp_dev_reset gated on reset_request > 0).  We do
     * the same: NeedsCoreReset starts 1 (cleared first kick after PnP),
     * is set after any kick that errors/times out, and otherwise stays 0
     * so successful back-to-back kicks don't disturb codec internal state.
     *
     * AV1 has no hardware-reset wiring yet (TODO: AV1 hang recovery).
     * But after a kick error the codec's reg state is unknown — we
     * may have observed an IOMMU fault, AXI bus error, or partial
     * decode that left some regs in an unexpected state.  Drop the
     * prev-mask so the next kick rewrites every reg from scratch,
     * eliminating any reliance on stale HW state.  Also covers the
     * case where an external power-cycle (suspend/resume, RaiseCluster
     * after PD power-gate) zeroed regs without us knowing. */
    if (RkMppQueryAndClearNeedsCoreReset(Device)) {
        RtlZeroMemory(q->PrevNonzeroMask, sizeof(q->PrevNonzeroMask));
    }

    /* Clear any stale latched status bits before the kick. */
    {
        ULONG sta = READ_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset));
        if (sta) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset),
                sta & ~1u);
        }
    }

    /* AV1 L2 cache config (CACHE window @ Mmios[1]).  Port of BSP
     * av1dec_set_l2_cache from mpp_av1dec.c.  Required for the codec's
     * PP output path to function — without it, the codec hits AXI bus
     * errors mid-decode (dec_bus_int with no IOMMU fault).
     *
     * For NV12 output (sw_pp_out_tile_size != AV1_PP_TILE_16X16):
     * configure PP0_Y and PP0_U cache channels with the output buffer
     * iovas + line geometry, mask cache IRQ, enable shaper, enable
     * cache.  AFBC config (Mmios[2]) is a no-op for NV12 — the AFBC
     * sub-block stays in reset state.
     *
     * Geometry derived from the regbuilder's output already substituted
     * into job->Writes (iovas in reg326/328, dims in reg4, pp_in_format
     * in reg322 bits 27-31).  We need the SUBSTITUTED iovas not the FDs
     * that the regbuilder originally wrote — hence reading from
     * job->Writes (post-substitution) rather than recomputing. */
    {
        /* Pull post-substitution values for relevant regs from job->Writes. */
        ULONG reg4 = 0, reg10 = 0, reg321 = 0, reg322 = 0;
        ULONG reg326 = 0, reg328 = 0;
        BOOLEAN haveOut = FALSE;
        for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
            ULONG idx = Job->Writes[i].Offset / 4;
            ULONG val = Job->Writes[i].Value;
            if      (idx == 4)   reg4   = val;
            else if (idx == 10)  reg10  = val;
            else if (idx == 321) reg321 = val;
            else if (idx == 322) reg322 = val;
            else if (idx == 326) { reg326 = val; haveOut = TRUE; }
            else if (idx == 328) reg328 = val;
        }

        ULONG cacheLen = 0;
        PVOID cacheBase = RkMppGetMmioWindow(Device, 1, &cacheLen);
        const ULONG AV1_PP_TILE_SIZE   = (3u << 9);  /* GENMASK(10,9) */
        const ULONG AV1_PP_TILE_16X16  = (1u << 10);
        BOOLEAN tile16 = ((reg321 & AV1_PP_TILE_SIZE) == AV1_PP_TILE_16X16);

        if (cacheBase && cacheLen >= 0x300 && haveOut && !tile16) {
            const ULONG width  = (reg4 >> 19) * 8;
            const ULONG height = ((reg4 >> 6) & 0x1fff) * 8;
            const ULONG pp_in_format = (reg322 >> 27) & 0x1F;
            const ULONG pixel_width  = (pp_in_format == 1) ? 8 : 16;
            const ULONG pre_fetch_height = 136;

            /* MPP_ALIGN(MPP_ALIGN(width * pixel_width, 8) / 8, 16) */
            ULONG line_size = ((width * pixel_width + 7) & ~7u) / 8;
            line_size = (line_size + 15) & ~15u;
            const ULONG line_stride = line_size >> 4;

#define CACHE_W(off, val) \
    WRITE_REGISTER_ULONG((volatile ULONG*)((PUCHAR)cacheBase + (off)), (val))
            /* TEST: keep cache disabled for all kicks to confirm whether the
             * L2 write cache is causing alternating-frame corruption.
             * PP writes bypass the cache and go directly to DRAM.
             * If all frames decode correctly without the cache, the root cause
             * is in the cache enable/disable/flush sequencing.
             * TODO: re-enable once correct flush sequence is identified. */
            CACHE_W(0x204, 0x00000000);
            /* PP0_Y channel */
            CACHE_W(0x84, reg326 + 1);                                 /* CONFIG0 */
            CACHE_W(0x8c, line_size | (line_stride << 16));            /* CONFIG2 */
            CACHE_W(0x90, height    | (pre_fetch_height << 16));       /* CONFIG3 */
            /* PP0_U channel (chroma half height) */
            CACHE_W(0x98, reg328 + 1);
            CACHE_W(0xa0, line_size | (line_stride << 16));
            CACHE_W(0xa4, (height >> 1) | ((pre_fetch_height >> 1) << 16));
            /* Mask cache IRQ */
            CACHE_W(0x30, 0xf);
            /* Shaper enable */
            CACHE_W(0x20, 0x1);
            /* Single-tile: enable cache for all reads (reg10 bit 1 == 0) */
            if (!(reg10 & (1u << 1))) {
                CACHE_W(0x208, 0x00000001);
            }
            /* DISABLED: do not write 0x81 (reorder_e | cache_e) — L2 cache
             * disabled for all kicks while diagnosing alternating-frame
             * corruption.  Without the enable, PP writes go directly to DRAM
             * bypassing the cache write-combining path entirely. */
            /* CACHE_W(0x204, 0x00000081); */
#undef CACHE_W
        }
    }

    /* Validate every (shifted) offset is in-range BEFORE issuing any MMIO
     * write, so we can reject malformed register lists without leaving
     * partial state on the hardware. */
    BOOLEAN hasKick = FALSE;
    for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
        const RKMPP_REG_WRITE *w = &Job->Writes[i];
        ULONG mmioOff = w->Offset + ops->SwregBase;
        if (mmioOff >= mmioLen ||
            (mmioLen - mmioOff) < sizeof(ULONG)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkav1d: job %llu swreg-off 0x%x (mmio 0x%x) out of "
                       "range (len 0x%x)\n",
                       (unsigned long long)Job->Id, w->Offset, mmioOff, mmioLen);
            RkMppJobComplete(Device, STATUS_INVALID_PARAMETER, 0);
            return;
        }
        if (w->Offset == ops->KickRegOffset &&
            (w->Value & ops->KickRegBit)) {
            hasKick = TRUE;
        }
    }

    /* Pre-kick clean: push CPU-dirty cache lines for the just-written
     * input buffers (tile_info, bitstream, prob_tbl, global_model) to
     * DRAM so the codec's first DMA read gets the fresh data. */
    for (UINT32 fi = 0; fi < Job->CleanMdlCount; fi++) {
        KeFlushIoBuffers(Job->CleanMdls[fi],
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }

    {
        /* AV1: VCD class is one flat array (regs 0..319, byte 0..0x4FC).
         * BSP's mpp_av1dec writes idx 1..319 ascending, en at idx 1 last
         * (per av1_bringup_table memory: reg_start=1, reg_end=319, en at
         * VCD+0x004 which IS idx 1).  Build a flat 320-entry shadow,
         * write in ascending order, skip the kick reg, then write kick
         * last.  Now extended through 511 — the PP cfg block lives at
         * swreg320..511 (byte 0x500..0x7FC) inside the same VCD window
         * (Mmios[0], length 0x800).  Without these writes the PP module
         * stays idle: VCD decodes correctly to tile_out_internal, but
         * the user-visible NV12 output buffer (pool_output) gets nothing
         * — codec returns hwstatus=success-clean but content is zeros.
         * No SWREG prefix — offset = idx*4 directly. */
        ULONG av1_vals[512] = {0};
        BOOLEAN av1_seen[512] = {0};
        ULONG cur_nonzero[16] = {0};   /* bitmap of regs nonzero this kick */
        for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
            const RKMPP_REG_WRITE *w = &Job->Writes[i];
            ULONG idx = w->Offset / 4;
            if (idx >= 512) continue;
            av1_vals[idx] = w->Value;
            av1_seen[idx] = TRUE;
            if (w->Value != 0)
                cur_nonzero[idx >> 5] |= (1u << (idx & 31));
        }
        const ULONG kick_idx = ops->KickRegOffset / 4;   /* 1 */
        /* Skip MMIO writes for regs that were zero last kick AND are zero
         * this kick — hardware retains their (zero) value, so writing zero
         * again is wasted bus work.  Regs that flipped nonzero→zero must
         * still be cleared.  Cuts per-kick MMIO from 511 writes down to
         * roughly the count of active regs (~80 typical) plus a few
         * "clear" writes for regs going inactive. */
        const ULONG *prev = q->PrevNonzeroMask;
        for (ULONG idx = 1; idx < 512; idx++) {
            if (idx == kick_idx) continue;
            const ULONG bit = 1u << (idx & 31);
            const BOOLEAN was_nz = (prev[idx >> 5] & bit) != 0;
            const BOOLEAN is_nz  = (cur_nonzero[idx >> 5] & bit) != 0;
            if (!was_nz && !is_nz) continue;  /* zero→zero: skip */
            TRACED_WRITE_ULONG(((PUCHAR)mmio + idx * 4), av1_vals[idx]);
        }
        /* Snapshot for next kick. */
        for (int i = 0; i < 16; i++)
            q->PrevNonzeroMask[i] = cur_nonzero[i];
        /* Kick last. */
        if (av1_seen[kick_idx]) {
            TRACED_WRITE_ULONG(((PUCHAR)mmio + ops->KickRegOffset),
                               av1_vals[kick_idx]);
        }
    }

    /* Flush the IOMMU's TLB right before the kick — matches BSP
     * rkvdec2_run -> mpp_iommu_flush_tlb.  Stale TLB entries from a
     * previous decode session caused the codec to read/write at obsolete
     * physical addresses (manifested as fault iovas in the 0xae0..0xe00
     * range). */
    {
        PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(Device);
        if (iommu && iommu->FlushTlb) {
            iommu->FlushTlb(iommu->Header.Context);
        }
    }

    if (hasKick) {
        /* Real decode — engage the poller to watch INT_STATUS. */
        KeSetEvent(&q->KickEvent, IO_NO_INCREMENT, FALSE);
    } else {
        /* Test-only register write set (smoke tests, peek-only).
         * Complete the job immediately without waiting for hardware. */
        RkMppJobComplete(Device, STATUS_SUCCESS, 0);
    }
}

/* -----------------------------------------------------------------------
 * RkMppJobComplete — finalise a job.
 *
 * Called from the software-completion DPC (Phase 2) or the ISR-DPC (Phase 3).
 * Runs at DISPATCH_LEVEL.  Must not allocate paged memory or block.
 * --------------------------------------------------------------------- */

static VOID
RkMppJobComplete(_In_ WDFDEVICE Device,
                 _In_ NTSTATUS Result,
                 _In_ UINT32 HardwareStatus)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);

    RKMPP_JOB *job = q->InFlight;
    if (!job) {
        KeReleaseSpinLock(&q->Lock, oldIrql);
        return;
    }

    /* Capture end timestamp and result. */
    job->EndQpc        = KeQueryPerformanceCounter(NULL);
    job->Result        = Result;
    job->HardwareStatus = HardwareStatus;

    /* BSP parity: only request a reset when an error/timeout was observed.
     * Plus we treat NTSTATUS failure as an error. */
    if (!NT_SUCCESS(Result) || (HardwareStatus & 0xF0u)) {
        RkMppSetNeedsCoreReset(Device);

        /* Charge the error to the file-object that submitted this job so
         * EvtFileCleanup can promote its session-end teardown to the
         * soft-tier IOMMU force-reset.  Mirrors BSP's per-task
         * `reset_request` accumulation that drives `mpp_dev_reset` on
         * task finish. */
        if (job->Owner) {
            PRKMPP_FILE_CTX fctx = RkMppFileGet(job->Owner);
            if (fctx) InterlockedIncrement(&fctx->ErrorCount);
        }
    }

    /* Cache invalidate DMA-output buffers the CPU will read after this kick.
     * Per MS docs, `ReadOperation=TRUE` means data flow is device→memory,
     * which on ARM64 emits the invalidate (`dc ivac`) needed to drop stale
     * CPU cache lines so the next CPU read reloads from DRAM.
     *
     * OutputFrameMdl    — pool_output NV12 (MmCached); fully committed before
     *   the PERF_WORKING_CNT drain exits.
     * InternalOutputMdl — AV1 pool_internal codec-tiled: user-mode dump
     *   path (RKMPP_AV1_DUMP_DIR) reads it for diffing against BSP captures.
     * AuxOutputMdl      — prob_tbl_out: user mode reads for CDF snapshot. */
    if (job->OutputFrameMdl) {
        KeFlushIoBuffers(job->OutputFrameMdl,
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }
    if (job->InternalOutputMdl) {
        KeFlushIoBuffers(job->InternalOutputMdl,
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }
    if (job->AuxOutputMdl) {
        KeFlushIoBuffers(job->AuxOutputMdl,
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }

    /* Move from InFlight to Completed. */
    q->InFlight = NULL;
    InsertTailList(&q->Completed, &job->Link);

    /* Signal the waiter before we potentially start the next job. */
    KeSetEvent(&job->Done, IO_NO_INCREMENT, FALSE);

    /* If another job is pending, promote it now — unless the queue is
     * being quiesced for D0Exit.  D0Exit Quiesce holds new kicks; the
     * paired D0Entry Resume restarts the chain by re-promoting head of
     * Pending.  We still null InFlight above so Resume's
     * "InFlight == NULL" check sees a clean slate. */
    RKMPP_JOB *next = NULL;
    if (q->Draining == 0 && !IsListEmpty(&q->Pending)) {
        PLIST_ENTRY entry = RemoveHeadList(&q->Pending);
        next = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
        q->InFlight = next;
    }

    KeReleaseSpinLock(&q->Lock, oldIrql);

    /* Per-job RaiseAv1Cluster/DropAv1Cluster removed — they were
     * always short-circuit no-ops nested under the device-lifetime
     * Raise from PrepareHardware, and DropAv1Cluster ran from DISPATCH
     * (DPC) which would have blocked the FAST_MUTEX serialization
     * added in rkmpp_ccu. */

    /* Kick the next job outside the spin lock. */
    if (next) {
        RkMppJobStart(Device, next);
    }
}

/* -----------------------------------------------------------------------
 * RkMppJobSubmit — IOCTL_RKMPP_SUBMIT_JOB handler
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobSubmit(_In_ WDFDEVICE Device,
               _In_ WDFFILEOBJECT File,
               _In_ const RKMPP_SUBMIT_JOB_IN *In,
               _Out_ RKMPP_SUBMIT_JOB_OUT *Out)
{
    /* Validate register-write and buffer-ref counts. */
    if (In->RegWriteCount > RKMPP_MAX_REG_WRITES ||
        In->BufRefCount   > RKMPP_MAX_BUF_REFS) {
        return STATUS_INVALID_PARAMETER;
    }

    /* Allocate the job from NonPagedPool — it must be accessible at
     * DISPATCH_LEVEL when the DPC fires. */
    RKMPP_JOB *job = (RKMPP_JOB *)ExAllocatePool2(
        POOL_FLAG_NON_PAGED, sizeof(RKMPP_JOB), 'JppM');
    if (!job) return STATUS_INSUFFICIENT_RESOURCES;

    RtlZeroMemory(job, sizeof(*job));
    KeInitializeEvent(&job->Done, NotificationEvent, FALSE);
    job->Owner = File;

    /* Copy the register-write list, performing iova-handle substitution
     * for any entry with BufferHandle != 0.  Iova is 64-bit but the
     * register field is 32-bit; rkiommu's address space is 32 bits so
     * the iova always fits. */
    job->RegWriteCount = In->RegWriteCount;
    for (UINT32 i = 0; i < In->RegWriteCount; i++) {
        const RKMPP_REG_WRITE *src = &In->Writes[i];
        RKMPP_REG_WRITE       *dst = &job->Writes[i];
        dst->Offset       = src->Offset;
        dst->BufferHandle = src->BufferHandle;
        dst->IovaOffset   = src->IovaOffset;
        dst->Reserved     = 0;

        if (src->BufferHandle == 0) {
            dst->Value = src->Value;
            continue;
        }

        UINT64 iova = 0;
        ULONG  bufSize = 0;
        NTSTATUS s = RkMppBufLookupIova(File, src->BufferHandle,
                                        &iova, &bufSize);
        if (!NT_SUCCESS(s)) {
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_HANDLE;
        }
        if (src->IovaOffset >= bufSize) {
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_PARAMETER;
        }
        dst->Value = (UINT32)(iova + src->IovaOffset);

        /* Direction-based cache maintenance:
         *
         *   AV1 offsets: codec writes user-visible NV12 to PP output regs
         *   326/328 = 0x518/0x520 (and reg65/99/133 internal scratch);
         *   inputs are reg167 tile_info=0x29C, reg169 bitstream=0x2A4,
         *   reg173 prob_tbl=0x2B4, reg83 global_model=0x14C.
         *
         *   Without this gating, AV1 user-mode writes sit in CPU cache
         *   (MmCached user mapping) and never reach DRAM before kick;
         *   codec reads zeros for tile_info / OBU / CDFs → timeouts. */
        PMDL mdl = NULL;
        if (!NT_SUCCESS(RkMppBufLookupMdl(File, src->BufferHandle, &mdl)) ||
            mdl == NULL) {
            continue;
        }
        /* Cache-maintenance MDL classification.
         *
         *   Output  — codec writes, CPU reads post-decode.  Need to
         *             invalidate user-mode-cached lines after kick.
         *   Input   — CPU writes (memcpy from user / regbuilder),
         *             codec reads.  Need to clean (push) cached lines
         *             before kick so codec's DMA sees fresh data. */
        if (src->Offset == 0x518u ||      /* AV1 PP output luma  */
            src->Offset == 0x520u) {      /* AV1 PP output chroma */
            if (job->OutputFrameMdl == NULL) job->OutputFrameMdl = mdl;
        } else if (src->Offset == 0x104u) {  /* AV1 reg65 = pool_internal Y */
            if (job->InternalOutputMdl == NULL) job->InternalOutputMdl = mdl;
        } else if (src->Offset == 0x2ACu) {  /* AV1 reg171 = prob_tbl_out */
            if (job->AuxOutputMdl == NULL) job->AuxOutputMdl = mdl;
        } else if (/* AV1 CPU-written inputs */
                   src->Offset == 0x29Cu ||  /* reg167 tile_info */
                   src->Offset == 0x2A4u ||  /* reg169 bitstream */
                   src->Offset == 0x2B4u ||  /* reg173 prob_tbl  */
                   src->Offset == 0x14Cu) {  /* reg83  global_model */
            BOOLEAN already = FALSE;
            for (UINT32 k = 0; k < job->CleanMdlCount; k++) {
                if (job->CleanMdls[k] == mdl) { already = TRUE; break; }
            }
            if (!already &&
                job->CleanMdlCount < RTL_NUMBER_OF(job->CleanMdls)) {
                job->CleanMdls[job->CleanMdlCount++] = mdl;
            }
        }
    }
    job->BufRefCount = In->BufRefCount;
    if (In->BufRefCount > 0) {
        RtlCopyMemory(job->BufRefs, In->BufRefs,
                      In->BufRefCount * sizeof(RKMPP_BUFFER_REF));
    }

    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    /* Assign a unique job ID atomically. */
    job->Id = (UINT64)InterlockedIncrement64(&q->NextId);

    /* Per-job RaiseAv1Cluster removed — the device-lifetime Raise
     * taken in PrepareHardware keeps the AV1 CCU refcount at >= 1
     * across the entire device lifetime, so the per-job 1<->2 bounce
     * was always a short-circuit no-op.  The matching per-job
     * DropAv1Cluster (which ran from DPC at DISPATCH_LEVEL) is
     * removed too — see the RkMppJobComplete comment for why this
     * matters for FAST_MUTEX serialization in rkmpp_ccu. */

    /* Enqueue the job under the spin lock.  Cap the pending-job count
     * as a kernel-side safeguard against runaway user-mode submission
     * — if a session keeps submitting faster than the codec can drain
     * (e.g. zero-copy ProcessInput pumping at decode rate while EVR
     * pulls at audio rate), the queue would grow without bound, holding
     * DPB slots and eventually feeding the codec bad refs that wedge
     * the hardware.  STATUS_DEVICE_BUSY tells the caller to back off
     * and call IOCTL_RKMPP_WAIT_JOB before submitting more. */
    enum { RKMPP_MAX_PENDING_JOBS = 8 };
    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);

    /* Count pending entries — list is short (capped here), no LIST_FOR_EACH
     * macro on Windows kernel so iterate manually. */
    ULONG pendingCount = 0;
    for (PLIST_ENTRY e = q->Pending.Flink;
         e != &q->Pending;
         e = e->Flink) {
        pendingCount++;
        if (pendingCount >= RKMPP_MAX_PENDING_JOBS) break;
    }
    if (pendingCount >= RKMPP_MAX_PENDING_JOBS) {
        KeReleaseSpinLock(&q->Lock, oldIrql);
        /* No per-job CCU raise to undo (removed — see comment above). */
        ExFreePoolWithTag(job, 'JppM');
        return STATUS_DEVICE_BUSY;
    }

    /* While Draining, force every new submit onto Pending; Resume restarts the chain. */
    BOOLEAN startNow = (q->InFlight == NULL) && (q->Draining == 0);
    if (startNow) {
        q->InFlight = job;
    } else {
        InsertTailList(&q->Pending, &job->Link);
    }

    KeReleaseSpinLock(&q->Lock, oldIrql);

    if (startNow) {
        RkMppJobStart(Device, job);
    }

    Out->JobId = job->Id;
    return STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------
 * RkMppJobPeek — IOCTL_RKMPP_PEEK_JOB handler
 *
 * Find a job by ID across Pending / InFlight / Completed and copy its
 * post-substitution register list back to user mode.  Used by tests to
 * confirm iova-handle substitution worked before the real hardware-kick
 * path is in (Phase 3b Task 11).
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobPeek(_In_ WDFDEVICE Device,
             _In_ UINT64 JobId,
             _Out_ RKMPP_PEEK_JOB_OUT *Out)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    KIRQL oldIrql;
    NTSTATUS status = STATUS_NOT_FOUND;

    KeAcquireSpinLock(&q->Lock, &oldIrql);

    RKMPP_JOB *job = NULL;
    if (q->InFlight && q->InFlight->Id == JobId) {
        job = q->InFlight;
    }
    if (!job) {
        for (PLIST_ENTRY e = q->Pending.Flink;
             e != &q->Pending; e = e->Flink) {
            RKMPP_JOB *cand = CONTAINING_RECORD(e, RKMPP_JOB, Link);
            if (cand->Id == JobId) { job = cand; break; }
        }
    }
    if (!job) {
        for (PLIST_ENTRY e = q->Completed.Flink;
             e != &q->Completed; e = e->Flink) {
            RKMPP_JOB *cand = CONTAINING_RECORD(e, RKMPP_JOB, Link);
            if (cand->Id == JobId) { job = cand; break; }
        }
    }

    if (job) {
        Out->RegWriteCount = job->RegWriteCount;
        Out->Reserved      = 0;
        if (job->RegWriteCount > 0) {
            RtlCopyMemory(Out->Writes, job->Writes,
                          job->RegWriteCount * sizeof(RKMPP_REG_WRITE));
        }
        status = STATUS_SUCCESS;
    }

    KeReleaseSpinLock(&q->Lock, oldIrql);
    return status;
}

/* -----------------------------------------------------------------------
 * RkMppJobWait — IOCTL_RKMPP_WAIT_JOB handler
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobWait(_In_ WDFDEVICE Device,
             _In_ UINT64 JobId,
             _In_ UINT32 TimeoutMs,
             _Out_ RKMPP_WAIT_JOB_OUT *Out)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);

    /* Search completed list first. */
    RKMPP_JOB *job = NULL;
    BOOLEAN alreadyComplete = FALSE;

    PLIST_ENTRY entry;
    for (entry = q->Completed.Flink;
         entry != &q->Completed;
         entry = entry->Flink) {
        RKMPP_JOB *candidate = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
        if (candidate->Id == JobId) {
            job = candidate;
            alreadyComplete = TRUE;
            break;
        }
    }

    if (!job) {
        /* Check InFlight. */
        if (q->InFlight && q->InFlight->Id == JobId) {
            job = q->InFlight;
        }
    }

    if (!job) {
        /* Check pending list. */
        for (entry = q->Pending.Flink;
             entry != &q->Pending;
             entry = entry->Flink) {
            RKMPP_JOB *candidate = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
            if (candidate->Id == JobId) {
                job = candidate;
                break;
            }
        }
    }

    if (!job) {
        KeReleaseSpinLock(&q->Lock, oldIrql);
        return STATUS_NOT_FOUND;
    }

    if (!alreadyComplete) {
        /* Job still in progress — wait for it with the caller's timeout. */
        KeReleaseSpinLock(&q->Lock, oldIrql);

        LARGE_INTEGER timeout;
        PLARGE_INTEGER pTimeout = NULL;
        if (TimeoutMs != 0 && TimeoutMs != (UINT32)-1) {
            /* Convert milliseconds to 100-ns units, negative = relative. */
            timeout.QuadPart = -((LONGLONG)TimeoutMs * 10000LL);
            pTimeout = &timeout;
        }

        NTSTATUS waitStatus = KeWaitForSingleObject(
            &job->Done, Executive, KernelMode, FALSE, pTimeout);

        if (waitStatus == STATUS_TIMEOUT) {
            /* Caller's WAIT_JOB timed out before the poller signalled
             * completion.  This happens when many kicks are backlogged
             * because each takes a long time (e.g. the codec is in a
             * partial-error state where the poller waits its full budget
             * per kick).
             *
             * The job may currently sit on Pending (not yet kicked),
             * be the InFlight (poller is processing it), or — if it
             * raced into completion just as we timed out — be on
             * Completed.  Re-acquire the lock, find it, and free it
             * SAFELY:
             *
             *   Pending  → remove + free (poller hasn't touched it yet).
             *   Completed → remove + free (poller is done with it).
             *   InFlight → leak the job (poller still holds a pointer
             *              and will dereference it; freeing here is a
             *              use-after-free).  The next session reset
             *              releases queue memory.  Set q->InFlight = NULL
             *              so subsequent submits don't wedge waiting
             *              for a "complete" signal that never comes. */
            job->Result = STATUS_TIMEOUT;
            Out->Status         = STATUS_TIMEOUT;
            Out->HardwareStatus = 0;
            Out->ElapsedQpc     = 0;

            KeAcquireSpinLock(&q->Lock, &oldIrql);
            BOOLEAN onPending   = FALSE;
            BOOLEAN onCompleted = FALSE;
            for (PLIST_ENTRY e = q->Pending.Flink; e != &q->Pending;
                 e = e->Flink) {
                if (CONTAINING_RECORD(e, RKMPP_JOB, Link) == job) {
                    onPending = TRUE; break;
                }
            }
            for (PLIST_ENTRY e = q->Completed.Flink; e != &q->Completed;
                 e = e->Flink) {
                if (CONTAINING_RECORD(e, RKMPP_JOB, Link) == job) {
                    onCompleted = TRUE; break;
                }
            }
            BOOLEAN safeToFree = FALSE;
            if (onPending) {
                RemoveEntryList(&job->Link);
                safeToFree = TRUE;
            } else if (onCompleted) {
                RemoveEntryList(&job->Link);
                safeToFree = TRUE;
            } else if (q->InFlight == job) {
                /* Poller still owns it — leak.  Detach so subsequent
                 * submits aren't gated on it. */
                q->InFlight = NULL;
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkav1d: WAIT_JOB timeout on InFlight job %llu — "
                           "leaking (poller still holds pointer)\n",
                           (unsigned long long)job->Id);
            }
            KeReleaseSpinLock(&q->Lock, oldIrql);

            if (safeToFree) {
                ExFreePoolWithTag(job, 'JppM');
            }
            return STATUS_TIMEOUT;
        }

        /* Job is now on the Completed list; re-acquire lock to remove it. */
        KeAcquireSpinLock(&q->Lock, &oldIrql);
    }

    /* Remove from Completed list and copy result. */
    RemoveEntryList(&job->Link);
    KeReleaseSpinLock(&q->Lock, oldIrql);

    LONGLONG elapsed = job->EndQpc.QuadPart - job->StartQpc.QuadPart;
    if (elapsed < 0) elapsed = 0;

    Out->Status         = job->Result;
    Out->HardwareStatus = job->HardwareStatus;
    Out->ElapsedQpc     = (UINT64)elapsed;

    ExFreePoolWithTag(job, 'JppM');
    return STATUS_SUCCESS;
}
