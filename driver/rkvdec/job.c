/* driver/rkmpp/job.c — register-list job submission queue for rkmpp.sys.
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
#include "devpub.h"       /* RKMPP_DEVICE_PUBLIC for UID query */
#include "../shared/rkmpp/bufpool.h"      /* for RkMppBufLookupIova (iova substitution) */
#include "../shared/rkmpp/ifc_client.h"   /* for RKMPP_CCU_INTERFACE via RkMppGetCcuIfc */

extern PRKIOMMU_INTERFACE RkMppGetIommuIfc(_In_ WDFDEVICE Device);
extern void RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out);

/* Defined in device.c — accessor pair for RKMPP_DEVICE.NeedsCoreReset. */
extern LONG RkMppQueryAndClearNeedsCoreReset(_In_ WDFDEVICE Device);
extern VOID RkMppSetNeedsCoreReset(_In_ WDFDEVICE Device);
/* 2-tier reset escalation: when narrow CoreReset doesn't recover the
 * codec (typically dec_e stuck at 1 across kicks), we set NeedsFullReset
 * which JobStart escalates to CcuFullCoreReset0/1 + IOMMU Reattach. */
extern LONG RkMppQueryAndClearNeedsFullReset(_In_ WDFDEVICE Device);
extern VOID RkMppSetNeedsFullReset(_In_ WDFDEVICE Device);
/* Returns previous value, atomically swapping in NewValue.  Used in
 * JobComplete to detect "this fail follows another fail" → escalate. */
extern LONG RkMppExchangeLastJobFailed(_In_ WDFDEVICE Device, LONG NewValue);

/* Tracing wrapper for codec MMIO writes — matches BSP's
 * mpp_dev_debug=DEBUG_SET_REG output format so the two traces can be
 * diffed directly.  Set to 0 to disable. */
#define RKMPP_TRACE_WRITES 0
#if RKMPP_TRACE_WRITES
#define TRACED_WRITE_ULONG(addr, val)                                     \
    do {                                                                  \
        ULONG _v = (val);                                                 \
        ULONG_PTR _a = (ULONG_PTR)(addr);                                 \
        ULONG _swreg_off = (ULONG)(_a - (ULONG_PTR)mmio - RKVDEC2_SWREG_BASE);\
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,               \
                   "rkmpp: write reg[%03d]: %04x: 0x%08x\n",              \
                   _swreg_off / 4, _swreg_off, _v);                       \
        WRITE_REGISTER_ULONG((volatile ULONG *)(_a), _v);                 \
    } while (0)
#else
#define TRACED_WRITE_ULONG(addr, val) WRITE_REGISTER_ULONG((volatile ULONG*)(addr), val)
#endif

/* -----------------------------------------------------------------------
 * INT_STATUS register offset — vdpu34x idx 224.
 * Same word holds enable bits when written and status bits when read;
 * write-1-to-clear acks the pending status.
 * --------------------------------------------------------------------- */
/* The ACPI MMIO resource for RKCP3550 (rkvdec0/1 client) maps phys
 * 0xfdc38000..0xfdc38800; SWREG[0] (REVISION) lives at +0x100 within
 * that mapping (the first 0x100 is the cluster-link auxiliary block).
 * BSP `mpp_rkvdec2.c` opens MMIO at 0xfdc38100 and treats register
 * offsets as `idx * 4` directly.  Our register-list writes need the
 * 0x100 prefix added before reaching the SWREG bank. */
#define RKVDEC2_SWREG_BASE            0x100u

/* SWREG[224] in BSP-relative space; with our MMIO mapping this is
 * 0x100 + 0x380 = 0x480. */
#define RKVDEC2_INT_STATUS_OFFSET     (RKVDEC2_SWREG_BASE + 0x380u)
#define RKVDEC2_INT_DEC_RDY_STA       (1u << 2)
#define RKVDEC2_INT_DEC_BUS_STA       (1u << 3)
#define RKVDEC2_INT_DEC_ERROR_STA     (1u << 4)
#define RKVDEC2_INT_DEC_TIMEOUT_STA   (1u << 5)
#define RKVDEC2_INT_BUF_EMPTY_STA     (1u << 6)
#define RKVDEC2_INT_COLMV_REF_ERROR   (1u << 7)
#define RKVDEC2_INT_DONE_MASK \
    (RKVDEC2_INT_DEC_RDY_STA | RKVDEC2_INT_DEC_ERROR_STA | \
     RKVDEC2_INT_DEC_TIMEOUT_STA | RKVDEC2_INT_DEC_BUS_STA | \
     RKVDEC2_INT_BUF_EMPTY_STA | RKVDEC2_INT_COLMV_REF_ERROR)
/* Matches BSP `RKVDEC_INT_ERROR_MASK` exactly (mpp_rkvdec2.h:76-79,
 * also mpp_rkvdec2.c:432-435 and mpp_rkvdec2_link.c:989): bits 4|5|6|7
 * (ERROR | TIMEOUT | BUF_EMPTY | COLMV_REF_ERR).  BUS_STA (bit 3) is
 * intentionally NOT included — earlier "bit 3 alone → stale FSM" was a
 * bring-up-era observation; the `!dec_rdy → FullCoreReset` escalation
 * (now dropped, see RkMppJobComplete) was the actual recovery path for
 * those genuine wedges.  Treating bit 3 as fatal triggered narrow
 * CoreReset on every kick where the codec also pulsed BUS_STA along
 * with DEC_RDY (clean done), which BSP considers success. */
#define RKVDEC2_INT_ERROR_MASK \
    (RKVDEC2_INT_DEC_ERROR_STA | RKVDEC2_INT_DEC_TIMEOUT_STA | \
     RKVDEC2_INT_BUF_EMPTY_STA | RKVDEC2_INT_COLMV_REF_ERROR)

/* dec_e=1 lives at byte offset 0x28 (idx 10).  A job that includes this
 * write is a real decode kick; jobs without it (smoke tests, register-
 * substitution probes) shouldn't engage the poller. */
#define RKVDEC2_REG_DEC_E_OFFSET   0x28u
#define RKVDEC2_REG_DEC_E_BIT      0x1u

/* Per-codec MMIO geometry table.  Job.c's hot path reads from this
 * struct so the kick / poll / drain code stays codec-agnostic. */
typedef struct _RKMPP_CODEC_OPS {
    /* Byte offset added to a regbuilder swreg index (idx*4) to land on
     * the actual MMIO register.  rkvdec2 has a 0x100 cluster-link
     * prefix; AV1 has none. */
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
     * AXI drain after DEC_RDY. For rkvdec2 this is SWREG[1] within the
     * 0x100-prefixed window (= 0x104). For AV1 the IP version lives at
     * REG0 (offset 0x000). Reading the wrong offset (e.g., the kick
     * register) doesn't provide AXI ordering and lets stale codec writes
     * remain in NoC buffers when WAIT_JOB returns. */
    ULONG RevisionOffset;
    /* Kick (dec_e) register offset and bit. */
    ULONG KickRegOffset;
    ULONG KickRegBit;
} RKMPP_CODEC_OPS;

static const RKMPP_CODEC_OPS g_ops = {
    .SwregBase             = RKVDEC2_SWREG_BASE,
    .IntStatusOffset       = RKVDEC2_INT_STATUS_OFFSET,
    .IntDoneMask           = RKVDEC2_INT_DONE_MASK,
    .IntErrorMask          = RKVDEC2_INT_ERROR_MASK,
    .PerfWorkingCntOffset  = RKVDEC2_SWREG_BASE + 0x41c,
    .RevisionOffset        = RKVDEC2_SWREG_BASE + 0x004,
    .KickRegOffset         = RKVDEC2_REG_DEC_E_OFFSET,
    .KickRegBit            = RKVDEC2_REG_DEC_E_BIT,
};

/* Forward declarations */
static VOID RkMppJobStart(_In_ WDFDEVICE Device, _In_ RKMPP_JOB *Job);
static VOID RkMppJobComplete(_In_ WDFDEVICE Device,
                             _In_ NTSTATUS Result,
                             _In_ UINT32 HardwareStatus);
static KSTART_ROUTINE RkMppPollerThread;

static BOOLEAN
RkMppJobReferencesBuffer(_In_ const RKMPP_JOB *Job,
                         _In_ WDFFILEOBJECT File,
                         _In_ UINT64 Cookie)
{
    if (!Job || Job->Owner != File || Cookie == 0) return FALSE;

    for (UINT32 i = 0; i < Job->DenseIovaSlotCount; i++) {
        if (Job->DenseIovaSlots[i].BufferHandle == Cookie) return TRUE;
    }
    for (UINT32 i = 0; i < Job->BufRefCount; i++) {
        if (Job->BufRefs[i].BufferHandle == Cookie) return TRUE;
    }
    return FALSE;
}

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
    Queue->LastOwner   = NULL;
    Queue->LastDecMode = 0;
    Queue->LastValid   = FALSE;
    RtlZeroMemory(Queue->OwnerLru, sizeof(Queue->OwnerLru));

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
                   "rkmpp: poller thread create failed 0x%08x\n", status);
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
                       "rkmpp: file-cleanup in-flight wait timed out\n");
            /* Sever the in-flight job's references to the buffer-pool
             * MDLs before the caller (EvtFileCleanup) runs BufFreeAll.
             * The poller is still alive and will eventually call
             * JobComplete, which dereferences OutputFrameMdl /
             * ColmvCurMdl in KeFlushIoBuffers.  Without this clear,
             * those flushes hit freed MDL memory → bugcheck.
             * JobComplete already null-checks both fields and skips
             * the flush; taking q->Lock here serializes against its
             * acquisition of the same lock around the flushes. */
            KeAcquireSpinLock(&q->Lock, &old);
            if (q->InFlight == inFlight && inFlight->Owner == File) {
                inFlight->OutputFrameMdl = NULL;
                inFlight->ColmvCurMdl    = NULL;
            }
            KeReleaseSpinLock(&q->Lock, old);
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
 * RkMppJobQueueHasOtherOwner — TRUE when InFlight or Pending contains at
 * least one job whose Owner is NOT `File`.  Intended for EvtFileCleanup
 * to decide whether the session-end PD power-cycle is safe (no other
 * concurrent decode session on this engine) or whether it would yank the
 * hardware out from under another File.
 *
 * Call AFTER RkMppJobsDrainOwner so this File's contribution is already
 * gone — the call then reduces to "is anyone else still using me".
 * --------------------------------------------------------------------- */
BOOLEAN
RkMppJobQueueHasOtherOwner(_In_ WDFDEVICE Device, _In_ WDFFILEOBJECT File)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    KIRQL old;
    BOOLEAN otherActive = FALSE;

    KeAcquireSpinLock(&q->Lock, &old);
    if (q->InFlight && q->InFlight->Owner != File) {
        otherActive = TRUE;
    } else {
        for (PLIST_ENTRY e = q->Pending.Flink;
             e != &q->Pending;
             e = e->Flink) {
            RKMPP_JOB *cand = CONTAINING_RECORD(e, RKMPP_JOB, Link);
            if (cand->Owner != File) { otherActive = TRUE; break; }
        }
    }
    KeReleaseSpinLock(&q->Lock, old);
    return otherActive;
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
             * read of the codec's REVISION register (offset 0x004 in
             * the SWREG window) goes through the same AXI bus the
             * codec used for its DMA writes; the read can't return
             * until prior writes from the same master domain have
             * committed, so it forces drain.  Plus a short
             * KeStallExecutionProcessor as a belt-and-suspenders
             * upper bound for any NoC-side buffering not covered by
             * the AXI ordering rule. */
            volatile ULONG drain = READ_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + ops->RevisionOffset));
            UNREFERENCED_PARAMETER(drain);
            /* AXI write-tail drain via polled idle indicator.  The
             * codec's PERF_WORKING_CNT (offset 0x41c, BSP
             * RKVDEC_PERF_WORKING_CNT) increments every cycle the codec
             * is processing.  Once DEC_RDY is asserted AND the counter
             * stops changing for several consecutive reads, the codec's
             * processing engine is idle — but the AXI write channel may
             * still be retiring buffered pixel writes through the NoC
             * to DRAM.
             *
             * BSP Linux gets away with no explicit drain because the
             * dma_buf_begin_cpu_access path (cache invalidate + memory
             * barrier) takes long enough naturally; our Windows IOCTL
             * → user-mode read path is much tighter and races the
             * codec's tail-end writes.  Manifests in mft_play (fast
             * pacing, no inter-kick delay) as non-deterministic
             * macroblock corruption starting once back-to-back kicks
             * stack up; mft_decode (slow PNG dump per frame) doesn't
             * hit it.
             *
             * Bounds: minimum 200 µs (covers basic AXI BVALID round-
             * trip), maximum 10000 µs (4K worst case with chains of
             * alt-refs).  Step 100 µs per poll.  Require 5 consecutive
             * equal reads (= 500 µs of unchanged counter) before
             * declaring the engine idle — earlier `>= 2` saw a 100 µs
             * gap between NoC bursts as "stable" and exited too soon.
             *
             * After PERF_WORKING_CNT settles, an unconditional 1500 µs
             * settle gives the AXI write channel time to retire the
             * codec's last pixel writes through the NoC to DRAM.  This
             * mirrors the original fixed 1500 µs stall (replaced in
             * 2026-05-04 when the polled drain went in) — restored as
             * a belt-and-suspenders since PERF_WORKING_CNT alone tracks
             * codec engine state, not the data fabric. */
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

        /* Ack pending status bits using the BSP rkvdec2 convention:
         * Linux reads RKVDEC_REG_INT_EN, then clears it by writing 0.
         * Keep this Linux-shaped while diagnosing the H.264 B-frame
         * Windows-only divergence; writing hwStatus back is a driver
         * asymmetry we do not need. */
        if (hwStatus) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset),
                0);
        }

        /* Diagnostic dump only when the codec genuinely didn't finish —
         * DONE bit not set within the poll budget.  Codec routinely
         * raises ERROR_STA / TIMEOUT_STA *together with* DEC_RDY_STA on
         * what is otherwise a successful decode (informational warnings
         * about the bitstream); BSP treats those as "errors" too but
         * just bumps reset_request and moves on.  Dumping on every such
         * "error+done" kick used to flood DbgView at 30 fps × multi-line
         * dumps.  Use no-DONE-at-all as the trigger — that's the case
         * where the codec is stuck and we have no useful state.
         * The dump format is rkvdec2-specific (bank layout differs on
         * AV1) — gate it accordingly. */
        if ((hwStatus & RKVDEC2_INT_DEC_RDY_STA) == 0) {
            /* Snapshot the in-flight job's switch state stamped by
             * RkMppJobStart so the dump can report dec_mode + switch
             * flags without inferring from log-line ordering.  Read
             * under the queue lock since RkMppJobComplete can null
             * InFlight concurrently; we still hold the kick event so
             * the job hasn't been freed yet, but the pointer load
             * needs to be ordered. */
            UINT64        kickJobId        = 0;
            WDFFILEOBJECT kickOwner        = NULL;
            UINT32        kickDecMode      = 0;
            UINT32        kickPrevDecMode  = 0;
            WDFFILEOBJECT kickPrevOwner    = NULL;
            BOOLEAN       kickSwitchOwner  = FALSE;
            BOOLEAN       kickSwitchMode   = FALSE;
            BOOLEAN       kickPrevValid    = FALSE;
            {
                KIRQL kIrql;
                KeAcquireSpinLock(&q->Lock, &kIrql);
                if (q->InFlight) {
                    kickJobId        = q->InFlight->Id;
                    kickOwner        = q->InFlight->Owner;
                    kickDecMode      = q->InFlight->KickDecMode;
                    kickPrevDecMode  = q->InFlight->KickPrevDecMode;
                    kickPrevOwner    = q->InFlight->KickPrevOwner;
                    kickSwitchOwner  = q->InFlight->KickSwitchOwner;
                    kickSwitchMode   = q->InFlight->KickSwitchMode;
                    kickPrevValid    = q->InFlight->KickPrevValid;
                }
                KeReleaseSpinLock(&q->Lock, kIrql);
            }
            ULONG bank[14] = {0};
            for (int i = 0; i < 14; i++) {
                bank[i] = READ_REGISTER_ULONG(
                    (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_SWREG_BASE + 0x380 + i * 4));
            }
#define RB(off) READ_REGISTER_ULONG((volatile ULONG*)((PUCHAR)mmio + RKVDEC2_SWREG_BASE + (off)))
            ULONG rb_mode  = RB(0x024);
            ULONG rb_dec_e = RB(0x028);
            ULONG rb_imp   = RB(0x02C);
            ULONG rb_sec   = RB(0x030);
            ULONG rb_err   = RB(0x034);
            ULONG rb_strln = RB(0x040);
            ULONG rb_rlc   = RB(0x200);
            ULONG rb_decout= RB(0x208);
            ULONG rb_pps   = RB(0x284);
            ULONG rb_rps   = RB(0x28c);
            ULONG rb_cab   = RB(0x314);
            ULONG rb_scan  = RB(0x2d0);
            ULONG rb_pochi0= RB(0x320);
            ULONG rb_pochi1= RB(0x324);
            ULONG rb_pochi4= RB(0x330);
#undef RB
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkmpp: poller INT=0x%08x result=0x%08x\n"
                       "  job=%llu owner=%p mode=%u prev_owner=%p prev_mode=%u "
                       "switch_owner=%d switch_mode=%d prev_valid=%d\n"
                       "  ctrl readback mode=0x%08x dec_e=0x%08x imp=0x%08x "
                       "sec=0x%08x err=0x%08x strlen=0x%08x rlc=0x%08x\n"
                       "  addr readback decout=0x%08x pps=0x%08x rps=0x%08x "
                       "cabac=0x%08x scanlist=0x%08x\n"
                       "  poc_hi readback [0]=0x%08x [1]=0x%08x [4]=0x%08x\n"
                       "  irqbank[224..231]: %08x %08x %08x %08x %08x %08x %08x %08x\n"
                       "  irqbank[232..237]: %08x %08x %08x %08x %08x %08x\n",
                       hwStatus, result,
                       (unsigned long long)kickJobId, kickOwner, kickDecMode,
                       kickPrevOwner, kickPrevDecMode,
                       kickSwitchOwner ? 1 : 0,
                       kickSwitchMode  ? 1 : 0,
                       kickPrevValid   ? 1 : 0,
                       rb_mode, rb_dec_e, rb_imp, rb_sec, rb_err, rb_strln, rb_rlc,
                       rb_decout, rb_pps, rb_rps, rb_cab, rb_scan,
                       rb_pochi0, rb_pochi1, rb_pochi4,
                       bank[0], bank[1], bank[2], bank[3],
                       bank[4], bank[5], bank[6], bank[7],
                       bank[8], bank[9], bank[10], bank[11],
                       bank[12], bank[13]);

            PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(q->Device);
            if (iommu && iommu->Snapshot) {
                RKIOMMU_FAULT_SNAPSHOT snap = {0};
                if (NT_SUCCESS(iommu->Snapshot(iommu->Header.Context, &snap))) {
                    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                               "  iommu#0 STATUS=0x%08x INT_RAWSTAT=0x%08x "
                               "INT_STATUS=0x%08x FAULT_ADDR=0x%08x DTE=0x%08x\n"
                               "  iommu#1 STATUS=0x%08x INT_RAWSTAT=0x%08x "
                               "INT_STATUS=0x%08x FAULT_ADDR=0x%08x DTE=0x%08x\n",
                               snap.Status, snap.IntRawStat, snap.IntStatus,
                               snap.PageFaultAddr, snap.DteAddr,
                               snap.Status1, snap.IntRawStat1, snap.IntStatus1,
                               snap.PageFaultAddr1, snap.DteAddr1);
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
 * Phase 2: reads + acks the INT_STATUS register (placeholder offset 0x100)
 * and queues the WDF DPC.  The ISR will not fire in Phase 2 because we
 * never kick the hardware; this is structural for Phase 3.
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
        /* Match Linux BSP rkvdec2: read status, then clear with 0. */
        WRITE_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + ops->IntStatusOffset),
            0);
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
 * RkMppJobStart — kick the job.
 *
 * Phase 2 (software-completion stub): capture start timestamp and schedule
 * the software DPC for immediate completion.  Phase 3 replaces this with
 * writing the register list to MMIO and asserting the hardware kick bit.
 * --------------------------------------------------------------------- */

/* RKVDEC2_SWREG_BASE is defined at file scope above; it accounts for
 * the 0x100 cluster-link prefix at the head of our ACPI MMIO mapping. */

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
                   "rkmpp: JobStart with no MmioBase — failing job %llu\n",
                   (unsigned long long)Job->Id);
        RkMppJobComplete(Device, STATUS_DEVICE_NOT_READY, 0);
        return;
    }

    /* Cross-kick scheduling probe.  Extract dec_mode from the kick's
     * dense bank (swreg9 = Common[1], low 5 bits) and compare to the
     * previous kick.  Stamp the result onto the job so the poller's
     * timeout dump can surface it directly (no inference from nearby
     * `kick-switch` log lines).  Log ONLY on transitions to keep noise
     * bounded (a clean playback emits roughly one line per ~thousand
     * kicks at steady state). */
    {
        UINT32 dec_mode = Job->DenseBank.Common[1] & 0x1Fu;
        BOOLEAN owner_switch = q->LastValid && (q->LastOwner != Job->Owner);
        BOOLEAN mode_switch  = q->LastValid && (q->LastDecMode != dec_mode);

        Job->KickDecMode     = dec_mode;
        Job->KickPrevDecMode = q->LastValid ? q->LastDecMode : 0;
        Job->KickPrevOwner   = q->LastValid ? q->LastOwner   : NULL;
        Job->KickSwitchOwner = owner_switch;
        Job->KickSwitchMode  = mode_switch;
        Job->KickPrevValid   = q->LastValid;

        if (!q->LastValid || owner_switch || mode_switch) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                       "rkmpp: kick-switch job=%llu owner=%p mode=%u (prev owner=%p mode=%u) "
                       "switch_owner=%d switch_mode=%d first=%d\n",
                       (unsigned long long)Job->Id,
                       Job->Owner,
                       dec_mode,
                       Job->KickPrevOwner,
                       Job->KickPrevDecMode,
                       owner_switch ? 1 : 0,
                       mode_switch  ? 1 : 0,
                       q->LastValid ? 0 : 1);
        }
        q->LastOwner   = Job->Owner;
        q->LastDecMode = dec_mode;
        q->LastValid   = TRUE;

        /* Upsert (Owner, Id) into the per-Owner LRU table used by
         * RkMppJobComplete's promotion path.  Find an existing slot
         * for this Owner, else the slot with the smallest LastKickId
         * (or an empty slot — File==NULL counts as 0). */
        const ULONG K = ARRAYSIZE(q->OwnerLru);
        ULONG hit   = K;
        ULONG evict = 0;
        for (ULONG i = 0; i < K; ++i) {
            if (q->OwnerLru[i].File == Job->Owner) { hit = i; break; }
            if (q->OwnerLru[i].LastKickId < q->OwnerLru[evict].LastKickId) evict = i;
        }
        ULONG slot = (hit < K) ? hit : evict;
        q->OwnerLru[slot].File       = Job->Owner;
        q->OwnerLru[slot].LastKickId = Job->Id;
    }

    /* Conditional core reset.  BSP only resets after error/timeout — see
     * mpp_common.c:2026 (mpp_dev_reset gated on reset_request > 0).  We do
     * the same: NeedsCoreReset starts 1 (cleared first kick after PnP),
     * is set after any kick that errors/times out, and otherwise stays 0
     * so successful back-to-back kicks don't disturb codec internal state.
     *
     * Earlier hypothesis was that per-kick reset was needed to flush stale
     * FSM after a failed kick — that's still served (RkMppJobComplete sets
     * the flag on error).  But on the *first* successful decode after a
     * fresh PnP boot, the codec apparently expects to retain initialisation
     * state across kicks; resetting clobbers something it relies on, which
     * manifests as INT=0x10 dec_error_sta partway through frame. */
    if (RkMppQueryAndClearNeedsFullReset(Device)) {
        /* Wide hang-recovery reset — kicks the codec out of stuck FSM
         * states (dec_e=1 across kicks, no interrupt) that the narrow
         * CON40-bits-6..9 toggle below can't recover.  The unwedge
         * recipe is: hclk_rkvdec0_root + aclk_rkvdec0_root reset (lives
         * inside the FullCoreReset0 SOFTRST_CON40 mask, bits 3..6),
         * with strict quiesce of all I/O to rkvdec0 AND the IOMMU
         * across the reset window.
         *
         * Cross-File concurrency note: this engine may be shared with
         * another open File (scenario A).  Resetting here is safe for
         * that File too because (a) we're at the *start* of a kick, not
         * mid-DMA — the JobStart caller (SubmitDense) just transitioned
         * us from Pending→InFlight under the queue lock, the codec is
         * idle; (b) the dense-bank programming below rewrites every
         * covered swreg for this kick so no register state from the
         * other File is being preserved-then-lost; (c) the IOMMU
         * Disable+Enable below preserves the domain mapping (just
         * drops the walk cache), so the other File's iovas remain
         * valid for their next kick.
         *
         * Sequence:
         *   1. MaskIrq    — detach ISR (no MMIO; safe with clocks gated)
         *   2. Disable    — final IOMMU MMIO write (AHB_CONTROL=0) before
         *                   we yank the bus
         *   3. FullCoreReset — PMU bus-idle → assert root+slave+core
         *                      resets → udelay → deassert → release idle
         *   4. Enable     — re-program DTE_ADDR + AHB paging on the
         *                   freshly-reset IOMMU (more than just Reattach
         *                   — this is the "must be set up again" the
         *                   reset recipe requires)
         *   5. UnmaskIrq  — re-arm ISR
         *
         * Without 1+2, a stray fault IRQ during the reset window reads
         * IOMMU MMIO held in reset → UB.  Without 4 the next codec
         * kick lands at iova 0 / undefined phys. */
        PRKMPP_CCU_INTERFACE    ccu   = RkMppGetCcuIfc(Device);
        PRKIOMMU_INTERFACE      iommu = RkMppGetIommuIfc(Device);
        RKMPP_DEVICE_PUBLIC pub;
        RkMppGetPublic(Device, &pub);
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkmpp: escalating to FullCoreReset on UID=%u (job %llu)\n",
                   pub.Uid, (unsigned long long)Job->Id);

        if (iommu && iommu->MaskIrq) {
            iommu->MaskIrq(iommu->Header.Context);
        }
        if (iommu && iommu->Disable) {
            (void)iommu->Disable(iommu->Header.Context);
        }
        if (ccu) {
            if (pub.Uid == 0 && ccu->FullCoreReset0) {
                ccu->FullCoreReset0(ccu->Header.Context);
            } else if (pub.Uid == 1 && ccu->FullCoreReset1) {
                ccu->FullCoreReset1(ccu->Header.Context);
            }
        }
        BOOLEAN reattachOk = TRUE;
        if (iommu && iommu->Enable) {
            NTSTATUS rs = iommu->Enable(iommu->Header.Context);
            if (!NT_SUCCESS(rs)) {
                reattachOk = FALSE;
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                           "rkmpp: post-FullReset IOMMU Enable failed 0x%08x; "
                           "re-flagging for next kick\n", rs);
                RkMppSetNeedsFullReset(Device);
            }
        }
        if (iommu && iommu->UnmaskIrq) {
            iommu->UnmaskIrq(iommu->Header.Context);
        }
        (void)reattachOk;
        /* FullCoreReset subsumes the narrow path; clear the narrow flag
         * so the per-job assert/deassert below skips. */
        (void)RkMppQueryAndClearNeedsCoreReset(Device);
    } else if (RkMppQueryAndClearNeedsCoreReset(Device)) {
        /* AssertRvdec{0,1}CoreReset/DeassertRvdec{0,1}CoreReset toggle the
         * narrow per-codec reset bundle.  CON40 bits 6..9 for RVD0,
         * CON41 bits 6..8 for RVD1.  Dispatch on UID — matches BSP
         * mpp_rkvdec2.c:1074-1095 rkvdec2_clk_on/off per-device clock list. */
        PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
        RKMPP_DEVICE_PUBLIC pub;
        RkMppGetPublic(Device, &pub);
        if (ccu) {
            if (pub.Uid == 0 && ccu->AssertRvdec0CoreReset && ccu->DeassertRvdec0CoreReset) {
                ccu->AssertRvdec0CoreReset(ccu->Header.Context);
                ccu->DeassertRvdec0CoreReset(ccu->Header.Context);
            } else if (pub.Uid == 1 && ccu->AssertRvdec1CoreReset && ccu->DeassertRvdec1CoreReset) {
                ccu->AssertRvdec1CoreReset(ccu->Header.Context);
                ccu->DeassertRvdec1CoreReset(ccu->Header.Context);
            }
        }
    }

    /* Ungate the codec's leaf clocks (clk_rkvdec0_core / ca / hevc_ca).
     * BSP runs this at the start of every task via `mpp_power_on →
     * clk_on`; we mirror it so that gate→ungate cycles between every
     * pair of kicks, draining in-flight AXI traffic and resetting
     * clock-domain-crossing flops without disturbing bus-root MMIO.
     * Without this, sustained back-to-back kicks (zero-copy or
     * non-ref-skip paths) wedge the codec — see memory note
     * `rkmpp_zero_copy_kicks_too_fast.md`.
     *
     * v7 ifc split: UngateRvdec0LeafClocks targets CLKGATE_CON40 bits
     * (RVD0's CABAC / HEVC CABAC / core leaf clocks), UngateRvdec1LeafClocks
     * targets CLKGATE_CON41 bits 6..8 (RVD1's analogous leaves).
     * Dispatch on UID so each codec only touches its own clocks —
     * matches BSP per-device clk_on semantics.
     *
     * MUST happen before the IntStatus clear below — the prior
     * JobComplete left leaf clocks gated; writing to slots in the
     * codec's leaf-gated domain (IntStatus at SWREG+0x380) before
     * ungating silently dropped the write. */
    {
        RKMPP_DEVICE_PUBLIC pub;
        RkMppGetPublic(Device, &pub);
        PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
        if (ccu) {
            if (pub.Uid == 0 && ccu->UngateRvdec0LeafClocks) {
                ccu->UngateRvdec0LeafClocks(ccu->Header.Context);
            } else if (pub.Uid == 1 && ccu->UngateRvdec1LeafClocks) {
                ccu->UngateRvdec1LeafClocks(ccu->Header.Context);
            }
        }
    }

    /* Hard-clear the IRQ status word before the kick.  BSP's
     * `mpp_write_req` walks the {reg_start=224, reg_num=16} bank every
     * task and overwrites slot 224 (RKVDEC_REG_INT_EN) with the
     * regbuilder's value (typically 0 in the user-mode wire), wiping
     * any latched ERROR_STA(4) / TIMEOUT_STA(5) / BUF_EMPTY_STA(6) /
     * COLMV_REF_ERR_STA(7).  Our hardcoded `bank_ranges` below skips
     * the 224..240 bank entirely, so without this explicit zero those
     * latches would survive and the next kick would re-enter the
     * poller with the same error bits already asserted — the
     * "H264 wedges with persistent IRQ error bits vs Linux" symptom.
     *
     * Must come AFTER the leaf-clock ungate above — the IntStatus slot
     * is in the codec's leaf-gated clock domain. */
    WRITE_REGISTER_ULONG(
        (volatile ULONG *)((PUCHAR)mmio + ops->IntStatusOffset), 0);

    /* Configure the rkvdec2 internal AXI read caches.  BSP `rkvdec2_run`
     * (mpp_rkvdec2.c:344-350) writes these BEFORE the kick:
     *   CACHE_PERMIT_CACHEABLE (bit 0) | READ_ALLOCATE (bit 1) |
     *   LINE_SIZE_64_BYTES (bit 4) = 0x13.
     *
     * The BSP-relative offsets are 0x51c (CACHE0_SIZE) etc., where the
     * BSP MMIO base is 0xfdc38100 (already past the cluster-link prefix).
     * Our ACPI MMIO base is 0xfdc38000 (without the prefix), so we must
     * add RKVDEC2_SWREG_BASE (0x100) to land at the same physical
     * address.  Earlier code was writing at the un-prefixed offsets,
     * stomping on swreg slots ~260 instead of the actual cache regs —
     * codec was running with cache config = whatever those swreg writes
     * left behind (i.e., not the 0x13 we intended). */
    if (mmioLen >= RKVDEC2_SWREG_BASE + 0x600) {
        const ULONG cache_cfg = 0x13;
#define CACHE_OFF(o) ((PUCHAR)mmio + RKVDEC2_SWREG_BASE + (o))
        TRACED_WRITE_ULONG(CACHE_OFF(0x51c), cache_cfg);
        TRACED_WRITE_ULONG(CACHE_OFF(0x55c), cache_cfg);
        TRACED_WRITE_ULONG(CACHE_OFF(0x59c), cache_cfg);
        /* clear caches */
        TRACED_WRITE_ULONG(CACHE_OFF(0x510), 1);
        TRACED_WRITE_ULONG(CACHE_OFF(0x550), 1);
        TRACED_WRITE_ULONG(CACHE_OFF(0x590), 1);
#undef CACHE_OFF
    }
    /* ---- Dense-bank dispatch ----------------------------------------
     * Mirrors upstream Linux rkvdec-vdpu381.c: every word in every
     * covered bank is written to MMIO every kick (in BSP ascending
     * order), then idx 10 (kick) last.  No skip-if-zero, no
     * PrevNonzeroMask — the bank's zero-init guarantees slots the
     * regbuilder didn't touch land at MMIO=0 rather than stale.
     *
     * Iova substitution already happened in RkMppJobSubmitDense (slots
     * carry the resolved (iova + offset)[31:0] in the bank itself), so
     * the bulk write is a plain byte stream. */
    const ULONG  swregBase = ops->SwregBase;
    const struct {
        ULONG          firstIdx;
        ULONG          count;
        const UINT32  *src;
    } banks[6] = {
        { RKMPP_DENSE_COMMON_FIRST,   RKMPP_DENSE_COMMON_WORDS,
          Job->DenseBank.Common      },
        { RKMPP_DENSE_CPARAM_FIRST,   RKMPP_DENSE_CPARAM_WORDS,
          Job->DenseBank.CodecParams },
        { RKMPP_DENSE_CADDR_FIRST,    RKMPP_DENSE_CADDR_WORDS,
          Job->DenseBank.CommonAddr  },
        { RKMPP_DENSE_CODADDR_FIRST,  RKMPP_DENSE_CODADDR_WORDS,
          Job->DenseBank.CodecAddr   },
        { RKMPP_DENSE_HIPOC_FIRST,    RKMPP_DENSE_HIPOC_WORDS,
          Job->DenseBank.HighPoc     },
        { RKMPP_DENSE_STAT_FIRST,     RKMPP_DENSE_STAT_WORDS,
          Job->DenseBank.Stat        },
    };

    /* Bounds-check every bank against the MMIO window BEFORE any
     * MMIO write so we never half-program the codec. */
    for (int b = 0; b < 6; b++) {
        const ULONG firstByte = banks[b].firstIdx * 4u + swregBase;
        const ULONG lastByte  = firstByte + banks[b].count * 4u;
        if (lastByte > mmioLen) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: dense job %llu bank %d (idx %lu..%lu) "
                       "out of MMIO range (len 0x%x)\n",
                       (unsigned long long)Job->Id, b,
                       banks[b].firstIdx,
                       banks[b].firstIdx + banks[b].count - 1,
                       mmioLen);
            RkMppJobComplete(Device, STATUS_INVALID_PARAMETER, 0);
            return;
        }
    }
    /* Kick reg (idx 10) must also fit; it's inside common but we
     * write it separately so double-check. */
    const ULONG kickByte = RKMPP_DENSE_KICK_REG_IDX * 4u + swregBase;
    if (kickByte + sizeof(ULONG) > mmioLen) {
        RkMppJobComplete(Device, STATUS_INVALID_PARAMETER, 0);
        return;
    }

    /* Pre-kick clean: push CPU-dirty cache lines for the just-written
     * input buffers (bitstream + packed PPS/RPS/scaling) to DRAM so
     * the codec's first DMA read gets the fresh data.  Narrowed from
     * "every referenced buffer" to "only buffers we know CPU wrote
     * this kick" — refs/colmv/RCB/etc. aren't CPU-written so cleaning
     * them is wasted work that dominated submit_us at 1440p+. */
    for (UINT32 fi = 0; fi < Job->CleanMdlCount; fi++) {
        KeFlushIoBuffers(Job->CleanMdls[fi],
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }

    /* Bulk-write each bank in BSP ascending order.  TRACED_WRITE_ULONG
     * per word so the diagnostic trace shape matches the BSP
     * mpp_write_req walk (one trace event per swreg).  Idx 10 inside
     * Common bank is skipped here and written last as the kick. */
    for (int b = 0; b < 6; b++) {
        for (ULONG p = 0; p < banks[b].count; p++) {
            const ULONG idx = banks[b].firstIdx + p;
            if (idx == RKMPP_DENSE_KICK_REG_IDX) continue;
            TRACED_WRITE_ULONG(
                ((PUCHAR)mmio + idx * 4u + swregBase),
                banks[b].src[p]);
        }
    }

    /* Flush IOMMU TLB BEFORE the kick — matches BSP rkvdec2_run
     * order (mpp_rkvdec2.c:363 → mpp_write START_EN at :372).
     * Stale TLB entries from a previous decode session caused the
     * codec to read/write at obsolete physical addresses (manifested
     * as fault iovas in the 0xae0..0xe00 range). */
    PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(Device);
    if (iommu && iommu->FlushTlb) {
        iommu->FlushTlb(iommu->Header.Context);
    }

    /* Kick: write idx 10 last with the regbuilder-supplied value.
     * Engaging the poller is gated on the kick bit; a zero KickValue
     * is treated as "test/peek-only" and completes immediately. */
    TRACED_WRITE_ULONG(
        ((PUCHAR)mmio + RKMPP_DENSE_KICK_REG_IDX * 4u + swregBase),
        Job->DenseKickValue);

    if (Job->DenseKickValue & ops->KickRegBit) {
        KeSetEvent(&q->KickEvent, IO_NO_INCREMENT, FALSE);
    } else {
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
     * RKVDEC_INT_ERROR_MASK = COLMV_REF_ERR | BUF_EMPTY | TIMEOUT | ERROR_STA
     * = bits 4,5,6,7.  Plus we treat NTSTATUS failure as an error. */
    BOOLEAN failed = (!NT_SUCCESS(Result) || (HardwareStatus & 0xF0u));
    if (failed) {
        /* 2-tier escalation, BSP-aligned:
         *
         *   - True wedge (DEC_TIMEOUT_STA from the codec's own watchdog,
         *     or our poll timeout because no INT_STATUS bit ever latched)
         *     → wide FullCoreReset.  Narrow CON40 toggle can't unwedge a
         *     stuck FSM.
         *   - Any other err_mask hit (ERROR / BUF_EMPTY / COLMV_REF_ERR,
         *     with or without DEC_RDY) → narrow CoreReset.  BSP would
         *     issue a soft IOMMU force-reset here — we can't (see
         *     [[rkmpp_force_reset_unsafe]]) so narrow CoreReset is our
         *     equivalent.  Earlier code escalated to FullCoreReset on
         *     every `!dec_rdy` status, which wide-reset the codec on
         *     ordinary soft errors (recoverable BUF_EMPTY at GOP
         *     boundaries, transient COLMV_REF_ERR on flaky streams) and
         *     disrupted peer decode sessions for no recovery benefit. */
        BOOLEAN dec_to  = (HardwareStatus & RKVDEC2_INT_DEC_TIMEOUT_STA) != 0;
        BOOLEAN poll_to = (Result == (NTSTATUS)0xC0000507L /* STATUS_IO_TIMEOUT */);
        BOOLEAN stuck   = dec_to || poll_to;
        if (stuck) {
            RkMppSetNeedsFullReset(Device);
        } else {
            RkMppSetNeedsCoreReset(Device);
        }
        /* Track for diagnostics; the previous "two consecutive failures"
         * gate has been replaced by the per-failure stuck classifier
         * above, so the value isn't load-bearing now but stays useful
         * if we want to add a back-off later. */
        (void)RkMppExchangeLastJobFailed(Device, 1);

        /* Charge the error to the file-object that submitted this job so
         * EvtFileCleanup can promote its session-end teardown to the
         * soft-tier IOMMU force-reset.  Mirrors BSP's per-task
         * `reset_request` accumulation that drives `mpp_dev_reset` on
         * task finish. */
        if (job->Owner) {
            PRKMPP_FILE_CTX fctx = RkMppFileGet(job->Owner);
            if (fctx) InterlockedIncrement(&fctx->ErrorCount);
        }
    } else {
        /* Successful kick clears the consecutive-failure tracker so a
         * single isolated error later doesn't immediately escalate. */
        RkMppExchangeLastJobFailed(Device, 0);
    }

    /* Cache invalidate DMA-output buffers the CPU will read after this kick.
     * Per MS docs, `ReadOperation=TRUE` means data flow is device→memory,
     * which on ARM64 emits the invalidate (`dc ivac`) needed to drop stale
     * CPU cache lines so the next CPU read reloads from DRAM.  An earlier
     * comment here had the boolean flipped — claiming FALSE invalidates —
     * which would explain the AV1 mft_play non-determinism: clean (FALSE)
     * leaves stale prefetched lines resident, and whether they survive to
     * the next user-mode read depends on cache pressure (mft_decode's PNG
     * write evicts them; mft_play's tight loop doesn't).
     *
     * OutputFrameMdl — pool_output NV12 (MmCached); fully committed before
     *   the PERF_WORKING_CNT drain exits.
     * ColmvCurMdl — pool_colmv for the current picture.  User-mode may
     *   dump it after WAIT_JOB, and subsequent kicks feed it back as a
     *   ref-colmv buffer, so drop stale CPU-side zero lines after DMA. */
    if (job->OutputFrameMdl) {
        KeFlushIoBuffers(job->OutputFrameMdl,
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }
    if (job->ColmvCurMdl) {
        KeFlushIoBuffers(job->ColmvCurMdl,
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }
    /* Move from InFlight to Completed. */
    q->InFlight = NULL;
    InsertTailList(&q->Completed, &job->Link);

    /* Signal the waiter before we potentially start the next job. */
    KeSetEvent(&job->Done, IO_NO_INCREMENT, FALSE);

    /* Promote the next pending job with per-Owner LRU fairness.
     *
     * Strict FIFO promotion would let a File submitting a burst of N
     * jobs kick all N before a peer File's first job runs.  Under
     * sustained multi-stream load that starves the peer past the
     * codec's reg32 watchdog (78 ms @ 4K) — see
     * `rkmpp_cross_file_starvation` + the Linux baseline showing BSP
     * multiplexes the same workload on one core with zero resets via
     * global per-task fair-share.
     *
     * LRU rule: for each Pending candidate, score = the Owner's
     * LastKickId from q->OwnerLru (0 = never kicked = oldest).  Pick
     * the minimum.  Stable tiebreak via list order (first encountered
     * wins) so within an Owner we still drain in submit order.  Two
     * streams → strict alternation (peer's LastKickId is always older).
     * Three+ streams → cycles fairly through all Owners regardless of
     * how bursty any one is. */
    RKMPP_JOB *next = NULL;
    if (!IsListEmpty(&q->Pending)) {
        PLIST_ENTRY chosen = q->Pending.Flink;
        UINT64 bestScore = (UINT64)-1;
        for (PLIST_ENTRY e = q->Pending.Flink;
             e != &q->Pending;
             e = e->Flink) {
            RKMPP_JOB *cand = CONTAINING_RECORD(e, RKMPP_JOB, Link);
            UINT64 score = 0;
            for (ULONG i = 0; i < ARRAYSIZE(q->OwnerLru); ++i) {
                if (q->OwnerLru[i].File == cand->Owner) {
                    score = q->OwnerLru[i].LastKickId;
                    break;
                }
            }
            if (score < bestScore) {
                bestScore = score;
                chosen = e;
                if (score == 0) break; /* never-kicked Owner — can't beat */
            }
        }
        RemoveEntryList(chosen);
        next = CONTAINING_RECORD(chosen, RKMPP_JOB, Link);
        q->InFlight = next;
    }

    KeReleaseSpinLock(&q->Lock, oldIrql);

    /* Gate the codec's leaf clocks now that this job is finished.
     * Pairs with the ungate at the head of RkMppJobStart; the
     * gate→ungate transition between successive kicks drains the
     * codec's internal AXI/clock-domain pipelines, matching what BSP
     * `mpp_power_off → clk_off` does at end-of-task.  Done before
     * starting the next job so the next kick sees a fresh ungate.
     *
     * Note: per-job RaiseCluster/DropCluster removed — they were
     * always short-circuit no-ops nested under the device-lifetime
     * Raise from PrepareHardware, and DropCluster ran from DISPATCH
     * (DPC) which would have blocked the FAST_MUTEX serialization
     * added in rkmpp_ccu.
     *
     * v7 ifc split: GateRvdec0LeafClocks targets CLKGATE_CON40 bits
     * (RVD0's leaves), GateRvdec1LeafClocks targets CLKGATE_CON41
     * bits 6..8 (RVD1's leaves).  Dispatch on UID so each codec only
     * gates its own clocks — no cross-codec wedge, and RVD1 now gets
     * the same per-kick clock-cycle break BSP rkvdec2_clk_off applies.
     *
     * RVD0 was previously skipped here as a workaround for a CON40
     * wedge; root cause turned out to be RVD0 leaf clocks running at
     * ~1/4 BSP rate (148/396/148 MHz vs BSP's 594/1000/594), making
     * the CDC settle after gate cycle exceed the codec FSM tolerance.
     * Fixed by raising RVD0 CLKSEL_CON90/91 values in rkmpp_ccu. */
    {
        RKMPP_DEVICE_PUBLIC pub;
        RkMppGetPublic(Device, &pub);
        PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
        if (ccu) {
            if (pub.Uid == 0 && ccu->GateRvdec0LeafClocks) {
                ccu->GateRvdec0LeafClocks(ccu->Header.Context);
            } else if (pub.Uid == 1 && ccu->GateRvdec1LeafClocks) {
                ccu->GateRvdec1LeafClocks(ccu->Header.Context);
            }
        }
    }

    /* Kick the next job outside the spin lock. */
    if (next) {
        RkMppJobStart(Device, next);
    }
}

/* -----------------------------------------------------------------------
 * Dense-bank submit / peek (rkvdec2 H.264 + H.265)
 *
 * Caller hands us a zero-init'd RKMPP_DENSE_BANK plus a list of
 * (RegIdx, BufferHandle, IovaOffset) substitution slots.  We resolve
 * each handle to an iova up-front and stamp (iova + offset)[31:0] into
 * the bank at slot.RegIdx.  RkMppJobStart bulk-writes each bank in
 * BSP order; idx 10 (kick) is written last as a separate MMIO write.
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobSubmitDense(_In_ WDFDEVICE Device,
                    _In_ WDFFILEOBJECT File,
                    _In_ const RKMPP_SUBMIT_DENSE_JOB_IN *In,
                    _Out_ RKMPP_SUBMIT_DENSE_JOB_OUT *Out)
{
    if (In->IovaSlotCount > RKMPP_MAX_DENSE_IOVA_SLOTS ||
        In->BufRefCount   > RKMPP_MAX_BUF_REFS) {
        return STATUS_INVALID_PARAMETER;
    }

    RKMPP_JOB *job = (RKMPP_JOB *)ExAllocatePool2(
        POOL_FLAG_NON_PAGED, sizeof(RKMPP_JOB), 'JppM');
    if (!job) return STATUS_INSUFFICIENT_RESOURCES;

    RtlZeroMemory(job, sizeof(*job));
    KeInitializeEvent(&job->Done, NotificationEvent, FALSE);
    job->Owner          = File;
    job->DenseKickValue = In->KickValue;

    /* Copy the bank verbatim; iova-substitution stamps over selected
     * slots below. */
    RtlCopyMemory(&job->DenseBank, &In->Bank, sizeof(job->DenseBank));

    /* Walk substitution slots: validate RegIdx falls in an address bank,
     * resolve buffer handle → iova, write (iova + offset)[31:0] into
     * the dense bank at the slot's idx.  Also classify the slot for
     * per-kick cache maintenance — reg130 = output, reg131 = current
     * colmv, reg128/129/161/163/180 = inputs the CPU just wrote and
     * the codec is about to DMA. */
    /* VP9-specific: reg162 (last_prob_base) is dual-mode — it points
     * at prob_default on keyframe/cold-context (CPU-written; needs
     * pre-kick clean) but at prob_loop[fcx] on inter (HW writeback
     * target — cleaning it would push stale CPU lines over HW's
     * adapted CDFs).  reg172 (update_prob_wr_base) is always the
     * writeback target.  If reg162's buffer differs from reg172's,
     * reg162 is prob_default and must be flushed; if equal, leave
     * the cache alone.  Capture handles here and post-process below. */
    UINT64 reg162Handle = 0;
    UINT64 reg172Handle = 0;

    job->DenseIovaSlotCount = In->IovaSlotCount;
    for (UINT32 i = 0; i < In->IovaSlotCount; i++) {
        const RKMPP_DENSE_IOVA_SLOT *src = &In->IovaSlots[i];
        RKMPP_DENSE_IOVA_SLOT       *dst = &job->DenseIovaSlots[i];
        *dst = *src;

        if (!RkMppDenseIsAddressReg(src->RegIdx)) {
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_PARAMETER;
        }
        if (src->BufferHandle == 0) {
            /* Caller filtered zero handles in user-mode (emit_iova) but
             * keep the check tight: an iova slot with handle == 0 is
             * indistinguishable from "no write" and would leave the
             * bank slot unsubstituted.  Reject. */
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_PARAMETER;
        }

        UINT64 iova    = 0;
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

        /* Stamp (iova + offset)[31:0] into the dense bank at RegIdx. */
        UINT32 *slot = NULL;
        if (src->RegIdx >= RKMPP_DENSE_CADDR_FIRST &&
            src->RegIdx <= RKMPP_DENSE_CADDR_LAST) {
            slot = &job->DenseBank.CommonAddr[src->RegIdx
                                              - RKMPP_DENSE_CADDR_FIRST];
        } else if (src->RegIdx >= RKMPP_DENSE_CODADDR_FIRST &&
                   src->RegIdx <= RKMPP_DENSE_CODADDR_LAST) {
            slot = &job->DenseBank.CodecAddr[src->RegIdx
                                              - RKMPP_DENSE_CODADDR_FIRST];
        }
        if (!slot) {
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_PARAMETER;
        }
        *slot = (UINT32)(iova + src->IovaOffset);

        /* Cache-maintenance classification.  Byte offset = RegIdx * 4. */
        PMDL mdl = NULL;
        if (!NT_SUCCESS(RkMppBufLookupMdl(File, src->BufferHandle, &mdl)) ||
            mdl == NULL) {
            continue;
        }
        const UINT32 byteOff = src->RegIdx * 4u;
        if (byteOff == 0x288u) {
            /* reg162 — defer cache decision until we know reg172's
             * handle (see comment above and post-loop block below). */
            reg162Handle = src->BufferHandle;
            continue;
        }
        if (byteOff == 0x2B0u) {                /* reg172 = HW writeback */
            reg172Handle = src->BufferHandle;
            continue;
        }
        if (byteOff == 0x208u) {                /* output frame */
            if (job->OutputFrameMdl == NULL) job->OutputFrameMdl = mdl;
        } else if (byteOff == 0x20Cu) {         /* current colmv */
            if (job->ColmvCurMdl == NULL) job->ColmvCurMdl = mdl;
            BOOLEAN already = FALSE;
            for (UINT32 k = 0; k < job->CleanMdlCount; k++) {
                if (job->CleanMdls[k] == mdl) { already = TRUE; break; }
            }
            if (!already &&
                job->CleanMdlCount < RTL_NUMBER_OF(job->CleanMdls)) {
                job->CleanMdls[job->CleanMdlCount++] = mdl;
            }
        } else if (byteOff == 0x200u || byteOff == 0x204u ||
                   byteOff == 0x280u ||
                   byteOff == 0x284u || byteOff == 0x28Cu ||
                   byteOff == 0x2D0u) {
            /* CPU-written buffers the codec DMAs as input:
             *   0x200 = reg128  RLC bitstream                  (per-kick)
             *   0x204 = reg129  RLC write                      (per-kick)
             *   0x280 = reg160  VP9 delta_prob_base — FillProbs output,
             *                   CPU rewrites every kick.
             *   0x284 = reg161  H.265 PPS / VP9 scratch
             *   0x28C = reg163  H.265 RPS / VP9 scratch
             *   0x2D0 = reg180  H.265 scanlist
             *
             * Deliberately NOT in this list (VP9):
             *   0x288 = reg162  last_prob_base.  CPU writes default
             *                   probs once at session init; thereafter
             *                   HW alternates write/read via reg172/162.
             *                   Pre-kick cleaning would push CPU's stale
             *                   defaults over HW's writeback on every
             *                   inter frame; the codec then reads back
             *                   defaults instead of the adapted CDFs.
             *   0x2A0 = reg168  segid_last_base — same hazard for seg
             *                   streams; revisit if seg paths land. */
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


    /* VP9 reg162 disambiguation: clean only when it points at a
     * different buffer than reg172.  Equal handles ⇒ prob_loop[fcx]
     * (HW writeback target, leave cache alone — clean would push
     * stale CPU lines over HW's adapted CDFs).  Different ⇒
     * prob_default (CPU-written; flush so codec's first read returns
     * the memcpy'd defaults instead of whatever cache last held). */
    if (reg162Handle != 0 && reg162Handle != reg172Handle) {
        PMDL reg162Mdl = NULL;
        if (NT_SUCCESS(RkMppBufLookupMdl(File, reg162Handle, &reg162Mdl)) &&
            reg162Mdl != NULL) {
            BOOLEAN already = FALSE;
            for (UINT32 k = 0; k < job->CleanMdlCount; k++) {
                if (job->CleanMdls[k] == reg162Mdl) { already = TRUE; break; }
            }
            if (!already &&
                job->CleanMdlCount < RTL_NUMBER_OF(job->CleanMdls)) {
                job->CleanMdls[job->CleanMdlCount++] = reg162Mdl;
            }
        }
    }

    job->BufRefCount = In->BufRefCount;
    if (In->BufRefCount > 0) {
        RtlCopyMemory(job->BufRefs, In->BufRefs,
                      In->BufRefCount * sizeof(RKMPP_BUFFER_REF));
    }

    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    job->Id = (UINT64)InterlockedIncrement64(&q->NextId);

    /* Two-tier admission control:
     *   - Per-device total cap (8): a hard backpressure ceiling for the
     *     whole engine, unchanged from single-File days.
     *   - Per-File cap (4): prevents one open handle from filling the
     *     queue and starving a concurrent decode session on the same
     *     engine.  Counts both Pending and InFlight owned by this File
     *     so a single greedy File can't have 4 pending + 1 in flight
     *     while a peer File starves. */
    enum {
        RKMPP_MAX_PENDING_JOBS          = 8,
        RKMPP_MAX_PENDING_JOBS_PER_FILE = 4,
    };
    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);

    ULONG pendingCount = 0;
    ULONG ownerCount   = 0;
    if (q->InFlight && q->InFlight->Owner == File) ownerCount++;
    for (PLIST_ENTRY e = q->Pending.Flink;
         e != &q->Pending;
         e = e->Flink) {
        RKMPP_JOB *cand = CONTAINING_RECORD(e, RKMPP_JOB, Link);
        pendingCount++;
        if (cand->Owner == File) ownerCount++;
        if (pendingCount >= RKMPP_MAX_PENDING_JOBS &&
            ownerCount   >= RKMPP_MAX_PENDING_JOBS_PER_FILE) break;
    }
    if (pendingCount >= RKMPP_MAX_PENDING_JOBS ||
        ownerCount   >= RKMPP_MAX_PENDING_JOBS_PER_FILE) {
        KeReleaseSpinLock(&q->Lock, oldIrql);
        ExFreePoolWithTag(job, 'JppM');
        return STATUS_DEVICE_BUSY;
    }

    BOOLEAN startNow = (q->InFlight == NULL);
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

NTSTATUS
RkMppJobPeekDense(_In_ WDFDEVICE Device,
                  _In_ WDFFILEOBJECT File,
                  _In_ UINT64 JobId,
                  _Out_ RKMPP_PEEK_DENSE_JOB_OUT *Out)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    KIRQL oldIrql;
    NTSTATUS status = STATUS_NOT_FOUND;

    KeAcquireSpinLock(&q->Lock, &oldIrql);

    RKMPP_JOB *job = NULL;
    if (q->InFlight && q->InFlight->Id == JobId) job = q->InFlight;
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
        if (job->Owner != File) {
            status = STATUS_NOT_FOUND;
            goto done;
        }
        Out->StructSize = sizeof(*Out);
        Out->KickValue  = job->DenseKickValue;
        RtlCopyMemory(&Out->Bank, &job->DenseBank, sizeof(Out->Bank));
        status = STATUS_SUCCESS;
    }

done:
    KeReleaseSpinLock(&q->Lock, oldIrql);
    return status;
}

BOOLEAN
RkMppJobBufferInUse(_In_ WDFDEVICE Device,
                    _In_ WDFFILEOBJECT File,
                    _In_ UINT64 Cookie)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    KIRQL oldIrql;
    BOOLEAN inUse = FALSE;

    KeAcquireSpinLock(&q->Lock, &oldIrql);

    if (RkMppJobReferencesBuffer(q->InFlight, File, Cookie)) {
        inUse = TRUE;
        goto done;
    }

    for (PLIST_ENTRY e = q->Pending.Flink; e != &q->Pending; e = e->Flink) {
        RKMPP_JOB *job = CONTAINING_RECORD(e, RKMPP_JOB, Link);
        if (RkMppJobReferencesBuffer(job, File, Cookie)) {
            inUse = TRUE;
            goto done;
        }
    }

    for (PLIST_ENTRY e = q->Completed.Flink; e != &q->Completed; e = e->Flink) {
        RKMPP_JOB *job = CONTAINING_RECORD(e, RKMPP_JOB, Link);
        if (RkMppJobReferencesBuffer(job, File, Cookie)) {
            inUse = TRUE;
            goto done;
        }
    }

done:
    KeReleaseSpinLock(&q->Lock, oldIrql);
    return inUse;
}

/* -----------------------------------------------------------------------
 * RkMppJobWait — IOCTL_RKMPP_WAIT_JOB handler
 * --------------------------------------------------------------------- */

NTSTATUS
RkMppJobWait(_In_ WDFDEVICE Device,
             _In_ WDFFILEOBJECT File,
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
    if (job->Owner != File) {
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
             *   InFlight → leave it attached.  The poller/hardware still
             *              owns the kick; detaching InFlight here would
             *              allow a later submit to start a second hardware
             *              job while the first is still active.
             *
             * The previous "free unconditionally" implementation left
             * the LIST_ENTRY in q->Pending pointing into freed pool —
             * the next SubmitJob iterating Pending crashed at
             * `e->Flink` once the slot was reused. */
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
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkmpp: WAIT_JOB timeout on InFlight job %llu — "
                           "leaving attached for poller completion\n",
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
