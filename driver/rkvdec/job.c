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
/* Conservative superset: BSP's `RKVDEC_INT_ERROR_MASK` is bits 4|5|6|7
 * (ERROR | TIMEOUT | BUF_EMPTY | COLMV_REF_ERR).  We include BUS_STA
 * (bit 3) too — empirically a status of bit 3 alone (without DEC_RDY)
 * has paired with stale codec FSM state that produces all-zero output
 * unless we trip NeedsCoreReset.  Slightly over-classifies vs BSP but
 * preserves the reset cadence the codec actually depends on. */
#define RKVDEC2_INT_ERROR_MASK \
    (RKVDEC2_INT_DEC_ERROR_STA | RKVDEC2_INT_DEC_TIMEOUT_STA | \
     RKVDEC2_INT_DEC_BUS_STA | RKVDEC2_INT_BUF_EMPTY_STA | \
     RKVDEC2_INT_COLMV_REF_ERROR)

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

    for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
        if (Job->Writes[i].BufferHandle == Cookie) return TRUE;
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
    RtlZeroMemory(Queue->PrevNonzeroMask, sizeof(Queue->PrevNonzeroMask));

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
                       "  ctrl readback mode=0x%08x dec_e=0x%08x imp=0x%08x "
                       "sec=0x%08x err=0x%08x strlen=0x%08x rlc=0x%08x\n"
                       "  addr readback decout=0x%08x pps=0x%08x rps=0x%08x "
                       "cabac=0x%08x scanlist=0x%08x\n"
                       "  poc_hi readback [0]=0x%08x [1]=0x%08x [4]=0x%08x\n"
                       "  irqbank[224..231]: %08x %08x %08x %08x %08x %08x %08x %08x\n"
                       "  irqbank[232..237]: %08x %08x %08x %08x %08x %08x\n",
                       hwStatus, result,
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
    if (RkMppQueryAndClearNeedsCoreReset(Device)) {
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
        /* Codec regs are now back to power-on zero — drop the prev-mask
         * so the next kick re-writes every nonzero reg (the skip logic
         * relies on the mask reflecting what's currently in HW). */
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
     * matches BSP per-device clk_on semantics. */
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
                       "rkmpp: job %llu swreg-off 0x%x (mmio 0x%x) out of "
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
     * input buffers (bitstream + packed PPS/RPS/scaling) to DRAM so
     * the codec's first DMA read gets the fresh data.  Narrowed from
     * "every referenced buffer" to "only buffers we know CPU wrote
     * this kick" — refs/colmv/RCB/etc. aren't CPU-written so cleaning
     * them is wasted work that dominated submit_us at 1440p+. */
    for (UINT32 fi = 0; fi < Job->CleanMdlCount; fi++) {
        KeFlushIoBuffers(Job->CleanMdls[fi],
                         /*ReadOperation*/ TRUE, /*DmaOperation*/ TRUE);
    }

    {
        /* Write the register list to MMIO in BSP-equivalent order.  BSP's
         * mpp_write_req walks regs[s..e] in strict ascending order and
         * writes EVERY entry in the bank (including zeros for unset slots).
         * Our regbuilder produces entries in code-emit order with gaps —
         * which leaves a different INTERMEDIATE codec state during the
         * write sequence.  To match BSP exactly:
         *   1. Build a flat 320-entry array indexed by reg idx.
         *   2. Mark which entries our regbuilder set; rest default to 0.
         *   3. For each BSP-bank range, write ALL entries (including zeros)
         *      in ascending order.
         * Skip idx 10 (kick) in this pass — it goes last as a separate write. */
        static const struct { ULONG first, last; } bank_ranges[6] = {
            {   8,  32 }, {  64, 112 }, { 128, 142 },
            { 160, 199 }, { 200, 204 }, { 256, 277 },
        };
        ULONG bank_vals[6][80] = {0};
        BOOLEAN bank_seen[6][80] = {0};
        ULONG cur_nonzero[16] = {0};   /* bitmap of regs nonzero this kick */

        for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
            const RKMPP_REG_WRITE *w = &Job->Writes[i];
            ULONG idx = w->Offset / 4;
            if (idx == 10) continue;
            for (int b = 0; b < 6; b++) {
                if (idx >= bank_ranges[b].first && idx <= bank_ranges[b].last) {
                    ULONG pos = idx - bank_ranges[b].first;
                    bank_vals[b][pos] = w->Value;
                    bank_seen[b][pos] = TRUE;
                    if (w->Value != 0)
                        cur_nonzero[idx >> 5] |= (1u << (idx & 31));
                    break;
                }
            }
        }
        /* Skip MMIO writes for regs that were zero last kick AND are
         * zero this kick — same optimization as the AV1 path.  Cuts
         * the rkvdec2 bank-walk from ~155 writes down to ~80. */
        const ULONG *prev = q->PrevNonzeroMask;
        for (int b = 0; b < 6; b++) {
            ULONG count = bank_ranges[b].last - bank_ranges[b].first + 1;
            for (ULONG p = 0; p < count; p++) {
                ULONG idx = bank_ranges[b].first + p;
                if (idx == 10) continue;
                const ULONG bit = 1u << (idx & 31);
                const BOOLEAN was_nz = (prev[idx >> 5] & bit) != 0;
                const BOOLEAN is_nz  = (cur_nonzero[idx >> 5] & bit) != 0;
                if (!was_nz && !is_nz) continue;
                TRACED_WRITE_ULONG(
                    ((PUCHAR)mmio + idx * 4 + RKVDEC2_SWREG_BASE),
                    bank_vals[b][p]);
            }
        }
        /* Snapshot for next kick. */
        for (int i = 0; i < 16; i++)
            q->PrevNonzeroMask[i] = cur_nonzero[i];
        /* Now the kick (idx 10) — written last, separately. */
        for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
            const RKMPP_REG_WRITE *w = &Job->Writes[i];
            if (w->Offset / 4 == 10) {
                TRACED_WRITE_ULONG(
                    ((PUCHAR)mmio + w->Offset + RKVDEC2_SWREG_BASE),
                    w->Value);
                break;
            }
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
     * RKVDEC_INT_ERROR_MASK = COLMV_REF_ERR | BUF_EMPTY | TIMEOUT | ERROR_STA
     * = bits 4,5,6,7.  Plus we treat NTSTATUS failure as an error. */
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

    /* If another job is pending, promote it now. */
    RKMPP_JOB *next = NULL;
    if (!IsListEmpty(&q->Pending)) {
        PLIST_ENTRY entry = RemoveHeadList(&q->Pending);
        next = CONTAINING_RECORD(entry, RKMPP_JOB, Link);
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
     * the same per-kick clock-cycle break BSP rkvdec2_clk_off applies. */
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
         *   Output frame  (reg130 / 0x208) — CPU will read post-decode.
         *     Captured into OutputFrameMdl for the post-IRQ invalidate.
         *
         *   Inputs (CPU writes, codec reads this kick):
         *     reg128 / 0x200 — RLC base (bitstream)
         *     reg129 / 0x204 — RLCWRITE base (alias of bitstream)
         *     reg131 / 0x20c — current colmv, zeroed by user-mode
         *     reg161 / 0x284 — packed PPS
         *     reg163 / 0x28C — packed RPS
         *     reg180 / 0x2D0 — packed scaling list
         *   Captured into CleanMdls[] for the pre-kick clean.
         *
         *   Everything else (refs, ref_colmv, RCB, error_ref, CABAC init)
         *   is skipped — see RKMPP_JOB comment for why. */
        PMDL mdl = NULL;
        if (!NT_SUCCESS(RkMppBufLookupMdl(File, src->BufferHandle, &mdl)) ||
            mdl == NULL) {
            continue;
        }
        /* Cache-maintenance MDL classification.
         *   Output  — codec writes, CPU reads post-decode.  Invalidate after kick.
         *   Input   — CPU writes, codec reads.  Clean before kick.
         *
         * rkvdec2 offsets: output reg130 = 0x208; current colmv reg131
         * = 0x20c; inputs reg128/129/161/163/180 =
         * 0x200/0x204/0x284/0x28C/0x2D0. */
        if (src->Offset == 0x208u) {      /* rkvdec2 output frame */
            if (job->OutputFrameMdl == NULL) job->OutputFrameMdl = mdl;
        } else if (src->Offset == 0x20Cu) { /* rkvdec2 current colmv */
            if (job->ColmvCurMdl == NULL) job->ColmvCurMdl = mdl;
            BOOLEAN already = FALSE;
            for (UINT32 k = 0; k < job->CleanMdlCount; k++) {
                if (job->CleanMdls[k] == mdl) { already = TRUE; break; }
            }
            if (!already &&
                job->CleanMdlCount < RTL_NUMBER_OF(job->CleanMdls)) {
                job->CleanMdls[job->CleanMdlCount++] = mdl;
            }
        } else if (src->Offset == 0x200u || src->Offset == 0x204u ||
                   src->Offset == 0x284u || src->Offset == 0x28Cu ||
                   src->Offset == 0x2D0u) {
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

    /* Per-job RaiseCluster removed — the device-lifetime Raise taken
     * in PrepareHardware keeps the CCU refcount at >= 1 across the
     * entire device lifetime, so the per-job 1<->2 bounce was always
     * a short-circuit no-op.  The matching per-job DropCluster (which
     * ran from DPC at DISPATCH_LEVEL) is removed too — see the
     * RkMppJobComplete comment for why this matters for the
     * FAST_MUTEX serialization in rkmpp_ccu. */

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
             _In_ WDFFILEOBJECT File,
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
        if (job->Owner != File) {
            status = STATUS_NOT_FOUND;
            goto done;
        }
        Out->RegWriteCount = job->RegWriteCount;
        Out->Reserved      = 0;
        if (job->RegWriteCount > 0) {
            RtlCopyMemory(Out->Writes, job->Writes,
                          job->RegWriteCount * sizeof(RKMPP_REG_WRITE));
        }
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
