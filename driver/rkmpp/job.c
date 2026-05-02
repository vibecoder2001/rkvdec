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
#include "bufpool.h"      /* for RkMppBufLookupIova (iova substitution) */
#include "ifc_client.h"   /* for RKMPP_CCU_INTERFACE via RkMppGetCcuIfc */

extern PRKIOMMU_INTERFACE RkMppGetIommuIfc(_In_ WDFDEVICE Device);

/* Defined in device.c — accessor pair for RKMPP_DEVICE.NeedsCoreReset. */
extern LONG RkMppQueryAndClearNeedsCoreReset(_In_ WDFDEVICE Device);
extern VOID RkMppSetNeedsCoreReset(_In_ WDFDEVICE Device);

/* Tracing wrapper for codec MMIO writes — matches BSP's
 * mpp_dev_debug=DEBUG_SET_REG output format so the two traces can be
 * diffed directly.  Set to 0 to disable. */
#define RKMPP_TRACE_WRITES 1
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
#define RKVDEC2_INT_ERROR_MASK \
    (RKVDEC2_INT_DEC_ERROR_STA | RKVDEC2_INT_DEC_TIMEOUT_STA | \
     RKVDEC2_INT_DEC_BUS_STA | RKVDEC2_INT_COLMV_REF_ERROR)

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
                (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_INT_STATUS_OFFSET));
            if (hwStatus & RKVDEC2_INT_DONE_MASK) goto have_status;
            KeStallExecutionProcessor(50);
        }
        {
            const ULONG kSleepIters = 200;
            LARGE_INTEGER iv;
            iv.QuadPart = -10000;   /* 1 ms relative, 100ns units */
            for (ULONG i = 0; i < kSleepIters; i++) {
                hwStatus = READ_REGISTER_ULONG(
                    (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_INT_STATUS_OFFSET));
                if (hwStatus & RKVDEC2_INT_DONE_MASK) goto have_status;
                KeDelayExecutionThread(KernelMode, FALSE, &iv);
            }
        }
have_status:
        if (hwStatus & RKVDEC2_INT_DONE_MASK) {
            result = (hwStatus & RKVDEC2_INT_ERROR_MASK)
                   ? STATUS_DEVICE_HARDWARE_ERROR
                   : STATUS_SUCCESS;
        }

        /* Dump the IRQ-status bank (idx 224..237 = bytes 0x380..0x3B4 in
         * SWREG-space, our_mmio +0x480..+0x4B4 with the 0x100 prefix). */
        ULONG bank[14] = {0};
        for (int i = 0; i < 14; i++) {
            bank[i] = READ_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_SWREG_BASE + 0x380 + i * 4));
        }

        /* Read back the control regs we just wrote — if the codec consumed
         * the kick we expect dec_e (idx 10 / off 0x28) to be self-cleared
         * to 0; if it stayed at 1 the kick never made it into the FSM. */
#define RB(off) READ_REGISTER_ULONG((volatile ULONG*)((PUCHAR)mmio + RKVDEC2_SWREG_BASE + (off)))
        ULONG rb_mode  = RB(0x024);
        ULONG rb_dec_e = RB(0x028);
        ULONG rb_imp   = RB(0x02C);
        ULONG rb_sec   = RB(0x030);
        ULONG rb_err   = RB(0x034);
        ULONG rb_strln = RB(0x040);
        ULONG rb_rlc   = RB(0x200);
        ULONG rb_decout= RB(0x208);     /* idx 130 DECOUT_BASE */
        ULONG rb_pps   = RB(0x284);     /* idx 161 PPS_BASE */
        ULONG rb_rps   = RB(0x28c);     /* idx 163 RPS_BASE */
        ULONG rb_cab   = RB(0x314);     /* idx 197 CABACTBL_BASE */
        ULONG rb_scan  = RB(0x2d0);     /* idx 180 SCANLIST_ADDR */
        ULONG rb_pochi0= RB(0x320);     /* idx 200 POC_HIGHBIT[0] */
        ULONG rb_pochi1= RB(0x324);     /* idx 201 */
        ULONG rb_pochi4= RB(0x330);     /* idx 204 */
#undef RB
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "  ctrl readback mode=0x%08x dec_e=0x%08x imp=0x%08x "
                   "sec=0x%08x err=0x%08x strlen=0x%08x rlc=0x%08x\n"
                   "  addr readback decout=0x%08x pps=0x%08x rps=0x%08x "
                   "cabac=0x%08x scanlist=0x%08x\n"
                   "  poc_hi readback [0]=0x%08x [1]=0x%08x [4]=0x%08x\n",
                   rb_mode, rb_dec_e, rb_imp, rb_sec, rb_err, rb_strln, rb_rlc,
                   rb_decout, rb_pps, rb_rps, rb_cab, rb_scan,
                   rb_pochi0, rb_pochi1, rb_pochi4);

        /* Ack pending status bits (write-1-to-clear). */
        if (hwStatus) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_INT_STATUS_OFFSET),
                hwStatus);
        }

        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp: poller INT=0x%08x result=0x%08x\n"
                   "  irqbank[224..231]: %08x %08x %08x %08x %08x %08x %08x %08x\n"
                   "  irqbank[232..237]: %08x %08x %08x %08x %08x %08x\n",
                   hwStatus, result,
                   bank[0], bank[1], bank[2], bank[3],
                   bank[4], bank[5], bank[6], bank[7],
                   bank[8], bank[9], bank[10], bank[11],
                   bank[12], bank[13]);

        /* Sample the IOMMU instance that owns this codec to find out
         * whether the hardware silently faulted on an iova — if it did,
         * INT_RAWSTAT bit 0 latches and PAGE_FAULT_ADDR carries the iova
         * the codec asked for.  Helps distinguish "codec didn't kick"
         * from "codec kicked but AXI traffic was page-faulted away". */
        PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(q->Device);
        if (iommu && iommu->Snapshot) {
            RKIOMMU_FAULT_SNAPSHOT snap = {0};
            if (NT_SUCCESS(iommu->Snapshot(iommu->Header.Context, &snap))) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                           "  iommu#0 STATUS=0x%08x INT_RAWSTAT=0x%08x "
                           "INT_STATUS=0x%08x FAULT_ADDR=0x%08x DTE=0x%08x\n",
                           snap.Status, snap.IntRawStat, snap.IntStatus,
                           snap.PageFaultAddr, snap.DteAddr);
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                           "  iommu#1 STATUS=0x%08x INT_RAWSTAT=0x%08x "
                           "INT_STATUS=0x%08x FAULT_ADDR=0x%08x DTE=0x%08x\n",
                           snap.Status1, snap.IntRawStat1, snap.IntStatus1,
                           snap.PageFaultAddr1, snap.DteAddr1);
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

    /* Read and acknowledge the interrupt-status register.
     * TODO (Phase 3): mask/clear the specific status bits correctly. */
    UINT32 hwStatus = 0;
    if (mmioBase) {
        hwStatus = READ_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + RKVDEC2_INT_STATUS_OFFSET));
        /* Ack by writing back the status bits (write-1-to-clear on rkvdec2). */
        WRITE_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + RKVDEC2_INT_STATUS_OFFSET),
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
}

/* -----------------------------------------------------------------------
 * RkMppJobStart — kick the job.
 *
 * Phase 2 (software-completion stub): capture start timestamp and schedule
 * the software DPC for immediate completion.  Phase 3 replaces this with
 * writing the register list to MMIO and asserting the hardware kick bit.
 * --------------------------------------------------------------------- */

/* dec_e=1 lives at byte offset 0x28 (idx 10).  A job that includes this
 * write is a real decode kick; jobs without it (smoke tests, register-
 * substitution probes) shouldn't engage the poller. */
#define RKVDEC2_REG_DEC_E_OFFSET   0x28u
#define RKVDEC2_REG_DEC_E_BIT      0x1u

/* RKVDEC2_SWREG_BASE is defined at file scope above; it accounts for
 * the 0x100 cluster-link prefix at the head of our ACPI MMIO mapping. */

static VOID
RkMppJobStart(_In_ WDFDEVICE Device, _In_ RKMPP_JOB *Job)
{
    PRKMPP_JOB_QUEUE q       = RkMppGetJobQueue(Device);
    PVOID            mmio    = RkMppGetMmioBase(Device);
    ULONG            mmioLen = RkMppGetMmioLength(Device);

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
        PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
        if (ccu && ccu->AssertCoreReset && ccu->DeassertCoreReset) {
            ccu->AssertCoreReset(ccu->Header.Context);
            ccu->DeassertCoreReset(ccu->Header.Context);
        }
    }

    /* Clear any stale latched status bits before the kick. */
    {
        ULONG sta = READ_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_INT_STATUS_OFFSET));
        if (sta) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_INT_STATUS_OFFSET),
                sta & ~1u);
        }
    }

    /* Pre-kick snapshot of perf-counter regs (idx 228..234) — useful to
     * tell whether 0x003c0130 in the post-kick dump is actual decode
     * progress or just a constant the codec returns regardless. */
    {
        ULONG pre[10] = {0};
        for (int i = 0; i < 10; i++) {
            pre[i] = READ_REGISTER_ULONG(
                (volatile ULONG *)((PUCHAR)mmio + RKVDEC2_SWREG_BASE
                                                + 228 * 4 + i * 4));
        }
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp: pre-kick perf[228..237]: "
                   "%08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
                   pre[0], pre[1], pre[2], pre[3], pre[4],
                   pre[5], pre[6], pre[7], pre[8], pre[9]);
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
        ULONG mmioOff = w->Offset + RKVDEC2_SWREG_BASE;
        if (mmioOff >= mmioLen ||
            (mmioLen - mmioOff) < sizeof(ULONG)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkmpp: job %llu swreg-off 0x%x (mmio 0x%x) out of "
                       "range (len 0x%x)\n",
                       (unsigned long long)Job->Id, w->Offset, mmioOff, mmioLen);
            RkMppJobComplete(Device, STATUS_INVALID_PARAMETER, 0);
            return;
        }
        if (w->Offset == RKVDEC2_REG_DEC_E_OFFSET &&
            (w->Value & RKVDEC2_REG_DEC_E_BIT)) {
            hasKick = TRUE;
        }
    }

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
    {
        static const struct { ULONG first, last; } bank_ranges[6] = {
            {   8,  32 }, {  64, 112 }, { 128, 142 },
            { 160, 199 }, { 200, 204 }, { 256, 277 },
        };
        ULONG bank_vals[6][80] = {0};
        BOOLEAN bank_seen[6][80] = {0};

        for (UINT32 i = 0; i < Job->RegWriteCount; i++) {
            const RKMPP_REG_WRITE *w = &Job->Writes[i];
            ULONG idx = w->Offset / 4;
            if (idx == 10) continue;
            for (int b = 0; b < 6; b++) {
                if (idx >= bank_ranges[b].first && idx <= bank_ranges[b].last) {
                    ULONG pos = idx - bank_ranges[b].first;
                    bank_vals[b][pos] = w->Value;
                    bank_seen[b][pos] = TRUE;
                    break;
                }
            }
        }
        for (int b = 0; b < 6; b++) {
            ULONG count = bank_ranges[b].last - bank_ranges[b].first + 1;
            for (ULONG p = 0; p < count; p++) {
                ULONG idx = bank_ranges[b].first + p;
                /* Skip idx 10 — BSP's mpp_write_req skips reg_en in the
                 * bank loop and writes it once at the end as the kick. */
                if (idx == 10) continue;
                /* BSP writes every slot in the bank, zeros included. */
                TRACED_WRITE_ULONG(
                    ((PUCHAR)mmio + idx * 4 + RKVDEC2_SWREG_BASE),
                    bank_vals[b][p]);
            }
        }
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

    /* Phase 3a: matching DropCluster for the per-job RaiseCluster taken in
     * RkMppJobSubmit.  Done from DPC context (here) so the refcount stays
     * balanced even if user mode never calls WaitJob. */
    PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
    if (ccu && ccu->DropCluster) {
        ccu->DropCluster(ccu->Header.Context);
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
    }
    job->BufRefCount = In->BufRefCount;
    if (In->BufRefCount > 0) {
        RtlCopyMemory(job->BufRefs, In->BufRefs,
                      In->BufRefCount * sizeof(RKMPP_BUFFER_REF));
    }

    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    /* Assign a unique job ID atomically. */
    job->Id = (UINT64)InterlockedIncrement64(&q->NextId);

    /* Phase 3a: raise the cluster refcount before queuing.  Dropped in
     * RkMppJobComplete (DPC context) regardless of whether WaitJob is
     * ever called, so refcount can't leak.  RaiseCluster is itself
     * refcounted in rkmpp_ccu, so this composes safely with the
     * device-lifetime raise taken in PrepareHardware. */
    PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
    if (ccu && ccu->RaiseCluster) {
        NTSTATUS s = ccu->RaiseCluster(ccu->Header.Context);
        if (!NT_SUCCESS(s)) {
            ExFreePoolWithTag(job, 'JppM');
            return s;
        }
    }

    /* Enqueue the job under the spin lock. */
    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);

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
            /* Phase 2: this path is unreachable in practice because the
             * software-completion DPC fires synchronously.  Phase 3 will
             * issue a hardware reset here. */
            job->Result = STATUS_TIMEOUT;
            Out->Status         = STATUS_TIMEOUT;
            Out->HardwareStatus = 0;
            Out->ElapsedQpc     = 0;
            /* Free the job — caller gives up ownership on timeout. */
            ExFreePoolWithTag(job, 'JppM');
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
