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
#include "../shared/rkmpp_log.h"
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

/* AV1 address-class swreg whitelist.  Mirrors the security-load-bearing
 * decision in upstream `kAv1DmaLsbIdx` (mft/engine/decode_engine_av1.cpp):
 * BSP's `trans_tbl_av1_vcd[]` defines which swreg slots the kernel
 * iommu-substitutor recognises as address-bearing.  We require BufferHandle
 * != 0 (i.e. iova substitution) for any RKMPP_REG_WRITE that lands on one
 * of these *_lsb indices OR its companion *_msb (idx - 1).  Conversely a
 * BufferHandle != 0 write targeting a non-address swreg is rejected — that
 * would let user mode point the codec at an arbitrary IOVA target.
 *
 * The set below was hand-mirrored from kAv1DmaLsbIdx in MFT and re-checked
 * against regbuilder_av1_reg.h's `*_base_lsb` field names; every entry has
 * a matching `sw_*_base_lsb` (or PP/lanczos lsb) declaration.  Adding new
 * address-class regs MUST be done in BOTH places (kernel whitelist here
 * + user-mode emit list in decode_engine_av1.cpp). */
static const UINT32 RKAV1D_DMA_LSB_IDX[] = {
    65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
    97, 99, 101, 103, 105, 107, 109, 111, 113, 133, 135, 137, 139,
    141, 143, 145, 147, 167, 169, 171, 173, 175, 177, 179, 183, 190,
    192, 194, 196, 198, 200, 202, 204, 224, 226, 228, 230, 232, 234,
    236, 238, 326, 328, 339, 341, 348, 350, 505, 507,
};

static BOOLEAN
RkAv1dIsAddressReg(_In_ UINT32 SwregIdx)
{
    for (size_t i = 0; i < RTL_NUMBER_OF(RKAV1D_DMA_LSB_IDX); i++) {
        if (SwregIdx == RKAV1D_DMA_LSB_IDX[i]) return TRUE;
        /* The msb companion sits at idx-1 (kernel ORs the high byte
         * during substitution).  User mode currently writes 0 there
         * with BufferHandle==0; keep that allowed as a plain write
         * but also accept BufferHandle!=0 if a future regbuilder
         * decides to substitute both halves. */
        if (SwregIdx + 1 == RKAV1D_DMA_LSB_IDX[i]) return TRUE;
    }
    return FALSE;
}

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
    Queue->Draining = 0;
    /* Vestigial PollerThread/KickEvent/ExitEvent removed in the
     * I14 cleanup — completion is interrupt-driven. */
}

VOID
RkMppJobQueueTeardown(_Inout_ RKMPP_JOB_QUEUE *Queue)
{
    UNREFERENCED_PARAMETER(Queue);
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
            RKMPP_LOG_WARN(
                       "rkav1d: file-cleanup in-flight wait timed out\n");
            /* Sever MDL refs AND mark orphan so the eventual natural
             * completion (poller-driven JobComplete) frees the job
             * memory instead of inserting it into Completed where it
             * would leak indefinitely.  Without OrphanOnComplete every
             * abandoned drain leaked one RKMPP_JOB — a DoS vector once
             * IOCTL is non-admin.  See [[critical_drainer_leak]]. */
            KeAcquireSpinLock(&q->Lock, &old);
            if (q->InFlight == inFlight && inFlight->Owner == File) {
                inFlight->OutputFrameMdl    = NULL;
                inFlight->InternalOutputMdl = NULL;
                inFlight->AuxOutputMdl      = NULL;
                inFlight->OrphanOnComplete  = TRUE;
            }
            KeReleaseSpinLock(&q->Lock, old);
            RkMppSetNeedsCoreReset(Device);
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
 * least one job whose Owner is NOT `File`.  Used by EvtFileCleanup to
 * decide whether session-close hygiene (IOMMU Reattach, etc.) is safe
 * to run inline (no peer) or must be skipped (peer is decoding right
 * now and would be disrupted).
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
        RKMPP_LOG_WARN(
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

    /* Read and acknowledge the interrupt-status register.
     *
     * Match BSP av1dec_vcd_irq (mpp_av1dec.c:634): write 0 to the
     * status/clear register, not the read-back value.  The AV1 VCD
     * status register at +0x004 is NOT write-1-to-clear — writing
     * the read value back leaves the bits set, the IRQ line stays
     * asserted (level-triggered ActiveHigh), and the ISR fires again
     * immediately → endless loop → machine hang.  Latent until
     * commit 3fdae48 made WdfInterruptCreate actually succeed; before
     * that the ISR never ran so the wrong ack was invisible. */
    UINT32 hwStatus = 0;
    if (mmioBase) {
        hwStatus = READ_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + ops->IntStatusOffset));
        WRITE_REGISTER_ULONG(
            (volatile ULONG *)((PUCHAR)mmioBase + ops->IntStatusOffset),
            0);
    }

    if (hwStatus == 0) {
        /* Not our interrupt (spurious). */
        return FALSE;
    }

    /* Hand hwStatus off to the DPC via the queue's LastIsrHwStatus
     * field.  Must be written BEFORE WdfInterruptQueueDpcForIsr — the
     * queue operation is the memory barrier that publishes this write
     * to the DPC running on (potentially) a different processor. */
    {
        PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(device);
        if (q) q->LastIsrHwStatus = hwStatus;
    }
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

    /* Read the ISR's INT_STATUS snapshot (volatile in the queue struct
     * — published via the implicit memory barrier in
     * WdfInterruptQueueDpcForIsr).  Classify error vs success the same
     * way the old poller did, then hand both result + hwStatus to
     * RkMppJobComplete, whose existing err_mask logic routes
     * AV1D_INT_TIMEOUT/BUS_ERROR/BUF_EMPTY/ASO/STRM/SLICE into a reset
     * request and the wait-for-keyframe firebreak.  Without this, the
     * codec could wedge with TIMEOUT and the DPC would silently
     * complete as STATUS_SUCCESS, hwStatus=0 — user-mode would treat
     * the garbage output as a valid frame and no reset would be
     * requested for the next kick. */
    UINT32   hwStatus = q->LastIsrHwStatus;
    NTSTATUS result   = (hwStatus & g_ops.IntErrorMask)
                      ? STATUS_DEVICE_HARDWARE_ERROR
                      : STATUS_SUCCESS;

    KIRQL oldIrql;
    KeAcquireSpinLock(&q->Lock, &oldIrql);
    RKMPP_JOB *job = q->InFlight;
    KeReleaseSpinLock(&q->Lock, oldIrql);

    if (job) {
        if (hwStatus & g_ops.IntErrorMask) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkav1d: DPC INT=0x%08x err_mask=0x%08x "
                       "(BUS=%d BUF_EMPTY=%d ASO=%d STRM=%d SLICE=%d TIMEOUT=%d) "
                       "job=%llu\n",
                       hwStatus, hwStatus & g_ops.IntErrorMask,
                       (hwStatus & AV1D_INT_BUS_ERROR)  ? 1 : 0,
                       (hwStatus & AV1D_INT_BUF_EMPTY)  ? 1 : 0,
                       (hwStatus & AV1D_INT_ASO_ERROR)  ? 1 : 0,
                       (hwStatus & AV1D_INT_STRM_ERROR) ? 1 : 0,
                       (hwStatus & AV1D_INT_SLICE)      ? 1 : 0,
                       (hwStatus & AV1D_INT_TIMEOUT)    ? 1 : 0,
                       (unsigned long long)job->Id);
        }
        RkMppJobComplete(device, result, hwStatus);
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
            /* AV1 L2 write cache intentionally left disabled.
             *
             * 2026-05-xx: enabling the cache (CACHE_W(0x204, 0x81)
             * — reorder_e | cache_e) caused alternating-frame
             * corruption.  PP writes bypassing the cache and going
             * straight to DRAM eliminated the artifact.  AV1
             * HW-decode is otherwise validated (see memory:
             * [[hevc_av1_10bit_wired]] / [[av1_bringup_table]]).
             *
             * Deferred: re-enable behind a correct flush sequence
             * (likely needs an inter-kick invalidate of the L2 write
             * cache once the precise trigger is isolated).  We still
             * configure the PP0_Y/PP0_U channel registers below so
             * the geometry is correct when the cache is re-enabled. */
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
            /* L2 write cache enable intentionally deferred — see the
             * "AV1 L2 write cache intentionally left disabled" note
             * above for the alternating-frame-corruption history.
             * CACHE_W(0x204, 0x00000081); */
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
            /* Skip MMIO only when (a) user-mode did NOT explicitly emit
             * this reg in Writes[] AND (b) it was zero last kick.  The
             * av1_seen[] check is what keeps the regbuilder's
             * kAv1ForceWriteIdx force-zeros (swreg 260, 266) actually
             * landing on the MMIO every kick — without it the
             * skip-zero-when-zero optimisation silently drops the very
             * write the user-mode list exists to guarantee, and the
             * right-edge corruption on 10-bit AV1 that 590f3c6 was
             * supposed to fix never actually got the clear it needed. */
            if (!av1_seen[idx] && !was_nz && !is_nz) continue;
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

    if (!hasKick) {
        /* Submissions lacking the kick bit are rejected at the IOCTL
         * boundary (commit 1), so this branch is unreachable from
         * user-mode.  Kept as a defensive trip for any future internal
         * caller — completes synchronously without going through the
         * ISR/DPC chain. */
        RkMppJobComplete(Device, STATUS_SUCCESS, 0);
    }
    /* Real kicks complete via the ISR → DPC chain. */
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
    if (!NT_SUCCESS(Result) || (HardwareStatus & g_ops.IntErrorMask)) {
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

    /* Orphan check — caller has abandoned this job (drainer/wait
     * timeout).  Skip Completed-list insert and arrange to free after
     * the lock release; without this, abandoned jobs leak forever. */
    BOOLEAN orphan = job->OrphanOnComplete;

    /* Move from InFlight to Completed (unless orphan). */
    q->InFlight = NULL;
    if (!orphan) {
        InsertTailList(&q->Completed, &job->Link);
    }

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

    /* Free orphan job after kicking the next one. */
    if (orphan) {
        ExFreePoolWithTag(job, 'JppM');
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

    /* Reject submissions lacking the codec's kick bit (test/peek-only
     * pattern).  Such jobs synchronously complete inside JobKickInner
     * → JobComplete → next-promote → JobKickInner — unbounded if the
     * caller queues many.  Kernel stack overflow vector.  Legitimate
     * AV1 decode always has the kick bit set on KickRegOffset; "peek
     * post-mortem reg state" is what PEEK is for.
     * See [[critical_kick_recursion]]. */
    {
        BOOLEAN hasKick = FALSE;
        for (UINT32 i = 0; i < In->RegWriteCount; i++) {
            const RKMPP_REG_WRITE *w = &In->Writes[i];
            if (w->Offset == AV1D_REG_KICK_OFFSET &&
                (w->Value & AV1D_REG_KICK_BIT)) {
                hasKick = TRUE;
                break;
            }
        }
        if (!hasKick) return STATUS_INVALID_PARAMETER;
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

        /* Reject byte-misaligned offsets early — AV1 swregs are 4-byte
         * aligned by construction; any unaligned offset would either land
         * in the gap between regs (no-op write inside the codec's MMIO
         * window) or split across two regs (impossible at 32-bit MMIO
         * granularity).  Both are security-relevant — a misaligned
         * offset bypasses RkAv1dIsAddressReg below. */
        if ((src->Offset & 3u) != 0 || src->Offset >= 512u * 4u) {
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_PARAMETER;
        }
        const UINT32 swregIdx = src->Offset / 4u;
        const BOOLEAN isAddr = RkAv1dIsAddressReg(swregIdx);

        if (src->BufferHandle == 0) {
            /* Plain write.  Allowed for every swreg — including the
             * msb companions of address regs, which BSP also writes
             * as 0 plain (kernel ORs the high byte). */
            dst->Value = src->Value;
            continue;
        }

        /* BufferHandle != 0 ⇒ iova substitution.  This MUST target an
         * address-class swreg; otherwise user mode could write an
         * arbitrary iova into a non-address register and divert the
         * codec to attacker-controlled DMA targets.  Review finding #1. */
        if (!isAddr) {
            ExFreePoolWithTag(job, 'JppM');
            return STATUS_INVALID_PARAMETER;
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
    /* Two-tier admission control:
     *   - Per-device total cap (8): the existing hard backpressure
     *     ceiling for the whole engine.
     *   - Per-File cap (4): prevents one open handle from filling the
     *     queue and starving a concurrent decode session on this same
     *     engine.  Counts Pending + InFlight owned by this File so a
     *     single greedy File can't have 4 pending + 1 in flight while
     *     a peer File starves. */
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
             *              job while the first is still active. */
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
                RKMPP_LOG_WARN(
                           "rkav1d: WAIT_JOB timeout on InFlight job %llu — "
                           "leaving attached for poller completion\n",
                           (unsigned long long)job->Id);
                /* Mark orphan so the poller-driven completion frees the
                 * job instead of moving it to Completed (caller has
                 * given up the JobId). */
                job->OrphanOnComplete = TRUE;
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
