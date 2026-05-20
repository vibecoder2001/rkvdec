/* driver/rkav1d/job.h — register-list job queue for rkav1d.sys.
 *
 * Phase 2: software-completion stub. Job submission queues a DPC that
 * immediately signals completion without touching hardware. Phase 3 replaces
 * RkMppJobStart() with a real hardware-kick path.
 */
#pragma once

#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ioctl.h"

/* -----------------------------------------------------------------------
 * Job descriptor — heap-allocated per submitted job.
 * --------------------------------------------------------------------- */

typedef struct _RKMPP_JOB {
    LIST_ENTRY      Link;           /* member of Pending or Completed list */
    UINT64          Id;             /* monotonically increasing job ID */
    KEVENT          Done;           /* signalled by RkMppJobComplete */
    NTSTATUS        Result;         /* STATUS_SUCCESS, STATUS_DEVICE_HUNG, ... */
    UINT32          HardwareStatus; /* raw INT_STATUS value (0 in Phase 2) */
    LARGE_INTEGER   StartQpc;
    LARGE_INTEGER   EndQpc;
    /* Owning file-object (the open handle that submitted this job).
     * EvtFileCleanup uses this to drain a process's outstanding jobs
     * before freeing its buffer pool — keeps the codec from DMA'ing
     * to iovas whose backing memory we just freed. */
    WDFFILEOBJECT   Owner;
    /* Snapshot of the submission inputs.  Phase 3 will translate BufRefs into
     * iova substitutions on the register list before kicking hardware. */
    UINT32          RegWriteCount;
    RKMPP_REG_WRITE Writes[RKMPP_MAX_REG_WRITES];
    UINT32          BufRefCount;
    RKMPP_BUFFER_REF BufRefs[RKMPP_MAX_BUF_REFS];
    /* MDLs requiring per-kick cache maintenance, tracked by direction.
     *
     * CleanMdls — user-mode-written inputs the codec reads this kick:
     *   bitstream (RLC), tile_info, prob_tbl, global_model.
     *   `dc cvac` issued at the head of RkMppJobStart so codec sees
     *   the freshly written CPU data on its first DMA read.
     *
     * OutputFrameMdl — the buffer the codec writes this kick that the
     *   CPU will read next.  `dc ivac` issued in RkMppJobComplete so
     *   user-mode reads from the cached mapping see codec's fresh
     *   data, not stale CPU cache lines.
     *
     * Buffers NOT tracked here (no per-kick maintenance needed):
     *   - Reference frames: codec reads them, CPU doesn't dirty them,
     *     and they were already invalidated when they were the output
     *     of their original kick.
     *   - Per-ref colmv: codec read-only this kick.
     *   - colmv_cur: codec writes it but CPU doesn't read it (only the
     *     codec consumes colmv as ref input on subsequent kicks).
     *   - RCB scratch / error_ref: codec-internal use only.
     *   - CABAC init table: filled once at engine init; CPU doesn't
     *     dirty it after, so no per-kick clean needed.
     *
     * Pre-narrowing this saved ~1 ms / frame at 1440p (~30 buffers
     * walked → ~4) and made it possible to keep up at 4K. */
    UINT32          CleanMdlCount;
    PMDL            CleanMdls[8];
    PMDL            OutputFrameMdl;
    /* AV1 only: pool_internal — codec-internal Y/UV/MV in codec-tiled
     * format (reg65 base).  Codec's VCD writes here; PP reads it and
     * emits user-visible NV12 to OutputFrameMdl.  Tracked here so the
     * post-decode KeFlushIoBuffers invalidates stale CPU lines for the
     * user-mode dump path (RKMPP_AV1_DUMP_DIR) which reads it for
     * byte-diffing against BSP captures. */
    PMDL            InternalOutputMdl;
    /* AV1 only: prob_tbl_out (reg171).  Codec writes post-decode CDF
     * state here each kick; user-mode reads it to snapshot saved_cdf[]
     * for the next inter-frame's CDF seed.  Needs dc ivac before user-
     * mode read, same as OutputFrameMdl. */
    PMDL            AuxOutputMdl;

    /* OrphanOnComplete — set TRUE under queue lock by a caller that has
     * given up waiting (drainer timeout, WAIT_JOB timeout).  When the
     * natural completion path eventually fires, RkMppJobComplete sees
     * the flag, skips the Completed-list insert, frees the job, and
     * returns.  Prevents per-timeout RKMPP_JOB leak on abandoned waits. */
    BOOLEAN         OrphanOnComplete;
} RKMPP_JOB, *PRKMPP_JOB;

/* -----------------------------------------------------------------------
 * Job queue — one per RKMPP_DEVICE.
 * --------------------------------------------------------------------- */

typedef struct _RKMPP_JOB_QUEUE {
    KSPIN_LOCK      Lock;
    LIST_ENTRY      Pending;        /* submitted, not yet started */
    LIST_ENTRY      Completed;      /* finished, not yet WaitJob'd */
    RKMPP_JOB      *InFlight;       /* currently executing (at most one) */
    volatile LONG64 NextId;         /* next job ID to assign */
    WDFDEVICE       Device;         /* back-pointer for poller context */
    WDFINTERRUPT    Interrupt;      /* WDFINTERRUPT for ISR/DPC */

    /* hwStatus handoff from RkMppEvtIsr → RkMppEvtDpc.
     *
     * The ISR reads INT_STATUS (acknowledging by writing 0), then
     * writes the value here BEFORE WdfInterruptQueueDpcForIsr.  The DPC
     * reads it on entry and passes it to RkMppJobComplete so the
     * err_mask classifier (TIMEOUT_STA → FullCoreReset, other err bits
     * → CoreReset, firebreak flags, etc.) sees the real codec status
     * rather than the placeholder 0 the DPC used during initial
     * bring-up.  Volatile because ISR/DPC run on different processors;
     * WdfInterruptQueueDpcForIsr provides the implicit memory barrier
     * between the write and the DPC's read. */
    volatile UINT32 LastIsrHwStatus;

    /* Completion is interrupt-driven (ISR → DPC → RkMppJobComplete).
     * Vestigial PollerThread/KickEvent/ExitEvent removed in I14. */

    /* Per-bit mask of register indices [0..511] that were nonzero in
     * the most recent kick.  Used to skip MMIO writes for regs that
     * were zero last kick AND are zero this kick — the hardware retains
     * their (zero) value so we don't need to re-write them.  Cuts
     * per-kick MMIO from ~511 (AV1) total writes down to ~80 active +
     * a few "clear" writes for regs going nonzero→zero.
     *
     * Cleared on reset paths because the reset returns codec regs to zero. */
    ULONG           PrevNonzeroMask[16];

    /* Set non-zero while EvtDeviceD0Exit is quiescing the poller; while
     * set, RkMppJobComplete does NOT pull the next job from Pending into
     * InFlight, so no new MMIO kick fires.  Cleared by
     * RkMppJobQueueResume on D0Entry.
     *
     * Locking: written via InterlockedExchange (outside Queue->Lock);
     * read under Queue->Lock by RkMppJobComplete and RkMppJobSubmit. */
    volatile LONG   Draining;
} RKMPP_JOB_QUEUE, *PRKMPP_JOB_QUEUE;

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/* Initialise the queue; call once during device creation. */
VOID RkMppJobQueueInit(_In_ WDFDEVICE Device, _Inout_ RKMPP_JOB_QUEUE *Queue);

/* Tear down the queue (stop poller thread, wait for it).  Call from
 * EvtReleaseHardware / EvtDriverContextCleanup. */
VOID RkMppJobQueueTeardown(_Inout_ RKMPP_JOB_QUEUE *Queue);

/* IOCTL_RKMPP_SUBMIT_JOB handler.  File is required for buffer-handle
 * resolution: any RKMPP_REG_WRITE with BufferHandle != 0 is rewritten to
 * the iova of the matching buffer in this file's allocation list before
 * the job is queued. */
NTSTATUS RkMppJobSubmit(_In_ WDFDEVICE Device,
                        _In_ WDFFILEOBJECT File,
                        _In_ const RKMPP_SUBMIT_JOB_IN *In,
                        _Out_ RKMPP_SUBMIT_JOB_OUT *Out);

/* IOCTL_RKMPP_WAIT_JOB handler. */
NTSTATUS RkMppJobWait(_In_ WDFDEVICE Device,
                      _In_ WDFFILEOBJECT File,
                      _In_ UINT64 JobId,
                      _In_ UINT32 TimeoutMs,
                      _Out_ RKMPP_WAIT_JOB_OUT *Out);

/* IOCTL_RKMPP_PEEK_JOB handler — return the post-substitution register
 * list for a queued or completed job.  Used by tests to verify iova
 * substitution before the real hardware-kick path is in. */
NTSTATUS RkMppJobPeek(_In_ WDFDEVICE Device,
                      _In_ WDFFILEOBJECT File,
                      _In_ UINT64 JobId,
                      _Out_ RKMPP_PEEK_JOB_OUT *Out);

/* Return TRUE when Cookie is referenced by any pending, in-flight, or
 * completed-not-yet-waited job owned by File.  Used by explicit
 * FREE_BUFFER to avoid tearing down IOVA mappings while a job can still
 * DMA through, or while user-mode still needs to wait the completed job. */
BOOLEAN RkMppJobBufferInUse(_In_ WDFDEVICE Device,
                            _In_ WDFFILEOBJECT File,
                            _In_ UINT64 Cookie);

/* Drain all jobs owned by a closing file-object so the codec is no
 * longer DMA'ing to iovas backed by buffers we're about to free.
 *
 * Pending jobs (queued, not yet kicked): removed and freed.
 * In-flight job (kicked, awaiting INT_RDY): waited up to TimeoutMs;
 *   if it doesn't complete, returned STATUS_TIMEOUT (caller may then
 *   force-reset).  When in-flight does complete in time we let the
 *   normal RkMppJobComplete path run.
 * Completed jobs (finished, not WaitJob'd): removed and freed.
 *
 * Caller is RkMppEvtFileCleanup; thread is the closing process /
 * a system worker if the process already exited.
 *
 * Returns STATUS_SUCCESS even when in-flight timed out; the bool
 * out tells the caller whether to follow up with a hardware reset. */
/* TRUE when the queue has an in-flight or pending job owned by something
 * other than `File`.  Use in EvtFileCleanup AFTER DrainOwner to decide
 * whether session-close hygiene that touches the IOMMU / codec block
 * (Reattach, FullReset) is safe to run inline (no peer) or must be
 * skipped (peer would be disrupted mid-DMA). */
BOOLEAN RkMppJobQueueHasOtherOwner(_In_ WDFDEVICE Device,
                                   _In_ WDFFILEOBJECT File);

NTSTATUS RkMppJobsDrainOwner(_In_ WDFDEVICE Device,
                             _In_ WDFFILEOBJECT File,
                             _In_ ULONG TimeoutMs,
                             _Out_ BOOLEAN *InFlightTimedOut);

/* Quiesce the poller — block tail-chain new-kick starts, then wait
 * up to TimeoutMs for any in-flight job to complete naturally.
 * Returns STATUS_SUCCESS if no in-flight or in-flight completed in
 * time; STATUS_TIMEOUT if the in-flight wait expired (caller may
 * still proceed to D3 but should escalate to a reset on D0Entry).
 * RkMppJobQueueResume MUST be paired 1:1 with each successful
 * Quiesce call.  PASSIVE_LEVEL only. */
NTSTATUS RkMppJobQueueQuiesce(_Inout_ RKMPP_JOB_QUEUE *Queue,
                              _In_ ULONG TimeoutMs);

/* Resume the queue after Quiesce.  Clears Draining.  If Pending is
 * non-empty (e.g. submits arrived while draining wasn't blocked,
 * or a tail-chain start was suppressed), promotes head of Pending
 * to InFlight and calls RkMppJobStart to restart the chain.
 * PASSIVE_LEVEL only. */
VOID RkMppJobQueueResume(_In_ WDFDEVICE Device,
                         _Inout_ RKMPP_JOB_QUEUE *Queue);

/* ISR — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_ISR  RkMppEvtIsr;

/* DPC — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_DPC  RkMppEvtDpc;
