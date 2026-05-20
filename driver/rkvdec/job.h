/* driver/rkmpp/job.h — register-list job queue for rkmpp.sys.
 *
 * Phase 2: software-completion stub. Job submission queues a DPC that
 * immediately signals completion without touching hardware. Phase 3 replaces
 * RkMppJobStart() with a real hardware-kick path.
 */
#pragma once

#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ioctl.h"
#include "../../shared/rkmpp_peer_worker_ifc.h"
#include "../shared/rkmpp/peer_attach.h"

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
    /* Optional buffer-reference table the caller supplies alongside the
     * iova slots.  Pins the listed handles for the lifetime of the job
     * (RkMppJobBufferInUse consults BufRefs even for handles not used
     * by the dense bank — useful for the H.264 output frame which is
     * referenced as DECOUT_BASE this kick but might also be needed by
     * later kicks as a ref). */
    UINT32          BufRefCount;
    RKMPP_BUFFER_REF BufRefs[RKMPP_MAX_BUF_REFS];
    /* MDLs requiring per-kick cache maintenance, tracked by direction.
     *
     * CleanMdls — user-mode-written inputs the codec reads this kick:
     *   bitstream (RLC), packed PPS, packed RPS, scaling list.
     *   `dc cvac` issued at the head of RkMppJobStart so codec sees
     *   the freshly written CPU data on its first DMA read.
     *
     * OutputFrameMdl — the buffer the codec writes this kick that the
     *   CPU will read next.  `dc ivac` issued in RkMppJobComplete so
     *   user-mode reads from the cached mapping see codec's fresh
     *   data, not stale CPU cache lines.
     *
     * ColmvCurMdl — the current picture's colmv buffer.  User-mode zeros
     *   it before the kick, so it needs a pre-kick clean; the codec then
     *   writes colmv data, so diagnostics that dump it need a post-kick
     *   invalidate just like output frames.
     *
     * Buffers NOT tracked here (no per-kick maintenance needed):
     *   - Reference frames: codec reads them, CPU doesn't dirty them,
     *     and they were already invalidated when they were the output
     *     of their original kick.
     *   - Per-ref colmv: codec read-only this kick.
     *   - RCB scratch / error_ref: codec-internal use only.
     *   - CABAC init table: filled once at engine init; CPU doesn't
     *     dirty it after, so no per-kick clean needed.
     *
     * Pre-narrowing this saved ~1 ms / frame at 1440p (~30 buffers
     * walked → ~4) and made it possible to keep up at 4K. */
    UINT32          CleanMdlCount;
    PMDL            CleanMdls[8];
    PMDL            OutputFrameMdl;
    PMDL            ColmvCurMdl;

    /* Dense register-bank payload — the only submission shape rkvdec
     * accepts (rkav1d has its own driver binary using the sparse
     * RKMPP_REG_WRITE[] format).  See mft/regbuilder_dense.h plus
     * shared/rkmpp_ioctl.h for the bank layout and substitution slot
     * convention. */
    UINT32                DenseKickValue;
    UINT32                DenseIovaSlotCount;
    RKMPP_DENSE_BANK      DenseBank;
    RKMPP_DENSE_IOVA_SLOT DenseIovaSlots[RKMPP_MAX_DENSE_IOVA_SLOTS];

    /* Stamped by RkMppJobStart from JobQueue.LastOwner/LastDecMode so
     * the poller's timeout dump can report whether THIS kick crossed
     * an owner / codec-mode boundary.  All zero / FALSE on the first
     * kick of a PnP cycle (KickPrevValid=FALSE). */
    UINT32          KickDecMode;
    UINT32          KickPrevDecMode;
    WDFFILEOBJECT   KickPrevOwner;
    BOOLEAN         KickSwitchOwner;
    BOOLEAN         KickSwitchMode;
    BOOLEAN         KickPrevValid;

    /* Phase 3 (Task 3.3): which core executes this job.  Set by
     * PromoteUntilFull at promote time; read by KickPromotions to
     * decide local-kick vs peer-dispatch, and by RkMppJobComplete /
     * RkMppPeerCompletion to identify which CorePending counter to
     * decrement.  0 = local (RVD0), 1 = peer (RVD1).  Phase 1
     * default value 0 keeps single-core semantics. */
    UINT32          TargetCore;

    /* OrphanOnComplete — set TRUE under queue lock by a caller that has
     * given up waiting (drainer timeout, WAIT_JOB timeout, RunForeign
     * timeout).  When the natural completion path (ISR/DPC, peer
     * completion) eventually fires, RkMppJobComplete /
     * RkMppPeerCompletion observe the flag, skip the Completed-list
     * insert (no one will dequeue it), free the job, and return.
     * Without this, abandoned jobs accumulate on Completed forever —
     * a non-paged-pool DoS once IOCTL is reachable from non-admin. */
    BOOLEAN         OrphanOnComplete;
} RKMPP_JOB, *PRKMPP_JOB;

/* -----------------------------------------------------------------------
 * Job queue — one per RKMPP_DEVICE.
 * --------------------------------------------------------------------- */

typedef struct _RKMPP_JOB_QUEUE {
    KSPIN_LOCK      Lock;
    LIST_ENTRY      Pending;        /* submitted, not yet started */
    LIST_ENTRY      Completed;      /* finished, not yet WaitJob'd */
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

    /* Cross-kick scheduling probe — updated in RkMppJobStart so the
     * next kick can log whether the codec just switched Owner (File)
     * or dec_mode (H.264=1 / H.265=0 / VP9=2 / etc.).  Used to
     * investigate cross-stream timeouts where the codec self-times-
     * out across mode boundaries.  Read+written under Queue->Lock or
     * from the poller's single-threaded JobStart caller — no
     * additional sync needed. */
    WDFFILEOBJECT   LastOwner;
    UINT32          LastDecMode;    /* low 5 bits of swreg9 */
    BOOLEAN         LastValid;      /* FALSE on first kick after PnP */

    /* Per-Owner last-kick table for round-robin promotion in
     * RkMppJobComplete.  At promotion the next Pending job is the one
     * whose Owner has the oldest LastKickId (NULL = never kicked, treat
     * as oldest).  Capacity 4 covers the realistic concurrent-session
     * count (per-File pending cap is 4; >4 decode sessions on one core
     * is already over-subscribed).  Evict LRU on overflow.  Read+written
     * only under Queue->Lock. */
    struct {
        WDFFILEOBJECT File;
        UINT64        LastKickId;
    } OwnerLru[4];

    /* Multi-core dispatch state (RVD0 only — RVD1 leaves these zero).
     * Filled by peer-watch callbacks; read by the promotion loop in
     * RkMppJobComplete.  All fields read+written under Lock.
     *
     * CoreCount: 1 (single-core / pre-attach) or 2 (RVD1 attached).
     * CoreIdle bit i = core i has no in-flight job.  Init = (1<<CoreCount)-1.
     * CorePending: BSP task_index analog used as a tiebreak when both
     *   cores are idle (lowest count wins).  Phase 3 reads these.
     * InFlightPerCore[]: replaces the old single InFlight field.
     *   Index 0 = local (RVD0 itself); index 1 = peer (RVD1).
     *   In Phase 1 only [0] is ever written (single-core path); Phase 3
     *   adds [1] writes.  RVD1's queue (UID==1) also uses [0] for its
     *   own local jobs handled via foreign-kick infrastructure.
     */
    ULONG                       CoreCount;
    ULONG                       CoreIdle;
    ULONG                       CorePending[2];
    RKMPP_JOB                  *InFlightPerCore[2];
    /* Tiebreak hint for PickTargetCore: index of the core last
     * dispatched to (0 or 1), or -1 before any dispatch.  When both
     * cores tie on CorePending, prefer the one that ISN'T this — so a
     * serial submitter alternates instead of pinning to core 0. */
    LONG                        LastKickCore;

    /* Peer worker client state.  PeerOpen=TRUE once we've successfully
     * QueryInterface'd RVD1's provider; FALSE during single-core mode
     * (peer driver absent or being unloaded).  Lifecycle: opened in the
     * peer-arrival callback, closed in the peer query-remove callback
     * (Phase 4) or in RVD0's ReleaseHardware. */
    BOOLEAN                     PeerOpen;
    RKMPP_PEER_WORKER_INTERFACE Peer;
    PFILE_OBJECT                PeerFileObj;
    RKMPP_PEER_WATCH            PeerWatch;

    /* Polling-completion thread.  WdfInterruptCreate currently fails for
     * rkmpp instances (STATUS_WDF_INVALID_INTERRUPT_CONFIG), so we run a
     * per-device kernel thread that polls INT_STATUS after each kick.
     * RkMppJobStart signals KickEvent; the thread polls + completes.
     * On teardown, ExitEvent shuts the thread down. */
    KEVENT          KickEvent;      /* signalled after register list written */
    KEVENT          ExitEvent;      /* signalled to terminate poller */
    PETHREAD        PollerThread;   /* referenced; ObDereferenced on teardown */
} RKMPP_JOB_QUEUE, *PRKMPP_JOB_QUEUE;

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/* Initialise the queue; call once during device creation. */
VOID RkMppJobQueueInit(_In_ WDFDEVICE Device, _Inout_ RKMPP_JOB_QUEUE *Queue);

/* Tear down the queue (stop poller thread, wait for it).  Call from
 * EvtReleaseHardware / EvtDriverContextCleanup. */
VOID RkMppJobQueueTeardown(_Inout_ RKMPP_JOB_QUEUE *Queue);

/* IOCTL_RKMPP_WAIT_JOB handler. */
NTSTATUS RkMppJobWait(_In_ WDFDEVICE Device,
                      _In_ WDFFILEOBJECT File,
                      _In_ UINT64 JobId,
                      _In_ UINT32 TimeoutMs,
                      _Out_ RKMPP_WAIT_JOB_OUT *Out);

/* Phase 3 (Task 3.2): runs a single dense kick from a pre-resolved
 * (bank, iova-slots) tuple supplied by RVD1's PEER_WORKER provider —
 * bypassing per-File handle resolution and per-Owner LRU updates.
 * Foreign jobs have Owner == NULL.  Caller (peer_worker's
 * PeerKickWorker) runs at PASSIVE on a system worker thread; this
 * function blocks until Job->Done is signalled by the ISR-DPC chain,
 * then invokes Cb(CbCtx, CompletionCookie, Result, HardwareStatus)
 * and returns.  Cb runs on the worker thread before this returns —
 * synchronous from the worker's perspective. */
typedef VOID (*RKMPP_FOREIGN_COMPLETION_CB)(
    _In_ PVOID    CbCtx,
    _In_ UINT64   CompletionCookie,
    _In_ NTSTATUS Result,
    _In_ UINT32   HardwareStatus);

NTSTATUS RkMppJobRunForeign(_In_ WDFDEVICE Device,
                            _In_ const VOID  *Bank,         /* RKMPP_DENSE_BANK */
                            _In_ UINT32       BankBytes,
                            _In_ UINT32       KickValue,
                            _In_ const VOID  *IovaSlots,    /* RKMPP_DENSE_IOVA_SLOT[] */
                            _In_ UINT32       IovaSlotCount,
                            _In_ UINT64       CompletionCookie,
                            _In_ RKMPP_FOREIGN_COMPLETION_CB Cb,
                            _In_ PVOID        CbCtx);

/* IOCTL_RKMPP_SUBMIT_DENSE_JOB handler.  The caller supplies a fully
 * zero-init'd RKMPP_DENSE_BANK plus a list of address-bank slots whose
 * buffer handles the kernel resolves into iovas (stamped into the bank
 * before the bulk MMIO write at kick time).  File is required for
 * handle lookup. */
NTSTATUS RkMppJobSubmitDense(_In_ WDFDEVICE Device,
                             _In_ WDFFILEOBJECT File,
                             _In_ const RKMPP_SUBMIT_DENSE_JOB_IN *In,
                             _Out_ RKMPP_SUBMIT_DENSE_JOB_OUT *Out);

/* IOCTL_RKMPP_PEEK_DENSE_JOB handler — return the post-substitution
 * dense bank for a queued or completed dense job.  Returns
 * STATUS_NOT_FOUND if the JobId doesn't match a dense-flavoured job
 * owned by `File`. */
NTSTATUS RkMppJobPeekDense(_In_ WDFDEVICE Device,
                           _In_ WDFFILEOBJECT File,
                           _In_ UINT64 JobId,
                           _Out_ RKMPP_PEEK_DENSE_JOB_OUT *Out);

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
NTSTATUS RkMppJobsDrainOwner(_In_ WDFDEVICE Device,
                             _In_ WDFFILEOBJECT File,
                             _In_ ULONG TimeoutMs,
                             _Out_ BOOLEAN *InFlightTimedOut);

/* TRUE when the queue has an in-flight or pending job owned by something
 * other than `File`.  Use in EvtFileCleanup AFTER DrainOwner to decide
 * whether the session-end PD power-cycle is safe (no other concurrent
 * decode session on this engine) or whether it would yank hardware out
 * from under another File. */
BOOLEAN RkMppJobQueueHasOtherOwner(_In_ WDFDEVICE Device,
                                   _In_ WDFFILEOBJECT File);

/* Phase 3 (Task 3.4): peer completion entry point.  RVD1's
 * PEER_WORKER provider invokes this via the consumer callback
 * registered at peer attach.  Identifies the completing job by
 * CompletionCookie (matches job->Id stamped in KickPromotions). */
VOID RkMppPeerCompletion(_In_ PVOID    ConsumerContext,
                         _In_ UINT64   CompletionCookie,
                         _In_ NTSTATUS JobStatus,
                         _In_ UINT32   HardwareStatus);

/* ISR — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_ISR  RkMppEvtIsr;

/* DPC — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_DPC  RkMppEvtDpc;
