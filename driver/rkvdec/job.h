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

    /* Polling-completion thread.  WdfInterruptCreate currently fails for
     * rkmpp instances (STATUS_WDF_INVALID_INTERRUPT_CONFIG), so we run a
     * per-device kernel thread that polls INT_STATUS after each kick.
     * RkMppJobStart signals KickEvent; the thread polls + completes.
     * On teardown, ExitEvent shuts the thread down. */
    KEVENT          KickEvent;      /* signalled after register list written */
    KEVENT          ExitEvent;      /* signalled to terminate poller */
    PETHREAD        PollerThread;   /* referenced; ObDereferenced on teardown */

    /* Per-bit mask of register indices [0..511] that were nonzero in
     * the most recent kick.  Used to skip MMIO writes for regs that
     * were zero last kick AND are zero this kick — the hardware retains
     * their (zero) value so we don't need to re-write them.  Cuts
     * per-kick MMIO from ~155 (rkvdec2) / ~511 (AV1) total writes down
     * to ~80 active + a few "clear" writes for regs going nonzero→zero.
     *
     * Cleared on reset paths because the reset returns codec regs to zero. */
    ULONG           PrevNonzeroMask[16];
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
NTSTATUS RkMppJobsDrainOwner(_In_ WDFDEVICE Device,
                             _In_ WDFFILEOBJECT File,
                             _In_ ULONG TimeoutMs,
                             _Out_ BOOLEAN *InFlightTimedOut);

/* ISR — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_ISR  RkMppEvtIsr;

/* DPC — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_DPC  RkMppEvtDpc;
