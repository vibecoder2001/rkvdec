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
    /* Snapshot of the submission inputs.  Phase 3 will translate BufRefs into
     * iova substitutions on the register list before kicking hardware. */
    UINT32          RegWriteCount;
    RKMPP_REG_WRITE Writes[RKMPP_MAX_REG_WRITES];
    UINT32          BufRefCount;
    RKMPP_BUFFER_REF BufRefs[RKMPP_MAX_BUF_REFS];
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
                      _In_ UINT64 JobId,
                      _In_ UINT32 TimeoutMs,
                      _Out_ RKMPP_WAIT_JOB_OUT *Out);

/* IOCTL_RKMPP_PEEK_JOB handler — return the post-substitution register
 * list for a queued or completed job.  Used by tests to verify iova
 * substitution before the real hardware-kick path is in. */
NTSTATUS RkMppJobPeek(_In_ WDFDEVICE Device,
                      _In_ UINT64 JobId,
                      _Out_ RKMPP_PEEK_JOB_OUT *Out);

/* ISR — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_ISR  RkMppEvtIsr;

/* DPC — declared here so device.c can pass it to WdfInterruptCreate. */
EVT_WDF_INTERRUPT_DPC  RkMppEvtDpc;
