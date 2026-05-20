/* driver/rkmpp/bufpool.h — DMA-coherent buffer pool for rkmpp.sys.
 *
 * Each file object that the user opens gets an RKMPP_FILE_CTX tracking the
 * set of buffers it has allocated.  Buffers are freed explicitly via
 * IOCTL_RKMPP_FREE_BUFFER or implicitly by EvtFileCleanup when the handle
 * is closed.
 */
#pragma once

#include <ntddk.h>
#include <wdf.h>

#include "../../../shared/rkmpp_ioctl.h"

/* -----------------------------------------------------------------------
 * Per-file-object context
 * --------------------------------------------------------------------- */

typedef struct _RKMPP_FILE_CTX {
    LIST_ENTRY    Buffers;    /* list of RKMPP_BUFFER.Link */
    KSPIN_LOCK    Lock;       /* guards Buffers list */
    WDFDEVICE     Device;     /* back-pointer so cleanup can reach the ifcs */
    UINT64        AllocatedBytes; /* rounded bytes currently charged to file */
    UINT32        BufferCount;    /* buffers currently charged to file */
    volatile LONG ErrorCount; /* session-cumulative count of error-flagged
                                 jobs.  Read by EvtFileCleanup to decide
                                 whether to invoke the soft-tier IOMMU
                                 force-reset for session isolation; reset
                                 to zero only there (on session end).
                                 INTENTIONALLY does not decrement on a
                                 subsequent successful kick — the value is
                                 a "did this file ever produce a hardware
                                 error?" signal, not a current-fault
                                 count.  Review I9. */
} RKMPP_FILE_CTX, *PRKMPP_FILE_CTX;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKMPP_FILE_CTX, RkMppFileGet);

/* -----------------------------------------------------------------------
 * Per-buffer tracking node (heap-allocated, lives on the file-ctx list)
 * --------------------------------------------------------------------- */

typedef struct _RKMPP_BUFFER {
    LIST_ENTRY  Link;           /* member of RKMPP_FILE_CTX.Buffers */
    UINT64      Cookie;         /* unique handle vended to user mode */
    PVOID       KernelVa;       /* system VA from MmAllocateContiguousNodeMemory */
    PMDL        Mdl;            /* non-paged MDL describing the pages */
    PVOID       UserVa;         /* user-mode VA from MmMapLockedPagesSpecifyCache */
    UINT64      Iova;           /* device-visible IOVA from rkiommu MapMdl */
    ULONG       Size;           /* rounded-up byte count */
    PEPROCESS   OwnerProcess;   /* ObReferenced at alloc; needed for safe unmap */
} RKMPP_BUFFER, *PRKMPP_BUFFER;

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/* Called from device.c EvtFileCreate (or the WDF FileObject EvtCreate
 * callback) to initialise the per-file context. */
NTSTATUS RkMppBufFileCtxInit(_In_ WDFFILEOBJECT File, _In_ WDFDEVICE Device);

/* Allocate a DMA-coherent buffer and map it into the calling process. */
NTSTATUS RkMppBufAlloc(_In_ WDFDEVICE           Device,
                       _In_ WDFFILEOBJECT        File,
                       _In_ const RKMPP_ALLOC_BUFFER_IN *In,
                       _Out_ RKMPP_ALLOC_BUFFER_OUT     *Out);

/* Free a single buffer identified by Cookie. */
NTSTATUS RkMppBufFree(_In_ WDFFILEOBJECT File, _In_ UINT64 Cookie);

/* Free every buffer on the file-object's list (called from EvtFileCleanup). */
VOID RkMppBufFreeAll(_In_ WDFFILEOBJECT File);

/* Look up a buffer by cookie within the file's allocation list and return
 * its iova + size.  STATUS_NOT_FOUND if the cookie is unknown.  Used by
 * RkMppJobSubmit for register-list iova substitution. */
NTSTATUS RkMppBufLookupIova(_In_ WDFFILEOBJECT File,
                            _In_ UINT64 Cookie,
                            _Out_ UINT64 *OutIova,
                            _Out_ ULONG  *OutSize);

/* Look up a buffer's MDL by cookie.  Used by RkMppJobSubmit to capture
 * MDLs into the job for later cache-maintenance (KeFlushIoBuffers) at
 * kick + completion time when buffers are mapped cached.
 * STATUS_NOT_FOUND if the cookie is unknown. */
NTSTATUS RkMppBufLookupMdl(_In_  WDFFILEOBJECT File,
                           _In_  UINT64 Cookie,
                           _Out_ PMDL *OutMdl);
