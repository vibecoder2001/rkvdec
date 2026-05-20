/* driver/rkmpp/bufpool.c — DMA-coherent buffer pool implementation.
 *
 * Each IOCTL_RKMPP_ALLOC_BUFFER:
 *   1. Rounds up the requested size to PAGE_SIZE.
 *   2. Allocates physically-contiguous memory < 4 GiB (rkiommu uses 32-bit
 *      page tables) via MmAllocateContiguousNodeMemory.
 *   3. Builds a non-paged MDL over those pages.
 *   4. Maps the MDL into the IOMMU address space via rkiommu MapMdl.
 *   5. Maps the MDL into the calling process's user address space via
 *      MmMapLockedPagesSpecifyCache(UserMode) — the simplest approach;
 *      no section handle, no NtMapViewOfSection.
 *   6. Assigns a unique cookie (global atomic counter).
 *   7. Tracks the buffer in the per-file-object RKMPP_FILE_CTX list.
 *
 * Freeing (explicit or at file-close cleanup):
 *   - Unmap user VA: must run in the owner process's context; we
 *     KeStackAttachProcess to the PEPROCESS recorded at allocation.
 *   - Unmap from IOMMU.
 *   - Free MDL and contiguous memory.
 */

/* ntifs.h is a superset of ntddk.h; include it first to get KAPC_STATE,
 * KeStackAttachProcess, and KeUnstackDetachProcess (used for safe user-VA
 * unmapping in a process context different from the original allocator). */
#include <ntifs.h>
#include <wdf.h>

#include "../../../shared/rkmpp_ioctl.h"
#include "../../../shared/rkiommu_ifc.h"
#include "bufpool.h"
#include "profile.h"

/* -----------------------------------------------------------------------
 * Forward declarations for the device-context accessor defined in device.c
 * --------------------------------------------------------------------- */
extern PRKIOMMU_INTERFACE RkMppGetIommuIfc(_In_ WDFDEVICE Device);

/* -----------------------------------------------------------------------
 * Cookie counter — one global, starts at 1, never wraps in practice
 * --------------------------------------------------------------------- */
static volatile LONG64 g_nextCookie = 1;
static volatile LONG64 g_totalAllocatedBytes = 0;

static const ULONG  RKMPP_MAX_BUFFER_BYTES = 128u * 1024u * 1024u;
static const UINT64 RKMPP_MAX_FILE_BYTES   = 2ull * 1024ull * 1024ull * 1024ull;
static const LONG64 RKMPP_MAX_GLOBAL_BYTES = 4ll * 1024ll * 1024ll * 1024ll;
static const UINT32 RKMPP_MAX_FILE_BUFFER_COUNT = 128u;

static BOOLEAN
RkMppBufUsageValid(_In_ UINT32 Usage)
{
    return Usage == RkMppBufferUsageBitstreamInput ||
           Usage == RkMppBufferUsageReferenceFrame ||
           Usage == RkMppBufferUsageOutputFrame ||
           Usage == RkMppBufferUsageScratch;
}

static NTSTATUS
RkMppBufRoundSize(_In_ UINT32 Size, _Out_ PULONG Rounded)
{
    if (Size == 0 || Size > RKMPP_MAX_BUFFER_BYTES)
        return STATUS_INVALID_PARAMETER;
    if (Size > MAXULONG - (PAGE_SIZE - 1))
        return STATUS_INVALID_PARAMETER;

    *Rounded = (Size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
    if (*Rounded == 0 || *Rounded > RKMPP_MAX_BUFFER_BYTES)
        return STATUS_INVALID_PARAMETER;

    return STATUS_SUCCESS;
}

static NTSTATUS
RkMppBufReserveQuota(_In_ PRKMPP_FILE_CTX Ctx, _In_ ULONG Size)
{
    KIRQL oldIrql;
    KeAcquireSpinLock(&Ctx->Lock, &oldIrql);

    if (Ctx->BufferCount >= RKMPP_MAX_FILE_BUFFER_COUNT ||
        Ctx->AllocatedBytes > RKMPP_MAX_FILE_BYTES - Size) {
        KeReleaseSpinLock(&Ctx->Lock, oldIrql);
        return STATUS_QUOTA_EXCEEDED;
    }

    Ctx->AllocatedBytes += Size;
    Ctx->BufferCount++;
    KeReleaseSpinLock(&Ctx->Lock, oldIrql);

    LONG64 newTotal = InterlockedAdd64(&g_totalAllocatedBytes, Size);
    if (newTotal > RKMPP_MAX_GLOBAL_BYTES) {
        InterlockedAdd64(&g_totalAllocatedBytes, -(LONG64)Size);

        KeAcquireSpinLock(&Ctx->Lock, &oldIrql);
        Ctx->AllocatedBytes -= Size;
        Ctx->BufferCount--;
        KeReleaseSpinLock(&Ctx->Lock, oldIrql);
        return STATUS_QUOTA_EXCEEDED;
    }

    return STATUS_SUCCESS;
}

static VOID
RkMppBufReleaseQuota(_In_ PRKMPP_FILE_CTX Ctx, _In_ ULONG Size)
{
    KIRQL oldIrql;
    KeAcquireSpinLock(&Ctx->Lock, &oldIrql);

    if (Ctx->AllocatedBytes >= Size)
        Ctx->AllocatedBytes -= Size;
    else
        Ctx->AllocatedBytes = 0;

    if (Ctx->BufferCount > 0)
        Ctx->BufferCount--;

    KeReleaseSpinLock(&Ctx->Lock, oldIrql);
    InterlockedAdd64(&g_totalAllocatedBytes, -(LONG64)Size);
}

/* -----------------------------------------------------------------------
 * RkMppBufFileCtxInit
 * --------------------------------------------------------------------- */
NTSTATUS
RkMppBufFileCtxInit(_In_ WDFFILEOBJECT File, _In_ WDFDEVICE Device)
{
    PRKMPP_FILE_CTX ctx = RkMppFileGet(File);
    InitializeListHead(&ctx->Buffers);
    KeInitializeSpinLock(&ctx->Lock);
    ctx->Device = Device;
    ctx->AllocatedBytes = 0;
    ctx->BufferCount = 0;
    return STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------
 * RkMppBufFreeOne — internal; unwinds a fully or partially initialised
 * RKMPP_BUFFER.  Caller must hold no spin lock (IRQL <= DISPATCH_LEVEL
 * on entry, but we attach to a process so we need PASSIVE_LEVEL).
 * --------------------------------------------------------------------- */
static VOID
RkMppBufFreeOne(_In_ PRKMPP_BUFFER Buf, _In_ WDFDEVICE Device)
{
    /* 1. Unmap user VA — must be in the owner process context.
     *
     * Skip the attach/unmap entirely when the owner process has
     * begun teardown: KeStackAttachProcess to a "zombie" EPROCESS
     * lands in a context whose page tables may already have been
     * reclaimed, and MmUnmapLockedPages then writes user-mode
     * VA backed by no valid VAD → BSOD (BAD_POOL_CALLER or
     * PAGE_FAULT_IN_NONPAGED_AREA).  Process-level address-space
     * teardown reclaims the mapping for us, so dropping the
     * explicit unmap on this path is safe.  PsGetProcessExitStatus
     * returns STATUS_PENDING while the process is live; anything
     * else (STATUS_SUCCESS / exit code) means PspExitThread has
     * begun the teardown.
     *
     * Code-review issue I3.  See [[bufpool_process_exit_race]]. */
    if (Buf->UserVa && Buf->Mdl && Buf->OwnerProcess) {
        if (PsGetProcessExitStatus(Buf->OwnerProcess) == STATUS_PENDING) {
            KAPC_STATE apcState;
            KeStackAttachProcess(Buf->OwnerProcess, &apcState);
            MmUnmapLockedPages(Buf->UserVa, Buf->Mdl);
            KeUnstackDetachProcess(&apcState);
        }
        /* else: process teardown already in progress; address-space
         * reclaim handles the VA mapping. */
        Buf->UserVa = NULL;
    }

    /* 2. Unmap from IOMMU. */
    if (Buf->Iova) {
        PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(Device);
        if (iommu && iommu->UnmapMdl) {
            iommu->UnmapMdl(iommu->Header.Context, Buf->Iova);
        }
        Buf->Iova = 0;
    }

    /* 3. Free MDL. */
    if (Buf->Mdl) {
        IoFreeMdl(Buf->Mdl);
        Buf->Mdl = NULL;
    }

    /* 4. Free contiguous memory. */
    if (Buf->KernelVa) {
        MmFreeContiguousMemory(Buf->KernelVa);
        Buf->KernelVa = NULL;
    }

    /* 5. Release process reference. */
    if (Buf->OwnerProcess) {
        ObDereferenceObject(Buf->OwnerProcess);
        Buf->OwnerProcess = NULL;
    }

    ExFreePoolWithTag(Buf, 'BppR');
}

/* -----------------------------------------------------------------------
 * RkMppBufLookupIova — return iova + size for a buffer cookie.  Walks the
 * file's allocation list under the spinlock; safe at <= DISPATCH_LEVEL.
 * --------------------------------------------------------------------- */
NTSTATUS
RkMppBufLookupIova(_In_ WDFFILEOBJECT File,
                   _In_ UINT64        Cookie,
                   _Out_ UINT64      *OutIova,
                   _Out_ ULONG       *OutSize)
{
    PRKMPP_FILE_CTX ctx = RkMppFileGet(File);
    KIRQL oldIrql;
    NTSTATUS status = STATUS_NOT_FOUND;

    KeAcquireSpinLock(&ctx->Lock, &oldIrql);
    for (PLIST_ENTRY entry = ctx->Buffers.Flink;
         entry != &ctx->Buffers;
         entry = entry->Flink) {
        PRKMPP_BUFFER buf = CONTAINING_RECORD(entry, RKMPP_BUFFER, Link);
        if (buf->Cookie == Cookie) {
            *OutIova = buf->Iova;
            *OutSize = buf->Size;
            status   = STATUS_SUCCESS;
            break;
        }
    }
    KeReleaseSpinLock(&ctx->Lock, oldIrql);
    return status;
}

/* -----------------------------------------------------------------------
 * RkMppBufLookupMdl — return the MDL for a buffer cookie.  Walks the
 * file's allocation list under the spinlock; safe at <= DISPATCH_LEVEL.
 * --------------------------------------------------------------------- */
NTSTATUS
RkMppBufLookupMdl(_In_  WDFFILEOBJECT File,
                  _In_  UINT64        Cookie,
                  _Out_ PMDL         *OutMdl)
{
    PRKMPP_FILE_CTX ctx = RkMppFileGet(File);
    KIRQL oldIrql;
    NTSTATUS status = STATUS_NOT_FOUND;

    *OutMdl = NULL;
    KeAcquireSpinLock(&ctx->Lock, &oldIrql);
    for (PLIST_ENTRY entry = ctx->Buffers.Flink;
         entry != &ctx->Buffers;
         entry = entry->Flink) {
        PRKMPP_BUFFER buf = CONTAINING_RECORD(entry, RKMPP_BUFFER, Link);
        if (buf->Cookie == Cookie) {
            *OutMdl = buf->Mdl;
            status  = STATUS_SUCCESS;
            break;
        }
    }
    KeReleaseSpinLock(&ctx->Lock, oldIrql);
    return status;
}

/* -----------------------------------------------------------------------
 * RkMppBufAlloc
 * --------------------------------------------------------------------- */
NTSTATUS
RkMppBufAlloc(_In_ WDFDEVICE                    Device,
              _In_ WDFFILEOBJECT               File,
              _In_ const RKMPP_ALLOC_BUFFER_IN *In,
              _Out_ RKMPP_ALLOC_BUFFER_OUT     *Out)
{
    NTSTATUS status;

    /* --- 1. Validate usage and round size up to PAGE_SIZE ------------- */
    if (!RkMppBufUsageValid(In->Usage))
        return STATUS_INVALID_PARAMETER;

    ULONG sizeRounded = 0;
    status = RkMppBufRoundSize(In->Size, &sizeRounded);
    if (!NT_SUCCESS(status))
        return status;

    PRKMPP_FILE_CTX fctx = RkMppFileGet(File);
    status = RkMppBufReserveQuota(fctx, sizeRounded);
    if (!NT_SUCCESS(status))
        return status;

    /* --- 2. Allocate physically-contiguous memory < 4 GiB ------------ */
    PHYSICAL_ADDRESS lo = {0};
    PHYSICAL_ADDRESS hi;
    hi.QuadPart = 0xFFFFFFFFLL;   /* < 4 GiB */
    PHYSICAL_ADDRESS boundary = {0};

    /* Cached mapping.  An earlier attempt produced top-half-fresh /
     * bottom-half-stale tearing; rediagnosed as an AXI write-buffer
     * drain race rather than a cache coherency hole — the codec's IRQ
     * asserts before its tail-end DMA writes reach DRAM, and our
     * uncached read was masking it by being slow enough.  Fix is in
     * the poller (RkMppPollerThread): dummy MMIO read + short stall
     * after observing DEC_RDY, before invalidate.  With that drain in
     * place, cached + per-job KeFlushIoBuffers gives ~4× faster CPU
     * reads of the 4K output buffer (12 ms uncached → 3 ms cached). */
    PVOID kernelVa = MmAllocateContiguousNodeMemory(
        sizeRounded, lo, hi, boundary,
        PAGE_READWRITE,
        MM_ANY_NODE_OK);
    if (!kernelVa) {
        RkMppBufReleaseQuota(fctx, sizeRounded);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    RtlZeroMemory(kernelVa, sizeRounded);

    /* --- 3. Build non-paged MDL --------------------------------------- */
    PMDL mdl = IoAllocateMdl(kernelVa, sizeRounded, FALSE, FALSE, NULL);
    if (!mdl) {
        MmFreeContiguousMemory(kernelVa);
        RkMppBufReleaseQuota(fctx, sizeRounded);
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    MmBuildMdlForNonPagedPool(mdl);

    /* Flush CPU caches for the underlying physical pages.  PAGE_NOCACHE
     * makes our kernel mapping uncached, so RtlZeroMemory above wrote
     * straight to DRAM through that mapping — but the same physical pages
     * may have been previously mapped cacheable under another VA (the OS
     * recycles pages across allocations).  Stale dirty lines in the L1/L2
     * for those aliasing VAs can be snooped by the codec's DMA on ARM64
     * and surface as residue.  Linux dma-heap is bit-exact deterministic
     * because it both zeroes and flushes; we match that here. */
    KeFlushIoBuffers(mdl, /*ReadOperation*/ FALSE, /*DmaOperation*/ TRUE);

    /* --- 4. Map into IOMMU -------------------------------------------- */
    PRKIOMMU_INTERFACE iommu = RkMppGetIommuIfc(Device);
    if (!iommu || !iommu->MapMdl) {
        IoFreeMdl(mdl);
        MmFreeContiguousMemory(kernelVa);
        RkMppBufReleaseQuota(fctx, sizeRounded);
        return STATUS_DEVICE_NOT_READY;
    }

    ULONG64 iova = 0;
    /* Pass the iommu's per-instance Header.Context so the provider can
     * resolve to the specific iommu instance we queried (per topology).
     * Passing our own wdmDev would land all maps in g_deviceList head
     * (VPMU UID=0). */
    PVOID provCtx = iommu->Header.Context;
    status = iommu->MapMdl(provCtx, mdl, In->Usage, &iova);
    if (!NT_SUCCESS(status)) {
        IoFreeMdl(mdl);
        MmFreeContiguousMemory(kernelVa);
        RkMppBufReleaseQuota(fctx, sizeRounded);
        return status;
    }

    /* --- 5. Map into calling process user space ----------------------- */
    /* All codecs use cached user-mode mappings.  The poller (job.c
     * RkMppPollerThread) issues a dummy MMIO read + PERF_WORKING_CNT
     * poll after DEC_RDY before signalling completion; RkMppJobComplete
     * then calls KeFlushIoBuffers on DMA-output MDLs to invalidate stale
     * CPU cache lines before any user-mode read.
     *
     * 2026-05-12 NOTE: a brief MmNonCached experiment made the RVD0
     * H.264 B-frame divergence WORSE, not better — wedge moved earlier
     * and more B-frames diverged.  Likely cause: with MmNonCached user
     * mappings, user writes bypass CPU cache, but the kernel-VA mapping
     * (cached PAGE_READWRITE) still holds the post-RtlZeroMemory zeros;
     * pre-kick KeFlushIoBuffers clean of those stale kernel-VA lines
     * clobbers the user's uncached writes in DRAM. */
    MEMORY_CACHING_TYPE userCacheType = MmCached;
    PVOID userVa = MmMapLockedPagesSpecifyCache(
        mdl, UserMode, userCacheType, NULL, FALSE, NormalPagePriority);
    if (!userVa) {
        iommu->UnmapMdl(provCtx, iova);
        IoFreeMdl(mdl);
        MmFreeContiguousMemory(kernelVa);
        RkMppBufReleaseQuota(fctx, sizeRounded);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    /* --- 6. Assign cookie --------------------------------------------- */
    UINT64 cookie = (UINT64)InterlockedIncrement64(&g_nextCookie);

    /* --- 7. Allocate tracking node ------------------------------------ */
    PRKMPP_BUFFER buf = (PRKMPP_BUFFER)ExAllocatePoolWithTag(
        NonPagedPoolNx, sizeof(RKMPP_BUFFER), 'BppR');
    if (!buf) {
        /* Must unmap in this process context before we return. */
        MmUnmapLockedPages(userVa, mdl);
        iommu->UnmapMdl(provCtx, iova);
        IoFreeMdl(mdl);
        MmFreeContiguousMemory(kernelVa);
        RkMppBufReleaseQuota(fctx, sizeRounded);
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    RtlZeroMemory(buf, sizeof(*buf));

    buf->Cookie    = cookie;
    buf->KernelVa  = kernelVa;
    buf->Mdl       = mdl;
    buf->UserVa    = userVa;
    buf->Iova      = iova;
    buf->Size      = sizeRounded;

    /* Record owner process so cleanup can KeStackAttachProcess safely. */
    buf->OwnerProcess = PsGetCurrentProcess();
    ObReferenceObject(buf->OwnerProcess);

    /* Insert under lock. */
    KIRQL oldIrql;
    KeAcquireSpinLock(&fctx->Lock, &oldIrql);
    InsertTailList(&fctx->Buffers, &buf->Link);
    KeReleaseSpinLock(&fctx->Lock, oldIrql);

    /* --- 8. Fill output ----------------------------------------------- */
    RtlZeroMemory(Out, sizeof(*Out));
    Out->StructSize    = sizeof(*Out);
    Out->BufferHandle  = cookie;
    Out->Iova          = iova;
    Out->UserVa        = userVa;
    Out->SizeRoundedUp = sizeRounded;

    return STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------
 * RkMppBufFree — look up by cookie, remove, and release
 * --------------------------------------------------------------------- */
NTSTATUS
RkMppBufFree(_In_ WDFFILEOBJECT File, _In_ UINT64 Cookie)
{
    PRKMPP_FILE_CTX fctx = RkMppFileGet(File);
    PRKMPP_BUFFER   found = NULL;

    KIRQL oldIrql;
    KeAcquireSpinLock(&fctx->Lock, &oldIrql);
    for (PLIST_ENTRY e = fctx->Buffers.Flink;
         e != &fctx->Buffers; e = e->Flink) {
        PRKMPP_BUFFER b = CONTAINING_RECORD(e, RKMPP_BUFFER, Link);
        if (b->Cookie == Cookie) {
            RemoveEntryList(&b->Link);
            found = b;
            break;
        }
    }
    KeReleaseSpinLock(&fctx->Lock, oldIrql);

    if (!found)
        return STATUS_NOT_FOUND;

    RkMppBufReleaseQuota(fctx, found->Size);
    RkMppBufFreeOne(found, fctx->Device);
    return STATUS_SUCCESS;
}

/* -----------------------------------------------------------------------
 * RkMppBufFreeAll — drain the list; called from EvtFileCleanup
 * --------------------------------------------------------------------- */
VOID
RkMppBufFreeAll(_In_ WDFFILEOBJECT File)
{
    PRKMPP_FILE_CTX fctx = RkMppFileGet(File);

    for (;;) {
        KIRQL oldIrql;
        KeAcquireSpinLock(&fctx->Lock, &oldIrql);
        if (IsListEmpty(&fctx->Buffers)) {
            KeReleaseSpinLock(&fctx->Lock, oldIrql);
            break;
        }
        PLIST_ENTRY e = RemoveHeadList(&fctx->Buffers);
        KeReleaseSpinLock(&fctx->Lock, oldIrql);

        PRKMPP_BUFFER b = CONTAINING_RECORD(e, RKMPP_BUFFER, Link);
        RkMppBufReleaseQuota(fctx, b->Size);
        RkMppBufFreeOne(b, fctx->Device);
    }
}
