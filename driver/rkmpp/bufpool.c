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

#include "../../shared/rkmpp_ioctl.h"
#include "../../shared/rkiommu_ifc.h"
#include "bufpool.h"

/* -----------------------------------------------------------------------
 * Forward declarations for the device-context accessor defined in device.c
 * --------------------------------------------------------------------- */
extern PRKIOMMU_INTERFACE RkMppGetIommuIfc(_In_ WDFDEVICE Device);

/* -----------------------------------------------------------------------
 * Cookie counter — one global, starts at 1, never wraps in practice
 * --------------------------------------------------------------------- */
static volatile LONG64 g_nextCookie = 1;

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
    /* 1. Unmap user VA — must be in the owner process context. */
    if (Buf->UserVa && Buf->Mdl && Buf->OwnerProcess) {
        KAPC_STATE apcState;
        KeStackAttachProcess(Buf->OwnerProcess, &apcState);
        MmUnmapLockedPages(Buf->UserVa, Buf->Mdl);
        KeUnstackDetachProcess(&apcState);
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

    /* --- 1. Round size up to PAGE_SIZE -------------------------------- */
    if (In->Size == 0)
        return STATUS_INVALID_PARAMETER;

    ULONG sizeRounded = (In->Size + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);

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
    if (!kernelVa)
        return STATUS_INSUFFICIENT_RESOURCES;

    RtlZeroMemory(kernelVa, sizeRounded);

    /* --- 3. Build non-paged MDL --------------------------------------- */
    PMDL mdl = IoAllocateMdl(kernelVa, sizeRounded, FALSE, FALSE, NULL);
    if (!mdl) {
        MmFreeContiguousMemory(kernelVa);
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
        return status;
    }

    /* --- 5. Map into calling process user space ----------------------- */
    PVOID userVa = MmMapLockedPagesSpecifyCache(
        mdl, UserMode, MmCached, NULL, FALSE, NormalPagePriority);
    if (!userVa) {
        iommu->UnmapMdl(provCtx, iova);
        IoFreeMdl(mdl);
        MmFreeContiguousMemory(kernelVa);
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
    PRKMPP_FILE_CTX fctx = RkMppFileGet(File);
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
        RkMppBufFreeOne(b, fctx->Device);
    }
}
