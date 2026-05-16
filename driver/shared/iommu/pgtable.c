/* driver/rkiommu/pgtable.c — Rockchip IOMMU page-table implementation.
 *
 * Two-level 32-bit IOVA space (v1 format):
 *   bits 31:22 — 10-bit PD index
 *   bits 21:12 — 10-bit PT index
 *   bits 11:0  — 12-bit page offset
 *
 * PTE/PDE bit definitions verified against:
 *   torvalds/linux drivers/iommu/rockchip-iommu.c (master branch)
 *   Lines 189-257: PDE valid bit 0; PTE valid bit 0, readable bit 1,
 *   writable bit 2.
 *
 * Physical memory constraints:
 *   The Rockchip IOMMU DTE_ADDR register is 32 bits wide, so the page
 *   directory and all page tables must reside in the low 4 GiB of
 *   physical RAM.  We enforce this by setting hi.QuadPart = 0xffffffff
 *   in all MmAllocateContiguousNodeMemory calls.
 *
 * Cache note:
 *   All page tables are allocated PAGE_NOCACHE, so CPU writes are
 *   immediately visible to the IOMMU without explicit cache flushes.
 *   The IOMMU's own TLB still needs a ZAP_CACHE command on each
 *   map/unmap; that is issued by the caller (device.c) when paging
 *   is enabled.
 */
#include <ntddk.h>
#include <wdf.h>
#include "pgtable.h"

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------- */

/* Allocate a 4 KiB physically-contiguous, PAGE_NOCACHE page below 4 GiB. */
static volatile ULONG *
AllocPageBelow4G(_Out_ ULONG *OutPhys)
{
    PHYSICAL_ADDRESS lo  = {0};
    PHYSICAL_ADDRESS hi  = {0};
    PHYSICAL_ADDRESS aln = {0};
    hi.QuadPart  = 0xffffffffull;   /* must stay in low 4 GiB */
    aln.QuadPart = RK_IOMMU_PAGE_SIZE;

    PVOID va = MmAllocateContiguousNodeMemory(
        RK_IOMMU_PAGE_SIZE, lo, hi, aln,
        PAGE_READWRITE | PAGE_NOCACHE,
        MM_ANY_NODE_OK);
    if (!va) {
        *OutPhys = 0;
        return NULL;
    }
    RtlZeroMemory(va, RK_IOMMU_PAGE_SIZE);
    *OutPhys = MmGetPhysicalAddress(va).LowPart;
    return (volatile ULONG *)va;
}

static VOID FreePageBelow4G(_In_ volatile ULONG *Va)
{
    MmFreeContiguousMemory((PVOID)Va);
}

/* ---------------------------------------------------------------------------
 * RkIommuDomainCreateVdec — v2 IOMMU domain (RKCP3570, no PTA).
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuDomainCreateVdec(PRKIOMMU_DOMAIN *Domain)
{
    PRKIOMMU_DOMAIN d = (PRKIOMMU_DOMAIN)ExAllocatePool2(
        POOL_FLAG_NON_PAGED, sizeof(*d), 'mDkR');
    if (!d) return STATUS_INSUFFICIENT_RESOURCES;
    RtlZeroMemory(d, sizeof(*d));
    KeInitializeSpinLock(&d->Lock);
    d->IsAv1d = FALSE;

    d->Pd = AllocPageBelow4G(&d->PdPhys);
    if (!d->Pd) { ExFreePoolWithTag(d, 'mDkR'); return STATUS_INSUFFICIENT_RESOURCES; }

    SIZE_T bitmapBytes = RK_IOMMU_IOVA_PAGES / 8;
    d->IovaBitmap = (ULONG_PTR *)ExAllocatePool2(POOL_FLAG_NON_PAGED, bitmapBytes, 'bIkR');
    if (!d->IovaBitmap) {
        FreePageBelow4G(d->Pd);
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    d->IovaStartBitmap = (ULONG_PTR *)ExAllocatePool2(POOL_FLAG_NON_PAGED, bitmapBytes, 'sIkR');
    if (!d->IovaStartBitmap) {
        ExFreePoolWithTag(d->IovaBitmap, 'bIkR');
        FreePageBelow4G(d->Pd);
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    RtlZeroMemory(d->IovaBitmap, bitmapBytes);
    RtlZeroMemory(d->IovaStartBitmap, bitmapBytes);
    d->IovaBitmap[0] |= (ULONG_PTR)1;       /* reserve iova page 0 */
    d->IovaStartBitmap[0] |= (ULONG_PTR)1;  /* mark page 0 as a boundary */

    /* Reserve BSP-prescribed RCB-SRAM IOVA range 0xFFF00000..0xFFFFFFFF.
     * Marked as a single allocation in the start-bitmap so any UnmapMdl
     * scanning up from a normal allocation placed just below this range
     * stops at the reservation boundary instead of over-unmapping into it. */
    {
        const ULONG kRcbPage  = 0xFFF00000u / RK_IOMMU_PAGE_SIZE;
        const ULONG kRcbCount = 0x00100000u / RK_IOMMU_PAGE_SIZE;
        const ULONG kBpw      = (ULONG)(sizeof(ULONG_PTR) * 8);
        for (ULONG p = kRcbPage; p < kRcbPage + kRcbCount &&
                                 p < RK_IOMMU_IOVA_PAGES; p++)
            d->IovaBitmap[p / kBpw] |= (ULONG_PTR)1 << (p % kBpw);
        if (kRcbPage < RK_IOMMU_IOVA_PAGES)
            d->IovaStartBitmap[kRcbPage / kBpw] |= (ULONG_PTR)1 << (kRcbPage % kBpw);
    }

    *Domain = d;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuDomainCreateAv1d — AV1D IOMMU domain (RKCP3571, allocates PTA).
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuDomainCreateAv1d(PRKIOMMU_DOMAIN *Domain)
{
    PRKIOMMU_DOMAIN d = (PRKIOMMU_DOMAIN)ExAllocatePool2(
        POOL_FLAG_NON_PAGED, sizeof(*d), 'mDkR');
    if (!d) return STATUS_INSUFFICIENT_RESOURCES;
    RtlZeroMemory(d, sizeof(*d));
    KeInitializeSpinLock(&d->Lock);
    d->IsAv1d = TRUE;

    d->Pd = AllocPageBelow4G(&d->PdPhys);
    if (!d->Pd) { ExFreePoolWithTag(d, 'mDkR'); return STATUS_INSUFFICIENT_RESOURCES; }

    ULONG ptaPhys = 0;
    d->Pta = (volatile ULONGLONG *)AllocPageBelow4G(&ptaPhys);
    if (!d->Pta) {
        FreePageBelow4G(d->Pd);
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    d->PtaPhys = ptaPhys;
    d->Pta[0] = ((ULONGLONG)d->PdPhys & AV1_PTA_DT_ADDR_MASK) | AV1_PTA_MODE_4K;

    SIZE_T bitmapBytes = RK_IOMMU_IOVA_PAGES / 8;
    d->IovaBitmap = (ULONG_PTR *)ExAllocatePool2(POOL_FLAG_NON_PAGED, bitmapBytes, 'bIkR');
    if (!d->IovaBitmap) {
        FreePageBelow4G((volatile ULONG *)d->Pta);
        FreePageBelow4G(d->Pd);
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    d->IovaStartBitmap = (ULONG_PTR *)ExAllocatePool2(POOL_FLAG_NON_PAGED, bitmapBytes, 'sIkR');
    if (!d->IovaStartBitmap) {
        ExFreePoolWithTag(d->IovaBitmap, 'bIkR');
        FreePageBelow4G((volatile ULONG *)d->Pta);
        FreePageBelow4G(d->Pd);
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    RtlZeroMemory(d->IovaBitmap, bitmapBytes);
    RtlZeroMemory(d->IovaStartBitmap, bitmapBytes);
    d->IovaBitmap[0] |= (ULONG_PTR)1;
    d->IovaStartBitmap[0] |= (ULONG_PTR)1;

    *Domain = d;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuDomainDestroy
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
VOID RkIommuDomainDestroy(PRKIOMMU_DOMAIN Domain)
{
    if (!Domain) return;

    /* Free all page tables */
    for (ULONG i = 0; i < RK_IOMMU_PD_ENTRIES; i++) {
        if (Domain->Pts[i]) {
            FreePageBelow4G(Domain->Pts[i]);
            Domain->Pts[i]    = NULL;
            Domain->PtPhys[i] = 0;
        }
    }

    if (Domain->Page0Scratch) {
        FreePageBelow4G(Domain->Page0Scratch);
        Domain->Page0Scratch = NULL;
        Domain->Page0Phys    = 0;
    }

    if (Domain->Pd) {
        FreePageBelow4G(Domain->Pd);
        Domain->Pd    = NULL;
        Domain->PdPhys = 0;
    }

    if (Domain->Pta) {
        FreePageBelow4G((volatile ULONG *)Domain->Pta);
        Domain->Pta     = NULL;
        Domain->PtaPhys = 0;
    }

    if (Domain->IovaBitmap) {
        ExFreePoolWithTag(Domain->IovaBitmap, 'bIkR');
        Domain->IovaBitmap = NULL;
    }

    if (Domain->IovaStartBitmap) {
        ExFreePoolWithTag(Domain->IovaStartBitmap, 'sIkR');
        Domain->IovaStartBitmap = NULL;
    }

    ExFreePoolWithTag(Domain, 'mDkR');
}

/* ---------------------------------------------------------------------------
 * RkIommuAllocIova — linear first-fit bitmap allocator
 *
 * Scans the bitmap for 'PageCount' consecutive clear bits, starting from
 * page 1 (page 0 is the NULL guard).  Returns STATUS_INSUFFICIENT_RESOURCES
 * if no contiguous range is found.
 *
 * Called with Domain->Lock held.
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuAllocIova(PRKIOMMU_DOMAIN Domain, ULONG PageCount,
                           PULONG64 OutIova)
{
    /* Total number of ULONG_PTR words in the bitmap */
    const ULONG bitsPerWord = (ULONG)(sizeof(ULONG_PTR) * 8);
    const ULONG totalWords  = RK_IOMMU_IOVA_PAGES / bitsPerWord;

    ULONG run   = 0;    /* current consecutive-clear-bit count */
    ULONG start = 0;    /* bit index where the current run started */

    /* Scan TOP-DOWN to match BSP's allocation strategy.  BSP gives the
     * first dma-heap allocation iova ~0xFFDC0000 and walks down from
     * there.  Our codec was faulting at low iovas in the 0xae0..0xe00
     * range; it's plausible the codec hardware masks high bits of some
     * register when the value is in the low-iova range, so allocations
     * in the high range avoid that quirk. */
    for (LONG w = (LONG)totalWords - 1; w >= 0; w--) {
        ULONG_PTR word = Domain->IovaBitmap[w];

        for (LONG b = (LONG)bitsPerWord - 1; b >= 0; b--) {
            ULONG bitIdx = (ULONG)w * bitsPerWord + (ULONG)b;

            if (!(word & ((ULONG_PTR)1 << b))) {
                /* free bit — extend the descending run */
                if (run == 0) start = bitIdx;       /* highest free bit  */
                run++;
                if (run == PageCount) {
                    ULONG base = bitIdx;            /* lowest bit in run */
                    for (ULONG k = base; k < base + PageCount; k++) {
                        ULONG kw = k / bitsPerWord;
                        ULONG kb = k % bitsPerWord;
                        Domain->IovaBitmap[kw] |= (ULONG_PTR)1 << kb;
                    }
                    /* Mark this range's first page as an allocation
                     * boundary so UnmapMdl's page-count recovery walk
                     * stops here and can't over-unmap into us from a
                     * neighbour that was placed immediately below. */
                    Domain->IovaStartBitmap[base / bitsPerWord]
                        |= (ULONG_PTR)1 << (base % bitsPerWord);
                    *OutIova = (ULONG64)base * RK_IOMMU_PAGE_SIZE;
                    return STATUS_SUCCESS;
                }
            } else {
                run = 0;
            }
        }
    }

    return STATUS_INSUFFICIENT_RESOURCES;
}

/* ---------------------------------------------------------------------------
 * RkIommuFreeIova — return IOVA pages to the bitmap
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
VOID RkIommuFreeIova(PRKIOMMU_DOMAIN Domain, ULONG64 Iova, ULONG PageCount)
{
    const ULONG bitsPerWord = (ULONG)(sizeof(ULONG_PTR) * 8);
    ULONG startPage = (ULONG)(Iova >> 12);

    for (ULONG i = 0; i < PageCount; i++) {
        ULONG page = startPage + i;
        ULONG w    = page / bitsPerWord;
        ULONG b    = page % bitsPerWord;
        Domain->IovaBitmap[w] &= ~((ULONG_PTR)1 << b);
    }
    /* Clear the start marker so a future allocation at this base can set
     * a fresh one (and so UnmapMdl can't be tricked into bounding at a
     * stale boundary if the bits get reused). */
    Domain->IovaStartBitmap[startPage / bitsPerWord]
        &= ~((ULONG_PTR)1 << (startPage % bitsPerWord));
}

/* ---------------------------------------------------------------------------
 * RkIommuMapAt — install physical→IOVA mappings
 *
 * Iova and PhysAddr must both be 4 KiB aligned.
 * Flags: RK_PTE_PAGE_READABLE | RK_PTE_PAGE_WRITABLE (at minimum readable).
 *
 * The iova arithmetic bug noted in the plan is fixed here:
 *   first_pdi / first_pti are the initial directory/table indices.
 *   For page i, the effective indices are computed via integer division and
 *   modulo so that they roll over correctly across PD boundaries.
 *
 * Called with Domain->Lock held.
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuMapAt(PRKIOMMU_DOMAIN Domain, ULONG64 Iova,
                       ULONG64 PhysAddr, ULONG PageCount, ULONG Flags)
{
    ULONG first_pdi = (ULONG)((Iova >> 22) & 0x3ffu);
    ULONG first_pti = (ULONG)((Iova >> 12) & 0x3ffu);

    for (ULONG i = 0; i < PageCount; i++) {
        ULONG pdi = first_pdi + (first_pti + i) / RK_IOMMU_PT_ENTRIES;
        ULONG pti = (first_pti + i) % RK_IOMMU_PT_ENTRIES;

        /* Bounds check: pdi must not overflow the directory */
        if (pdi >= RK_IOMMU_PD_ENTRIES) {
            /* Roll back already-mapped pages and return error */
            if (i > 0) {
                RkIommuUnmapAt(Domain, Iova, i);
            }
            return STATUS_INVALID_PARAMETER;
        }

        /* Ensure the page table for this pdi exists */
        if (!Domain->Pts[pdi]) {
            volatile ULONG *pt = AllocPageBelow4G(&Domain->PtPhys[pdi]);
            if (!pt) {
                if (i > 0) {
                    RkIommuUnmapAt(Domain, Iova, i);
                }
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            Domain->Pts[pdi] = pt;

            /* Install the DTE: PT phys | valid.  Mainline-v2 uses the
             * full 32 bits of address (mask 0xfffff000); AV1D uses
             * mask 0xffffffc0 (64-byte alignment).  Both work with
             * 4 KiB-aligned PTs we allocate. */
            ULONG dte = Domain->IsAv1d
                ? (Domain->PtPhys[pdi] & AV1_DTE_PT_ADDR_MASK) | AV1_DTE_VALID
                : (Domain->PtPhys[pdi] | RK_DTE_PT_VALID);
            Domain->Pd[pdi] = dte;
        }

        /* Install the PTE.  Mainline-v2: address[31:12] | r/w/v (bits 0/1/2).
         * AV1D: address[31:12] | (address[39:32] << 28) | v (bit 0) | w (bit 2);
         * no "readable" bit. */
        ULONG64 pagePhys = PhysAddr + (ULONG64)i * RK_IOMMU_PAGE_SIZE;
        ULONG pte;
        if (Domain->IsAv1d) {
            ULONG lo  = (ULONG)(pagePhys & AV1_PTE_PAGE_ADDR_LOW_MASK);
            ULONG hi8 = (ULONG)((pagePhys >> 32) & AV1_PTE_PAGE_ADDR_HIGH_MASK);
            pte = lo | (hi8 << AV1_PTE_PAGE_ADDR_HIGH_SHIFT) | AV1_PTE_VALID;
            if (Flags & RK_PTE_PAGE_WRITABLE) pte |= AV1_PTE_WRITABLE;
        } else {
            pte = (ULONG)(pagePhys & RK_PTE_PAGE_ADDRESS_MASK) |
                  (Flags & (RK_PTE_PAGE_READABLE | RK_PTE_PAGE_WRITABLE)) |
                  RK_PTE_PAGE_VALID;
        }
        Domain->Pts[pdi][pti] = pte;
    }

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuUnmapAt — clear PTE entries for an IOVA range
 *
 * Does not free empty page tables (leave them for re-use; domain destroy
 * handles the reclaim).
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
VOID RkIommuUnmapAt(PRKIOMMU_DOMAIN Domain, ULONG64 Iova, ULONG PageCount)
{
    ULONG first_pdi = (ULONG)((Iova >> 22) & 0x3ffu);
    ULONG first_pti = (ULONG)((Iova >> 12) & 0x3ffu);

    for (ULONG i = 0; i < PageCount; i++) {
        ULONG pdi = first_pdi + (first_pti + i) / RK_IOMMU_PT_ENTRIES;
        ULONG pti = (first_pti + i) % RK_IOMMU_PT_ENTRIES;

        if (pdi >= RK_IOMMU_PD_ENTRIES) break;
        if (!Domain->Pts[pdi]) continue;

        Domain->Pts[pdi][pti] = 0;
    }
}
