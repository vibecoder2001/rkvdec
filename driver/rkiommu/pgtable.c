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
 * RkIommuDomainCreate
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuDomainCreate(PRKIOMMU_DOMAIN *Domain)
{
    PRKIOMMU_DOMAIN d = (PRKIOMMU_DOMAIN)ExAllocatePool2(
        POOL_FLAG_NON_PAGED, sizeof(*d), 'mDkR');
    if (!d) return STATUS_INSUFFICIENT_RESOURCES;
    RtlZeroMemory(d, sizeof(*d));
    KeInitializeSpinLock(&d->Lock);

    /* Allocate the page directory */
    d->Pd = AllocPageBelow4G(&d->PdPhys);
    if (!d->Pd) {
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    /* Allocate the IOVA bitmap (128 KiB) from non-paged pool */
    SIZE_T bitmapBytes = RK_IOMMU_IOVA_PAGES / 8;  /* 128 KiB */
    d->IovaBitmap = (ULONG_PTR *)ExAllocatePool2(
        POOL_FLAG_NON_PAGED, bitmapBytes, 'bIkR');
    if (!d->IovaBitmap) {
        FreePageBelow4G(d->Pd);
        ExFreePoolWithTag(d, 'mDkR');
        return STATUS_INSUFFICIENT_RESOURCES;
    }
    RtlZeroMemory(d->IovaBitmap, bitmapBytes);

    /* Reserve IOVA page 0 (NULL guard) — restored after system-abrupt-
     * shutdown.  Letting the bitstream buffer land at iova 0 caused the
     * codec to AXI-write/read at iova 0..0xFFF in ways that triggered a
     * critical SError or thermal/power shutdown.  Safer to fault the
     * IOMMU and let the driver report the bad iova. */
    d->IovaBitmap[0] |= (ULONG_PTR)1;

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

    if (Domain->Pd) {
        FreePageBelow4G(Domain->Pd);
        Domain->Pd    = NULL;
        Domain->PdPhys = 0;
    }

    if (Domain->IovaBitmap) {
        ExFreePoolWithTag(Domain->IovaBitmap, 'bIkR');
        Domain->IovaBitmap = NULL;
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

    for (ULONG w = 0; w < totalWords; w++) {
        ULONG_PTR word = Domain->IovaBitmap[w];

        for (ULONG b = 0; b < bitsPerWord; b++) {
            ULONG bitIdx = w * bitsPerWord + b;

            if (!(word & ((ULONG_PTR)1 << b))) {
                /* This bit is clear (free) */
                if (run == 0) start = bitIdx;
                run++;
                if (run == PageCount) {
                    /* Found a suitable run — mark it as allocated */
                    for (ULONG k = start; k < start + PageCount; k++) {
                        ULONG kw = k / bitsPerWord;
                        ULONG kb = k % bitsPerWord;
                        Domain->IovaBitmap[kw] |= (ULONG_PTR)1 << kb;
                    }
                    *OutIova = (ULONG64)start * RK_IOMMU_PAGE_SIZE;
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

            /* Install the PDE: PA | valid */
            Domain->Pd[pdi] = Domain->PtPhys[pdi] | RK_DTE_PT_VALID;
        }

        /* Install the PTE: PA | flags | valid */
        ULONG64 pagePhys = PhysAddr + (ULONG64)i * RK_IOMMU_PAGE_SIZE;
        ULONG pte = (ULONG)(pagePhys & RK_PTE_PAGE_ADDRESS_MASK) |
                    (Flags & (RK_PTE_PAGE_READABLE | RK_PTE_PAGE_WRITABLE)) |
                    RK_PTE_PAGE_VALID;
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
