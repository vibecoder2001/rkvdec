/* driver/rkiommu/pgtable.h — Rockchip IOMMU page table abstraction.
 *
 * Two-level 32-bit IOVA space:
 *   10-bit PD index | 10-bit PT index | 12-bit page offset
 *   Directory: 1024 PDEs, each pointing to a 4 KiB page table
 *   Page table: 1024 PTEs, each covering a 4 KiB page
 *
 * Register constants sourced from mainline Linux:
 *   drivers/iommu/rockchip-iommu.c
 *   (https://github.com/torvalds/linux, master branch, 2024)
 *   Lines 32-43 (offsets), 58-64 (commands), 189-257 (PTE/PDE format)
 */
#pragma once
#include <ntddk.h>
#include <wdf.h>

/* ---------------------------------------------------------------------------
 * Rockchip IOMMU MMIO register offsets (v1 format)
 * Source: torvalds/linux drivers/iommu/rockchip-iommu.c lines 32-43
 * --------------------------------------------------------------------------- */
#define RK_MMU_DTE_ADDR         0x00u   /* Page directory base (physical) */
#define RK_MMU_STATUS           0x04u   /* Status register */
#define RK_MMU_COMMAND          0x08u   /* Command register */
#define RK_MMU_PAGE_FAULT_ADDR  0x0Cu   /* IOVA of last page fault */
#define RK_MMU_ZAP_ONE_LINE     0x10u   /* Shootdown single IOTLB entry */
#define RK_MMU_INT_RAWSTAT      0x14u   /* IRQ raw status (ignores mask) */
#define RK_MMU_INT_CLEAR        0x18u   /* Acknowledge / re-arm IRQ */
#define RK_MMU_INT_MASK         0x1Cu   /* IRQ enable mask */
#define RK_MMU_INT_STATUS       0x20u   /* IRQ status after masking */
#define RK_MMU_AUTO_GATING      0x24u   /* Auto-gating control */

/* Status bits (RK_MMU_STATUS) */
#define RK_MMU_STATUS_PAGING_ENABLED        (1u << 0)
#define RK_MMU_STATUS_PAGE_FAULT_ACTIVE     (1u << 1)
#define RK_MMU_STATUS_STALL_ACTIVE          (1u << 2)
#define RK_MMU_STATUS_IDLE                  (1u << 3)

/* Command values (write to RK_MMU_COMMAND) */
#define RK_MMU_CMD_ENABLE_PAGING    0u
#define RK_MMU_CMD_DISABLE_PAGING   1u
#define RK_MMU_CMD_ENABLE_STALL     2u
#define RK_MMU_CMD_DISABLE_STALL    3u
#define RK_MMU_CMD_ZAP_CACHE        4u
#define RK_MMU_CMD_PAGE_FAULT_DONE  5u
#define RK_MMU_CMD_FORCE_RESET      6u
/* BSP reset command — alias used by Phase 3a RkIommuEnable.
 * Value 0x01 matches the Linux rockchip-iommu driver. */
#define RK_MMU_CMD_RESET            0x01u

/* Interrupt flags */
#define RK_MMU_IRQ_PAGE_FAULT   0x01u
#define RK_MMU_IRQ_BUS_ERROR    0x02u
#define RK_MMU_IRQ_MASK         (RK_MMU_IRQ_PAGE_FAULT | RK_MMU_IRQ_BUS_ERROR)

/* ---------------------------------------------------------------------------
 * AV1D IOMMU MMIO register offsets — RKCP3571 variant.  Layout from
 * linux-rockchip BSP drivers/video/rockchip/mpp/mpp_iommu_av1d.c.
 * Completely distinct from the v2 layout above; do NOT reuse the
 * RK_MMU_* constants on AV1D bases. */
#define AV1_MMU_FLUSH                  0x184u   /* bit 4 = flush */
#define AV1_MMU_FLUSH_BIT              (1u << 4)
#define AV1_MMU_CONFIG1                0x1ACu   /* bit 28 = OUT_OF_BOUND  */
#define AV1_MMU_CONFIG1_OUT_OF_BOUND   (1u << 28)
#define AV1_MMU_PAGE_FAULT_ADDR_AV1    0x380u
#define AV1_MMU_STATUS_AV1             0x384u   /* IRQ status */
#define AV1_MMU_STATUS_AV1_IRQ_MASK    0x7u
#define AV1_MMU_AHB_EXCEPTION          0x380u   /* same offset as fault-addr;
                                                 * write 1 to enable exceptions */
#define AV1_MMU_AHB_CONTROL            0x388u
#define AV1_MMU_AHB_CONTROL_ENABLE     (1u << 0)
#define AV1_MMU_AHB_TBL_ARRAY_BASE_L   0x38Cu   /* PTA dma low 32 bits  */
#define AV1_MMU_AHB_TBL_ARRAY_BASE_H   0x390u   /* PTA dma high 8 bits  */
#define AV1_MMU_AHB_CTX_PD             0x3B4u

/* AV1D PTE format — 40-bit page address packed into 32 bits.
 *   bits 31:12 — low 20 bits of page-aligned 40-bit address
 *   bits 11:4  — high 8 bits (bits 39:32) of the 40-bit address
 *   bit  2     — writable
 *   bit  0     — valid
 * Mainline-v2 PTE (bit 1=readable) does NOT exist on AV1D. */
#define AV1_PTE_VALID                  (1u << 0)
#define AV1_PTE_WRITABLE               (1u << 2)
#define AV1_PTE_PAGE_ADDR_LOW_MASK     0xfffff000u
#define AV1_PTE_PAGE_ADDR_HIGH_MASK    0x000000FFu  /* bits 39:32 of phys */
#define AV1_PTE_PAGE_ADDR_HIGH_SHIFT   28           /* land at bits 11:4 */

/* AV1D DTE format — bits 31:6 = PT phys (64-byte aligned), bit 0 = valid. */
#define AV1_DTE_VALID                  (1u << 0)
#define AV1_DTE_PT_ADDR_MASK           0xffffffc0u

/* AV1D PTA entry — 64-bit; bits 31:10 = DT phys (1KB aligned), bit 0 = mode.
 * Mode 0 = 4K pages.  We use only PTA[0] since we have a single domain. */
#define AV1_PTA_DT_ADDR_MASK           0xfffffc00u
#define AV1_PTA_MODE_4K                0u

/* ---------------------------------------------------------------------------
 * Page table entry (PTE) format — v1 32-bit addresses
 * Source: torvalds/linux drivers/iommu/rockchip-iommu.c lines 230-257
 *   Bits 31:12 — page physical address (4 KiB aligned)
 *   Bit  2     — writable
 *   Bit  1     — readable
 *   Bit  0     — valid
 * --------------------------------------------------------------------------- */
#define RK_PTE_PAGE_VALID       (1u << 0)
#define RK_PTE_PAGE_READABLE    (1u << 1)
#define RK_PTE_PAGE_WRITABLE    (1u << 2)
#define RK_PTE_PAGE_ADDRESS_MASK 0xfffff000u

/* ---------------------------------------------------------------------------
 * Directory table entry (DTE/PDE) format — v1
 * Source: torvalds/linux drivers/iommu/rockchip-iommu.c lines 189-197
 *   Bits 31:12 — page-table physical address (4 KiB aligned)
 *   Bit  0     — valid
 * --------------------------------------------------------------------------- */
#define RK_DTE_PT_VALID         (1u << 0)
#define RK_DTE_PT_ADDRESS_MASK  0xfffff000u

/* ---------------------------------------------------------------------------
 * Page table geometry
 * --------------------------------------------------------------------------- */
#define RK_IOMMU_PD_ENTRIES     1024u           /* entries in the directory */
#define RK_IOMMU_PT_ENTRIES     1024u           /* entries per page table */
#define RK_IOMMU_PAGE_SIZE      4096u           /* 4 KiB */
#define RK_IOMMU_IOVA_BITS      32u             /* 32-bit IOVA space */
#define RK_IOMMU_IOVA_PAGES     (1u << 20)      /* 2^32 / 4096 = 1 M pages */
#define RK_IOMMU_BITMAP_WORDS   (RK_IOMMU_IOVA_PAGES / (sizeof(ULONG_PTR)*8))

/* ---------------------------------------------------------------------------
 * RKIOMMU_DOMAIN — per-IOMMU address space
 *
 * All modifications (PT writes + bitmap updates) are serialized by Lock.
 * The PD is PAGE_NOCACHE so CPU writes reach RAM without explicit flushing;
 * page tables are also PAGE_NOCACHE for the same reason.
 * Both the PD and the PT pool must reside in the low 4 GiB of physical RAM
 * because the IOMMU DTE_ADDR register is 32 bits wide.
 * --------------------------------------------------------------------------- */
typedef struct _RKIOMMU_DOMAIN {
    /* TRUE for AV1D IOMMU (RKCP3571).  Set by RkIommuDomainCreate from
     * the variant flag.  Map/Unmap encode different PTE/DTE formats. */
    BOOLEAN          IsAv1d;

    /* AV1D PTA — 1024-entry × 8-byte page-table-array.  Only PTA[0] is
     * used (4K mode, points at our DT/Pd).  AV1D's IOMMU is programmed
     * with PtaPhys (not PdPhys).  NULL on v2 domains. */
    volatile ULONGLONG *Pta;        /* kernel VA, 4 KiB, AV1D only */
    ULONG               PtaPhys;    /* 32-bit physical, AV1D only */

    /* Page directory: RK_IOMMU_PD_ENTRIES * sizeof(ULONG) = 4 KiB */
    volatile ULONG  *Pd;            /* kernel VA of the page directory */
    ULONG            PdPhys;        /* 32-bit physical address of Pd */

    /* Per-entry page table pointers (kernel VA), NULL until first use */
    volatile ULONG  *Pts[RK_IOMMU_PD_ENTRIES];
    ULONG            PtPhys[RK_IOMMU_PD_ENTRIES]; /* 32-bit PA of each PT */

    /* IOVA allocator: 1M-bit bitmap (128 KiB), 1 bit per 4 KiB page.
     * Bit 0 is reserved (NULL guard).  Non-paged pool.  */
    ULONG_PTR       *IovaBitmap;    /* 128 KiB bitmap, non-paged pool */

    /* IOVA allocation-start bitmap.  Set at the first page of every range
     * returned by RkIommuAllocIova plus the static reservations placed at
     * domain-create (page 0, RCB-SRAM at 0xFFF00000).  Cleared by
     * RkIommuFreeIova.  Used by UnmapMdl to validate the caller's iova
     * starts an allocation we issued (defense against double-unmap / stray
     * pointers). */
    ULONG_PTR       *IovaStartBitmap; /* 128 KiB bitmap, non-paged pool */

    /* IOVA allocation-end bitmap.  Set at index (base + PageCount) — one
     * past the last page of every allocation — by RkIommuAllocIova; cleared
     * at the same position by RkIommuFreeIova.  UnmapMdl scans FORWARD
     * from (startPage + 1) until it hits an end-marker; the distance is
     * the allocation's PageCount.
     *
     * Rationale: the allocator is top-down (a new allocation gets a LOWER
     * base than older neighbours).  IovaStartBitmap alone is insufficient
     * because the OLDER allocation's start-bit is ABOVE the new one — an
     * upward IovaBitmap scan from the new base would never hit it before
     * running straight through the older allocation's contiguous bits and
     * over-unmapping into a peer File's PTEs that the peer's HW is still
     * DMA'ing through.  The end-marker is anchored at the allocation's
     * upper boundary so the unmap walk stops exactly at our own range.
     *
     * Bitmap is sized to (RK_IOMMU_IOVA_PAGES + 1) bits — an allocation
     * ending at the final page legitimately sets bit RK_IOMMU_IOVA_PAGES. */
    ULONG_PTR       *IovaEndBitmap;   /* (1M+1)/8 bytes, non-paged pool */

    KSPIN_LOCK       Lock;          /* serializes PT writes + bitmap alloc */
} RKIOMMU_DOMAIN, *PRKIOMMU_DOMAIN;

/* ---------------------------------------------------------------------------
 * Public API (implemented in pgtable.c)
 * --------------------------------------------------------------------------- */
/* RkIommuDomainCreateVdec — allocates a v2 IOMMU domain (RKCP3570).
 * Sets Domain->IsAv1d = FALSE; no PTA allocation. */
NTSTATUS RkIommuDomainCreateVdec(_Out_ PRKIOMMU_DOMAIN *Domain);

/* RkIommuDomainCreateAv1d — allocates an AV1D IOMMU domain (RKCP3571).
 * Sets Domain->IsAv1d = TRUE; allocates PTA level above the DT. */
NTSTATUS RkIommuDomainCreateAv1d(_Out_ PRKIOMMU_DOMAIN *Domain);

/* PASSIVE_LEVEL only — MmFreeContiguousMemory + ExFreePoolWithTag
 * paths reach back to PASSIVE-only kernel services.  Previously
 * annotated DISPATCH_LEVEL which was wrong; the only existing caller
 * (ReleaseHardware) is PASSIVE so it worked, but would mislead a
 * future DISPATCH-level caller into a BSOD.  Review IOMMU #4. */
_IRQL_requires_(PASSIVE_LEVEL)
VOID RkIommuDomainDestroy(_In_ PRKIOMMU_DOMAIN Domain);

/* PASSIVE_LEVEL only — MapAt may need to allocate a new page-table page
 * via MmAllocateContiguousNodeMemory (PASSIVE-only).  Previously annotated
 * DISPATCH_LEVEL, which would have BSOD'd on the first miss; all current
 * callers (MapMdl from WDF IOCTL handlers) run at PASSIVE.  Review #6. */
_IRQL_requires_(PASSIVE_LEVEL)
NTSTATUS RkIommuMapAt(
    _In_  PRKIOMMU_DOMAIN Domain,
    _In_  ULONG64 Iova,
    _In_  ULONG64 PhysAddr,
    _In_  ULONG   PageCount,
    _In_  ULONG   Flags);       /* RK_PTE_PAGE_READABLE | RK_PTE_PAGE_WRITABLE */

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID RkIommuUnmapAt(
    _In_ PRKIOMMU_DOMAIN Domain,
    _In_ ULONG64 Iova,
    _In_ ULONG   PageCount);

/* Allocate a contiguous IOVA range of PageCount 4-KiB pages.
 * Returns the first IOVA in *OutIova.  Caller must then call RkIommuMapAt. */
_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS RkIommuAllocIova(
    _In_  PRKIOMMU_DOMAIN Domain,
    _In_  ULONG   PageCount,
    _Out_ PULONG64 OutIova);

/* Free an IOVA range previously returned by RkIommuAllocIova. */
_IRQL_requires_max_(DISPATCH_LEVEL)
VOID RkIommuFreeIova(
    _In_ PRKIOMMU_DOMAIN Domain,
    _In_ ULONG64 Iova,
    _In_ ULONG   PageCount);
