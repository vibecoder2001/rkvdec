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

/* Interrupt flags */
#define RK_MMU_IRQ_PAGE_FAULT   0x01u
#define RK_MMU_IRQ_BUS_ERROR    0x02u
#define RK_MMU_IRQ_MASK         (RK_MMU_IRQ_PAGE_FAULT | RK_MMU_IRQ_BUS_ERROR)

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
    /* Page directory: RK_IOMMU_PD_ENTRIES * sizeof(ULONG) = 4 KiB */
    volatile ULONG  *Pd;            /* kernel VA of the page directory */
    ULONG            PdPhys;        /* 32-bit physical address of Pd */

    /* Per-entry page table pointers (kernel VA), NULL until first use */
    volatile ULONG  *Pts[RK_IOMMU_PD_ENTRIES];
    ULONG            PtPhys[RK_IOMMU_PD_ENTRIES]; /* 32-bit PA of each PT */

    /* IOVA allocator: 1M-bit bitmap (128 KiB), 1 bit per 4 KiB page.
     * Bit 0 is reserved (NULL guard).  Non-paged pool.  */
    ULONG_PTR       *IovaBitmap;    /* 128 KiB bitmap, non-paged pool */

    KSPIN_LOCK       Lock;          /* serializes PT writes + bitmap alloc */
} RKIOMMU_DOMAIN, *PRKIOMMU_DOMAIN;

/* ---------------------------------------------------------------------------
 * Public API (implemented in pgtable.c)
 * --------------------------------------------------------------------------- */
_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS RkIommuDomainCreate(_Out_ PRKIOMMU_DOMAIN *Domain);

_IRQL_requires_max_(DISPATCH_LEVEL)
VOID RkIommuDomainDestroy(_In_ PRKIOMMU_DOMAIN Domain);

_IRQL_requires_max_(DISPATCH_LEVEL)
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
