/* shared/rkiommu_ifc.h — In-kernel device interface exported by
 * rkiommu.sys for codec cores to consume.
 *
 * Phase 2: full MapMdl/UnmapMdl + fault registration.
 */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKIOMMU,
    0x4f9b1c23, 0x82a9, 0x4cd8, 0xb3, 0x14, 0x57, 0xa1, 0x0e, 0x44, 0x9d, 0x12);

/* v3 adds Hid/Uid to the returned interface so consumers can pick the
 * specific rkiommu instance that owns the codec they're driving (per
 * the static topology table in rkiommu/topology.c).  v2 had a single
 * "first match" shortcut that broke any system with more than one
 * rkiommu instance — buffer mappings would land in the wrong page
 * table and the codec hardware would silently see untranslated iovas.
 *
 * Calling convention (v3): the first parameter of MapMdl / UnmapMdl /
 * RegisterFaultHandler is ProviderContext — the consumer MUST pass
 * `iommu->Header.Context` (the value the provider set when publishing
 * its interface).  This is what lets the provider route the call back
 * to the specific iommu instance the consumer queried.  Passing any
 * other value (e.g. the consumer's own device object) routes to the
 * wrong instance.  RegisterFaultHandler additionally takes a separate
 * ConsumerContext that is opaque to the provider and passed back
 * verbatim to the fault callback. */
/* v4: add Reattach for cross-session walk-cache flush.  Linux's
 * `mpp_iommu_refresh` (mpp_common.c, called from mpp_dev_reset) detaches
 * and re-attaches the IOMMU domain on every full reset; the source
 * comment is explicit: "if the domain does not change, iommu attach will
 * be return as an empty operation. Therefore, force to close and then
 * open, will update the domain. In this way, domain can really attach."
 * Our `FlushTlb` only invalidates page-table TLB entries; it does NOT
 * flush walk caches / prefetch buffers.  Killing a process mid-decode
 * leaves walk-cache state from the killed session that the next session
 * can chase into stale physical addresses — `Reattach` resets that
 * state.  Body: STALL → mask IRQs → DISABLE_PAGING → zero DTE_ADDR
 * → reprogram DTE_ADDR with the same domain → ENABLE_PAGING → UN-STALL. */
/* v5: add ForceReset for soft-tier session-end recovery.  Mirrors BSP
 * `rk_iommu_force_reset` (rockchip-iommu.c): STALL → write
 * RK_MMU_CMD_FORCE_RESET → poll DTE_ADDR == 0 → UN-STALL.  Resets the
 * MMU's internal state machine (walk caches, prefetcher, fault state)
 * without touching the codec/AXI/AHB/NIU CRU bits the wide
 * `Ccu.FullCoreReset` toggles.  Caller MUST follow with `Reattach` to
 * reprogram DTE_ADDR (FORCE_RESET zeroes it). */
#define RKIOMMU_IFC_VERSION 5u

typedef NTSTATUS (*RKIOMMU_QUERY_VERSION)(_Out_ PUINT32);

typedef NTSTATUS (*RKIOMMU_MAP_MDL)(
    _In_  PVOID ProviderContext,
    _In_  PMDL  Mdl,
    _In_  ULONG Role,
    _Out_ PULONG64 Iova);

typedef NTSTATUS (*RKIOMMU_UNMAP_MDL)(
    _In_ PVOID ProviderContext,
    _In_ ULONG64 Iova);

typedef VOID (*RKIOMMU_FAULT_CALLBACK)(
    _In_ PVOID ConsumerContext,
    _In_ ULONG64 FaultIova,
    _In_ ULONG StatusReg);

typedef NTSTATUS (*RKIOMMU_REGISTER_FAULT)(
    _In_ PVOID ProviderContext,
    _In_ PVOID ConsumerContext,
    _In_ RKIOMMU_FAULT_CALLBACK Callback);

/* Diagnostic snapshot used by the consumer post-kick to find out whether
 * the codec's last AXI traffic produced an IOMMU page fault and, if so,
 * at what iova.  All registers are read-only sampling — no state change. */
typedef struct _RKIOMMU_FAULT_SNAPSHOT {
    ULONG       Status;        /* RK_MMU_STATUS */
    ULONG       IntRawStat;    /* RK_MMU_INT_RAWSTAT */
    ULONG       IntStatus;     /* RK_MMU_INT_STATUS */
    ULONG       PageFaultAddr; /* RK_MMU_PAGE_FAULT_ADDR */
    ULONG       DteAddr;       /* RK_MMU_DTE_ADDR readback */
    /* MMU#1 (second 0x40 register bank) - all-zero if MmioLength < 0x80. */
    ULONG       Status1;
    ULONG       IntRawStat1;
    ULONG       IntStatus1;
    ULONG       PageFaultAddr1;
    ULONG       DteAddr1;
} RKIOMMU_FAULT_SNAPSHOT, *PRKIOMMU_FAULT_SNAPSHOT;

typedef NTSTATUS (*RKIOMMU_SNAPSHOT)(
    _In_  PVOID                    ProviderContext,
    _Out_ PRKIOMMU_FAULT_SNAPSHOT  Out);

/* Flush the IOMMU's TLB.  BSP calls this before every codec kick (see
 * rkvdec2_run -> mpp_iommu_flush_tlb).  Required when the codec reuses
 * the same iovas across multiple decode sessions; without a pre-kick
 * flush, stale page-table entries cause the codec to read/write from
 * obsolete physical addresses. */
typedef NTSTATUS (*RKIOMMU_FLUSH_TLB)(_In_ PVOID ProviderContext);

/* Detach + reattach the IOMMU domain.  See RKIOMMU_IFC_VERSION header
 * comment for rationale.  Existing iova→phys mappings in the domain are
 * preserved; only the hardware-side walk caches and DTE binding are
 * rebuilt.  Safe to call even when paging is already disabled. */
typedef NTSTATUS (*RKIOMMU_REATTACH)(_In_ PVOID ProviderContext);

/* Soft-tier reset: issues RK_MMU_CMD_FORCE_RESET to all MMU instances
 * under stall, then polls DTE_ADDR == 0 for completion.  Returns
 * STATUS_TIMEOUT if any MMU instance fails to ack within 100 ms. */
typedef NTSTATUS (*RKIOMMU_FORCE_RESET)(_In_ PVOID ProviderContext);

typedef struct _RKIOMMU_INTERFACE {
    INTERFACE                Header;
    UINT32                   Hid;     /* e.g. 0x3570 / 0x3571 */
    UINT32                   Uid;     /* per-HID instance number */
    RKIOMMU_QUERY_VERSION    QueryVersion;
    RKIOMMU_MAP_MDL          MapMdl;
    RKIOMMU_UNMAP_MDL        UnmapMdl;
    RKIOMMU_REGISTER_FAULT   RegisterFaultHandler;
    RKIOMMU_SNAPSHOT         Snapshot;
    RKIOMMU_FLUSH_TLB        FlushTlb;
    RKIOMMU_REATTACH         Reattach;
    RKIOMMU_FORCE_RESET      ForceReset;
} RKIOMMU_INTERFACE, *PRKIOMMU_INTERFACE;
