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
 * `Ccu.FullCoreReset0/1`/`FullAv1Reset` toggles.  Caller MUST follow with `Reattach` to
 * reprogram DTE_ADDR (FORCE_RESET zeroes it). */
/* v6: add Enable/Disable/MaskIrq/UnmaskIrq so the codec driver (rkav1d)
 * can orchestrate the whole cluster's D0Entry/D0Exit sequence on a
 * single thread.  Design rationale: WDF does NOT synchronize D-state
 * transitions between sibling devices under the same parent — if both
 * rkav1d and rkiommu_av1d registered their own EvtDeviceD0Entry/Exit,
 * the framework would dispatch the two callbacks concurrently and
 * neither side could guarantee the IOMMU MMIO writes happen on the
 * correct side of the CCU clock-gate toggle.  Instead, rkav1d owns the
 * full sequence and drives the IOMMU's hardware state remotely through
 * these ifc calls.  Ordering rkav1d will use:
 *   D0Exit  : poller-quiesce → MaskIrq → Disable → Ccu.GateAv1LeafClocks
 *   D0Entry : Ccu.UngateAv1LeafClocks → Enable → UnmaskIrq
 * Enable/Disable wrap the existing internal helpers (idempotent on
 * re-enable, AHB_CONTROL=0 on disable).  Mask/UnmaskIrq use
 * WdfInterruptDisable/Enable — these only disconnect the ISR from
 * kernel dispatch (no MMIO), so they are safe to call when the
 * underlying clocks are gated.  All four methods require PASSIVE_LEVEL
 * (WdfInterruptDisable/Enable constraint).  The pre-existing
 * `Reattach` method (Disable-then-Enable atomically) is kept for
 * cross-session walk-cache flush; it intentionally does NOT expose the
 * intermediate state needed to interleave clock-gate writes, so
 * D0Entry/D0Exit need the new split methods. */
/* v7: add IsPtAttached for slave page-table readiness query.  Slave
 * rkiommu instances (Task 2.2) leave page tables NULL until the
 * MASTER instance arrives and they re-program their DTE_ADDR with
 * the master's PdPhys.  Codec drivers paired with the slave (RVD1)
 * must wait on this before issuing any IOCTL that hits the iommu.
 * Master always returns TRUE; slave returns its PtAttached field. */
#define RKIOMMU_IFC_VERSION 7u

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

/* v6 D-state orchestration (PASSIVE_LEVEL only).  Enable programs
 * DTE_ADDR and asserts AHB paging; Disable writes AHB_CONTROL=0 and
 * clears the PagingEnabled flag.  MaskIrq/UnmaskIrq call
 * WdfInterruptDisable/Enable to detach/reattach the ISR from the
 * kernel dispatcher without touching MMIO — safe to invoke when
 * clocks are gated.  See RKIOMMU_IFC_VERSION header for sequencing. */
typedef NTSTATUS (*RKIOMMU_ENABLE)(_In_ PVOID ProviderContext);
typedef NTSTATUS (*RKIOMMU_DISABLE)(_In_ PVOID ProviderContext);
typedef NTSTATUS (*RKIOMMU_MASK_IRQ)(_In_ PVOID ProviderContext);
typedef NTSTATUS (*RKIOMMU_UNMASK_IRQ)(_In_ PVOID ProviderContext);

typedef BOOLEAN (*RKIOMMU_IS_PT_ATTACHED)(_In_ PVOID ProviderContext);

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
    RKIOMMU_ENABLE           Enable;
    RKIOMMU_DISABLE          Disable;
    RKIOMMU_MASK_IRQ         MaskIrq;
    RKIOMMU_UNMASK_IRQ       UnmaskIrq;
    RKIOMMU_IS_PT_ATTACHED   IsPtAttached;    /* v7 */
} RKIOMMU_INTERFACE, *PRKIOMMU_INTERFACE;
