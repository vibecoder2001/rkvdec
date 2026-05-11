/* shared/rkmpp_ccu_ifc.h — In-kernel device interface exported by
 * rkmpp_ccu.sys for codec cores in its cluster to consume.
 *
 * Phase 2: full cluster raise/drop + per-core reset.
 */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKMPP_CCU,
    0x3b2a8e02, 0x6d31, 0x4a08, 0x9a, 0x77, 0x3c, 0x6b, 0xb8, 0x10, 0xe2, 0x55);

/* v3: add FullCoreReset for hang-recovery wide-bundle resets framed by a
 * PMU bus-idle handshake.  AssertCoreReset/DeassertCoreReset are the
 * narrow per-job toggle (CON40 bits 6..9) used on first-kick post-PnP
 * and post-error.  FullCoreReset is for the kill-mid-decode case where
 * the codec is genuinely wedged and even bus-side blocks need cycling.
 *
 * Mirrors Linux `rkvdec2_reset` CRU-bundle path: PMU idle req → assert
 * {NIU_a, NIU_h, AXI, AHB, CORE, CABAC, HEVC_CABAC} → udelay(5) →
 * deassert in reverse → PMU idle release.  The caller MUST follow with
 * an `RKIOMMU_INTERFACE::Reattach` so the IOMMU's DTE_ADDR is
 * reprogrammed (bits 2..5 reset the AXI/AHB bus blocks the IOMMU sits
 * on, and a paging-disabled IOMMU silently drops AXI traffic). */
/* v4: add per-kick leaf-clock gate/ungate.  Linux BSP `rkvdec2_clk_off`
 * runs at the end of every task (mpp_task_finish → mpp_power_off →
 * clk_off) and `rkvdec2_clk_on` at the start (mpp_power_on → clk_on),
 * gating + ungating the codec leaf clocks (clk_rkvdec0_core,
 * clk_rkvdec0_ca, clk_rkvdec0_hevc_ca) between every kick.  This
 * clock-cycle break drains in-flight AXI traffic and resets
 * clock-domain-crossing flops without touching the codec FSM.  Without
 * it, our 12 ms uncached output-buffer read was accidentally providing
 * the same settle window — and removing that read (zero-copy or
 * non-ref skip) collapsed inter-kick spacing and wedged the codec.
 * Bus-root clocks (CON40 bits 0,1,2 — hclk/aclk/aclk_ccu) stay UNgated
 * so MMIO register access between kicks still works. */
/* v5: add RaiseAv1Cluster / DropAv1Cluster.  AV1 (RKCP3560) lives in its
 * own power domain (PD_AV1) with independent clocks/resets from the
 * rkvdec0/1 cluster.  The base RaiseCluster powers PD_VCODEC + PD_VDPU
 * + PD_RKVDEC0/1; the AV1 path powers PD_VCODEC + PD_VDPU + PD_AV1.
 * The two parent PDs are ref-counted so both clusters can coexist
 * without one's drop yanking power from the other. */
/* v6: add GateAv1LeafClocks / UngateAv1LeafClocks.  Splits the CLKGATE_CON68
 * leaf-clock toggle out from RaiseAv1Cluster/DropAv1Cluster so rkav1d
 * can gate clocks at D3 entry while PD_AV1 stays raised across the
 * D-state transition.  Mirrors the rkvdec0/1
 * GateCoreLeafClocks/UngateCoreLeafClocks pattern, but for CON68 bits
 * (AV1 BIU + AV1 leaf gates) rather than CON40.  Linux BSP runtime-PM
 * model (mpp_av1dec.c / mpp_iommu_av1d.c) keeps PD_AV1 raised across
 * RPM_SUSPEND and only cycles clocks; this pair lets us match that
 * topology without touching the existing PrepareHardware/ReleaseHardware
 * Raise/Drop semantics.  Not refcounted — paired 1:1 per device with
 * the matching Ungate/Gate call. */
/* v7: split rkvdec leaf-clock + core-reset methods into per-instance
 * pairs.  The v6 GateCoreLeafClocks / UngateCoreLeafClocks /
 * AssertCoreReset / DeassertCoreReset methods only wrote
 * CLKGATE_CON40 / SOFTRST_CON40 (RVD0's bits) despite the generic
 * naming — calling them from RVD1's job path cross-talked, gating /
 * resetting RVD0 and wedging any in-flight RVD0 decode.  The
 * band-aid in commit fb73e0c was to skip the per-kick gate on RVD1
 * entirely (`if (pub.Uid == 0)` guards in driver/rkvdec/job.c) — that
 * killed the cross-talk but also dropped RVD1's per-task clock-cycle
 * break, diverging from BSP `mpp_rkvdec2.c:1074-1095`
 * rkvdec2_clk_on/clk_off (per-device clock list — no shared state).
 *
 * v7 renames the existing v6 fields to make their RVD0 scope explicit
 * (GateRvdec0LeafClocks / UngateRvdec0LeafClocks / AssertRvdec0CoreReset
 * / DeassertRvdec0CoreReset) and adds the symmetrical RVD1 pair
 * targeting CLKGATE_CON41 / SOFTRST_CON41.  FullCoreReset stays
 * unchanged — it's an RVD0-specific hang-recovery method tied to the
 * CON40 wide-bundle CRU dance and a separate concern.
 *
 * AV1 surface (GateAv1LeafClocks / RaiseAv1Cluster / etc.) is
 * untouched. */
/* v8: split FullCoreReset per-codec, narrow (drop shared NIU bits).
 * The v7 FullCoreReset was hardcoded RVD0-only despite the generic
 * name — same scoping pitfall the v6→v7 leaf-clock / core-reset
 * split fixed for the narrow per-kick toggle.  v8 splits it into
 * per-codec hang-recovery methods:
 *
 *   FullCoreReset0 — RVD0 (was FullCoreReset; renamed)
 *   FullCoreReset1 — RVD1 (new — CON41 wide bundle)
 *   FullAv1Reset   — AV1  (new — CON68 wide bundle)
 *
 * Each method does PMU bus-idle request → assert codec-cluster CRU
 * bits → udelay(5) → deassert → release.  All three follow the BSP
 * `rkvdec2_reset` shape; the AV1 variant mirrors the CON68 dance
 * RaiseAv1Cluster already uses on bring-up.
 *
 * v8 ALSO removes the shared SOFTRST_CON44 NIU bits (4..6 —
 * VDPU aclk / aclk_low / hclk NIU) from all three methods.  That
 * NIU is shared between every VDPU-child codec (RVD0, RVD1, JPEG,
 * IEP, AV1's parent path, …) and resetting it from a single
 * codec's hang-recovery path would disrupt any peer codec
 * mid-decode.  If a coordinated wide NIU reset is ever needed it
 * will be a separate ifc method gated on all-codecs-idle.
 *
 * Callers must still follow each FullXxxReset with
 * RKIOMMU_INTERFACE::Reattach — the CRU bundles still reset the
 * AXI/AHB bus blocks the IOMMU sits behind, so DTE_ADDR is
 * cleared as a side effect. */
#define RKMPP_CCU_IFC_VERSION 8u

typedef NTSTATUS (*RKMPP_CCU_QUERY_VERSION)(_Out_ PUINT32);
typedef NTSTATUS (*RKMPP_CCU_RAISE_CLUSTER) (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_DROP_CLUSTER)  (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_ASSERT_RESET)  (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_DEASSERT_RESET)(_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_FULL_RESET)    (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_LEAF_GATE)     (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_LEAF_UNGATE)   (_In_ PVOID ClientCookie);

typedef struct _RKMPP_CCU_INTERFACE {
    INTERFACE                 Header;
    RKMPP_CCU_QUERY_VERSION   QueryVersion;
    RKMPP_CCU_RAISE_CLUSTER   RaiseCluster;
    RKMPP_CCU_DROP_CLUSTER    DropCluster;
    /* RVD0 (rkvdec0) per-instance reset + leaf-clock gating.
     * Targets CLKGATE_CON40 / SOFTRST_CON40 bits.  Mirrors BSP
     * `rkvdec2_clk_on/off` for the device whose CRU bank is CON40. */
    RKMPP_CCU_ASSERT_RESET    AssertRvdec0CoreReset;     /* was AssertCoreReset */
    RKMPP_CCU_DEASSERT_RESET  DeassertRvdec0CoreReset;   /* was DeassertCoreReset */
    RKMPP_CCU_LEAF_GATE       GateRvdec0LeafClocks;      /* was GateCoreLeafClocks */
    RKMPP_CCU_LEAF_UNGATE     UngateRvdec0LeafClocks;    /* was UngateCoreLeafClocks */
    /* RVD1 (rkvdec1) per-instance reset + leaf-clock gating — new in v7.
     * Targets CLKGATE_CON41 / SOFTRST_CON41 bits.  Replaces the
     * fb73e0c `if (pub.Uid == 0)` skip in driver/rkvdec/job.c. */
    RKMPP_CCU_ASSERT_RESET    AssertRvdec1CoreReset;
    RKMPP_CCU_DEASSERT_RESET  DeassertRvdec1CoreReset;
    RKMPP_CCU_LEAF_GATE       GateRvdec1LeafClocks;
    RKMPP_CCU_LEAF_UNGATE     UngateRvdec1LeafClocks;
    /* Per-codec wide-bundle hang-recovery resets (v8).  Each does a
     * PMU bus-idle request → assert that codec's CRU bundle bits →
     * udelay(5) → deassert → release.  FullCoreReset0 targets
     * SOFTRST_CON40 (RVD0), FullCoreReset1 targets SOFTRST_CON41
     * (RVD1).  The shared VDPU NIU bits in SOFTRST_CON44 (4..6)
     * are NOT touched — that NIU is shared with all VDPU-child
     * codecs and resetting it would disrupt peer codecs mid-decode.
     * Callers MUST follow with RKIOMMU_INTERFACE::Reattach (the
     * CRU bundle clears DTE_ADDR via the AXI/AHB bus reset). */
    RKMPP_CCU_FULL_RESET      FullCoreReset0;
    RKMPP_CCU_FULL_RESET      FullCoreReset1;
    /* AV1 cluster (RKCP3560) — independent PD_AV1 / clocks / resets
     * from the rkvdec0/1 cluster.  PD_VCODEC + PD_VDPU parents are
     * ref-counted internally so both clusters can coexist. */
    RKMPP_CCU_RAISE_CLUSTER   RaiseAv1Cluster;
    RKMPP_CCU_DROP_CLUSTER    DropAv1Cluster;
    /* Per-D0-state AV1 leaf-clock gate.  PD_AV1 stays raised; only the
     * CLKGATE_CON68 bits toggle.  Called from rkav1d
     * EvtDeviceD0Exit/D0Entry.  Not refcounted — paired 1:1 with the
     * matching Ungate/Gate call. */
    RKMPP_CCU_LEAF_GATE       GateAv1LeafClocks;
    RKMPP_CCU_LEAF_UNGATE     UngateAv1LeafClocks;
    /* AV1 wide-bundle hang-recovery reset (v8).  PMU bus-idle on
     * PD_AV1, assert SOFTRST_CON68 bundle, udelay(5), deassert,
     * release.  Same Reattach-after-call rule as FullCoreReset0/1. */
    RKMPP_CCU_FULL_RESET      FullAv1Reset;
} RKMPP_CCU_INTERFACE, *PRKMPP_CCU_INTERFACE;
