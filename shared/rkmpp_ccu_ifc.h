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
#define RKMPP_CCU_IFC_VERSION 5u

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
    RKMPP_CCU_ASSERT_RESET    AssertCoreReset;
    RKMPP_CCU_DEASSERT_RESET  DeassertCoreReset;
    RKMPP_CCU_FULL_RESET      FullCoreReset;
    RKMPP_CCU_LEAF_GATE       GateCoreLeafClocks;     /* per-kick gate */
    RKMPP_CCU_LEAF_UNGATE     UngateCoreLeafClocks;   /* per-kick ungate */
    /* AV1 cluster (RKCP3560) — independent PD_AV1 / clocks / resets
     * from the rkvdec0/1 cluster.  PD_VCODEC + PD_VDPU parents are
     * ref-counted internally so both clusters can coexist. */
    RKMPP_CCU_RAISE_CLUSTER   RaiseAv1Cluster;
    RKMPP_CCU_DROP_CLUSTER    DropAv1Cluster;
} RKMPP_CCU_INTERFACE, *PRKMPP_CCU_INTERFACE;
