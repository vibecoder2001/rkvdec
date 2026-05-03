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
#define RKMPP_CCU_IFC_VERSION 3u

typedef NTSTATUS (*RKMPP_CCU_QUERY_VERSION)(_Out_ PUINT32);
typedef NTSTATUS (*RKMPP_CCU_RAISE_CLUSTER) (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_DROP_CLUSTER)  (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_ASSERT_RESET)  (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_DEASSERT_RESET)(_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_FULL_RESET)    (_In_ PVOID ClientCookie);

typedef struct _RKMPP_CCU_INTERFACE {
    INTERFACE                 Header;
    RKMPP_CCU_QUERY_VERSION   QueryVersion;
    RKMPP_CCU_RAISE_CLUSTER   RaiseCluster;
    RKMPP_CCU_DROP_CLUSTER    DropCluster;
    RKMPP_CCU_ASSERT_RESET    AssertCoreReset;
    RKMPP_CCU_DEASSERT_RESET  DeassertCoreReset;
    RKMPP_CCU_FULL_RESET      FullCoreReset;
} RKMPP_CCU_INTERFACE, *PRKMPP_CCU_INTERFACE;
