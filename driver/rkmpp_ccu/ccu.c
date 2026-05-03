/* driver/rkmpp_ccu/ccu.c — RKCP3503 (RDCC) clock/reset/power.
 *
 * IMPORTANT TOPOLOGY NOTE
 *
 * RDCC at 0xfdc30000 (the MMIO range exposed by the RKCP3503 ACPI device)
 * is the rkv-decoder Cluster *Coordinator* (task arbitration / reset
 * orchestration), NOT the clock control unit.  Clock gating for the
 * RVD0/RVD1 cores lives in the system CRU at 0xfd7c0000 — a region the
 * firmware does NOT expose as an ACPI device.  Linux gets at it via the
 * generic clock framework; on Windows we have no such abstraction, so
 * this driver maps the system CRU directly by hardcoded physical address
 * (in driver.c::RkMppCcuEvtPrepareHardware) and uses that mapping for
 * RaiseCluster / DropCluster / *CoreReset.
 *
 * Two MMIO ranges are therefore in play:
 *   g_rdcc_mmio  = ACPI _CRS for RDCC, currently unused (Phase 3 task arb)
 *   g_cru_mmio   = direct map of system CRU at 0xfd7c0000 (this file)
 *
 * Register offsets sourced from rockchip-linux BSP kernel,
 *   commit 44d4aaaa9843057b32946f595948387db152be01
 *   drivers/clk/rockchip/clk-rk3588.c
 *   include/dt-bindings/clock/rk3588-cru.h
 *
 * CRU base (from rk3588s.dtsi):  0xfd7c0000, size 0x5c000.
 * Register offset formulas:
 *   RK3588_CLKGATE_CON(n) = n*4 + 0x800
 *   RK3588_SOFTRST_CON(n) = n*4 + 0xa00  (16 resets/reg)
 *
 * Clock-gate register CLKGATE_CON(40) = 0x8A0:
 *   bit 0  hclk_rkvdec0_root
 *   bit 1  aclk_rkvdec0_root
 *   bit 2  aclk_rkvdec_ccu
 *
 * Soft-reset register SOFTRST_CON(40) = 0xAA0:
 *   bit 9  SRST_RKVDEC0_CORE
 *
 * HIWORD-MASK convention
 *
 * RK3588 CRU CON-registers use the standard Rockchip hi-word-mask format:
 *   - Upper 16 bits are a write-enable mask (bit N set in the upper half
 *     means "update bit N in the lower half"; bit N clear means "leave
 *     bit N alone").
 *   - Lower 16 bits are the value bits.
 *   - Reads return only the lower 16 bits as live state; the upper half
 *     reads as zero, so plain read-modify-write does NOT work — we MUST
 *     write an explicit mask in the upper half.
 *
 * To set bits M to value V (where V uses bits 0..15):
 *   write (M << 16) | V
 *
 * Polarity:
 *   Clock gate: 1 = GATED (stopped), 0 = UNGATED (running).
 *   Soft reset: 1 = ASSERTED, 0 = DEASSERTED.
 */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ccu_ifc.h"
#include "pmu.h"

/* CRU register offsets relative to g_cru_mmio (which maps 0xfd7c0000+). */
/* CLKGATE_CON(40) bits we ungate:
 *   0 hclk_rkvdec0_root  7 clk_rkvdec0_ca
 *   1 aclk_rkvdec0_root  8 clk_rkvdec0_hevc_ca
 *   2 aclk_rkvdec_ccu    9 clk_rkvdec0_core
 *
 * CLKGATE_CON(41) bits we ungate:
 *   0 hclk_rkvdec1_root  7 clk_rkvdec1_hevc_ca
 *   1 aclk_rkvdec1_root  8 clk_rkvdec1_core
 *   6 clk_rkvdec1_ca
 *
 * Phase 2 ungated only the bus-roots (bits 0,1,2 of CON(40)) which left the
 * codec core clocks gated — codec MMIO reads SError'd on first hardware run.
 */
#define RDCC_CRU_CLKGATE_CON40   0x8A0u
#define RDCC_CRU_CLKGATE_CON41   0x8A4u
#define RDCC_CRU_CLKGATE_CON44   0x8B0u  /* VDPU root clocks: aclk/aclk_low/hclk */
#define RDCC_CRU_SOFTRST_CON40   0xAA0u
#define RDCC_CRU_SOFTRST_CON41   0xAA4u
#define RDCC_CRU_SOFTRST_CON44   0xAB0u  /* VDPU NIU resets at bits 4..6 */

/* CLKSEL_CON(89) = 0x300 + 89*4 = 0x464 — rkvdec0 root clock muxes/dividers.
 * Per linux-rockchip clk-rk3588.c the COMPOSITE clocks for aclk_rkvdec_ccu,
 * aclk_rkvdec0_root and hclk_rkvdec0_root all share this register:
 *
 *   [15:14] aclk_rkvdec_ccu  mux  (gpll|cpll|aupll|spll)   → 0 = GPLL
 *   [13:9 ] aclk_rkvdec_ccu  div  (actual_div − 1)          → 1 = /2 (594 MHz)
 *   [8:7  ] aclk_rkvdec0_root mux (gpll|cpll|aupll|spll)   → 0 = GPLL
 *   [6:2  ] aclk_rkvdec0_root div (actual_div − 1)          → 1 = /2 (594 MHz)
 *   [1:0  ] hclk_rkvdec0_root mux (200m|100m|50m|24m)       → 0 = 200 MHz
 *
 * Without this, ACLK_RKVDEC_CCU comes out of reset with an unconfigured mux
 * and the clock never actually ticks even after we ungate the gate bit.  The
 * rkvdec0 NIU then can't advance its bus-idle FSM and our PMU idle-deassert
 * never gets an ack.
 *
 * Hi-word-mask write: full 16-bit field set, value 0x0204 (bits 9 + 2). */
#define RDCC_CRU_CLKSEL_CON89    0x464u
#define RDCC_CRU_CLKSEL_CON89_MASK   0xFFFFu
#define RDCC_CRU_CLKSEL_CON89_VALUE  0x0204u

/* CLKSEL_CON(90) = 0x300 + 90*4 = 0x468 — leaf clocks for CABAC and HEVC CABAC.
 * Per edk2-rk3588 RK3588.h:
 *   bits  4..0  CLK_RKVDEC0_CA       div  → /8 = 187 MHz   (BSP wants 200 MHz)
 *   bit      5  CLK_RKVDEC0_CA       mux  → 0 = CPLL (1500 MHz)
 *   bits 10..6  CLK_RKVDEC0_HEVC_CA  div  → /3 = 333 MHz   (BSP wants 300 MHz)
 *   bits 12..11 CLK_RKVDEC0_HEVC_CA  mux  → 0 = MATRIX_1000M
 *
 * Without explicit rate setup the leaves come up at the 24 MHz reset default,
 * which is too slow for 200 ms decode budgets and explains "kick succeeds,
 * INT_STATUS reads 0 forever" on real hardware.  Div field = actual_div - 1. */
#define RDCC_CRU_CLKSEL_CON90    0x468u
#define RDCC_CRU_CLKSEL_CON90_MASK   0x1FFFu
#define RDCC_CRU_CLKSEL_CON90_VALUE  ((7u << 0) | (0u << 5) | (2u << 6) | (0u << 11))

/* CLKSEL_CON(91) = 0x300 + 91*4 = 0x46C — clk_rkvdec0_core.
 *   bits 4..0  div  → /8 = 187 MHz   (BSP wants 200 MHz)
 *   bit     5  mux  → 0 = CPLL */
#define RDCC_CRU_CLKSEL_CON91    0x46Cu
#define RDCC_CRU_CLKSEL_CON91_MASK   0x3Fu
#define RDCC_CRU_CLKSEL_CON91_VALUE  ((7u << 0) | (0u << 5))

typedef struct _RDCC_REGS {
    ULONG ClkGateCon44;     /* VDPU root clocks (must be on before PD_VDPU) */
    ULONG ClkGateCon44Mask;
    ULONG ClkGateCon40;
    ULONG ClkGateCon40Mask;
    ULONG ClkGateCon41;
    ULONG ClkGateCon41Mask;
    ULONG SoftRstCon40;
    ULONG SoftRstCon40Mask;
    ULONG SoftRstCon41;
    ULONG SoftRstCon41Mask;
    ULONG SoftRstCon44;
    ULONG SoftRstCon44Mask;
} RDCC_REGS;

/* Clock + reset masks per linux-rockchip clk-rk3588.c gate descriptor table.
 *
 * CON(44) bits 0,1,2 are ACLK_VDPU_ROOT / ACLK_VDPU_LOW_ROOT / HCLK_VDPU_ROOT.
 * These must be ungated BEFORE the PD_VDPU bus-idle handshake — without them
 * the bus has no clock and ack never propagates.
 *
 * AssertCoreReset / DeassertCoreReset (hang-recovery ifc) toggle only the
 * CORE bit (CON40 bit 9, CON41 bit 8). */
static const RDCC_REGS g_rdcc = {
    .ClkGateCon44     = RDCC_CRU_CLKGATE_CON44,
    .ClkGateCon44Mask = 0x00000077u,  /* VDPU root  bits 0..2 (aclk/aclk_low/hclk)
                                       * + VDPU NIU bits 4..6 (aclk/aclk_low/hclk_NIU)
                                       * The NIU bits sit between PD_VDPU and the
                                       * downstream PDs (RKVDEC0/1, IEP, RGA, etc.);
                                       * the rkvdec0 idle-ack travels through the
                                       * VDPU NIU, so its clocks must run for the
                                       * handshake FSM to advance.  Per
                                       * edk2-rk3588 RK3588.h: ACLK_VDPU_NIU_GATE=708,
                                       * ACLK_VDPU_LOW_NIU_GATE=709, HCLK_VDPU_NIU_GATE=710
                                       * → CLKGATE_CON(44) bits 4,5,6.            */
    .ClkGateCon40     = RDCC_CRU_CLKGATE_CON40,
    .ClkGateCon40Mask = 0x000003FFu,  /* bits 0..4,7,8,9 = 0x39F; mask 0x3FF
                                       * also covers 5,6 to be safe         */
    .ClkGateCon41     = RDCC_CRU_CLKGATE_CON41,
    .ClkGateCon41Mask = 0x000001CFu,  /* bits 0..3,6,7,8                    */
    .SoftRstCon40     = RDCC_CRU_SOFTRST_CON40,
    .SoftRstCon40Mask = 0x000003FCu,  /* bits 2..9 — CCU + rkvdec0 group    */
    .SoftRstCon41     = RDCC_CRU_SOFTRST_CON41,
    .SoftRstCon41Mask = 0x000001FCu,  /* bits 2..8 — rkvdec1 group          */
    .SoftRstCon44     = RDCC_CRU_SOFTRST_CON44,
    .SoftRstCon44Mask = 0x00000070u,  /* bits 4..6 — VDPU NIU resets        */
};

/* Per-job reset bundle (used by AssertCoreReset / DeassertCoreReset).
 * Toggle the codec's *internal* sub-block resets (CABAC engine, HEVC
 * CABAC, core clock, core FSM) but NOT the AXI/AHB/CCU bus resets.
 * Resetting the bus drops the IOMMU's page-directory state because
 * the rkvdec_ccu and rkvdec0_a/h resets sit on the same bus path —
 * empirically observed: dec_bus_sta IRQ + DTE_ADDR readback=0 after
 * a wider reset.
 *
 * CON40 bit layout (per RK3588 BSP DTS):
 *   bit 2  rkvdec_ccu_a    — cluster AXI       (DO NOT reset)
 *   bit 3  rkvdec_ccu_h    — cluster AHB       (DO NOT reset)
 *   bit 4  rkvdec0_a       — codec AXI         (DO NOT reset)
 *   bit 5  rkvdec0_h       — codec AHB         (DO NOT reset)
 *   bit 6  ca_rkvdec0      — CABAC engine      ✓
 *   bit 7  hevc_ca_rkvdec0 — HEVC CABAC        ✓
 *   bit 8  rkvdec0_c       — core clock        ✓
 *   bit 9  rkvdec0_core    — core FSM          ✓ */
#define RDCC_CON40_CORE_RESET_BIT  0x000003C0u  /* bits 6..9 */
#define RDCC_CON41_CORE_RESET_BIT  0x000001C0u  /* bits 6..8 (RVD1) */

/* Defined in driver.c.  g_rdcc_mmio is the ACPI MMIO (Phase 3 task arb).
 * g_cru_mmio is a direct physical-address map of the system CRU; we use
 * that for clock and reset control in Phase 2. */
extern volatile UCHAR *g_rdcc_mmio;
extern volatile UCHAR *g_cru_mmio;
extern LONG            g_raise_refcount;

/* Hi-word-mask write helper.  Bits set in `mask` (lower 16) get updated to
 * the corresponding bit in `value`; other bits are unaffected.  The upper
 * 16 bits of the actual MMIO write are the mask itself. */
static FORCEINLINE void
RkCcuHiwordWrite(_In_ volatile UCHAR *base, _In_ ULONG offset,
                 _In_ ULONG mask, _In_ ULONG value)
{
    ULONG word = (mask << 16) | (value & mask);
    WRITE_REGISTER_ULONG((volatile ULONG*)(base + offset), word);
}

NTSTATUS RkMppCcuQueryVersion(_Out_ PUINT32 v)
{
    *v = RKMPP_CCU_IFC_VERSION;
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuRaiseCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    if (InterlockedIncrement(&g_raise_refcount) != 1) return STATUS_SUCCESS;

    /* Linux pm_domains.c per-domain sequence:
     *   ungate the PD's clocks → bus-idle deassert → PMU power-on.
     * The bus-idle ack only updates when the bus's own clocks are running,
     * so clocks come BEFORE the PMU handshake.  We unfold per-domain order:
     *
     *   PD_VCODEC      (no clocks, no idle)
     *   ungate VDPU root clocks (CON44 b0..2)
     *   PD_VDPU        (idle handshake + pwr)
     *   ungate rkvdec0 cluster clocks (CON40 mask 0x3FF)
     *   PD_RKVDEC0     (idle handshake + pwr)
     *   ungate rkvdec1 cluster clocks (CON41 mask 0x1CF)
     *   PD_RKVDEC1     (idle handshake + pwr)
     *   deassert resets
     */
    NTSTATUS s = RkMppPmuPowerOn(&g_pdVcodec);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_VCODEC power-on failed 0x%08x\n", s);
        InterlockedDecrement(&g_raise_refcount);
        return s;
    }

    /* Ungate VDPU root clocks BEFORE PD_VDPU's bus-idle handshake. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44, g_rdcc.ClkGateCon44Mask, 0);

    /* Deassert VDPU NIU resets (CON44 bits 4..6).  The VDPU NIU sits between
     * PD_VDPU and downstream PDs (RKVDEC0/1, IEP, RGA, ...).  UEFI leaves
     * these asserted; downstream codec MMIO reads SError on a NIU in reset
     * even after the rkvdec0 NIU is brought up. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon44, g_rdcc.SoftRstCon44Mask, 0);

    s = RkMppPmuPowerOn(&g_pdVdpu);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_VDPU power-on failed 0x%08x\n", s);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44,
                         g_rdcc.ClkGateCon44Mask, g_rdcc.ClkGateCon44Mask);
        RkMppPmuPowerOff(&g_pdVcodec);
        InterlockedDecrement(&g_raise_refcount);
        return s;
    }

    /* Configure rkvdec0 root + leaf clocks BEFORE ungating their gates and
     * before the PD_RKVDEC0 idle handshake.  Three CLKSEL_CON registers:
     *
     *   CON(89): aclk_rkvdec_ccu / aclk_rkvdec0_root / hclk_rkvdec0_root
     *   CON(90): clk_rkvdec0_ca + clk_rkvdec0_hevc_ca (CABAC + HEVC CABAC)
     *   CON(91): clk_rkvdec0_core
     *
     * ACLK_RKVDEC_CCU's COMPOSITE mux must select a real PLL parent or the
     * clock won't tick — the rkvdec0 NIU needs it ticking to ack
     * idle-deassert.
     *
     * Leaves (core, ca, hevc_ca) need rate setup too: BSP kernel sets them
     * to 200/200/300 MHz at probe.  Without that they'd run at 24 MHz OSC
     * default, fast enough for register access but too slow for an actual
     * decode within our 200 ms poll budget — the symptom is "kick fires,
     * INT_STATUS reads 0 forever".  Pick CPLL/8 = 187 MHz for core/ca and
     * MATRIX_1000M/3 = 333 MHz for hevc_ca; close enough to BSP targets. */
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON89,
                     RDCC_CRU_CLKSEL_CON89_MASK, RDCC_CRU_CLKSEL_CON89_VALUE);
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON90,
                     RDCC_CRU_CLKSEL_CON90_MASK, RDCC_CRU_CLKSEL_CON90_VALUE);
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON91,
                     RDCC_CRU_CLKSEL_CON91_MASK, RDCC_CRU_CLKSEL_CON91_VALUE);

    /* Ungate rkvdec0 cluster clocks BEFORE PD_RKVDEC0 handshake. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40, g_rdcc.ClkGateCon40Mask, 0);

    /* CRITICAL: deassert SOFTRST_CON(40) bits 2..9 BEFORE the PD_RKVDEC0
     * idle handshake.  UEFI leaves the rkvdec_ccu / rkvdec0 / rkvdec0_NIU
     * resets ASSERTED (we observed SOFTRST_CON40 = 0x000003fc on probe).
     * A NIU in reset cannot advance its bus-idle FSM, so the PMU
     * idle-deassert ack stays stuck — exactly what we were timing out on.
     *
     * Originally we deasserted these resets at the *end* of RaiseCluster
     * (after all PD power-ons), but the rkvdec0 power-on never returns
     * because of this very issue.  Order matters. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40, g_rdcc.SoftRstCon40Mask, 0);

    s = RkMppPmuPowerOn(&g_pdRkvdec0);
    if (!NT_SUCCESS(s)) {
        ULONG g40 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + 0x8A0));
        ULONG g44 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + 0x8B0));
        ULONG s89 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + 0x4A4));
        ULONG r40 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + 0xAA0));
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_RKVDEC0 power-on failed 0x%08x "
                   "CLKGATE40=0x%08x CLKGATE44=0x%08x CLKSEL89=0x%08x SOFTRST40=0x%08x\n",
                   s, g40, g44, s89, r40);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                         g_rdcc.ClkGateCon40Mask, g_rdcc.ClkGateCon40Mask);
        RkMppPmuPowerOff(&g_pdVdpu);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44,
                         g_rdcc.ClkGateCon44Mask, g_rdcc.ClkGateCon44Mask);
        RkMppPmuPowerOff(&g_pdVcodec);
        InterlockedDecrement(&g_raise_refcount);
        return s;
    }

    /* Ungate rkvdec1 cluster clocks BEFORE PD_RKVDEC1 handshake. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41, g_rdcc.ClkGateCon41Mask, 0);

    /* Same reset-before-handshake fix for rkvdec1 group (CON41 bits 2..8). */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41, g_rdcc.SoftRstCon41Mask, 0);

    s = RkMppPmuPowerOn(&g_pdRkvdec1);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_RKVDEC1 power-on failed 0x%08x\n", s);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                         g_rdcc.ClkGateCon41Mask, g_rdcc.ClkGateCon41Mask);
        RkMppPmuPowerOff(&g_pdRkvdec0);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                         g_rdcc.ClkGateCon40Mask, g_rdcc.ClkGateCon40Mask);
        RkMppPmuPowerOff(&g_pdVdpu);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44,
                         g_rdcc.ClkGateCon44Mask, g_rdcc.ClkGateCon44Mask);
        RkMppPmuPowerOff(&g_pdVcodec);
        InterlockedDecrement(&g_raise_refcount);
        return s;
    }

    /* Resets were deasserted earlier (before each PD's idle handshake) —
     * the NIU has to be out of reset for the PMU bus-idle ack to fire. */

    /* Codec MMIO at 0xFDC30000 / 0xFDC38xxx / 0xFDC48xxx is now reachable. */
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDropCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_SUCCESS;
    if (InterlockedDecrement(&g_raise_refcount) != 0) return STATUS_SUCCESS;

    /* Reverse of RaiseCluster, child-first:
     *   reassert resets → power off PD_RKVDEC1 → gate rkvdec1 clocks
     *   → power off PD_RKVDEC0 → gate rkvdec0 clocks
     *   → power off PD_VDPU → gate VDPU root clocks
     *   → power off PD_VCODEC */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     g_rdcc.SoftRstCon40Mask, g_rdcc.SoftRstCon40Mask);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41,
                     g_rdcc.SoftRstCon41Mask, g_rdcc.SoftRstCon41Mask);

    RkMppPmuPowerOff(&g_pdRkvdec1);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                     g_rdcc.ClkGateCon41Mask, g_rdcc.ClkGateCon41Mask);

    RkMppPmuPowerOff(&g_pdRkvdec0);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                     g_rdcc.ClkGateCon40Mask, g_rdcc.ClkGateCon40Mask);

    RkMppPmuPowerOff(&g_pdVdpu);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon44,
                     g_rdcc.SoftRstCon44Mask, g_rdcc.SoftRstCon44Mask);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44,
                     g_rdcc.ClkGateCon44Mask, g_rdcc.ClkGateCon44Mask);

    RkMppPmuPowerOff(&g_pdVcodec);
    return STATUS_SUCCESS;
}

/* Hang-recovery reset toggles for the RVD0 core only.  RaiseCluster
 * deasserts the full reset bundle on bring-up; these helpers target
 * just the CORE bit (CON40 bit 9) so a stuck job can be killed without
 * teardown of bus/cabac/HEVC clocks.  RVD1 needs a per-core ifc
 * extension (Phase 3b later task). */
NTSTATUS RkMppCcuAssertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     RDCC_CON40_CORE_RESET_BIT, RDCC_CON40_CORE_RESET_BIT);
    KeStallExecutionProcessor(20);  /* 20 µs — hardware latches the reset */
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     RDCC_CON40_CORE_RESET_BIT, 0);
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}

/* RkMppCcuFullCoreReset — wide hang-recovery reset.  Mirrors Linux
 * `rkvdec2_reset` CRU-bundle path that fires when soft-reset times out:
 * PMU idle req → assert {NIU_a/h, AXI/AHB, CABAC, HEVC_CABAC, CORE} →
 * udelay(5) → deassert in reverse → PMU idle release.
 *
 * This resets the AXI/AHB bus blocks the IOMMU sits on, so DTE_ADDR is
 * cleared as a side effect.  CALLER MUST follow with
 * RKIOMMU_INTERFACE::Reattach to reprogram DTE_ADDR before any further
 * codec activity, or AXI traffic will land at iova 0 / undefined phys.
 *
 * Only RVD0 covered today (CON40 group); RVD1 is a Phase 3b extension. */
NTSTATUS RkMppCcuFullCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;

    /* Wide reset bundle for the rkvdec0 cluster:
     *   CON40 bits 2..9 — rkvdec_ccu_a/h, rkvdec0_a/h, ca, hevc_ca, c, core
     *   CON44 bits 4..6 — VDPU NIU resets (aclk/aclk_low/hclk_NIU)
     * Frame the entire bundle with a PMU bus-idle request so any
     * in-flight AXI traffic on PD_RKVDEC0 gets quiesced before the
     * resets land — otherwise the bus could trap on a half-issued
     * burst when the AXI block itself is held in reset. */
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
               "rkmpp_ccu: FullCoreReset — wide CRU reset for RVD0\n");

    /* Quiesce the PD_RKVDEC0 bus.  Best-effort: if the bus is wedged
     * the ack will time out, but we proceed anyway (the reset itself
     * is what unwedges it). */
    (void)RkMppPmuIdleRequest(&g_pdRkvdec0, TRUE);

    /* Assert the wide reset bundle. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     g_rdcc.SoftRstCon40Mask, g_rdcc.SoftRstCon40Mask);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon44,
                     g_rdcc.SoftRstCon44Mask, g_rdcc.SoftRstCon44Mask);
    KeStallExecutionProcessor(5);

    /* Deassert.  Order doesn't matter for hi-word-mask writes (each
     * register is independent), but we mirror Linux's reverse-order
     * convention: NIU first, then cluster. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon44,
                     g_rdcc.SoftRstCon44Mask, 0);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     g_rdcc.SoftRstCon40Mask, 0);
    KeStallExecutionProcessor(20);

    /* Release the bus-idle request. */
    (void)RkMppPmuIdleRequest(&g_pdRkvdec0, FALSE);

    return STATUS_SUCCESS;
}
