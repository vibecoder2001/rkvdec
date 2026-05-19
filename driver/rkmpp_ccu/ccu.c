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
#include "../shared/rkmpp_log.h"
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
 * Per linux-rockchip clk-rk3588.c the COMPOSITE clocks share this register:
 *
 *   [15:14] aclk_rkvdec_ccu   mux (gpll|cpll|aupll|spll) → 0 = GPLL
 *   [13:9 ] aclk_rkvdec_ccu   div (actual_div − 1)         → 1 = /2 (594 MHz)
 *   [8:7  ] aclk_rkvdec0_root mux (gpll|cpll|aupll|npll)  → 2 = AUPLL
 *   [6:2  ] aclk_rkvdec0_root div (actual_div − 1)         → 0 = /1 (786 MHz)
 *   [1:0  ] hclk_rkvdec0_root mux (200m|100m|50m|24m)      → 0 = 200 MHz
 *
 * 2026-05-12: rates updated to match BSP / RVD1 (UEFI-set) defaults.
 * Previous values gave aclk_rkvdec0_root at gpll/2 = 594 MHz (vs BSP 786)
 * and the leaf clocks at ~1/4 BSP.  The rate mismatch correlated with the
 * RVD0 H.265 / H.264 wedge under per-kick gate cycling: at the slow
 * leaf rates the gate cycle's CDC settle window after ungate is rate-
 * relative and didn't fit the codec FSM's recovery window. */
#define RDCC_CRU_CLKSEL_CON89    0x464u
#define RDCC_CRU_CLKSEL_CON89_MASK   0xFFFFu
#define RDCC_CRU_CLKSEL_CON89_VALUE  ((1u << 9) | (2u << 7))   /* 0x0300 */

/* CLKSEL_CON(90) = 0x300 + 90*4 = 0x468 — leaf clocks for CABAC + HEVC CABAC.
 * Per linux-rockchip clk-rk3588.c:
 *   bits  4..0  clk_rkvdec0_ca      div → 1 = /2 (594 MHz from gpll)
 *   bit      5  clk_rkvdec0_ca      mux → 0 = gpll
 *   bits 10..6  clk_rkvdec0_hevc_ca div → 0 = /1 (1000 MHz from 1000m_src)
 *   bits 12..11 clk_rkvdec0_hevc_ca mux → 3 = 1000m_src
 *
 * 2026-05-12: rates updated to match BSP / RVD1.  Previous values gave
 * 148 / 396 MHz, slow enough that the per-kick CON40 leaf-clock gate
 * cycle's CDC settle window exceeded the codec FSM's tolerance. */
#define RDCC_CRU_CLKSEL_CON90    0x468u
#define RDCC_CRU_CLKSEL_CON90_MASK   0x1FFFu
#define RDCC_CRU_CLKSEL_CON90_VALUE  ((1u << 0) | (3u << 11))  /* 0x1801 */

/* CLKSEL_CON(91) = 0x300 + 91*4 = 0x46C — clk_rkvdec0_core.
 *   bits 4..0  div → 1 = /2 (594 MHz)
 *   bit     5  mux → 0 = gpll
 *
 * 2026-05-12: rate updated from gpll/8 (148 MHz) to gpll/2 (594 MHz)
 * to match BSP / RVD1.  See CON90 comment. */
#define RDCC_CRU_CLKSEL_CON91    0x46Cu
#define RDCC_CRU_CLKSEL_CON91_MASK   0x3Fu
#define RDCC_CRU_CLKSEL_CON91_VALUE  (1u << 0)                 /* 0x0001 */

/* AV1 decoder clock + reset registers (jammy-branch BSP):
 *   CLKSEL_CON(163) at 0x300 + 163*4 = 0x58C
 *     bits [4:0]   ACLK_AV1 divider  (actual_div − 1)
 *     bits [6:5]   ACLK_AV1 mux      (0=gpll | 1=cpll | 2=aupll)
 *     bits [8:7]   PCLK_AV1_ROOT mux (0=200m | 1=100m | 2=50m | 3=24m)
 *   (Earlier comment claimed bit 7 overlapped between the two muxes —
 *   wrong; per BSP clk-rk3588.c aclk mux is bits[6:5], pclk mux is
 *   bits[8:7], no overlap.)
 *   CLKGATE_CON(68) at 0x800 + 68*4 = 0x910
 *     bit 0  ACLK_AV1_ROOT  bit 3  PCLK_AV1_ROOT
 *   SOFTRST_CON(68) at 0xa00 + 68*4 = 0xB10
 *     bit 1  SRST_A_AV1_BIU   bit 4  SRST_P_AV1_BIU
 *     bit 2  SRST_A_AV1       bit 5  SRST_P_AV1
 *
 * 2026-05-12: rates updated to match BSP (verified via rk's
 * /sys/kernel/debug/clk/clk_summary): aclk_av1 = gpll/3 = 396 MHz,
 * pclk_av1 = mux 0 = 200 MHz.  Previous values (gpll/4 = 297 MHz +
 * pclk mux 2 = 50 MHz) underclocked aclk by 25 % and pclk by 4×
 * because of a div-off-by-one and the bogus "bit-7 overlap" comment
 * making the author pick `1u << 7` thinking it was "100m mux=1" —
 * actually selects 50m (mux=2). */
#define RDCC_CRU_CLKSEL_CON163        0x58Cu
#define RDCC_CRU_CLKSEL_CON163_MASK   0x01FFu  /* [8:0] */
#define RDCC_CRU_CLKSEL_CON163_VALUE  ((2u << 0) | (0u << 5) | (0u << 7))  /* 0x0002 */

#define RDCC_CRU_CLKGATE_CON68        0x910u
#define RDCC_CRU_CLKGATE_CON68_MASK   ((1u << 0) | (1u << 3))

#define RDCC_CRU_SOFTRST_CON68        0xB10u
#define RDCC_CRU_SOFTRST_CON68_MASK   ((1u << 1) | (1u << 2) | \
                                       (1u << 4) | (1u << 5))

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
 * AssertRvdec0CoreReset / DeassertRvdec0CoreReset (and the matching Rvdec1
 * pair) toggle only the codec-internal sub-block bits (CON40 6..9 for RVD0,
 * CON41 6..8 for RVD1). */
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

/* Per-job reset bundle (used by AssertRvdec0CoreReset / DeassertRvdec0CoreReset).
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

/* AV1 cluster bring-up is independent of the rkvdec0/1 cluster — it
 * has its own PD (PD_AV1) with its own clock + reset bundle.  Use a
 * separate refcount so AV1 device probe doesn't end up as a no-op
 * just because rkvdec0 already raised the rkvdec cluster. */
static LONG g_av1_refcount = 0;
/* Both cluster paths share the VD_LOGIC + VD_VCODEC parents (PD_VCODEC
 * + PD_VDPU).  Track those separately so power-off on one cluster's
 * Drop doesn't yank the rug from under the other. */
static LONG g_parents_refcount = 0;

/* Serializes all Raise/Drop entry points. The previous refcount-only
 * pattern was racy: when two sibling devices (e.g., rkvdec RVD0 + RVD1)
 * raced into Raise on parallel KMDF threads, the second's
 * InterlockedIncrement returned 2 and short-circuited to SUCCESS while
 * the first was still mid-PMU-handshake. The second consumer's MMIO
 * read then landed before its codec PD was powered → WHEA UE.
 *
 * Now all callers run at PASSIVE_LEVEL (per-job Raise/Drop in
 * driver/rkvdec/job.c and driver/rkav1d/job.c removed — they were
 * always short-circuit no-ops nested under the device-lifetime Raise
 * from PrepareHardware, and one of them ran at DISPATCH which blocked
 * a simpler mutex fix).
 *
 * Initialized in DriverEntry. PASSIVE_LEVEL only. */
static FAST_MUTEX g_ccu_mutex;

VOID RkMppCcuInitMutex(VOID)
{
    ExInitializeFastMutex(&g_ccu_mutex);
}

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

/* Parents (PD_VCODEC + PD_VDPU) raise — shared between rkvdec cluster
 * and AV1 cluster bring-up.  Refcounted so the second caller skips the
 * full PMU sequence and just bumps the count.
 *
 * Called with g_ccu_mutex held. */
static NTSTATUS RaiseParents(void)
{
    if (++g_parents_refcount != 1) return STATUS_SUCCESS;

    NTSTATUS s = RkMppPmuPowerOn(&g_pdVcodec);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_VCODEC power-on failed 0x%08x\n", s);
        --g_parents_refcount;
        return s;
    }

    /* VDPU root clocks must tick before its bus-idle handshake; NIU resets
     * sit on the same path so deassert them too. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44, g_rdcc.ClkGateCon44Mask, 0);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon44, g_rdcc.SoftRstCon44Mask, 0);

    s = RkMppPmuPowerOn(&g_pdVdpu);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_VDPU power-on failed 0x%08x\n", s);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44,
                         g_rdcc.ClkGateCon44Mask, g_rdcc.ClkGateCon44Mask);
        RkMppPmuPowerOff(&g_pdVcodec);
        --g_parents_refcount;
        return s;
    }
    return STATUS_SUCCESS;
}

/* Called with g_ccu_mutex held. */
static void DropParents(void)
{
    if (--g_parents_refcount != 0) return;
    RkMppPmuPowerOff(&g_pdVdpu);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon44,
                     g_rdcc.SoftRstCon44Mask, g_rdcc.SoftRstCon44Mask);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon44,
                     g_rdcc.ClkGateCon44Mask, g_rdcc.ClkGateCon44Mask);
    RkMppPmuPowerOff(&g_pdVcodec);
}

NTSTATUS RkMppCcuRaiseCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;

    ExAcquireFastMutex(&g_ccu_mutex);
    if (g_raise_refcount > 0) {
        g_raise_refcount++;
        ExReleaseFastMutex(&g_ccu_mutex);
        return STATUS_SUCCESS;
    }

    /* refcount == 0: do the full Raise. */

    /* Linux pm_domains.c per-domain sequence:
     *   ungate the PD's clocks → bus-idle deassert → PMU power-on.
     * The bus-idle ack only updates when the bus's own clocks are running,
     * so clocks come BEFORE the PMU handshake.  Per-domain order:
     *
     *   parents: PD_VCODEC then PD_VDPU (factored out in RaiseParents)
     *   ungate rkvdec0 cluster clocks (CON40 mask 0x3FF)
     *   PD_RKVDEC0     (idle handshake + pwr)
     *   ungate rkvdec1 cluster clocks (CON41 mask 0x1CF)
     *   PD_RKVDEC1     (idle handshake + pwr)
     */
    NTSTATUS s = RaiseParents();
    if (!NT_SUCCESS(s)) {
        ExReleaseFastMutex(&g_ccu_mutex);
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
     * Leaves (core, ca, hevc_ca) need rate setup too: per
     * /sys/kernel/debug/clk/clk_summary on rk's BSP kernel they run at
     * GPLL/2 = 594 MHz for core + ca and 1000m_src/1 = 1000 MHz for
     * hevc_ca.  Without that they'd run at 24 MHz OSC default, fast
     * enough for register access but too slow for an actual decode
     * within our 200 ms poll budget — the symptom is "kick fires,
     * INT_STATUS reads 0 forever".  An earlier version aimed for an
     * older BSP target (200/200/300 MHz) and undershot to 148/396/148
     * MHz due to mux misinterpretation; that triggered the RVD0 CON40
     * gate-cycle wedge (see h264_bframe_divergence_open.md history). */
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
        DropParents();
        ExReleaseFastMutex(&g_ccu_mutex);
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
        DropParents();
        ExReleaseFastMutex(&g_ccu_mutex);
        return s;
    }

    /* Resets were deasserted earlier (before each PD's idle handshake) —
     * the NIU has to be out of reset for the PMU bus-idle ack to fire. */

    /* Codec MMIO at 0xFDC30000 / 0xFDC38xxx / 0xFDC48xxx is now reachable. */
    g_raise_refcount = 1;
    ExReleaseFastMutex(&g_ccu_mutex);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDropCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_SUCCESS;

    ExAcquireFastMutex(&g_ccu_mutex);
    if (g_raise_refcount > 1) {
        g_raise_refcount--;
        ExReleaseFastMutex(&g_ccu_mutex);
        return STATUS_SUCCESS;
    }
    if (g_raise_refcount == 0) {
        /* Defensive: Drop without matching Raise.  Skip teardown. */
        RKMPP_LOG_WARN(
                   "rkmpp_ccu: DropCluster with refcount==0 — ignored\n");
        ExReleaseFastMutex(&g_ccu_mutex);
        return STATUS_SUCCESS;
    }

    /* refcount == 1: do the full Drop. */

    /* Reverse of RaiseCluster, child-first:
     *   power off PD_RKVDEC1 → power off PD_RKVDEC0
     *   → reassert resets → gate rkvdec1 + rkvdec0 clocks
     *   → DropParents (handles VDPU + VCODEC ref-counted)
     *
     * PowerOff BEFORE SOFTRST assert — mirrors Raise's deassert-before-PowerOn
     * discipline. The BIU NIU has to be out of reset to ack the PMU's
     * idle-request assertion; if SOFTRST is asserted first, the idle ack
     * stays stuck and PowerOff times out (10ms), leaving PMU half-configured
     * and wedging the next Raise. The post-PowerOff reassert is belt-and-
     * suspenders to keep CRU state deterministic for the next Raise's
     * deassert. Status from PowerOff is logged but not propagated — Drop
     * must complete teardown even on timeout or refcounts go out of sync. */
    NTSTATUS s1 = RkMppPmuPowerOff(&g_pdRkvdec1);
    if (!NT_SUCCESS(s1)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_RKVDEC1 power-off failed 0x%08x\n", s1);
    }

    NTSTATUS s0 = RkMppPmuPowerOff(&g_pdRkvdec0);
    if (!NT_SUCCESS(s0)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_RKVDEC0 power-off failed 0x%08x\n", s0);
    }

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     g_rdcc.SoftRstCon40Mask, g_rdcc.SoftRstCon40Mask);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41,
                     g_rdcc.SoftRstCon41Mask, g_rdcc.SoftRstCon41Mask);

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                     g_rdcc.ClkGateCon41Mask, g_rdcc.ClkGateCon41Mask);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                     g_rdcc.ClkGateCon40Mask, g_rdcc.ClkGateCon40Mask);

    DropParents();
    g_raise_refcount = 0;
    ExReleaseFastMutex(&g_ccu_mutex);
    return STATUS_SUCCESS;
}

/* AV1 cluster bring-up — separate ref-counted path from rkvdec0/1.
 * Sequence:
 *   parents (PD_VCODEC + PD_VDPU)
 *   configure ACLK_AV1_ROOT mux+div / PCLK_AV1_ROOT mux (CLKSEL_CON163)
 *   ungate CLKGATE_CON(68) bits 0,3
 *   deassert SOFTRST_CON(68) bits 1,2,4,5
 *   PD_AV1 power on (idle handshake + pwr clear)
 */
NTSTATUS RkMppCcuRaiseAv1Cluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;

    ExAcquireFastMutex(&g_ccu_mutex);
    if (g_av1_refcount > 0) {
        g_av1_refcount++;
        ExReleaseFastMutex(&g_ccu_mutex);
        return STATUS_SUCCESS;
    }

    /* refcount == 0: do the full Raise. */
    NTSTATUS s = RaiseParents();
    if (!NT_SUCCESS(s)) {
        ExReleaseFastMutex(&g_ccu_mutex);
        return s;
    }

    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON163,
                     RDCC_CRU_CLKSEL_CON163_MASK, RDCC_CRU_CLKSEL_CON163_VALUE);
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKGATE_CON68,
                     RDCC_CRU_CLKGATE_CON68_MASK, 0);
    /* Deassert AV1 BIU + AV1 + PCLK_AV1_BIU + PCLK_AV1 resets BEFORE the
     * PD_AV1 idle handshake — same reasoning as the rkvdec0/1 reset
     * ordering: the BIU NIU has to be out of reset for its idle FSM to
     * advance and ack the deassert. */
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_SOFTRST_CON68,
                     RDCC_CRU_SOFTRST_CON68_MASK, 0);

    s = RkMppPmuPowerOn(&g_pdAv1);
    if (!NT_SUCCESS(s)) {
        ULONG g68 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + RDCC_CRU_CLKGATE_CON68));
        ULONG s163 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + RDCC_CRU_CLKSEL_CON163));
        ULONG r68 = READ_REGISTER_ULONG((volatile ULONG*)(g_cru_mmio + RDCC_CRU_SOFTRST_CON68));
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_AV1 power-on failed 0x%08x "
                   "CLKGATE68=0x%08x CLKSEL163=0x%08x SOFTRST68=0x%08x\n",
                   s, g68, s163, r68);
        RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKGATE_CON68,
                         RDCC_CRU_CLKGATE_CON68_MASK,
                         RDCC_CRU_CLKGATE_CON68_MASK);
        DropParents();
        ExReleaseFastMutex(&g_ccu_mutex);
        return s;
    }
    g_av1_refcount = 1;
    ExReleaseFastMutex(&g_ccu_mutex);
    return STATUS_SUCCESS;
}

/* Reverse of RaiseAv1Cluster:
 *   power off PD_AV1 → reassert SOFTRST_CON68 → gate CLKGATE_CON68
 *   → DropParents.
 *
 * PowerOff BEFORE SOFTRST assert — mirrors Raise's deassert-before-PowerOn
 * discipline (see ccu.c:449). The AV1 BIU NIU has to be out of reset to
 * ack the PMU's idle-request assertion; if SOFTRST is asserted first the
 * idle ack stays stuck and PowerOff times out (10ms), leaving PMU half-
 * configured and wedging the next Raise. Status from PowerOff is logged
 * but not propagated — Drop must complete teardown even on timeout or
 * the refcount goes out of sync. */
NTSTATUS RkMppCcuDropAv1Cluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_SUCCESS;

    ExAcquireFastMutex(&g_ccu_mutex);
    if (g_av1_refcount > 1) {
        g_av1_refcount--;
        ExReleaseFastMutex(&g_ccu_mutex);
        return STATUS_SUCCESS;
    }
    if (g_av1_refcount == 0) {
        /* Defensive: Drop without matching Raise.  Skip teardown. */
        RKMPP_LOG_WARN(
                   "rkmpp_ccu: DropAv1Cluster with refcount==0 — ignored\n");
        ExReleaseFastMutex(&g_ccu_mutex);
        return STATUS_SUCCESS;
    }

    /* refcount == 1: do the full Drop. */
    NTSTATUS s = RkMppPmuPowerOff(&g_pdAv1);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_AV1 power-off failed 0x%08x\n", s);
    }

    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_SOFTRST_CON68,
                     RDCC_CRU_SOFTRST_CON68_MASK, RDCC_CRU_SOFTRST_CON68_MASK);
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKGATE_CON68,
                     RDCC_CRU_CLKGATE_CON68_MASK, RDCC_CRU_CLKGATE_CON68_MASK);
    DropParents();
    g_av1_refcount = 0;
    ExReleaseFastMutex(&g_ccu_mutex);
    return STATUS_SUCCESS;
}

/* Per-D0-state AV1 leaf-clock gate.  PD_AV1 stays raised; only the
 * CLKGATE_CON68 bits toggle.  Called from rkav1d EvtDeviceD0Exit/D0Entry.
 * Not refcounted — paired 1:1 with the matching Ungate/Gate call.
 * RaiseAv1Cluster/DropAv1Cluster keep their own refcount for the
 * long-lived PD-up state established at PrepareHardware. */
NTSTATUS RkMppCcuGateAv1LeafClocks(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Set gate bits = stop the AV1 leaf clocks. */
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKGATE_CON68,
                     RDCC_CRU_CLKGATE_CON68_MASK,
                     RDCC_CRU_CLKGATE_CON68_MASK);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuUngateAv1LeafClocks(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Clear gate bits = restart the AV1 leaf clocks. */
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKGATE_CON68,
                     RDCC_CRU_CLKGATE_CON68_MASK, 0);
    return STATUS_SUCCESS;
}

/* RKVDEC0 leaf-clock gate bits in CLKGATE_CON40:
 *   bit 3  hclk_rkvdec0       (codec hclk leaf — child of hclk_rkvdec0_root)
 *   bit 4  aclk_rkvdec0       (codec aclk leaf — child of aclk_rkvdec0_root)
 *   bit 7  clk_rkvdec0_ca     (CABAC)
 *   bit 8  clk_rkvdec0_hevc_ca (HEVC CABAC)
 *   bit 9  clk_rkvdec0_core   (decode pipeline)
 *
 * Mapping from BSP rkvdec2_clk_off / clk_on:
 *   dec->aclk_info.clk        → bit 4   (DTS "aclk_rkvdec0")
 *   dec->hclk_info.clk        → bit 3   (DTS "hclk_rkvdec0")
 *   dec->core_clk_info.clk    → bit 9
 *   dec->cabac_clk_info.clk   → bit 7
 *   dec->hevc_cabac_clk_info  → bit 8
 *
 * Verified against upstream drivers/clk/rockchip/clk-rk3588.c gate
 * descriptors (CLK_RKVDEC0_CA / _HEVC_CA / _CORE at bits 7/8/9 of
 * RK3588_CLKGATE_CON(40); HCLK_RKVDEC0 / ACLK_RKVDEC0 at bits 3/4).
 *
 * Root clocks at bits 0,1,2 (hclk_rkvdec0_root / aclk_rkvdec0_root /
 * aclk_rkvdec_ccu) are deliberately EXCLUDED — they are refcounted
 * shared parents in CCF and BSP does not toggle them per task.
 *
 * Bits 3 and 4 (HCLK / ACLK) are ALSO deliberately EXCLUDED from the
 * per-kick gate mask: they are the AHB/AXI bus clocks shared with the
 * rkvdec0 IOMMU (HID=RKCP3570 UID=9, RD0M).  Gating them between kicks
 * leaves the IOMMU's MMIO unreachable — any user-mode-triggered
 * MapMdl call (IOCTL_RKMPP_ALLOC_BUFFER) lands on gated MMIO and
 * silently fails: writes get cached on the bus, reads return the
 * last-written value (classic gated-MMIO signature, e.g. STATUS and
 * INT_MASK both echoing the last DTE_ADDR write in pre-enable
 * readback logs).  Linux BSP rkvdec2_clk_off does gate these bits,
 * but Linux IOMMU operations are bracketed by pm_runtime_get_sync /
 * pm_runtime_put which transparently re-ungates them; our Windows
 * IOCTL path has no such bracket, so bits 3,4 must stay UNGATED
 * across runtime until a queuing/deferring layer is added.
 *
 * Codec MMIO between kicks: not accessed.  JobStart ungates BEFORE
 * the register-list writes and the kick; the poller polls INT_STATUS
 * while clocks are on; JobComplete gates AFTER the poller exits.
 * Bits 7,8,9 are codec-internal (CABAC, HEVC CABAC, core) and no
 * other consumer touches them between kicks, so gating them per-kick
 * is safe. */
#define RDCC_CON40_LEAF_GATE_MASK  0x00000380u  /* bits 7,8,9 */

/* RKVDEC1 leaf-clock gate bits in CLKGATE_CON41:
 *   bit 0  hclk_rkvdec1_root  (cluster AHB root  — DO NOT toggle per-kick)
 *   bit 1  aclk_rkvdec1_root  (cluster AXI root  — DO NOT toggle per-kick)
 *   bit 2  hclk_rkvdec1       (codec hclk leaf   — child of *_root)
 *   bit 3  aclk_rkvdec1       (codec aclk leaf   — child of *_root)
 *   bit 6  clk_rkvdec1_ca     (CABAC)
 *   bit 7  clk_rkvdec1_hevc_ca (HEVC CABAC)
 *   bit 8  clk_rkvdec1_core   (decode pipeline)
 *
 * Verified against upstream drivers/clk/rockchip/clk-rk3588.c:
 *   CLK_RKVDEC1_CA / _HEVC_CA / _CORE at bits 6/7/8 of CLKGATE_CON(41)
 *     (clk-rk3588.c:1616-1624);
 *   HCLK_RKVDEC1 / ACLK_RKVDEC1 at bits 2/3 of CLKGATE_CON(41)
 *     (clk-rk3588.c:2336-2339);
 *   hclk_rkvdec1_root / aclk_rkvdec1_root at bits 0/1
 *     (clk-rk3588.c:1610-1615).
 *
 * Layout exactly mirrors CON40 with a -1 shift on the codec-internal
 * bits (CON40 7,8,9 → CON41 6,7,8) and a -1 shift on the bus leaves
 * (CON40 3,4 → CON41 2,3).
 *
 * Mapping from BSP rkvdec2_clk_off / clk_on (per-device clock list —
 * see mpp_rkvdec2.c:1074-1095):
 *   dec->aclk_info.clk        → bit 3   (DTS "aclk_rkvdec1")
 *   dec->hclk_info.clk        → bit 2   (DTS "hclk_rkvdec1")
 *   dec->core_clk_info.clk    → bit 8
 *   dec->cabac_clk_info.clk   → bit 6
 *   dec->hevc_cabac_clk_info  → bit 7
 *
 * Bus clocks (bits 2,3) are deliberately EXCLUDED from the per-kick
 * gate mask — same reason as CON40 bits 3,4 (see commit bdd6f3c):
 * they are shared with the rkvdec1 IOMMU's MMIO bus.  Gating them
 * between kicks leaves the IOMMU MMIO unreachable to MapMdl /
 * IOCTL_RKMPP_ALLOC_BUFFER, silently corrupting DTE programming.
 * Linux BSP rkvdec2_clk_off does gate these, but pm_runtime_get_sync /
 * put brackets IOMMU operations there; Windows has no such bracket.
 *
 * Codec-internal bits 6,7,8 (CABAC / HEVC CABAC / core) are safe to
 * cycle per-kick: no other consumer touches them between kicks. */
#define RDCC_CON41_LEAF_GATE_MASK  0x000001C0u  /* bits 6,7,8 (RVD1) */

NTSTATUS RkMppCcuGateRvdec0LeafClocks(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Set gate bits = stop the RVD0 leaf clocks. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                     RDCC_CON40_LEAF_GATE_MASK,
                     RDCC_CON40_LEAF_GATE_MASK);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuUngateRvdec0LeafClocks(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Clear gate bits = restart the RVD0 leaf clocks. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                     RDCC_CON40_LEAF_GATE_MASK, 0);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuGateRvdec1LeafClocks(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Set gate bits = stop the RVD1 leaf clocks. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                     RDCC_CON41_LEAF_GATE_MASK,
                     RDCC_CON41_LEAF_GATE_MASK);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuUngateRvdec1LeafClocks(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Clear gate bits = restart the RVD1 leaf clocks. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                     RDCC_CON41_LEAF_GATE_MASK, 0);
    return STATUS_SUCCESS;
}

/* Hang-recovery reset toggles for the rkvdec0/1 cores.  RaiseCluster
 * deasserts the full reset bundle on bring-up; these helpers target
 * just the core / CABAC / HEVC CABAC bits (CON40 / CON41 bits 6..9 /
 * 6..8 respectively) so a stuck job can be killed without teardown of
 * bus/cluster clocks.  Per BSP mpp_rkvdec2.c:1074-1095 each device
 * resets only its own bits; v7 mirrors that with per-instance methods. */
NTSTATUS RkMppCcuAssertRvdec0CoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     RDCC_CON40_CORE_RESET_BIT, RDCC_CON40_CORE_RESET_BIT);
    KeStallExecutionProcessor(20);  /* 20 µs — hardware latches the reset */
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertRvdec0CoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     RDCC_CON40_CORE_RESET_BIT, 0);
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuAssertRvdec1CoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41,
                     RDCC_CON41_CORE_RESET_BIT, RDCC_CON41_CORE_RESET_BIT);
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertRvdec1CoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41,
                     RDCC_CON41_CORE_RESET_BIT, 0);
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}

/* RkMppCcuFullCoreReset0 — wide hang-recovery reset for RVD0
 * (renamed from FullCoreReset in v8).  Mirrors Linux `rkvdec2_reset`
 * CRU-bundle path that fires when soft-reset times out: PMU idle req →
 * assert CON40 bundle → udelay(5) → deassert → PMU idle release.
 *
 * v8 NOTE: SOFTRST_CON44 NIU bits (4..6 — VDPU aclk/aclk_low/hclk NIU)
 * are NO LONGER touched here.  That NIU is shared between all
 * VDPU-child codecs, and resetting it from a single codec's
 * hang-recovery would disrupt any peer codec mid-decode.  If a
 * coordinated wide NIU reset is ever needed, it will be a separate
 * ifc method gated on all-codecs-idle.
 *
 * This still resets the AXI/AHB bus blocks the IOMMU sits on (those
 * bits live inside the CON40 bundle, not CON44), so DTE_ADDR is
 * cleared as a side effect.  CALLER MUST follow with
 * RKIOMMU_INTERFACE::Reattach to reprogram DTE_ADDR before any further
 * codec activity, or AXI traffic will land at iova 0 / undefined phys. */
NTSTATUS RkMppCcuFullCoreReset0(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;

    RKMPP_LOG_WARN(
               "rkmpp_ccu: FullCoreReset0 — power-cycle PD_RKVDEC0 + wide CRU reset\n");

    /* Empirical recipe (proven to unwedge by CCU driver reinstall):
     * full PD power-cycle around the SOFTRST bundle.  The earlier
     * "PMU bus-idle only" sequence let the codec's internal FSM
     * survive the SOFTRST toggle (clock domain stayed powered the
     * whole time), so dec_e=1 stuck states persisted.  Power-cycling
     * PD_RKVDEC0 forces the entire codec island through reset.
     *
     * Sequence mirrors Drop(rkvdec0-only) → Raise(rkvdec0-only)
     * extracted from RaiseCluster/DropCluster:
     *   1. PowerOff PD_RKVDEC0  (idle handshake → PMU pwr_off)
     *   2. Assert SOFTRST_CON40 mask  (rkvdec0 group, no shared CON44)
     *   3. Reapply CLKSEL_CON89/90/91  (defensive — sticky in practice)
     *   4. Ungate CLKGATE_CON40 mask
     *   5. Deassert SOFTRST_CON40 mask
     *   6. PowerOn PD_RKVDEC0  (PMU pwr_off=0 → idle deassert)
     *
     * Serialised with RaiseCluster/DropCluster via g_ccu_mutex. */
    ExAcquireFastMutex(&g_ccu_mutex);

    NTSTATUS s = RkMppPmuPowerOff(&g_pdRkvdec0);
    if (!NT_SUCCESS(s)) {
        RKMPP_LOG_WARN(
                   "rkmpp_ccu: FullCoreReset0 PowerOff failed 0x%08x — "
                   "continuing reset anyway\n", s);
    }

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     g_rdcc.SoftRstCon40Mask, g_rdcc.SoftRstCon40Mask);
    KeStallExecutionProcessor(20);

    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON89,
                     RDCC_CRU_CLKSEL_CON89_MASK, RDCC_CRU_CLKSEL_CON89_VALUE);
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON90,
                     RDCC_CRU_CLKSEL_CON90_MASK, RDCC_CRU_CLKSEL_CON90_VALUE);
    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_CLKSEL_CON91,
                     RDCC_CRU_CLKSEL_CON91_MASK, RDCC_CRU_CLKSEL_CON91_VALUE);

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                     g_rdcc.ClkGateCon40Mask, 0);

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40,
                     g_rdcc.SoftRstCon40Mask, 0);
    KeStallExecutionProcessor(20);

    s = RkMppPmuPowerOn(&g_pdRkvdec0);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: FullCoreReset0 PowerOn failed 0x%08x\n", s);
    }

    ExReleaseFastMutex(&g_ccu_mutex);
    return s;
}

/* RkMppCcuFullCoreReset1 — wide hang-recovery reset for RVD1 (v8 new).
 * Same shape as FullCoreReset0 but targets PD_RKVDEC1 + SOFTRST_CON41.
 * No shared SOFTRST_CON44 bits touched — see FullCoreReset0 comment.
 * Caller must follow with RKIOMMU_INTERFACE::Reattach. */
NTSTATUS RkMppCcuFullCoreReset1(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;

    RKMPP_LOG_WARN(
               "rkmpp_ccu: FullCoreReset1 — power-cycle PD_RKVDEC1 + wide CRU reset\n");

    /* Mirror of FullCoreReset0 for the CON41 / PD_RKVDEC1 group.  No
     * shared CON44 NIU bits touched.  See FullCoreReset0 for the
     * sequence rationale (PD power-cycle is what the empirical
     * "CCU driver reinstall unwedges" recipe boils down to). */
    ExAcquireFastMutex(&g_ccu_mutex);

    NTSTATUS s = RkMppPmuPowerOff(&g_pdRkvdec1);
    if (!NT_SUCCESS(s)) {
        RKMPP_LOG_WARN(
                   "rkmpp_ccu: FullCoreReset1 PowerOff failed 0x%08x — "
                   "continuing reset anyway\n", s);
    }

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41,
                     g_rdcc.SoftRstCon41Mask, g_rdcc.SoftRstCon41Mask);
    KeStallExecutionProcessor(20);

    /* RVD1 leaf clocks (CON93/94) are UEFI-set to BSP-equivalent
     * values; we never re-program them.  No CLKSEL reapply needed
     * for the RVD1 path (matches RaiseCluster for rkvdec1). */

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                     g_rdcc.ClkGateCon41Mask, 0);

    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon41,
                     g_rdcc.SoftRstCon41Mask, 0);
    KeStallExecutionProcessor(20);

    s = RkMppPmuPowerOn(&g_pdRkvdec1);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: FullCoreReset1 PowerOn failed 0x%08x\n", s);
    }

    ExReleaseFastMutex(&g_ccu_mutex);
    return s;
}

/* RkMppCcuFullAv1Reset — wide hang-recovery reset for AV1 (v8 new).
 * Same shape as FullCoreReset0 but targets PD_AV1 + SOFTRST_CON68
 * (the CRU bundle RaiseAv1Cluster already drives on bring-up).
 * No shared SOFTRST_CON44 bits touched — see FullCoreReset0 comment.
 * Caller must follow with RKIOMMU_INTERFACE::Reattach. */
NTSTATUS RkMppCcuFullAv1Reset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;

    RKMPP_LOG_WARN(
               "rkmpp_ccu: FullAv1Reset — wide CRU reset for AV1\n");

    (void)RkMppPmuIdleRequest(&g_pdAv1, TRUE);

    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_SOFTRST_CON68,
                     RDCC_CRU_SOFTRST_CON68_MASK, RDCC_CRU_SOFTRST_CON68_MASK);
    KeStallExecutionProcessor(5);

    RkCcuHiwordWrite(g_cru_mmio, RDCC_CRU_SOFTRST_CON68,
                     RDCC_CRU_SOFTRST_CON68_MASK, 0);
    KeStallExecutionProcessor(20);

    (void)RkMppPmuIdleRequest(&g_pdAv1, FALSE);

    return STATUS_SUCCESS;
}
