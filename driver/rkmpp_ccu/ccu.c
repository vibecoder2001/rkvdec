/* driver/rkmpp_ccu/ccu.c — RKCP3503 (RDCC) clock/reset/power.
 *
 * Register offsets sourced from rockchip-linux BSP kernel,
 * commit 44d4aaaa9843057b32946f595948387db152be01 at
 *   drivers/clk/rockchip/clk-rk3588.c
 * Header values (reset IDs) from:
 *   include/dt-bindings/clock/rk3588-cru.h
 * (via memory: linux_rkvdec_source_refs.md)
 *
 * CRU base (from rk3588s.dtsi):  0xfd7c0000, size 0x5c000
 * Register offset formulas (from clk.h / softrst.c):
 *   RK3588_CLKGATE_CON(n) = n*4 + 0x800
 *   RK3588_SOFTRST_CON(n) = n*4 + 0xa00  (16 resets/reg, HIWORD_MASK)
 *
 * Clock-gate register CLKGATE_CON(40) = offset 0x8A0 (from CRU base):
 *   bit 0  hclk_rkvdec0_root
 *   bit 1  aclk_rkvdec0_root
 *   bit 2  aclk_rkvdec_ccu       <- CCU bus clock
 *   bit 7  clk_rkvdec0_ca
 *   bit 8  clk_rkvdec0_hevc_ca
 *   bit 9  clk_rkvdec0_core
 *
 * Clock-gate register CLKGATE_CON(41) = offset 0x8A4:
 *   bit 0  hclk_rkvdec1_root
 *   bit 1  aclk_rkvdec1_root
 *   bit 6  clk_rkvdec1_ca
 *   bit 7  clk_rkvdec1_hevc_ca
 *   bit 8  clk_rkvdec1_core
 *
 * Soft-reset register SOFTRST_CON(40) = offset 0xAA0 (16 resets/reg):
 *   reset_id 640..655 -> CON(40), bits 0..15
 *   SRST_A_RKVDEC_CCU = 642 -> bit 2
 *   SRST_H_RKVDEC0    = 643 -> bit 3
 *   SRST_A_RKVDEC0    = 644 -> bit 4
 *   SRST_RKVDEC0_CORE = 649 -> bit 9
 *
 * Soft-reset register SOFTRST_CON(41) = offset 0xAA4:
 *   reset_id 656..671 -> CON(41), bits 0..15
 *   SRST_H_RKVDEC1    = 658 -> bit 2
 *   SRST_A_RKVDEC1    = 659 -> bit 3
 *   SRST_RKVDEC1_CORE = 664 -> bit 8
 *
 * v1 single-instance note:
 *   For v1 we only service RKCP3503 (rkv-decoder-v2 CCU).  The RDCC_REGS
 *   struct covers rkvdec0 (CON 40).  Extending to rkvdec1 (CON 41) and the
 *   full clock tree is left for v2.
 *
 * Rockchip HIWORD_MASK write convention:
 *   Upper 16 bits of each write are a "write-enable" mask; set bit N in both
 *   halves to change bit N, or set bit N only in the upper half to leave it
 *   unchanged.  READ_REGISTER_ULONG returns the full 32-bit value; only the
 *   lower 16 bits carry live state.  We use plain read-modify-write here
 *   because the HIWORD_MASK convention requires bit-precise upper-half masks
 *   which are harder to express cleanly — and at kernel IRQL the register
 *   window is fully owned by this driver, so RMW is safe.
 *
 * Rockchip clock-gate polarity: 1 = GATED (stopped), 0 = UNGATED (running).
 *   RaiseCluster clears gate bits; DropCluster sets them.
 * Rockchip soft-reset polarity:  1 = RESET ASSERTED, 0 = DEASSERTED.
 */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ccu_ifc.h"

/* CRU register offsets from the CRU MMIO base (0xfd7c0000).
 * All offsets are byte offsets; registers are 32-bit.
 */
#define RDCC_CRU_CLKGATE_CON40   0x8A0u  /* CLKGATE_CON(40): rkvdec0 + CCU */
#define RDCC_CRU_SOFTRST_CON40   0xAA0u  /* SOFTRST_CON(40): rkvdec0 resets */

typedef struct _RDCC_REGS {
    ULONG ClkGate;        /* CRU clock-gate register offset (from CRU base) */
    ULONG ClkGateMask;    /* bits we own in ClkGate (1 = may gate, 0 = leave) */
    ULONG SoftReset;      /* CRU soft-reset register offset (from CRU base)  */
    ULONG SoftResetMask;  /* bits we own in SoftReset */
} RDCC_REGS;

static const RDCC_REGS g_rdcc = {
    /* CLKGATE_CON(40) offset 0x8A0:
     *   bit 0 hclk_rkvdec0_root, bit 1 aclk_rkvdec0_root, bit 2 aclk_rkvdec_ccu
     *   Gate all three to fully idle the cluster.
     */
    .ClkGate       = RDCC_CRU_CLKGATE_CON40,
    .ClkGateMask   = 0x00000007u,  /* bits 0+1+2 */

    /* SOFTRST_CON(40) offset 0xAA0:
     *   bit 9 SRST_RKVDEC0_CORE (reset_id 649)
     *   Assert/deassert only the core reset for v1.
     */
    .SoftReset     = RDCC_CRU_SOFTRST_CON40,
    .SoftResetMask = 0x00000200u,  /* bit 9 */
};

extern volatile UCHAR *g_rdcc_mmio;
extern LONG            g_raise_refcount;

NTSTATUS RkMppCcuQueryVersion(_Out_ PUINT32 v)
{
    *v = RKMPP_CCU_IFC_VERSION;
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuRaiseCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_rdcc_mmio) return STATUS_DEVICE_NOT_READY;
    if (InterlockedIncrement(&g_raise_refcount) == 1) {
        /* Clear gate bits → ungate (clock running). */
        ULONG v = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate));
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate),
            v & ~g_rdcc.ClkGateMask);
    }
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDropCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_rdcc_mmio) return STATUS_SUCCESS;
    if (InterlockedDecrement(&g_raise_refcount) == 0) {
        /* Set gate bits → gate (clock stopped). */
        ULONG v = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate));
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate),
            v | g_rdcc.ClkGateMask);
    }
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuAssertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_rdcc_mmio) return STATUS_DEVICE_NOT_READY;
    /* Set reset bits → assert reset (core held in reset). */
    ULONG v = READ_REGISTER_ULONG(
        (volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset));
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset),
        v | g_rdcc.SoftResetMask);
    KeStallExecutionProcessor(20);  /* 20 µs — hardware must latch the reset */
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_rdcc_mmio) return STATUS_DEVICE_NOT_READY;
    /* Clear reset bits → deassert (core running). */
    ULONG v = READ_REGISTER_ULONG(
        (volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset));
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset),
        v & ~g_rdcc.SoftResetMask);
    return STATUS_SUCCESS;
}
