/* driver/rkmpp_ccu/pmu.c — RK3588 PMU power-domain control for rkvdec.
 *
 * Bring-up sequence sourced from linux-rockchip pm_domains.c — see
 * memory phase3b_codec_bringup_table.md.  PMU base 0xFD8D8000.
 *
 * **rk3588 PMU IS hi-word-mask.**  Per-domain `pwr_w_mask = pwr << 16` and
 * `req_w_mask = req << 16` are set in the BSP table (DOMAIN_M_O_R_G macro),
 * and Linux uses regmap_write with `(mask << 16) | value` for both pwr and
 * req registers.  An earlier RMW attempt was wrong direction.
 *
 * **"Domain on" status check** uses the **repair_status_offset (0x290)**
 * for domains where `repair_status_mask != 0`, with bit-set = on (1=on).
 * Linux's rockchip_pmu_domain_is_on:
 *
 *   if (info->repair_status_mask) {
 *       regmap_read(repair_status_offset, &val);
 *       return val & info->repair_status_mask;
 *   }
 *   if (info->status_mask == 0) return !is_idle(pd);
 *   regmap_read(status_offset, &val);
 *   return !(val & info->status_mask);
 *
 * For RK3588 codec PDs:
 *   PD_VCODEC : repair_status_mask=0, status_mask=BIT(2)
 *               → poll status_offset (0x180) for bit 2 CLEAR
 *   PD_VDPU   : repair_status_mask=BIT(9)
 *               → poll repair_status_offset (0x290) for bit 9 SET
 *   PD_RKVDEC0: repair_status_mask=BIT(7) → poll 0x290 bit 7 SET
 *   PD_RKVDEC1: repair_status_mask=BIT(8) → poll 0x290 bit 8 SET
 *
 * This is the third-time-lucky version: the previous two builds had the
 * status check reading the wrong register or wrong polarity, so PowerOn
 * timed out even when the hardware actually came up.
 */
#include "pmu.h"

extern volatile UCHAR *g_pmu_mmio;

/* PMU register offsets relative to RKMPP_PMU_PHYS_BASE (0xFD8D8000). */
#define RK3588_PMU_PWR_GATE_CON         0x14Cu  /* pwr-off bits          */
#define RK3588_PMU_BUS_IDLE_REQ         0x10Cu  /* idle-request bits     */
#define RK3588_PMU_BUS_IDLE_ACK         0x118u  /* idle-ack (read-only)  */
#define RK3588_PMU_BUS_IDLE_ST          0x120u  /* idle-status (RO)      */
#define RK3588_PMU_PWR_GATE_STATUS      0x180u  /* status_offset (RO)    */
#define RK3588_PMU_PWR_REPAIR_STATUS    0x290u  /* repair_status_offset  */

/* PD_VCODEC — VD_VCODEC voltage parent.  No idle handshake.
 *   pwr=BIT(2) at 0x14C
 *   status_mask=BIT(2) at 0x180; bit clear = on. */
const RKMPP_PMU_DOMAIN g_pdVcodec = {
    .PwrOffset    = RK3588_PMU_PWR_GATE_CON,
    .PwrBit       = (1u << 2),
    .StatusOffset = RK3588_PMU_PWR_GATE_STATUS,
    .StatusBit    = (1u << 2),  /* polled for clear (0 = on) */
    .RepairStatusBit = 0,        /* not used for VCODEC      */
    .IdleReqOffset = 0,          /* no idle handshake        */
    .IdleReqBit    = 0,
    .IdleAckOffset = 0, .IdleAckBit = 0,
    .IdleStOffset  = 0, .IdleStBit  = 0,
};

/* PD_VDPU — VD_LOGIC voltage parent for codec PDs.
 *   pwr=BIT(10), repair_status=BIT(9) (1=on)
 *   req=idle=BIT(8). */
const RKMPP_PMU_DOMAIN g_pdVdpu = {
    .PwrOffset       = RK3588_PMU_PWR_GATE_CON,
    .PwrBit          = (1u << 10),
    .StatusOffset    = 0,                /* repair path */
    .StatusBit       = 0,
    .RepairStatusBit = (1u << 9),        /* polled for set (1 = on) */
    .IdleReqOffset = RK3588_PMU_BUS_IDLE_REQ,
    .IdleReqBit    = (1u << 8),
    .IdleAckOffset = RK3588_PMU_BUS_IDLE_ACK,
    .IdleAckBit    = (1u << 8),
    .IdleStOffset  = RK3588_PMU_BUS_IDLE_ST,
    .IdleStBit     = (1u << 8),
};

/* PD_RKVDEC0:
 *   pwr=BIT(8), repair_status=BIT(7) (1=on)
 *   req=idle=BIT(6). */
const RKMPP_PMU_DOMAIN g_pdRkvdec0 = {
    .PwrOffset       = RK3588_PMU_PWR_GATE_CON,
    .PwrBit          = (1u << 8),
    .StatusOffset    = 0,
    .StatusBit       = 0,
    .RepairStatusBit = (1u << 7),
    .IdleReqOffset = RK3588_PMU_BUS_IDLE_REQ,
    .IdleReqBit    = (1u << 6),
    .IdleAckOffset = RK3588_PMU_BUS_IDLE_ACK,
    .IdleAckBit    = (1u << 6),
    .IdleStOffset  = RK3588_PMU_BUS_IDLE_ST,
    .IdleStBit     = (1u << 6),
};

/* PD_RKVDEC1:
 *   pwr=BIT(9), repair_status=BIT(8) (1=on)
 *   req=idle=BIT(7). */
const RKMPP_PMU_DOMAIN g_pdRkvdec1 = {
    .PwrOffset       = RK3588_PMU_PWR_GATE_CON,
    .PwrBit          = (1u << 9),
    .StatusOffset    = 0,
    .StatusBit       = 0,
    .RepairStatusBit = (1u << 8),
    .IdleReqOffset = RK3588_PMU_BUS_IDLE_REQ,
    .IdleReqBit    = (1u << 7),
    .IdleAckOffset = RK3588_PMU_BUS_IDLE_ACK,
    .IdleAckBit    = (1u << 7),
    .IdleStOffset  = RK3588_PMU_BUS_IDLE_ST,
    .IdleStBit     = (1u << 7),
};

/* Hi-word-mask write: upper 16 = bit-enable mask, lower 16 = value.
 * Per-domain pwr_w_mask = pwr << 16, so the value 0 clears the bit and the
 * value (mask) sets the bit.  Same convention for req. */
static FORCEINLINE void
PmuHiwordWrite(_In_ ULONG offset, _In_ ULONG mask, _In_ ULONG value)
{
    ULONG word = (mask << 16) | (value & mask);
    WRITE_REGISTER_ULONG((volatile ULONG*)(g_pmu_mmio + offset), word);
}

/* Returns TRUE when the domain is currently on. */
static BOOLEAN PmuDomainIsOn(_In_ const RKMPP_PMU_DOMAIN *D)
{
    if (D->RepairStatusBit) {
        ULONG val = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_pmu_mmio + RK3588_PMU_PWR_REPAIR_STATUS));
        return (val & D->RepairStatusBit) != 0;  /* 1 = on */
    }
    if (D->StatusBit) {
        ULONG val = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_pmu_mmio + D->StatusOffset));
        return (val & D->StatusBit) == 0;  /* 0 = on */
    }
    return TRUE;  /* no status mechanism — assume on */
}

NTSTATUS RkMppPmuPowerOn(_In_ const RKMPP_PMU_DOMAIN *D)
{
    if (!g_pmu_mmio) return STATUS_DEVICE_NOT_READY;

    /* Step 1: clear pwr-off bit via hi-word-mask write (value 0). */
    PmuHiwordWrite(D->PwrOffset, D->PwrBit, 0);

    /* Step 2: wait for is-on status. */
    for (ULONG i = 0; i < 10000; i++) {
        if (PmuDomainIsOn(D)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                       "rkmpp_ccu: PD pwrBit=0x%x powered on after %u µs\n",
                       D->PwrBit, i);
            goto powered_on;
        }
        KeStallExecutionProcessor(1);
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp_ccu: power-on status timeout (PD pwrBit=0x%x)\n", D->PwrBit);
    return STATUS_DEVICE_HARDWARE_ERROR;

powered_on:
    /* Step 3: if domain has bus-idle handshake, deassert it now. */
    if (D->IdleReqOffset == 0) return STATUS_SUCCESS;

    PmuHiwordWrite(D->IdleReqOffset, D->IdleReqBit, 0);

    /* Wait for ack to clear (Linux: target_ack=0 when idle=false). */
    for (ULONG i = 0; i < 1000; i++) {
        ULONG ack = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_pmu_mmio + D->IdleAckOffset));
        if ((ack & D->IdleAckBit) == 0) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                       "rkmpp_ccu: PD pwrBit=0x%x bus active after %u µs\n",
                       D->PwrBit, i);
            return STATUS_SUCCESS;
        }
        KeStallExecutionProcessor(1);
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp_ccu: bus-idle-ack timeout (PD pwrBit=0x%x)\n", D->PwrBit);
    return STATUS_DEVICE_HARDWARE_ERROR;
}

NTSTATUS RkMppPmuPowerOff(_In_ const RKMPP_PMU_DOMAIN *D)
{
    if (!g_pmu_mmio) return STATUS_DEVICE_NOT_READY;

    /* Step 1: assert idle-req (if applicable) and wait for ack. */
    if (D->IdleReqOffset != 0) {
        PmuHiwordWrite(D->IdleReqOffset, D->IdleReqBit, D->IdleReqBit);

        for (ULONG i = 0; i < 1000; i++) {
            ULONG ack = READ_REGISTER_ULONG(
                (volatile ULONG*)(g_pmu_mmio + D->IdleAckOffset));
            if ((ack & D->IdleAckBit) != 0) goto bus_idled;
            KeStallExecutionProcessor(1);
        }
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: bus-idle-ack-on timeout (PD pwrBit=0x%x)\n",
                   D->PwrBit);
        return STATUS_DEVICE_HARDWARE_ERROR;
bus_idled:;
    }

    /* Step 2: set pwr-off bit (hi-word-mask write with value=mask). */
    PmuHiwordWrite(D->PwrOffset, D->PwrBit, D->PwrBit);
    return STATUS_SUCCESS;
}
