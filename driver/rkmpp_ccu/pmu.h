/* driver/rkmpp_ccu/pmu.h — RK3588 PMU power-domain control for rkvdec.
 *
 * Authoritative source: linux-rockchip device tree.
 *   rk3588s.dtsi `pmu: power-management@fd8d8000` — base address.
 *   drivers/soc/rockchip/pm_domains.c — rk3588_pmu offset constants and
 *                                       rk3588_pm_domains[] table.
 *   See memory: phase3b_codec_bringup_table.md for the full extraction.
 *
 * The PMU at 0xFD8D8000 is NOT exposed via ACPI in this firmware (we author
 * the firmware ourselves and have not yet added _PR0/_ON/_OFF methods).  We
 * map it directly by physical address from driver.c::EvtPrepareHardware
 * (only when HID == RKCP3503), the same pattern used for the system CRU.
 *
 * Phase 3a was off by 0x8000 (mapped 0xFD8D0000) which silently misdirected
 * every PMU write into a different peripheral and explains why codec MMIO
 * SError'd despite "successful" cluster bring-up.
 *
 * Base offsets from rk3588_pmu (struct rockchip_pmu_info):
 *   pwr_offset    = 0x14c  PMU_PWR_GATE_CON      (write hi-word-mask; 1=off, 0=on)
 *   status_offset = 0x180  PMU_PWR_GATE_STATUS   (read-only; 0=on, 1=off)
 *   req_offset    = 0x10c  PMU_BUS_IDLE_REQ      (write hi-word-mask)
 *   ack_offset    = 0x118  PMU_BUS_IDLE_ACK      (read-only)
 *   idle_offset   = 0x120  PMU_BUS_IDLE_ST       (read-only)
 *
 * Per-domain DOMAIN_RK3588 expansions (rk3588_pm_domains[]):
 *   PD_VCODEC : pwr=BIT(2)   status=BIT(2)              no idle handshake
 *   PD_VDPU   : pwr=BIT(10)  r_status=BIT(9)            req=idle=BIT(8)
 *   PD_RKVDEC0: pwr=BIT(8)   r_status=BIT(7)            req=idle=BIT(6)
 *   PD_RKVDEC1: pwr=BIT(9)   r_status=BIT(8)            req=idle=BIT(7)
 *
 * Parent hierarchy from rk3588s.dtsi `power-controller`:
 *   VD_VCODEC  → PD_RKVDEC0 / PD_RKVDEC1   (also includes PD_VENC0/1)
 *   VD_LOGIC   → PD_VDPU → PD_RKVDEC0 / PD_RKVDEC1   (also PD_AV1, RGA, etc.)
 *
 * The rkvdec power domains are declared in BOTH voltage-domain subtrees —
 * the standard Rockchip pattern for codec PDs that straddle two voltage
 * islands.  Both voltage parents must be powered before the bus to
 * PD_RKVDEC0/1 will respond to idle-deassert.  Bring-up order:
 *   PD_VCODEC → PD_VDPU → PD_RKVDEC0 → PD_RKVDEC1
 * Tear-down order is exactly reversed.
 *
 * Note that PD_VCODEC uses the *direct* status mask (status=BIT(2)), while
 * PD_RKVDEC0/1 expose r_status (BIT(7)/BIT(8)) at the same status_offset.
 * Linux's rockchip_pmu_domain_is_on uses `r_status` when `status_mask` is
 * zero — so for our purposes both encode "poll bit X of 0x180 until clear".
 *
 * Sequence per domain, per Linux rockchip_do_pmu_set_power_domain():
 *   power on:  (if req!=0) clear req, wait ack=0, wait idle=0;
 *              clear pwr bit; wait status bit=0.
 *   power off: (if req!=0) assert req, wait ack=1, wait idle=1;
 *              set pwr bit.
 *
 * Hi-word-mask convention applies to all writable registers (pwr, req).
 */
#pragma once
#include <ntddk.h>

#define RKMPP_PMU_PHYS_BASE   0xFD8D8000ULL
#define RKMPP_PMU_MAP_LENGTH  0x1000u

typedef struct _RKMPP_PMU_DOMAIN {
    ULONG PwrOffset;       /* write hi-word-mask; 1=off, 0=on                   */
    ULONG PwrBit;          /* bit mask within PwrOffset register                */

    /* "Domain on" status check.  Two register paths used by RK3588 PDs:
     *   - RepairStatusBit != 0: poll PMU_PWR_REPAIR_STATUS (0x290), bit set = on
     *   - StatusBit       != 0: poll PMU_PWR_GATE_STATUS (0x180), bit clear = on
     * The two are mutually exclusive per domain. */
    ULONG StatusOffset;
    ULONG StatusBit;
    ULONG RepairStatusBit; /* mask in fixed PMU_PWR_REPAIR_STATUS = 0x290       */

    ULONG IdleReqOffset;   /* write hi-word-mask; 1=request idle, 0=unidle.
                            * Zero when this domain has no AXI bus to quiesce
                            * (e.g. PD_VCODEC parent voltage domain).          */
    ULONG IdleReqBit;
    ULONG IdleAckOffset;   /* read-only PMU_BUS_IDLE_ACK = 0x118                */
    ULONG IdleAckBit;
    ULONG IdleStOffset;    /* read-only PMU_BUS_IDLE_ST  = 0x120                */
    ULONG IdleStBit;
} RKMPP_PMU_DOMAIN;

extern const RKMPP_PMU_DOMAIN g_pdVcodec;   /* VD_VCODEC parent — power on FIRST  */
extern const RKMPP_PMU_DOMAIN g_pdVdpu;     /* VD_LOGIC parent — power on SECOND  */
extern const RKMPP_PMU_DOMAIN g_pdRkvdec0;  /* depends on both VCODEC and VDPU    */
extern const RKMPP_PMU_DOMAIN g_pdRkvdec1;  /* depends on both VCODEC and VDPU    */

NTSTATUS RkMppPmuPowerOn (_In_ const RKMPP_PMU_DOMAIN *D);
NTSTATUS RkMppPmuPowerOff(_In_ const RKMPP_PMU_DOMAIN *D);
