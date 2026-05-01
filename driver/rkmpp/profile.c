/* driver/rkmpp/profile.c — HID + _UID → per-instance profile */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "profile.h"

static const RKMPP_PROFILE g_profiles[] = {
    /* RVD0 / RVD1 — rkv-decoder-v2 cores. v1 only enables H.264.
     * REVISION lives at MMIO base + 0x100.  MmioBase points at the
     * link region (0xFDC38000) since Phase 3b Task 11; the regs region
     * starts at offset 0x100, and reg064 (the first word of the H.264
     * codec-params bank) is the hardware-ID register on rkvdec2.
     * Empirically returns 0x53813f05 on RK3588. */
    /* RKCP3550 / RVD0 + RVD1: rkv-decoder-v2 cores share the same SWREG
     * layout for H.264 + HEVC (the BSP's trans_tbl_h264d/h265d carry
     * different reg-index lists for iova substitution but the underlying
     * register file is the same; the user-mode regbuilder picks the
     * codec mode in reg009 + reg012).  rkmpp.sys is codec-agnostic on
     * the kick path: it copies the user-mode RKMPP_REG_WRITE list to
     * MMIO verbatim, so the driver doesn't need a per-codec gate.
     * Advertise both. */
    { 0x3550, 0, RKMPP_CODEC_H264 | RKMPP_CODEC_HEVC, 0x0100 },
    { 0x3550, 1, RKMPP_CODEC_H264 | RKMPP_CODEC_HEVC, 0x0100 },
    /* All other matching HIDs probe but expose no codec capability in Phase 1. */
    { 0x3510, 0, 0, 0x0000 },
    { 0x3511, 0, 0, 0x0000 },
    { 0x3512, 0, 0, 0x0000 },
    { 0x3520, 0, 0, 0x0000 },
    { 0x3521, 0, 0, 0x0000 },
    { 0x3521, 1, 0, 0x0000 },
    { 0x3521, 2, 0, 0x0000 },
    { 0x3521, 3, 0, 0x0000 },
    { 0x3540, 0, 0, 0x0000 },
    { 0x3540, 1, 0, 0x0000 },
    { 0x3560, 0, 0, 0x0000 },
};

const RKMPP_PROFILE*
RkMppFindProfile(_In_ UINT32 Hid, _In_ UINT32 Uid)
{
    for (ULONG i = 0; i < ARRAYSIZE(g_profiles); i++) {
        if (g_profiles[i].Hid == Hid && g_profiles[i].Uid == Uid) {
            return &g_profiles[i];
        }
    }
    return NULL;
}
