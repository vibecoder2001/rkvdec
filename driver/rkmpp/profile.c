/* driver/rkmpp/profile.c — HID + _UID → per-instance profile */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "profile.h"

static const RKMPP_PROFILE g_profiles[] = {
    /* RVD0 / RVD1 — rkv-decoder-v2 cores. v1 only enables H.264.
     * RKVDEC_REG_HW_ID_INDEX=0 → byte offset 0x0000 per
     * rockchip-linux/kernel develop-5.10
     * drivers/video/rockchip/mpp/mpp_rkvdec2.h (c606a84)
     * mpp_read(mpp, reg_id=0) reads MMIO base+0x0000. */
    { 0x3550, 0, RKMPP_CODEC_H264, 0x0000 },
    { 0x3550, 1, RKMPP_CODEC_H264, 0x0000 },
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
