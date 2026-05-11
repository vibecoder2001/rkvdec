/* driver/rkvdec/profile.c — codec profile for rkvdec.sys (RKCP3550 only). */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ioctl.h"
#include "profile.h"

static const RKMPP_PROFILE g_profiles[] = {
    { 0x3550, 0, RKMPP_CODEC_H264 | RKMPP_CODEC_HEVC, 0x0100, RKMPP_PERSONALITY_RKVDEC2 },
    { 0x3550, 1, RKMPP_CODEC_H264 | RKMPP_CODEC_HEVC, 0x0100, RKMPP_PERSONALITY_RKVDEC2 },
};

const RKMPP_PROFILE*
RkMppFindProfile(_In_ UINT32 Hid, _In_ UINT32 Uid)
{
    for (ULONG i = 0; i < ARRAYSIZE(g_profiles); i++) {
        if (g_profiles[i].Hid == Hid && g_profiles[i].Uid == Uid)
            return &g_profiles[i];
    }
    return NULL;
}
