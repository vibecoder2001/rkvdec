/* driver/rkav1d/profile.c — codec profile for rkav1d.sys (RKCP3560 only). */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ioctl.h"
#include "profile.h"

static const RKMPP_PROFILE g_profiles[] = {
    { 0x3560, 0, RKMPP_CODEC_AV1, 0x0000, RKMPP_PERSONALITY_AV1D },
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
