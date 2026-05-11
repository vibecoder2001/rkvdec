/* driver/rkav1d/profile.h — shared RKMPP_PROFILE definition */
#pragma once

#include <ntddk.h>

/* Codec personality.  Selects per-codec MMIO offsets, register-bank walk
 * strategy, kick/IRQ register location, and whether the device has 1 or
 * 3 MMIO windows.  Devices with no SupportedCodecs flags still get a
 * personality so the probe path knows whether to even read MMIO. */
typedef enum _RKMPP_CODEC_PERSONALITY {
    RKMPP_PERSONALITY_NONE   = 0,
    RKMPP_PERSONALITY_RKVDEC2 = 1,  /* H.264 / HEVC / VP9 — vdpu34x family */
    RKMPP_PERSONALITY_AV1D    = 2,  /* AV1 — vdpu family, 3 MMIO windows */
} RKMPP_CODEC_PERSONALITY;

typedef struct _RKMPP_PROFILE {
    UINT32 Hid;
    UINT32 Uid;
    UINT32 SupportedCodecs;     /* bitmap from rkmpp_ioctl.h */
    UINT32 RevisionRegOffset;   /* MMIO offset of the REVISION register */
    RKMPP_CODEC_PERSONALITY Personality;
} RKMPP_PROFILE;

const RKMPP_PROFILE* RkMppFindProfile(_In_ UINT32 Hid, _In_ UINT32 Uid);
