/* driver/rkmpp/profile.h — shared RKMPP_PROFILE definition */
#pragma once

#include <ntddk.h>

typedef struct _RKMPP_PROFILE {
    UINT32 Hid;
    UINT32 Uid;
    UINT32 SupportedCodecs;     /* bitmap from rkmpp_ioctl.h */
    UINT32 RevisionRegOffset;   /* MMIO offset of the REVISION register */
} RKMPP_PROFILE;

const RKMPP_PROFILE* RkMppFindProfile(_In_ UINT32 Hid, _In_ UINT32 Uid);
