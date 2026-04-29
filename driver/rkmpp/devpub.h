/* driver/rkmpp/devpub.h — public device-context fields shared between
 * device.c and ioctl.c.  Neither file needs access to the full RKMPP_DEVICE
 * context; only these four words are exported across the boundary.
 */
#pragma once

typedef struct _RKMPP_DEVICE_PUBLIC {
    UINT32 Hid;
    UINT32 Uid;
    UINT32 RevisionWord;
    UINT32 SupportedCodecs;
} RKMPP_DEVICE_PUBLIC;
