/* driver/rkav1d/devpub.h — public device-context fields shared between
 * device.c and ioctl.c.  Neither file needs access to the full RKMPP_DEVICE
 * context; only these exported words are shared across the boundary.
 */
#pragma once

typedef struct _RKMPP_DEVICE_PUBLIC {
    UINT32 Hid;
    UINT32 Uid;
    UINT32 RevisionWord;
    UINT32 SupportedCodecs;
} RKMPP_DEVICE_PUBLIC;

/* Phase 3a: IOMMU fault state snapshot.  All fields are written atomically
 * by RkMppOnIommuFault; read atomically by the IOCTL handler. */
typedef struct _RKMPP_FAULT_STATE {
    LONG   Triggered;    /* 1 if a fault was observed */
    LONG   StatusReg;    /* IOMMU INT_STATUS captured by the ISR */
    LONG64 FaultIova;    /* faulting IOVA captured by the ISR */
} RKMPP_FAULT_STATE;
