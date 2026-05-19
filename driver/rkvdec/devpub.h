/* driver/rkmpp/devpub.h — public device-context fields shared between
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

/* Multi-core dispatch — RVD1 (UID==1) publishes a PEER_WORKER interface
 * that RVD0 consumes.  See driver/rkvdec/peer_worker.c. */
struct _WDF_OBJECT_HANDLE;
NTSTATUS RkMppPeerWorkerPublish(_In_ WDFDEVICE Device, _In_ UINT32 Uid);
VOID     RkMppPeerWorkerNotifyQueryRemove(VOID);

/* Called from peer_worker.c when RVD0 connects via RegisterCompletion.
 * Triggers a lazy Iommu.Enable + flips IommuAttached=TRUE on RVD1 so
 * slave attachment that completed asynchronously after RVD1's
 * PrepareHardware becomes visible.  Idempotent. */
VOID RkMppPeerOnRvd0Connected(_In_ WDFDEVICE Device);
