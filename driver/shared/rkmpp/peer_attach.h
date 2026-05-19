/* driver/shared/rkmpp/peer_attach.h — PnP arrival/removal notification
 * helper.  Both rkvdec (peer worker open) and rkiommu_vdec (slave →
 * master attach) use this to handle late-start and runtime install/
 * uninstall without blocking PrepareHardware.  Spec: "Start-order
 * races". */
#pragma once
#include <ntddk.h>
#include <wdf.h>

typedef VOID (*RKMPP_PEER_ARRIVAL_FN)(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink);
typedef VOID (*RKMPP_PEER_REMOVAL_FN)(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink);

typedef struct _RKMPP_PEER_WATCH {
    PVOID                  NotificationEntry;   /* IoRegisterPlugPlayNotification handle */
    PVOID                  Ctx;
    RKMPP_PEER_ARRIVAL_FN  OnArrival;
    RKMPP_PEER_REMOVAL_FN  OnRemoval;           /* may be NULL */
} RKMPP_PEER_WATCH;

/* Register a notification on the given device interface GUID.  Fires
 * once synchronously for every currently-present instance, then on
 * future arrivals/removals.  Caller MUST eventually call
 * RkMppUnwatchPeer (typically in EvtReleaseHardware). */
NTSTATUS RkMppWatchPeer(_In_  const GUID            *InterfaceGuid,
                        _In_  PVOID                  Ctx,
                        _In_  RKMPP_PEER_ARRIVAL_FN  OnArrival,
                        _In_opt_ RKMPP_PEER_REMOVAL_FN OnRemoval,
                        _Out_ RKMPP_PEER_WATCH      *Watch);

VOID RkMppUnwatchPeer(_Inout_ RKMPP_PEER_WATCH *Watch);
