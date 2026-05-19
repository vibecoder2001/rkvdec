/* driver/shared/rkmpp/peer_attach.c — see peer_attach.h */
#include "peer_attach.h"

/* Device interface GUIDs from ntddk.h / PnP specs */
static const GUID GUID_DEVICE_INTERFACE_ARRIVAL_LOCAL = {
    0xCB3A4004, 0x46F0, 0x11D0, { 0xB0, 0x8F, 0x00, 0x60, 0x97, 0x13, 0x05, 0x3F }
};
static const GUID GUID_DEVICE_INTERFACE_REMOVAL_LOCAL = {
    0xCB3A4005, 0x46F0, 0x11D0, { 0xB0, 0x8F, 0x00, 0x60, 0x97, 0x13, 0x05, 0x3F }
};

DRIVER_NOTIFICATION_CALLBACK_ROUTINE RkMppPnpCallback;

_Use_decl_annotations_
NTSTATUS
RkMppPnpCallback(PVOID NotificationStructure, PVOID Context)
{
    RKMPP_PEER_WATCH *w = (RKMPP_PEER_WATCH*)Context;
    if (!w || !NotificationStructure) return STATUS_SUCCESS;

    DEVICE_INTERFACE_CHANGE_NOTIFICATION *n =
        (DEVICE_INTERFACE_CHANGE_NOTIFICATION*)NotificationStructure;

    if (IsEqualGUID(&n->Event, &GUID_DEVICE_INTERFACE_ARRIVAL_LOCAL)) {
        if (w->OnArrival) w->OnArrival(w->Ctx, n->SymbolicLinkName);
    } else if (IsEqualGUID(&n->Event, &GUID_DEVICE_INTERFACE_REMOVAL_LOCAL)) {
        if (w->OnRemoval) w->OnRemoval(w->Ctx, n->SymbolicLinkName);
    }
    return STATUS_SUCCESS;
}

NTSTATUS RkMppWatchPeer(_In_  const GUID            *InterfaceGuid,
                        _In_  PVOID                  Ctx,
                        _In_  RKMPP_PEER_ARRIVAL_FN  OnArrival,
                        _In_opt_ RKMPP_PEER_REMOVAL_FN OnRemoval,
                        _Out_ RKMPP_PEER_WATCH      *Watch)
{
    Watch->Ctx        = Ctx;
    Watch->OnArrival  = OnArrival;
    Watch->OnRemoval  = OnRemoval;
    Watch->NotificationEntry = NULL;

    /* PNPNOTIFY_DEVICE_INTERFACE_INCLUDE_EXISTING_INTERFACES makes the
     * callback fire once for every already-present instance — handles
     * "peer already there at our PrepareHardware time" without a
     * separate enumeration pass. */
    return IoRegisterPlugPlayNotification(
        EventCategoryDeviceInterfaceChange,
        PNPNOTIFY_DEVICE_INTERFACE_INCLUDE_EXISTING_INTERFACES,
        (LPGUID)InterfaceGuid,
        WdfDriverWdmGetDriverObject(WdfGetDriver()),
        RkMppPnpCallback,
        Watch,
        &Watch->NotificationEntry);
}

VOID RkMppUnwatchPeer(_Inout_ RKMPP_PEER_WATCH *Watch)
{
    if (Watch->NotificationEntry) {
        IoUnregisterPlugPlayNotificationEx(Watch->NotificationEntry);
        Watch->NotificationEntry = NULL;
    }
}
