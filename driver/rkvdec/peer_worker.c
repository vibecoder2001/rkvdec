/* driver/rkvdec/peer_worker.c — RVD1's PEER_WORKER interface provider.
 *
 * Phase 1 (Task 1.5): published the interface with stub bodies.
 * Phase 3 (Task 3.2): KickJob queues a work item that calls
 *                     RkMppJobRunForeign on RVD1's local device. */
#include <initguid.h>
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_peer_worker_ifc.h"
#include "../../shared/rkmpp_ioctl.h"
#include "devpub.h"
#include "job.h"

/* Mirror of RKVDEC2_REG_DEC_E_BIT in job.c (kept private there to avoid
 * polluting job.h's API surface).  Review finding #7 uses this to gate
 * zero-kick peer submissions; keep the two definitions in sync. */
#ifndef RKVDEC2_REG_DEC_E_BIT
#define RKVDEC2_REG_DEC_E_BIT 0x1u
#endif

typedef struct _RKMPP_PEER_PROVIDER_CTX {
    WDFDEVICE                       Device;
    PVOID                           CompletionCtx;
    RKMPP_PEER_COMPLETION_CB        CompletionCb;
    PVOID                           QueryRemoveCtx;
    RKMPP_PEER_QUERY_REMOVE_CB      QueryRemoveCb;
    KSPIN_LOCK                      Lock;
    LONG volatile                   RefCount;
} RKMPP_PEER_PROVIDER_CTX;

static RKMPP_PEER_PROVIDER_CTX g_PeerProvider;

static VOID PeerIfcReference(PVOID c)   { InterlockedIncrement(&((RKMPP_PEER_PROVIDER_CTX*)c)->RefCount); }
static VOID PeerIfcDereference(PVOID c) { InterlockedDecrement(&((RKMPP_PEER_PROVIDER_CTX*)c)->RefCount); }

static NTSTATUS
PeerRegisterCompletion(PVOID provCtx, PVOID consCtx, RKMPP_PEER_COMPLETION_CB cb)
{
    RKMPP_PEER_PROVIDER_CTX *p = provCtx;
    KIRQL irql;
    KeAcquireSpinLock(&p->Lock, &irql);
    p->CompletionCtx = consCtx;
    p->CompletionCb  = cb;
    WDFDEVICE selfDev = p->Device;
    KeReleaseSpinLock(&p->Lock, irql);

    /* RVD0 just connected to us — retry our lazy Iommu.Enable in case
     * slave PT wasn't yet attached when RVD1's PrepareHardware ran.
     * Skip when cb is NULL (RVD0 is disconnecting, not connecting). */
    if (cb && selfDev) {
        RkMppPeerOnRvd0Connected(selfDev);
    }
    return STATUS_SUCCESS;
}

static NTSTATUS
PeerRegisterQueryRemove(PVOID provCtx, PVOID consCtx, RKMPP_PEER_QUERY_REMOVE_CB cb)
{
    RKMPP_PEER_PROVIDER_CTX *p = provCtx;
    KIRQL irql;
    KeAcquireSpinLock(&p->Lock, &irql);
    p->QueryRemoveCtx = consCtx;
    p->QueryRemoveCb  = cb;
    KeReleaseSpinLock(&p->Lock, irql);
    return STATUS_SUCCESS;
}

/* Per-kick work item.  Allocated by PeerKickJob, freed by the worker
 * after RkMppJobRunForeign returns and the consumer CB has been
 * invoked.  All caller-supplied buffers are copied here so PeerKickJob
 * can return immediately.
 *
 * IoWorkItem is allocated via IoAllocateWorkItem (not the deprecated
 * ExInitializeWorkItem / ExQueueWorkItem pair). */
typedef struct _PEER_KICK_WORK {
    PIO_WORKITEM            IoWorkItem;
    UINT32                  KickValue;
    UINT32                  IovaSlotCount;
    UINT64                  CompletionCookie;
    RKMPP_DENSE_BANK        BankCopy;
    RKMPP_DENSE_IOVA_SLOT   IovaCopy[RKMPP_MAX_DENSE_IOVA_SLOTS];
} PEER_KICK_WORK;

/* Trampoline matching RKMPP_FOREIGN_COMPLETION_CB.  Forwards to the
 * consumer's RKMPP_PEER_COMPLETION_CB under provider lock. */
static VOID
PeerForeignCompletionTrampoline(_In_ PVOID    CbCtx,
                                _In_ UINT64   CompletionCookie,
                                _In_ NTSTATUS Result,
                                _In_ UINT32   HardwareStatus)
{
    UNREFERENCED_PARAMETER(CbCtx);
    RKMPP_PEER_PROVIDER_CTX *p = &g_PeerProvider;
    KIRQL irql;
    KeAcquireSpinLock(&p->Lock, &irql);
    RKMPP_PEER_COMPLETION_CB cb = p->CompletionCb;
    PVOID consCtx               = p->CompletionCtx;
    KeReleaseSpinLock(&p->Lock, irql);
    if (cb) cb(consCtx, CompletionCookie, Result, HardwareStatus);
}

static IO_WORKITEM_ROUTINE PeerKickWorker;
static VOID
PeerKickWorker(_In_ PDEVICE_OBJECT DeviceObject, _In_opt_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(DeviceObject);
    PEER_KICK_WORK *w = (PEER_KICK_WORK*)Ctx;
    if (!w) return;

    /* RkMppJobRunForeign blocks until completion, then calls our
     * trampoline before returning. */
    (void)RkMppJobRunForeign(g_PeerProvider.Device,
                             &w->BankCopy,
                             sizeof(w->BankCopy),
                             w->KickValue,
                             w->IovaCopy,
                             w->IovaSlotCount,
                             w->CompletionCookie,
                             PeerForeignCompletionTrampoline,
                             NULL);

    IoFreeWorkItem(w->IoWorkItem);
    ExFreePoolWithTag(w, 'kWPK');
}

static NTSTATUS
PeerKickJob(PVOID provCtx, const RKMPP_PEER_KICK_PARAMS *params)
{
    UNREFERENCED_PARAMETER(provCtx);

    if (!params || !params->Bank) return STATUS_INVALID_PARAMETER;
    if (params->BankBytes != sizeof(RKMPP_DENSE_BANK)) return STATUS_INVALID_PARAMETER;
    if (params->IovaSlotCount > RKMPP_MAX_DENSE_IOVA_SLOTS) return STATUS_INVALID_PARAMETER;
    /* Reject zero-kick: SubmitDense gates this at the IOCTL boundary,
     * but a buggy/hostile peer consumer could synthesise one and the
     * resulting synchronous JobComplete → KickPromotions loop has no
     * recursion bound (same hazard as the original SubmitDense gate
     * — see [[critical_kick_recursion]]).  Review finding #7. */
    if ((params->KickValue & RKVDEC2_REG_DEC_E_BIT) == 0) {
        return STATUS_INVALID_PARAMETER;
    }

    PDEVICE_OBJECT devObj = WdfDeviceWdmGetDeviceObject(g_PeerProvider.Device);
    if (!devObj) return STATUS_DEVICE_NOT_READY;

    PEER_KICK_WORK *w = (PEER_KICK_WORK*)ExAllocatePool2(POOL_FLAG_NON_PAGED,
                                                         sizeof(*w), 'kWPK');
    if (!w) return STATUS_INSUFFICIENT_RESOURCES;
    RtlZeroMemory(w, sizeof(*w));

    w->IoWorkItem = IoAllocateWorkItem(devObj);
    if (!w->IoWorkItem) {
        ExFreePoolWithTag(w, 'kWPK');
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    RtlCopyMemory(&w->BankCopy, params->Bank, sizeof(w->BankCopy));
    if (params->IovaSlotCount) {
        RtlCopyMemory(w->IovaCopy, params->IovaSlots,
                      params->IovaSlotCount * sizeof(RKMPP_DENSE_IOVA_SLOT));
    }
    w->KickValue        = params->KickValue;
    w->IovaSlotCount    = params->IovaSlotCount;
    w->CompletionCookie = params->CompletionCookie;

    IoQueueWorkItem(w->IoWorkItem, PeerKickWorker, DelayedWorkQueue, w);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppPeerWorkerPublish(_In_ WDFDEVICE Device, _In_ UINT32 Uid)
{
    if (Uid != 1) return STATUS_SUCCESS;

    RtlZeroMemory(&g_PeerProvider, sizeof(g_PeerProvider));
    g_PeerProvider.Device = Device;
    KeInitializeSpinLock(&g_PeerProvider.Lock);

    RKMPP_PEER_WORKER_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    ifc.Header.Size                = sizeof(ifc);
    ifc.Header.Version             = RKMPP_PEER_WORKER_IFC_VERSION;
    ifc.Header.Context             = &g_PeerProvider;
    ifc.Header.InterfaceReference  = PeerIfcReference;
    ifc.Header.InterfaceDereference= PeerIfcDereference;
    ifc.Hid                        = 0x3550;
    ifc.Uid                        = 1;
    ifc.RegisterCompletion         = PeerRegisterCompletion;
    ifc.KickJob                    = PeerKickJob;
    ifc.RegisterQueryRemove        = PeerRegisterQueryRemove;

    WDF_QUERY_INTERFACE_CONFIG qic;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&qic, (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKMPP_PEER_WORKER, NULL);
    NTSTATUS s = WdfDeviceAddQueryInterface(Device, &qic);
    if (!NT_SUCCESS(s)) return s;
    return WdfDeviceCreateDeviceInterface(Device,
                                          &GUID_DEVINTERFACE_RKMPP_PEER_WORKER, NULL);
}

VOID RkMppPeerWorkerNotifyQueryRemove(VOID)
{
    KIRQL irql;
    RKMPP_PEER_QUERY_REMOVE_CB cb = NULL;
    PVOID ctx = NULL;
    KeAcquireSpinLock(&g_PeerProvider.Lock, &irql);
    cb  = g_PeerProvider.QueryRemoveCb;
    ctx = g_PeerProvider.QueryRemoveCtx;
    KeReleaseSpinLock(&g_PeerProvider.Lock, irql);
    if (cb) cb(ctx);
}
