/* driver/rkmpp/device.c — per-instance device for rkmpp.sys.
 *
 * Phase 1 responsibilities:
 *   - parse HID + _UID from the ACPI hardware-ID list
 *   - look up the profile
 *   - map MMIO _CRS resource 0
 *   - read the REVISION register and stash it
 *   - register GUID_DEVINTERFACE_RKMPP so user mode can find this instance
 */
#include <initguid.h>
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "../shared/rkmpp_log.h"
#include "profile.h"
#include "devpub.h"
#include "../shared/rkmpp/ifc_client.h"
#include "../shared/rkmpp/bufpool.h"
#include "../shared/acpi_uid.h"
#include "job.h"
#include "../../shared/rkiommu_master_ifc.h"

/* Per-codec memory window count.  rkvdec2 declares 1 or 2 contiguous
 * regions (we merge to one MmioBase).  AV1 declares 3 separate windows
 * (VCD, CACHE, AFBC) at 64 KB-spaced base addresses with un-allocated
 * phys in between, so they CANNOT be merged: we map each independently
 * and look them up via index. */
/* rkvdec2 (link+regs are contiguous in ACPI _CRS so we merge to one
 * MmioBase) — N=1.  Symmetry with rkav1d/device.c (which uses N=3)
 * keeps the device-context shape consistent; the array machinery
 * compiles to a single addressable entry and is no overhead.
 * Review I5: documented as intentional design symmetry rather
 * than removed; review comment suggested dead code but the cost is
 * zero and it makes the two device.c files line-for-line parallel. */
#define RKMPP_MAX_MMIO_WINDOWS 1

typedef struct _RKMPP_MMIO_WINDOW {
    PVOID  Base;       /* virt mapping, NULL if slot unused */
    SIZE_T Length;
    PHYSICAL_ADDRESS Phys;
} RKMPP_MMIO_WINDOW;

typedef struct _RKMPP_DEVICE {
    UINT32                 Hid;
    UINT32                 Uid;
    UINT32                 RevisionWord;
    UINT32                 SupportedCodecs;
    RKMPP_CODEC_PERSONALITY Personality;
    /* Primary MMIO base — first window (rkvdec2: link+regs merged;
     * AV1: VCD).  Job/IOCTL paths still address through MmioBase as
     * the codec's main register window.  Additional windows live in
     * Mmios[1..]. */
    PVOID                  MmioBase;
    SIZE_T                 MmioLength;
    RKMPP_MMIO_WINDOW      Mmios[RKMPP_MAX_MMIO_WINDOWS];
    UINT32                 MmioCount;
    RKMPP_IFC_CLIENT       Ifcs;
    RKMPP_JOB_QUEUE        JobQueue;

    /* Phase 3a: IOMMU fault state — written by RkMppOnIommuFault at
     * DISPATCH_LEVEL, read by IOCTL_RKMPP_INJECT_IOMMU_FAULT at any level. */
    volatile LONG          FaultTriggered;   /* set to 1 by the callback */
    volatile LONG          FaultStatusReg;   /* status reg captured */
    volatile LONG64        FaultIova;        /* iova captured */

    /* Per-kick core reset is harmful — BSP only resets after error/timeout
     * (mpp_common.c:2026: if reset_request > 0 mpp_dev_reset).  We mirror
     * that: 1 before first kick, set after error, cleared after reset. */
    volatile LONG          NeedsCoreReset;
    /* Escalation flag set when a failure follows another recent failure —
     * narrow CoreReset (CON40 bits 6..9 toggle) doesn't recover the codec
     * once dec_e is stuck at 1 (mid-decode wedge).  When set, the next
     * JobStart runs the wider FullCoreReset0/1 (PMU idle + NIU + CABAC +
     * CORE bundle) and re-Attaches the IOMMU (FullCoreResetN zeroes
     * DTE_ADDR as a side effect). */
    volatile LONG          NeedsFullReset;
    /* Tracks whether the most recent JobComplete reported failure, so we
     * can detect "two failures in a row" and escalate. */
    volatile LONG          LastJobFailed;

    /* Phase 4 (Task 4.2): TRUE when our paired rkiommu is operational
     * (master rkiommu has PT-attached us).  Flipped FALSE by
     * RkMppOnMasterIommuQueryRemove when master is being uninstalled.
     * IOCTLs that require IOMMU return STATUS_DEVICE_NOT_READY when
     * this is FALSE — forces MFT to re-open from scratch when master
     * comes back. */
    BOOLEAN                  IommuAttached;
    /* Master rkiommu interface (only opened when our paired rkiommu
     * is master, i.e. RVD0's iommu UID 9 on RK3588).  Used to receive
     * cascade query-remove notifications. */
    RKIOMMU_MASTER_INTERFACE MasterIommuIfc;
    PFILE_OBJECT             MasterIommuFileObj;
    BOOLEAN                  MasterIommuOpen;
} RKMPP_DEVICE, *PRKMPP_DEVICE;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKMPP_DEVICE, RkMppDeviceGet);

/* Phase 4 (Task 4.2): thin accessor so ioctl.c can read IommuAttached
 * without needing to see the full RKMPP_DEVICE definition. */
BOOLEAN
RkMppIsIommuAttached(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return ctx->IommuAttached;
}

LONG
RkMppQueryAndClearNeedsCoreReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return InterlockedExchange(&ctx->NeedsCoreReset, 0);
}

VOID
RkMppSetNeedsCoreReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    InterlockedExchange(&ctx->NeedsCoreReset, 1);
}

LONG
RkMppQueryAndClearNeedsFullReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return InterlockedExchange(&ctx->NeedsFullReset, 0);
}

VOID
RkMppSetNeedsFullReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    InterlockedExchange(&ctx->NeedsFullReset, 1);
    /* Cascade: full reset is a strict superset of core reset, but we
     * also flag the narrow one so any code path that only checks the
     * core flag still triggers SOMETHING. */
    InterlockedExchange(&ctx->NeedsCoreReset, 1);
}

LONG
RkMppExchangeLastJobFailed(_In_ WDFDEVICE Device, LONG NewValue)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return InterlockedExchange(&ctx->LastJobFailed, NewValue);
}

EVT_WDF_DEVICE_PREPARE_HARDWARE     RkMppEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE     RkMppEvtReleaseHardware;
EVT_WDF_DEVICE_QUERY_REMOVE         RkMppEvtDeviceQueryRemove;  /* Task 4.1 */
EVT_WDF_OBJECT_CONTEXT_CLEANUP      RkMppEvtDeviceContextCleanup;
EVT_WDF_FILE_CLEANUP                RkMppEvtFileCleanup;
EVT_WDF_FILE_CLOSE                  RkMppEvtFileClose;

/* Phase 3a: IOMMU fault callback — registered in PrepareHardware. */
static VOID RkMppOnIommuFault(_In_ PVOID  ClientCookie,
                               _In_ ULONG64 FaultIova,
                               _In_ ULONG   StatusReg);

/* Phase 4 (Task 4.2): cascade-detach callback invoked by master rkiommu
 * EvtDeviceQueryRemove when the master is being uninstalled. */
static VOID RkMppOnMasterIommuQueryRemove(_In_ PVOID ConsumerContext);

/* Phase 1 (Task 1.6): peer-watch callbacks for RVD0 watching RVD1. */
static VOID RkMppPeerArrival(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink);
static VOID RkMppPeerRemoval(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink);

/* Phase 3 (Task 3.4): peer completion callback — defined in job.c.
 * RVD1's provider invokes this when a foreign-dispatched job finishes. */
extern VOID RkMppPeerCompletion(_In_ PVOID    ConsumerContext,
                                _In_ UINT64   CompletionCookie,
                                _In_ NTSTATUS JobStatus,
                                _In_ UINT32   HardwareStatus);

/* Forward decl: RkMppGetJobQueue is defined later in this file (used by
 * the peer callbacks above and by PrepareHardware/ReleaseHardware). */
PRKMPP_JOB_QUEUE RkMppGetJobQueue(_In_ WDFDEVICE Device);

/* ISR and DPC live in job.c but are declared via EVT_WDF_INTERRUPT_* here
 * so we can pass them to WdfInterruptCreate in RkMppDeviceCreate. */
extern EVT_WDF_INTERRUPT_ISR  RkMppEvtIsr;
extern EVT_WDF_INTERRUPT_DPC  RkMppEvtDpc;

extern NTSTATUS RkMppQueueInit(_In_ WDFDEVICE Device);  /* in ioctl.c */

static NTSTATUS RkMppReadAcpiId(_In_ WDFDEVICE Device,
                                _Out_ PUINT32 Hid, _Out_ PUINT32 Uid);

NTSTATUS
RkMppDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit)
{
    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = RkMppEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = RkMppEvtReleaseHardware;
    pnp.EvtDeviceQueryRemove     = RkMppEvtDeviceQueryRemove;  /* Task 4.1 */
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    /* Configure per-file-object context so bufpool.c can track allocations
     * per open handle.  EvtFileCleanup is invoked while the process is still
     * alive (or we KeStackAttachProcess for safety), before EvtFileClose. */
    WDF_FILEOBJECT_CONFIG foCfg;
    WDF_FILEOBJECT_CONFIG_INIT(&foCfg,
                               WDF_NO_EVENT_CALLBACK,  /* EvtFileCreate */
                               RkMppEvtFileClose,
                               RkMppEvtFileCleanup);

    WDF_OBJECT_ATTRIBUTES foAttr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&foAttr, RKMPP_FILE_CTX);

    WdfDeviceInitSetFileObjectConfig(DeviceInit, &foCfg, &foAttr);

    WDF_OBJECT_ATTRIBUTES attr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr, RKMPP_DEVICE);
    attr.EvtCleanupCallback = RkMppEvtDeviceContextCleanup;

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, &attr, &device);
    if (!NT_SUCCESS(status)) return status;

    /* GUID_DEVINTERFACE_RKMPP (the user-mode device interface) is NOT
     * registered here — UID isn't known until PrepareHardware reads
     * ACPI _UID, and we only want RVD0 (UID==0) reachable to user-mode.
     * RVD1 publishes only the in-kernel PEER_WORKER ifc for RVD0 to
     * dispatch through.  Registration moved to PrepareHardware. */

    /* Initialise the job queue (spin lock, lists, DPC) before any IOCTL
     * can arrive.  Must be called before RkMppQueueInit so the IOCTL
     * handlers can safely call RkMppGetJobQueue. */
    PRKMPP_DEVICE devCtx = RkMppDeviceGet(device);
    RkMppJobQueueInit(device, &devCtx->JobQueue);

    /* Wire the codec IRQ from EvtDeviceAdd-time with the default config.
     * Calling WdfInterruptCreate from EvtDevicePrepareHardware with
     * explicit InterruptRaw+InterruptTranslated descriptors fails with
     * STATUS_WDF_INVALID_INTERRUPT_CONFIG (0xC020020F) on our ARM64 GIC
     * SPI lines — WDF's strict-binding validation rejects the descriptors
     * even though the same shape works for rkiommu_vdec.  EvtDeviceAdd's
     * lenient path lets WDF auto-bind during normal PnP resource
     * assignment, matching rkiommu_vdec's pattern.
     *
     * Fatal on failure: completion is interrupt-driven exclusively.
     * The previous fall-back poller has been gated off — the polling
     * thread still runs but skips its INT_STATUS poll loop on every
     * KickEvent wake. */
    {
        WDF_INTERRUPT_CONFIG intCfg;
        WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkMppEvtIsr, RkMppEvtDpc);
        NTSTATUS intStatus = WdfInterruptCreate(
            device, &intCfg, WDF_NO_OBJECT_ATTRIBUTES,
            &devCtx->JobQueue.Interrupt);
        if (!NT_SUCCESS(intStatus)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: WdfInterruptCreate failed 0x%08x — "
                       "refusing to load (interrupt-driven completion "
                       "is non-optional)\n",
                       (ULONG)intStatus);
            return intStatus;
        }
    }

    /* Create the deferred-kick workitem.  Completion (DPC, DISPATCH)
     * only does bookkeeping and wakes this workitem; RkMppKickWorker
     * runs PromoteUntilFull + the next hardware kick at PASSIVE.  Fatal
     * on failure — deferred kick dispatch is non-optional, same as the
     * interrupt above. */
    {
        NTSTATUS wiStatus = RkMppJobQueueCreateWorker(device, &devCtx->JobQueue);
        if (!NT_SUCCESS(wiStatus)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: RkMppJobQueueCreateWorker failed 0x%08x — "
                       "refusing to load (deferred kick dispatch is "
                       "non-optional)\n",
                       (ULONG)wiStatus);
            return wiStatus;
        }
    }

    return RkMppQueueInit(device);
}

/* Phase 4 (Task 4.1): runtime peer detach.  Called from the peer's
 * RKMPP_PEER_QUERY_REMOVE_CB at PASSIVE when RVD1 is about to be
 * uninstalled.  Snapshots peer ifc state under queue lock, transitions
 * RVD0 back to single-core, drains any peer-targeted job briefly,
 * then dereferences the peer interface + releases the file-object.
 *
 * Also called from RkMppEvtReleaseHardware (the rkvdec.sys PnP-stop
 * path) to share the same teardown logic with a cleaner ordering. */
static VOID
RkMppDetachPeer(_In_ WDFDEVICE Device)
{
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    KIRQL irql;
    KeAcquireSpinLock(&q->Lock, &irql);
    BOOLEAN wasOpen = q->PeerOpen;
    RKMPP_PEER_WORKER_INTERFACE peerIfc = q->Peer;
    PFILE_OBJECT peerFo = q->PeerFileObj;
    RKMPP_JOB *peerJob = q->InFlightPerCore[1];
    q->PeerOpen = FALSE;
    q->CoreCount = 1;
    q->CoreIdle &= 1u;   /* clear peer idle bit */
    q->CorePending[1] = 0;
    q->PeerFileObj = NULL;
    RtlZeroMemory(&q->Peer, sizeof(q->Peer));
    KeReleaseSpinLock(&q->Lock, irql);

    if (!wasOpen) return;

    /* If a peer-targeted job was in flight at the moment of detach, the
     * "all jobs stopped" precondition was violated.  Best effort: wait
     * briefly for its Done event.  Spec acknowledges surprise-remove is
     * non-graceful; the codec watchdog + reset escalation will eventually
     * fire if hardware truly hung. */
    if (peerJob) {
        LARGE_INTEGER to; to.QuadPart = -((LONGLONG)500 * 10000);
        KeWaitForSingleObject(&peerJob->Done, Executive,
                              KernelMode, FALSE, &to);
    }

    /* Order: Dereference the interface (provider's Dereference callback
     * runs while its binary is still pinned by the file-object ref),
     * then release the file-object.  See [[wdf_query_interface.md]] —
     * inverting this order races with provider unload and BSODs 0xCE. */
    if (peerIfc.Header.InterfaceDereference)
        peerIfc.Header.InterfaceDereference(peerIfc.Header.Context);
    if (peerFo) ObDereferenceObject(peerFo);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: peer detached, single-core mode\n");
}

/* Phase 4 (Task 4.1): consumer-side callback invoked by RVD1's
 * PEER_WORKER provider from RVD1's EvtDeviceQueryRemove.  Runs at
 * PASSIVE on RVD1's thread.  Returning from this callback grants
 * RVD1 permission to proceed with query-remove. */
static VOID
RkMppPeerOnQueryRemove(_In_ PVOID ConsumerContext)
{
    WDFDEVICE Device = (WDFDEVICE)ConsumerContext;
    RkMppDetachPeer(Device);
}

static VOID
RkMppPeerArrival(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink)
{
    WDFDEVICE Device = (WDFDEVICE)Ctx;
    PRKMPP_DEVICE devCtx = RkMppDeviceGet(Device);
    if (devCtx->Uid != 0) return;   /* only RVD0 consumes peer */

    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    RKMPP_PEER_WORKER_INTERFACE ifc;
    PFILE_OBJECT fo = NULL;
    NTSTATUS s = RkMppQueryPeerWorkerBySymlink(SymbolicLink, &ifc, &fo);
    if (!NT_SUCCESS(s)) return;

    /* Topology filter: only RVD1 (Hid 0x3550, Uid 1) is our peer. */
    if (ifc.Hid != 0x3550 || ifc.Uid != 1) {
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        ObDereferenceObject(fo);
        return;
    }

    KIRQL irql;
    KeAcquireSpinLock(&q->Lock, &irql);
    if (q->PeerOpen) {
        /* Already attached — duplicate arrival, drop the second ref. */
        KeReleaseSpinLock(&q->Lock, irql);
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        ObDereferenceObject(fo);
        return;
    }
    q->Peer        = ifc;
    q->PeerFileObj = fo;
    q->PeerOpen    = TRUE;
    q->CoreCount   = 2;
    q->CoreIdle   |= (1u << 1);
    q->CorePending[1] = 0;
    KeReleaseSpinLock(&q->Lock, irql);

    /* Phase 3 (Task 3.4): register our completion callback on the
     * peer.  RVD1 invokes RkMppPeerCompletion when a foreign job's
     * codec interrupt completes.  Done outside the queue lock —
     * peer.RegisterCompletion takes its own lock. */
    if (ifc.RegisterCompletion) {
        ifc.RegisterCompletion(ifc.Header.Context, Device,
                               RkMppPeerCompletion);
    }

    /* Phase 4 (Task 4.1): register inverse hook so RVD1 can cascade
     * tell us to detach before it goes away. */
    if (ifc.RegisterQueryRemove) {
        ifc.RegisterQueryRemove(ifc.Header.Context, Device,
                                RkMppPeerOnQueryRemove);
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: peer worker attached (RVD1), CoreCount=2\n");
}

static VOID
RkMppPeerRemoval(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink)
{
    UNREFERENCED_PARAMETER(Ctx);
    UNREFERENCED_PARAMETER(SymbolicLink);
    /* Authoritative detach is via the peer's QueryRemove callback,
     * wired in Phase 4.  Removal notification just logs. */
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: peer worker removal notification\n");
}

NTSTATUS
RkMppEvtPrepareHardware(_In_ WDFDEVICE Device,
                        _In_ WDFCMRESLIST ResourcesRaw,
                        _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);  /* WDF auto-binds the IRQ
                                            * descriptor for the interrupt
                                            * registered in DeviceCreate;
                                            * we don't touch raw resources. */
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    NTSTATUS status = RkMppReadAcpiId(Device, &ctx->Hid, &ctx->Uid);
    if (!NT_SUCCESS(status)) return status;

    /* Register the user-mode GUID_DEVINTERFACE_RKMPP only for RVD0
     * (UID==0).  RVD1 has no user-mode IOCTL surface — only RVD0 is
     * opened by MFT, RVD0 dispatches to RVD1 via the in-kernel
     * PEER_WORKER ifc.  Holding registration until UID is known keeps
     * RVD1 invisible to user-mode enumeration entirely (vs registering
     * for both then disabling, which leaves a brief enumerable window
     * + lets MFT cache stale symlinks). */
    if (ctx->Uid == 0) {
        NTSTATUS si = WdfDeviceCreateDeviceInterface(
            Device, &GUID_DEVINTERFACE_RKMPP, NULL);
        if (!NT_SUCCESS(si)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec UID=0: WdfDeviceCreateDeviceInterface failed 0x%x\n", si);
            return si;
        }
    }


    /* First kick after PnP MUST reset: empirically, a fresh PD power-on
     * leaves the codec FSM in a state where dec_e=1 is accepted but the
     * core never starts (perf[229] doesn't advance, no AXI traffic).
     * Confirmed by skip-first-reset test: zero decode progress.
     * Subsequent kicks only reset after error/timeout (BSP parity via
     * mpp_common.c:2026 — gates mpp_dev_reset on reset_request > 0). */
    InterlockedExchange(&ctx->NeedsCoreReset, 1);

    /* Step 1: walk resources to capture MMIO windows.  The codec IRQ
     * was already registered at EvtDeviceAdd time via WdfInterruptCreate
     * (see RkMppDeviceCreate); WDF auto-binds it against the
     * CmResourceTypeInterrupt entry in this resource list during PnP
     * assignment, so we don't touch the descriptor here. */

    /* RVD0/RVD1 (RKCP3550) declares TWO physically contiguous memory regions
     *   link : 0xFDC38000 / 0x100  ← idx 0..63
     *   regs : 0xFDC38100 / 0x400  ← idx 64..319
     * which we merge into one MmioBase so the BSP idx*4 byte offsets line up.
     *
     * Strategy: take the convex hull and map as a single block.
     * The primary MmioBase always points at Mmios[0].Base. */
    PHYSICAL_ADDRESS mmioLow  = {0};
    PHYSICAL_ADDRESS mmioHigh = {0};
    BOOLEAN          mmioFound = FALSE;
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            PHYSICAL_ADDRESS start = d->u.Memory.Start;
            ULONG len = d->u.Memory.Length;
            PHYSICAL_ADDRESS end;
            end.QuadPart = start.QuadPart + len;
            if (!mmioFound) {
                mmioLow   = start;
                mmioHigh  = end;
                mmioFound = TRUE;
            } else {
                if (start.QuadPart < mmioLow.QuadPart)  mmioLow  = start;
                if (end.QuadPart   > mmioHigh.QuadPart) mmioHigh = end;
            }
        }
        /* CmResourceTypeInterrupt entries are consumed by WDF's auto-bind
         * for the WDFINTERRUPT registered in RkMppDeviceCreate — we don't
         * extract them here. */
    }

    if (!mmioFound) return STATUS_INSUFFICIENT_RESOURCES;
    ULONG mmioLen = (ULONG)(mmioHigh.QuadPart - mmioLow.QuadPart);
    if (mmioLen < 0x800) mmioLen = 0x800;  /* extend to cover cache regs */
    PVOID v = MmMapIoSpaceEx(mmioLow, mmioLen, PAGE_READWRITE | PAGE_NOCACHE);
    if (!v) return STATUS_INSUFFICIENT_RESOURCES;
    ctx->Mmios[0].Base   = v;
    ctx->Mmios[0].Length = mmioLen;
    ctx->Mmios[0].Phys   = mmioLow;
    ctx->MmioCount       = 1;
    ctx->MmioBase        = v;
    ctx->MmioLength      = mmioLen;
    RKMPP_LOG_INFO(
               "rkvdec: HID=RKCP%04x UID=%u MmioBase=phys 0x%llx len 0x%x\n",
               ctx->Hid, ctx->Uid, mmioLow.QuadPart, mmioLen);

    /* WDF interrupt registration moved to RkMppDeviceCreate
     * (EvtDeviceAdd-time) to avoid the STATUS_WDF_INVALID_INTERRUPT_CONFIG
     * failure that the EvtPrepareHardware-time explicit-descriptor path
     * hit on our ARM64 GIC SPI lines.  See the comment block in
     * RkMppDeviceCreate. */

    /* Step 2: open the in-kernel ifcs of rkmpp_ccu.sys and rkiommu.sys.
     * Without them we cannot service a single IOCTL beyond GET_CAPS, so refuse
     * to load (PnP code 31, no bugcheck). */
    status = RkMppOpenIfcs(Device, ctx->Hid, ctx->Uid, &ctx->Ifcs);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp: ccu/iommu ifcs unavailable (0x%08x) — install "
                   "rkmpp_ccu.sys and rkiommu.sys before rkmpp.sys\n", status);
        return STATUS_DEVICE_NOT_READY;
    }

    if (ctx->Ifcs.Iommu.Header.Version != RKIOMMU_IFC_VERSION ||
        ctx->Ifcs.Ccu.Header.Version   != RKMPP_CCU_IFC_VERSION) {
        RkMppCloseIfcs(&ctx->Ifcs);
        return STATUS_REVISION_MISMATCH;
    }
    RKMPP_LOG_INFO(
               "rkmpp: ifcs opened (iommu v%u, ccu v%u)\n",
               ctx->Ifcs.Iommu.Header.Version, ctx->Ifcs.Ccu.Header.Version);

    ctx->IommuAttached = FALSE;  /* set TRUE below after Iommu.Enable succeeds */

    /* Phase 4 (Task 4.2): if our paired rkiommu is the master, also open
     * its MASTER interface so we receive cascade query-remove notifications.
     * Slave rkiommu doesn't need this — its lifecycle is bound to master's
     * via the master→slave RegisterQueryRemove already wired in Task 2.2. */
    if (RkMppIsMasterIommu(ctx->Ifcs.Iommu.Hid, ctx->Ifcs.Iommu.Uid)) {
        PWSTR symlinks = NULL;
        NTSTATUS sl = IoGetDeviceInterfaces((LPGUID)&GUID_DEVINTERFACE_RKIOMMU_MASTER,
                                            NULL, 0, &symlinks);
        if (NT_SUCCESS(sl) && symlinks && *symlinks) {
            UNICODE_STRING uname;
            RtlInitUnicodeString(&uname, symlinks);
            PFILE_OBJECT fo = NULL;
            PDEVICE_OBJECT devObj = NULL;
            sl = IoGetDeviceObjectPointer(&uname, FILE_READ_DATA, &fo, &devObj);
            if (NT_SUCCESS(sl)) {
                RtlZeroMemory(&ctx->MasterIommuIfc, sizeof(ctx->MasterIommuIfc));
                sl = RkMppQueryOne(devObj, &GUID_DEVINTERFACE_RKIOMMU_MASTER,
                                   RKIOMMU_MASTER_IFC_VERSION,
                                   &ctx->MasterIommuIfc,
                                   sizeof(ctx->MasterIommuIfc));
                if (NT_SUCCESS(sl)) {
                    ctx->MasterIommuFileObj = fo;
                    ctx->MasterIommuOpen    = TRUE;
                    /* Register our cascade-detach callback. */
                    if (ctx->MasterIommuIfc.RegisterQueryRemove) {
                        ctx->MasterIommuIfc.RegisterQueryRemove(
                            ctx->MasterIommuIfc.Header.Context,
                            Device,
                            RkMppOnMasterIommuQueryRemove);
                    }
                } else {
                    ObDereferenceObject(fo);
                }
            }
        }
        if (symlinks) ExFreePool(symlinks);
        /* Non-fatal: if master ifc isn't queryable, we just don't get
         * the cascade callback.  Surprise-remove path still works (with
         * the 0xCE risk that wdf_query_interface memory documents). */
    }

    /* RVD1 (rkvdec UID==1) is paired with the slave rkiommu (UID 10),
     * whose page tables are not attached to master until slave's PnP
     * arrival callback fires (asynchronously after master rkiommu
     * starts).  Don't block PrepareHardware on that — instead, allow
     * PnP to complete with IommuAttached=FALSE, publish PEER_WORKER,
     * and rely on RVD0's later RegisterCompletion call (when RVD0
     * comes online) to trigger our lazy Enable.  This handles every
     * PnP order: RVD1-before-master, master-first, and master+RVD0
     * cycling while we stay loaded.  See RkMppPeerOnRvd0Connected
     * below for the enable trigger. */

    /* Phase 1 (Task 1.5): if this is RVD1, publish PEER_WORKER so RVD0
     * can dispatch jobs to us in dual-core mode.  Stub bodies for now —
     * Phase 3 wires the actual kick path. */
    {
        NTSTATUS sPeerPub = RkMppPeerWorkerPublish(Device, ctx->Uid);
        if (!NT_SUCCESS(sPeerPub)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: PEER_WORKER publish failed 0x%x (UID=%u)\n",
                       sPeerPub, ctx->Uid);
            RkMppCloseIfcs(&ctx->Ifcs);
            return sPeerPub;
        }
    }

    /* Raise the cluster.  Refcounted; matching DropCluster in ReleaseHardware.
     * Picks the rkvdec0/1 path or the AV1 path based on personality, which
     * we look up from the profile early because the per-codec branch below
     * (where ctx->Personality is finalized) hasn't run yet. */
    PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
    status = ctx->Ifcs.Ccu.RaiseCluster(cookie);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp: RaiseCluster failed 0x%08x\n", status);
        RkMppCloseIfcs(&ctx->Ifcs);
        return status;
    }

    /* Cluster bring-up confirmed working on real hardware as of Phase 3b
     * Task 2 — PD_VCODEC + PD_VDPU + PD_RKVDEC0/1 all power on cleanly,
     * NIU/CCU resets deassert, codec MMIO at 0xFDC38100/0xFDC48100 reads
     * REVISION = 0x53813f05.  Now the rest of PrepareHardware is safe. */

    /* Phase 2 fix: slave rkiommu (UID 10, paired with RVD1) has its
     * shadow Domain ready (set by OnMasterArrival via the PnP watch)
     * but its MMU MMIO is NOT yet programmed — that step was deferred
     * to avoid SError on slave's bus clocks before RaiseCluster.  Now
     * that RVD1's cluster is up and bus clocks are ungated, drive an
     * explicit Iommu.Enable through the ifc to program slave's DTE_ADDR
     * + ENABLE_PAGING.
     *
     * RVD0 (master rkiommu): Enable always succeeds (Domain already
     * allocated at master PrepareHardware).  Sets IommuAttached=TRUE.
     *
     * RVD1 (slave rkiommu): if slave hasn't yet attached to master
     * (Domain still NULL), Enable returns DEVICE_NOT_READY.  That's
     * fine — leave IommuAttached=FALSE and let RkMppPeerOnRvd0Connected
     * retry the Enable when RVD0 later calls RegisterCompletion. */
    ctx->IommuAttached = FALSE;
    if (ctx->Ifcs.Iommu.Enable) {
        NTSTATUS se = ctx->Ifcs.Iommu.Enable(ctx->Ifcs.Iommu.Header.Context);
        if (NT_SUCCESS(se)) {
            ctx->IommuAttached = TRUE;
        } else if (ctx->Uid == 0) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec UID=0: Iommu.Enable after RaiseCluster failed 0x%x\n", se);
            ctx->Ifcs.Ccu.DropCluster(cookie);
            RkMppCloseIfcs(&ctx->Ifcs);
            return se;
        } else {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                       "rkvdec UID=1: Iommu.Enable not ready yet (0x%x) — "
                       "will retry on RVD0 connect\n", se);
        }
    }

    const RKMPP_PROFILE *p = RkMppFindProfile(ctx->Hid, ctx->Uid);
    if (p) {
        ctx->SupportedCodecs = p->SupportedCodecs;
        ctx->Personality     = p->Personality;
    }

    /* Only touch codec MMIO + IOMMU wiring for devices we actually plan
     * to drive in this phase (decoders only — RVD0/RVD1).  RaiseCluster
     * powers the decoder PDs only; encoder/JPEG/IEP cores live in
     * PD_VENC0/1, etc., which we don't bring up here, so reading their
     * MMIO would SError.  The non-decoder probes still get a profile
     * row (SupportedCodecs=0) so subsequent IOCTLs can report
     * "unsupported"; they just don't get any bring-up activity. */
    if (p && p->SupportedCodecs != 0) {
        /* Wide hang-recovery reset on every PrepareHardware so partial
         * driver reinstall self-heals codec FSM + IOMMU state without
         * needing a full driver-chain reload.  v7 FullCoreReset was
         * RVD0-only; v8 has per-instance narrow variants that skip the
         * shared VDPU NIU reset (CON44 bits 4..6) to avoid disrupting
         * peer codecs.
         *
         * FullCoreResetN zeroes IOMMU DTE_ADDR as a side effect (the
         * AXI/AHB bus reset lives on the same NIU the IOMMU sits
         * behind), so we MUST follow with Reattach to reprogram
         * DTE_ADDR before any further codec activity. */
        if (ctx->Ifcs.CcuOpen) {
            if (ctx->Uid == 0 && ctx->Ifcs.Ccu.FullCoreReset0) {
                ctx->Ifcs.Ccu.FullCoreReset0(ctx->Ifcs.Ccu.Header.Context);
            } else if (ctx->Uid == 1 && ctx->Ifcs.Ccu.FullCoreReset1) {
                ctx->Ifcs.Ccu.FullCoreReset1(ctx->Ifcs.Ccu.Header.Context);
            }
        }
        if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Reattach) {
            NTSTATUS rs = ctx->Ifcs.Iommu.Reattach(ctx->Ifcs.Iommu.Header.Context);
            if (!NT_SUCCESS(rs)) {
                RKMPP_LOG_WARN(
                           "rkmpp: post-Raise Reattach failed 0x%08x\n", rs);
            }
        }

        /* Capture REVISION as ground truth that codec MMIO is alive. */
        ctx->RevisionWord = READ_REGISTER_ULONG(
            (volatile ULONG*)((PUCHAR)ctx->MmioBase + p->RevisionRegOffset));

        /* Wire the IOMMU fault callback.  rkiommu's DPC invokes this when
         * a translation fault posts; we just stash the fault state for the
         * IOCTL surface to surface to userspace.  Safe to register before
         * RkIommuEnable runs — the IRQ stays masked until then.
         *
         * Hardening item: this currently registers globally per device
         * cookie; once we have a job queue, the fault should be attributed
         * to the in-flight job and trigger AssertRvdec{0,1}CoreReset on its core. */
        if (ctx->Ifcs.Iommu.RegisterFaultHandler) {
            ctx->Ifcs.Iommu.RegisterFaultHandler(
                ctx->Ifcs.Iommu.Header.Context,         /* iommu instance */
                WdfDeviceWdmGetDeviceObject(Device),    /* our device, for callback */
                RkMppOnIommuFault);
        }

        /* Pulse the core reset.  RaiseCluster deasserts the bring-up reset
         * bundle (NIU + bus + cabac + core), but UEFI leaves the hardware
         * in an undefined "never reset" state — without a clean
         * assert→deassert pulse the codec sometimes accepts dec_e=1 yet
         * never issues AXI traffic (verified via post-kick IOMMU snapshot:
         * STATUS=idle, INT_RAWSTAT=0, no fault).  Mirror what BSP's
         * platform-init reset_control_bulk_assert+deassert dance does.
         *
         * v7 ifc split: AssertRvdec0CoreReset / DeassertRvdec0CoreReset
         * pulse the CON40 narrow reset bundle (bits 6..9); the matching
         * Rvdec1 pair pulses CON41 bits 6..8.  Dispatch on UID so each
         * codec's PrepareHardware only resets its own bits — matches
         * BSP per-device platform-init reset_control_bulk dance. */
        if (ctx->Uid == 0 &&
            ctx->Ifcs.Ccu.AssertRvdec0CoreReset && ctx->Ifcs.Ccu.DeassertRvdec0CoreReset) {
            ctx->Ifcs.Ccu.AssertRvdec0CoreReset(ctx->Ifcs.Ccu.Header.Context);
            ctx->Ifcs.Ccu.DeassertRvdec0CoreReset(ctx->Ifcs.Ccu.Header.Context);
        } else if (ctx->Uid == 1 &&
                   ctx->Ifcs.Ccu.AssertRvdec1CoreReset && ctx->Ifcs.Ccu.DeassertRvdec1CoreReset) {
            ctx->Ifcs.Ccu.AssertRvdec1CoreReset(ctx->Ifcs.Ccu.Header.Context);
            ctx->Ifcs.Ccu.DeassertRvdec1CoreReset(ctx->Ifcs.Ccu.Header.Context);
        }
    }

    RKMPP_LOG_INFO(
               "rkmpp: HID=RKCP%04x UID=%u rev=0x%08x codecs=0x%08x\n",
               ctx->Hid, ctx->Uid, ctx->RevisionWord, ctx->SupportedCodecs);

    /* RVD0 watches for RVD1's PEER_WORKER device interface arrival.
     * Registered LAST so that the synchronous-arrival callback (which
     * runs RVD1's full reset via RkMppPeerOnRvd0Connected) happens
     * AFTER our own FullCoreReset0 + Reattach + Assert/Deassert above.
     * Otherwise RVD0's later reset would disturb RVD1's freshly-reset
     * codec state and break RVD1 playback — surfaced as "disable +
     * re-enable RVD0 alone breaks RVD1 until RVD1 reinstall."
     * Non-fatal if registration fails — driver stays in single-core
     * mode silently. */
    if (ctx->Uid == 0) {
        NTSTATUS sw = RkMppWatchPeer(&GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
                                     Device,
                                     RkMppPeerArrival,
                                     RkMppPeerRemoval,
                                     &RkMppGetJobQueue(Device)->PeerWatch);
        if (!NT_SUCCESS(sw)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: RkMppWatchPeer failed 0x%x (single-core fallback)\n",
                       sw);
        }
    }

    return STATUS_SUCCESS;
}

NTSTATUS
RkMppEvtReleaseHardware(_In_ WDFDEVICE Device, _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);

    /* Phase 1 (Task 1.6) / Phase 4 (Task 4.1): unwatch peer + drop any
     * held peer ref.  Order matters: stop the PnP notification first (no
     * more arrival callbacks can fire), then call RkMppDetachPeer which
     * does the under-lock snapshot + deref dance.  Dereference of the
     * interface MUST precede release of the file-object — provider's
     * Dereference callback runs while its binary is still pinned (see
     * ifc_client.c:241-252, [[wdf_query_interface.md]] for the 0xCE
     * rationale). */
    if (ctx->Uid == 0) {
        RkMppUnwatchPeer(&RkMppGetJobQueue(Device)->PeerWatch);
        RkMppDetachPeer(Device);
    }

    /* Phase 4 (Task 4.2): drop master rkiommu ifc ref if we opened it.
     *
     * Critical: scrub our cascade callback out of master's Consumers[]
     * registry BEFORE releasing the file-object.  If we release first
     * and master gets disabled next (e.g. ACPI _DEP-triggered order
     * leaves master alive momentarily after we're gone), master would
     * invoke our (now stale) callback against this freed Device →
     * bugcheck → reboot to recover.  Symptom seen on RKCP3570 UID 9
     * disable: Device Manager demands reboot. */
    if (ctx->MasterIommuOpen) {
        if (ctx->MasterIommuIfc.UnregisterQueryRemove) {
            ctx->MasterIommuIfc.UnregisterQueryRemove(
                ctx->MasterIommuIfc.Header.Context, Device);
        }
        if (ctx->MasterIommuIfc.Header.InterfaceDereference)
            ctx->MasterIommuIfc.Header.InterfaceDereference(
                ctx->MasterIommuIfc.Header.Context);
        if (ctx->MasterIommuFileObj) {
            ObDereferenceObject(ctx->MasterIommuFileObj);
            ctx->MasterIommuFileObj = NULL;
        }
        ctx->MasterIommuOpen = FALSE;
    }

    /* Stop the poller before anything that takes MMIO out from under it.
     * The poller's tight `READ_REGISTER_ULONG(mmio + IntStatusOffset)`
     * loop runs at PASSIVE_LEVEL with no synchronization against PnP
     * teardown.  If a kick is in flight when PnP-stop fires, the poller
     * is still inside that read when DropCluster powers the codec PD
     * off (→ SError) or MmUnmapIoSpace tears down the kernel VA
     * (→ PAGE_FAULT_IN_NONPAGED_AREA).  Quiesce it first; the second
     * call from EvtDeviceContextCleanup is a no-op once PollerThread
     * is NULL. */
    RkMppJobQueueTeardown(&ctx->JobQueue);

    /* Mirror PrepareHardware in reverse: drop the cluster raise we took
     * there, then release the ifcs and unmap MMIO.
     *
     * Quiesce the IOMMU BEFORE the cluster Drop so that:
     *   - Any in-flight fault IRQ is masked at the device level
     *     (RK_MMU_INT_MASK=0 in RkIommuDisableHw); KMDF's WDF interrupt
     *     vector stays untouched (cross-device WdfInterruptDisable
     *     BSODs on shutdown per commit 9a71579).
     *   - Paging is disabled and DTE_ADDR zeroed while clocks are still
     *     on (PD raised); after Drop the IOMMU MMIO sits in a gated PD
     *     and any access would SError → WHEA UE.
     *
     * Fixes WHEA_UNCORRECTABLE_ERROR observed when reinstalling rkvdec
     * without reinstalling rkiommu_vdec or rkmpp_ccu.
     *
     * Best-effort: if Ifcs.Iommu.Disable is NULL (rkiommu_vdec not yet
     * rebuilt against v6 ifc) or returns failure, log and continue —
     * teardown must always complete. */
    if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Disable) {
        NTSTATUS s = ctx->Ifcs.Iommu.Disable(ctx->Ifcs.Iommu.Header.Context);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: Iommu.Disable in ReleaseHardware failed 0x%08x — "
                       "continuing teardown\n", s);
        }
    }
    if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.DropCluster) {
        PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
        ctx->Ifcs.Ccu.DropCluster(cookie);
    }
    RkMppCloseIfcs(&ctx->Ifcs);

    for (UINT32 i = 0; i < ctx->MmioCount; i++) {
        if (ctx->Mmios[i].Base) {
            MmUnmapIoSpace(ctx->Mmios[i].Base, ctx->Mmios[i].Length);
            ctx->Mmios[i].Base = NULL;
        }
    }
    ctx->MmioBase   = NULL;
    ctx->MmioLength = 0;
    ctx->MmioCount  = 0;
    return STATUS_SUCCESS;
}

/* Phase 4 (Task 4.1): EvtDeviceQueryRemove — called by PnP before
 * uninstalling or disabling this device.  On RVD1 (the PEER_WORKER
 * provider), we synchronously notify RVD0 (the consumer) so it can
 * transition to single-core mode and dereference the interface before
 * RVD1's driver binary is unloaded.  Returning STATUS_SUCCESS grants
 * permission to proceed with query-remove. */
NTSTATUS
RkMppEvtDeviceQueryRemove(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    /* RVD1 only — when our PEER_WORKER provider is going away,
     * notify the consumer (RVD0).  This invokes RVD0's registered
     * RKMPP_PEER_QUERY_REMOVE_CB synchronously; on return RVD0 has
     * detached and we can finalize PnP query-remove. */
    if (ctx->Uid == 1) {
        RkMppPeerWorkerNotifyQueryRemove();
    }
    return STATUS_SUCCESS;
}

/* Phase 4 (Task 4.2): master rkiommu is going away — flip IommuAttached
 * to FALSE so subsequent IOCTLs return STATUS_DEVICE_NOT_READY, drain
 * in-flight jobs briefly, then deref our master ifc.  Runs at PASSIVE on
 * master rkiommu's EvtDeviceQueryRemove thread; returning grants master
 * permission to proceed.  When master reinstalls, our (re-attached)
 * iommu's PtAttached flips back on its own; MFT re-opens us and
 * IommuAttached goes TRUE again via a new PrepareHardware cycle. */
static VOID
RkMppOnMasterIommuQueryRemove(_In_ PVOID ConsumerContext)
{
    WDFDEVICE Device = (WDFDEVICE)ConsumerContext;
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    ctx->IommuAttached = FALSE;

    /* Drain in-flight on both cores (best effort; 500ms each). */
    for (ULONG c = 0; c < 2; c++) {
        KIRQL irql;
        KeAcquireSpinLock(&q->Lock, &irql);
        RKMPP_JOB *jf = q->InFlightPerCore[c];
        KEVENT *done = jf ? &jf->Done : NULL;
        KeReleaseSpinLock(&q->Lock, irql);
        if (done) {
            LARGE_INTEGER to; to.QuadPart = -((LONGLONG)500 * 10000);
            KeWaitForSingleObject(done, Executive, KernelMode, FALSE, &to);
        }
    }

    /* Deref master ifc.  Order: Dereference callback first (runs while
     * binary is still pinned by fo), then release file-object.
     * See [[wdf_query_interface.md]] — inverting races with provider
     * unload and BSODs 0xCE. */
    if (ctx->MasterIommuOpen) {
        if (ctx->MasterIommuIfc.Header.InterfaceDereference)
            ctx->MasterIommuIfc.Header.InterfaceDereference(
                ctx->MasterIommuIfc.Header.Context);
        if (ctx->MasterIommuFileObj) {
            ObDereferenceObject(ctx->MasterIommuFileObj);
            ctx->MasterIommuFileObj = NULL;
        }
        ctx->MasterIommuOpen = FALSE;
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: master rkiommu cascade-detach, IommuAttached=FALSE\n");
}

/* Called from peer_worker.c when RVD0 (re-)connects to RVD1 via
 * RegisterCompletion.  RVD0's connection signals "master rkiommu is
 * up, slave should be attached, dispatch is ready" — so this is the
 * point at which RVD1 must do a fresh-PrepareHardware-equivalent
 * reset: Iommu.Enable + FullCoreReset1 + Reattach + narrow codec
 * reset pulse.  Without the codec reset, RVD1's FSM remains in the
 * half-initialized state from when initial PrepareHardware reset it
 * with no working IOMMU — symptom is broken playback that only a
 * driver reinstall clears.  Runs at PASSIVE on RVD0's PnP arrival
 * thread.  Idempotent — second connection just does the same reset
 * dance (RVD0 isn't dispatching during disconnect, so safe). */
VOID RkMppPeerOnRvd0Connected(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    if (!ctx) return;

    /* Slave PT might still be propagating.  Event-driven wait via
     * the v8 ifc — blocks on the slave's PtAttachedEvent (signalled
     * by RkIommuSlaveOnMasterArrival) up to 1 s.  Replaces the
     * 100 ms × 10 KeDelayExecutionThread loop.  Common case wakes
     * immediately if PtAttached was already true.  Review I7. */
    if (ctx->Ifcs.Iommu.WaitPtAttached) {
        (void)ctx->Ifcs.Iommu.WaitPtAttached(
            ctx->Ifcs.Iommu.Header.Context, 1000);
    } else if (ctx->Ifcs.Iommu.IsPtAttached) {
        /* Older ifc (pre-v8) — fall back to the original poll. */
        for (ULONG i = 0; i < 10; i++) {
            if (ctx->Ifcs.Iommu.IsPtAttached(ctx->Ifcs.Iommu.Header.Context))
                break;
            LARGE_INTEGER interval; interval.QuadPart = -1LL * 100 * 10000;
            KeDelayExecutionThread(KernelMode, FALSE, &interval);
        }
    }

    /* Step 1: program slave's MMU (idempotent for master). */
    if (ctx->Ifcs.Iommu.Enable) {
        NTSTATUS s = ctx->Ifcs.Iommu.Enable(ctx->Ifcs.Iommu.Header.Context);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec UID=%u: RVD0 connect → Iommu.Enable 0x%x — "
                       "leaving IommuAttached=FALSE\n", ctx->Uid, s);
            return;
        }
    }

    /* Step 2-4: same reset chain RkMppEvtPrepareHardware does after
     * cluster raise.  FullCoreReset zeroes IOMMU DTE as a side effect,
     * so Reattach is required to reprogram it before any kick.  The
     * narrow Assert/Deassert pulse re-initializes the codec FSM — this
     * is the step that was missing on initial RVD1 bring-up when slave
     * PT wasn't yet attached, and that a driver reinstall fixes. */
    if (ctx->Uid == 1 && ctx->Ifcs.CcuOpen) {
        if (ctx->Ifcs.Ccu.FullCoreReset1) {
            ctx->Ifcs.Ccu.FullCoreReset1(ctx->Ifcs.Ccu.Header.Context);
        }
        if (ctx->Ifcs.Iommu.Reattach) {
            NTSTATUS rs = ctx->Ifcs.Iommu.Reattach(ctx->Ifcs.Iommu.Header.Context);
            if (!NT_SUCCESS(rs)) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                           "rkvdec UID=1: RVD0 connect → Reattach 0x%x\n", rs);
                return;
            }
        }
        if (ctx->Ifcs.Ccu.AssertRvdec1CoreReset &&
            ctx->Ifcs.Ccu.DeassertRvdec1CoreReset) {
            ctx->Ifcs.Ccu.AssertRvdec1CoreReset(ctx->Ifcs.Ccu.Header.Context);
            ctx->Ifcs.Ccu.DeassertRvdec1CoreReset(ctx->Ifcs.Ccu.Header.Context);
        }
    }

    ctx->IommuAttached = TRUE;
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec UID=%u: RVD0 connect → reset+Enable OK, "
               "IommuAttached=TRUE\n", ctx->Uid);
}

/* Per-device context cleanup — fires when the WDFDEVICE is destroyed
 * (driver unload or device removal).  Stops the polling-completion
 * thread and releases its reference. */
VOID
RkMppEvtDeviceContextCleanup(_In_ WDFOBJECT Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet((WDFDEVICE)Device);
    if (ctx) {
        RkMppJobQueueTeardown(&ctx->JobQueue);
    }
}

/* Reads ACPI _HID and _UID from the device-instance properties.
 * Hardware-ID list may contain multiple IDs (the matching ACPI HID plus
 * `PRP0001` and any compatible IDs). Walk the multi-sz list and pick the
 * first one starting with "ACPI\\RKCP35".
 */
/* RkMppReadAcpiId — wrapper for back-compat with existing call sites.
 * Implementation now lives in driver/shared/acpi_uid.c. */
static NTSTATUS RkMppReadAcpiId(_In_ WDFDEVICE Device,
                                _Out_ PUINT32 Hid, _Out_ PUINT32 Uid)
{
    return RkSharedReadAcpiHidUid(Device, Hid, Uid);
}

/* EvtFileCreate is WDF_NO_EVENT_CALLBACK; the file context memory is
 * zero-initialised by WDF.  We initialise it lazily on first IOCTL, but
 * it is cleaner to do it in EvtFileClose's counterpart.  Actually the
 * standard pattern: initialise in EvtFileCreate.  Since we used
 * WDF_NO_EVENT_CALLBACK for create, WDF still allocates the context block
 * (zeroed).  RkMppBufAlloc handles a zero-initialised context because
 * RkMppBufFileCtxInit is called from the first alloc path via ioctl.c.
 * EvtFileCleanup frees all outstanding buffers; EvtFileClose is a no-op
 * (WDF requires it when EvtFileCleanup is set). */

VOID
RkMppEvtFileCleanup(_In_ WDFFILEOBJECT FileObject)
{
    /* Initialise context if it was never used (no allocs occurred). */
    PRKMPP_FILE_CTX ctx = RkMppFileGet(FileObject);
    if (!ctx->Device) {
        /* Never initialised — nothing to do. */
        return;
    }

    RKMPP_LOG_INFO(
               "rkmpp: EvtFileCleanup fileobject=%p — draining jobs\n",
               FileObject);

    /* Drain this file-object's outstanding jobs BEFORE freeing its
     * buffer pool.  If a kicked job's bitstream / output / colmv
     * buffers vanish from under the codec mid-DMA, the IOMMU tears
     * down the mappings and the codec's next AXI transaction wedges
     * the cluster (and historically the whole driver).  500ms cap is
     * plenty for a single decode-kick to finish.
     *
     * Only force a core reset on the next kick when we ACTUALLY hit
     * the timeout — if the in-flight finished cleanly, the codec is
     * idle and another session's clean kick shouldn't pay the reset
     * cost.  Forcing reset unconditionally regressed the common case
     * (process exits cleanly post-playback → next session's first
     * kick burned a reset for nothing, and the inter-session timing
     * disrupted ongoing pipelines on other cores). */
    BOOLEAN timedOut = FALSE;
    (void)RkMppJobsDrainOwner(ctx->Device, FileObject, 500, &timedOut);

    LONG sessionErrors = InterlockedExchange(&ctx->ErrorCount, 0);
    PRKMPP_DEVICE devCtx = RkMppDeviceGet(ctx->Device);

    if (timedOut) {
        RKMPP_LOG_WARN(
                   "rkmpp: FileCleanup in-flight wait timed out — "
                   "session-end PD power-cycle below; in-flight work lost\n");
    } else if (sessionErrors > 0) {
        RKMPP_LOG_INFO(
                   "rkmpp: drain clean fileobject=%p but %d error-flagged "
                   "jobs — session-end PD power-cycle below\n",
                   FileObject, sessionErrors);
    } else {
        RKMPP_LOG_INFO(
                   "rkmpp: drain done fileobject=%p clean — "
                   "session-end PD power-cycle below\n", FileObject);
    }

    RkMppBufFreeAll(FileObject);

    /* Decide between two session-close hygiene paths:
     *
     *   A. NO other File active on this engine — do the full PD power-
     *      cycle (FullCoreReset0/1 = PD_RKVDEC{0,1} PowerOff → SOFTRST
     *      toggle → PowerOn).  Clears codec FSM + IOMMU walk-cache state
     *      so the next session never inherits anything from this one.
     *      This is the empirical recipe that removes whole classes of
     *      "wedge persists across mpv invocations" failures.
     *
     *   B. ANOTHER File still has Pending / InFlight on this engine —
     *      a second concurrent decode is using us right now.  The wide
     *      PD-cycle would yank the codec and IOMMU out from under their
     *      in-flight DMA, corrupting their stream.  Downgrade to:
     *        - IOMMU Reattach (Disable+Enable) to drop walk-cache state
     *          tied to our just-unmapped iovas; preserves other Files'
     *          domain mappings.
     *        - Flag NeedsCoreReset so the OTHER File's next kick eats
     *          a cheap narrow CON40/CON41 core-reset bundle — softer
     *          cleanse of codec FSM than the full PD-cycle, but it
     *          still scrubs any FSM bits our last kick left behind.
     *      When the last remaining File on this engine closes, path A
     *      runs and performs the full hygiene cycle then.
     *
     * The `timedOut` recovery path stays on A regardless: if our drain
     * failed, the codec is wedged and the other File's next kick was
     * going to fail anyway — better to recover the engine now.
     *
     * Note on race: the otherActive check is not atomic with the heavy
     * path below — a peer can submit between observation and PD-cycle.
     * That's acceptable: if the race-stomp causes the peer's first
     * post-reset kick to error, JobStart's NeedsFullReset recovery
     * path issues a fresh hygiene cycle on the kick after.  The
     * alternative (a Quiescing latch checked in SubmitDense) added
     * complexity without measurable benefit. */
    BOOLEAN otherActive = RkMppJobQueueHasOtherOwner(ctx->Device, FileObject);

    if (devCtx) {
        PRKIOMMU_INTERFACE   iommu = devCtx->Ifcs.IommuOpen ? &devCtx->Ifcs.Iommu : NULL;
        PRKMPP_CCU_INTERFACE ccu   = devCtx->Ifcs.CcuOpen   ? &devCtx->Ifcs.Ccu   : NULL;

        if (otherActive && !timedOut) {
            RKMPP_LOG_INFO(
                       "rkmpp: FileCleanup with concurrent File on UID=%u — "
                       "skipping IOMMU/CCU touches (would disrupt peer DMA); "
                       "flagging NeedsCoreReset for peer's next kick\n",
                       devCtx->Uid);
            /* Do NOT call Iommu.Reattach here.  It Disable+Enables the
             * IOMMU, opening a ~10µs window with paging OFF — the peer's
             * in-flight AXI transactions during that window go
             * untranslated and fault.  BufFreeAll above already issued
             * per-iova ZAP_CACHE via UnmapMdl, which is the only
             * walk-cache state that needed flushing for our cleanup.
             *
             * The narrow per-codec reset bundle on the peer's next kick
             * scrubs any codec FSM state our last kick left behind, at
             * kick-start timing (codec idle) so it's peer-safe. */
            RkMppSetNeedsCoreReset(ctx->Device);
        } else {
            if (timedOut && otherActive) {
                RKMPP_LOG_WARN(
                           "rkmpp: FileCleanup drain timed out on UID=%u with "
                           "concurrent File active — PD-cycle will disrupt it\n",
                           devCtx->Uid);
            }
            /* iommu->MaskIrq disabled — calling rkiommu_vdec's
             * WdfInterruptDisable from rkvdec's FileCleanup is a
             * cross-device interrupt disconnect that races KMDF's own
             * teardown of that interrupt (and concurrent FileCleanups on
             * simultaneous stop): IoDisconnectInterrupt → KeRemoveQueueDpc
             * runs on an already-removed DPC → ACCESS_VIOLATION →
             * bugcheck 0x3B.  Confirmed by the simultaneous-stop crash
             * (FileCleanup → RkIommuMaskIrq → WdfInterruptDisable →
             * KeRemoveQueueDpc).  Same hazard already removed from the
             * JobKickLocalInner reset path and from ReleaseHardware
             * (commit 9a71579); see `wdf_interrupt_disable_cross_device`.
             * The Disable below drops IOMMU paging (AHB_CONTROL=0) before
             * the reset, so the codec generates no faults during the
             * window — the mask was only belt-and-suspenders.
             *
             * if (iommu && iommu->MaskIrq) iommu->MaskIrq(iommu->Header.Context);
             */
            if (iommu && iommu->Disable) {
                (void)iommu->Disable(iommu->Header.Context);
            }
            if (ccu) {
                if (devCtx->Uid == 0 && ccu->FullCoreReset0) {
                    ccu->FullCoreReset0(ccu->Header.Context);
                } else if (devCtx->Uid == 1 && ccu->FullCoreReset1) {
                    ccu->FullCoreReset1(ccu->Header.Context);
                }
            }
            if (iommu && iommu->Enable) {
                NTSTATUS rs = iommu->Enable(iommu->Header.Context);
                if (!NT_SUCCESS(rs)) {
                    RKMPP_LOG_WARN(
                               "rkmpp: post-FileCleanup IOMMU Enable failed 0x%08x — "
                               "next session will lazy-Enable on first MapMdl\n", rs);
                }
            }
            /* iommu->UnmaskIrq disabled — paired with the MaskIrq removal
             * above.  KMDF re-arms rkiommu_vdec's ISR on its own D0 path;
             * rkvdec must not WdfInterruptEnable it cross-device. */
            /* if (iommu && iommu->UnmaskIrq) iommu->UnmaskIrq(iommu->Header.Context); */

            /* Power-cycle subsumes both narrow CoreReset and wide
             * FullReset that JobStart would otherwise apply to the
             * next session's first kick — clear the flags. */
            (void)RkMppQueryAndClearNeedsCoreReset(ctx->Device);
            (void)RkMppQueryAndClearNeedsFullReset(ctx->Device);
        }
    }

    /* LastJobFailed is per-device; clear regardless of path so it doesn't
     * leak across to the next kick on either File. */
    (void)RkMppExchangeLastJobFailed(ctx->Device, 0);
}

VOID
RkMppEvtFileClose(_In_ WDFFILEOBJECT FileObject)
{
    UNREFERENCED_PARAMETER(FileObject);
    /* Nothing to do; cleanup was done in EvtFileCleanup. */
}

/* ---------------------------------------------------------------------------
 * RkMppOnIommuFault — RKIOMMU_FAULT_CALLBACK invoked by rkiommu.sys DPC
 * (DISPATCH_LEVEL) when the IOMMU raises a page-fault interrupt.
 *
 * ClientCookie is the PDEVICE_OBJECT passed to RegisterFaultHandler; we
 * recover the WDFDEVICE via WdfWdmDeviceGetWdfDeviceHandle, then the
 * per-device context.  Atomic stores ensure safe cross-level visibility.
 * --------------------------------------------------------------------------- */
static VOID
RkMppOnIommuFault(_In_ PVOID   ClientCookie,
                  _In_ ULONG64 FaultIova,
                  _In_ ULONG   StatusReg)
{
    PDEVICE_OBJECT wdmDev = (PDEVICE_OBJECT)ClientCookie;
    WDFDEVICE wdfDev = WdfWdmDeviceGetWdfDeviceHandle(wdmDev);
    if (!wdfDev) return;
    PRKMPP_DEVICE ctx = RkMppDeviceGet(wdfDev);
    if (!ctx) return;

    InterlockedExchange64((LONG64*)&ctx->FaultIova, (LONG64)FaultIova);
    InterlockedExchange((LONG*)&ctx->FaultStatusReg, (LONG)StatusReg);
    InterlockedExchange((LONG*)&ctx->FaultTriggered, 1);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp: IOMMU fault iova=0x%llx status=0x%lx\n",
               (unsigned long long)FaultIova, (unsigned long)StatusReg);
}

/* Accessor used by bufpool.c to reach the rkiommu interface without
 * exposing the full RKMPP_DEVICE structure outside device.c. */
PRKIOMMU_INTERFACE
RkMppGetIommuIfc(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return &ctx->Ifcs.Iommu;
}

/* Accessor used by job.c to reach the CCU interface (raise/drop cluster). */
PRKMPP_CCU_INTERFACE
RkMppGetCcuIfc(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return &ctx->Ifcs.Ccu;
}

/* Accessor used by job.c to reach the MMIO base for ISR register reads. */
PVOID
RkMppGetMmioBase(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return ctx->MmioBase;
}

/* Accessor used by job.c for offset bounds checking on register writes.
 * MmioLength is SIZE_T but every codec MMIO range we map is < 4 GiB, so
 * a ULONG return is fine (bounds-clamped on the off chance). */
ULONG
RkMppGetMmioLength(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    if (!ctx) return 0;
    return (ctx->MmioLength > MAXULONG) ? MAXULONG : (ULONG)ctx->MmioLength;
}

/* Accessor used by job.c to reach the job queue. */
PRKMPP_JOB_QUEUE
RkMppGetJobQueue(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return &ctx->JobQueue;
}

/* Codec-personality accessor — selects the kick path in job.c. */
RKMPP_CODEC_PERSONALITY
RkMppGetPersonality(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return ctx->Personality;
}

/* Auxiliary MMIO window accessor.  Index 0 is the primary (same as
 * MmioBase); 1..N-1 hit additional windows declared in _CRS.  Returns
 * NULL if the slot is unused.  AV1 uses indices 0=VCD, 1=CACHE, 2=AFBC. */
PVOID
RkMppGetMmioWindow(_In_ WDFDEVICE Device,
                   _In_ UINT32 Index,
                   _Out_opt_ PULONG Length)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    if (Index >= ctx->MmioCount) {
        if (Length) *Length = 0;
        return NULL;
    }
    if (Length) {
        SIZE_T l = ctx->Mmios[Index].Length;
        *Length = (l > MAXULONG) ? MAXULONG : (ULONG)l;
    }
    return ctx->Mmios[Index].Base;
}

void RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    Out->Hid             = ctx->Hid;
    Out->Uid             = ctx->Uid;
    Out->RevisionWord    = ctx->RevisionWord;
    Out->SupportedCodecs = ctx->SupportedCodecs;
}

/* Accessor used by ioctl.c to read the atomically-maintained IOMMU fault
 * state written by RkMppOnIommuFault.  Uses InterlockedOr(x,0) as a
 * barrier-safe atomic load on both x86 and ARM64. */
void RkMppGetFaultState(_In_ WDFDEVICE Device, _Out_ RKMPP_FAULT_STATE *Out)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    Out->Triggered  = InterlockedOr((LONG*)&ctx->FaultTriggered, 0);
    Out->StatusReg  = InterlockedOr((LONG*)&ctx->FaultStatusReg, 0);
    Out->FaultIova  = InterlockedOr64((LONG64*)&ctx->FaultIova, 0);
}
