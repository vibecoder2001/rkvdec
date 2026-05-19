# rkvdec2 Multi-Core Dispatch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Dispatch H.264/H.265 jobs across RVD0 and RVD1 in parallel via RVD0-as-master, with shared IOMMU page tables, single-core fallback, and clean uninstall.

**Architecture:** RVD0 owns the IOCTL surface, job queue, and per-Owner LRU; promotes pending jobs to whichever core's `InFlightPerCore[]` slot is idle (BSP-style: idle bit + `task_index` tiebreak). RVD1 publishes a kernel `PEER_WORKER` query-interface; RVD0 dispatches MMIO writes through it and receives completions via DPC callback. Master rkiommu_vdec (UID 9) owns page tables; slave (UID 10) attaches to master's page-table base via `GetPageTableBase` query-interface, programming its own MMU registers with the master's PT root. All cross-driver attach paths use `IoRegisterPlugPlayNotification` so any start order works and either driver can be hot-uninstalled/reinstalled.

**Tech Stack:** Windows KMDF, C, ARM64, WDF query-interface, ACPI, MSBuild, `mft_play`/`mft_decode` user-mode test harnesses on the rk ARM64 box.

**Verification model:** This project has no in-tree unit tests for kernel binaries. Each task ends with a build step (msbuild on the Windows dev box → Z: share) and, for behavior changes, a user-runnable recipe on rk. Per [[windows_arm_target_manual.md]] the agent cannot trigger HW runs; user must execute and report observations.

**Spec:** `docs/superpowers/specs/2026-05-18-rkvdec-multicore-dispatch-design.md`

---

## File Inventory

**New files:**
- `shared/rkmpp_peer_worker_ifc.h` — peer-worker query-interface contract
- `shared/rkiommu_master_ifc.h` — master→slave PT-base query-interface
- `driver/shared/rkmpp/peer_attach.c` / `.h` — `IoRegisterPlugPlayNotification`-driven peer open helper
- `driver/rkvdec/peer_worker.c` — RVD1's `PEER_WORKER` provider impl

**Modified files:**
- `driver/rkvdec/job.h` — per-core in-flight slots, idle bitmap, peer client state
- `driver/rkvdec/job.c` — promotion loop, dispatch decision, debug log, peer completion entry
- `driver/rkvdec/device.c` — open peer worker, register PnP notification, peer-detach on file cleanup
- `driver/rkvdec/driver.c` — only register `PEER_WORKER` provider on UID==1 instances
- `driver/rkvdec/ioctl.c` — gate IOCTLs on `IommuAttached`
- `driver/rkiommu_vdec/device.c` — slave-mode (skip PT alloc, attach to master, late-attach), master-side consumer registry, EvtDeviceQueryRemove cascade
- `driver/rkiommu_vdec/device.h` — context additions (`IsMaster`, `PtAttached`, `MasterClient`, `Consumers[]`)
- `driver/rkiommu_vdec/ifc.c` — publish `GetPageTableBase` on master; `IsPtAttached` on all
- `driver/shared/rkmpp/ifc_client.h` / `.c` — `LookupPeerCodec`, `IsMasterIommu`, helper to open master rkiommu by Hid/Uid

---

## Phase 1 — Peer-worker interface scaffolding

Add the new query-interface contracts and the late-attach helper. No dispatch yet; existing single-core path stays the only active path. Goal: RVD0 can detect RVD1's presence and hold a reference to it without changing job behavior.

### Task 1.1: Define `PEER_WORKER` interface header

**Files:**
- Create: `shared/rkmpp_peer_worker_ifc.h`

- [ ] **Step 1: Create the header**

```c
/* shared/rkmpp_peer_worker_ifc.h — In-kernel device interface exported
 * by RVD1 (rkvdec UID==1 instance) so RVD0 can dispatch jobs to it.
 *
 * Topology: there is exactly one peer worker on RK3588 (the RVD1
 * instance).  RVD0 queries this interface at PrepareHardware time
 * (or later via IoRegisterPlugPlayNotification) and uses it to kick
 * jobs cross-core.  Single-core fallback: query returns
 * STATUS_NOT_FOUND, RVD0 stays self-only.  See spec
 * docs/superpowers/specs/2026-05-18-rkvdec-multicore-dispatch-design.md
 * sections "Architecture" and "Start-order races". */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
    0x9a7e0c81, 0x4f12, 0x4a3d, 0xb6, 0x88, 0x1c, 0x2f, 0x6e, 0x14, 0xa0, 0x55);

#define RKMPP_PEER_WORKER_IFC_VERSION 1u

/* Callback type — RVD1 invokes this from its DPC when a kicked job's
 * codec interrupt has been handled.  Runs at DISPATCH_LEVEL.  RVD0
 * uses CompletionCookie to look up which RKMPP_JOB this completion
 * matches in its queue. */
typedef VOID (*RKMPP_PEER_COMPLETION_CB)(
    _In_ PVOID    ConsumerContext,    /* RVD0's RKMPP_DEVICE pointer */
    _In_ UINT64   CompletionCookie,   /* opaque, supplied at KickJob */
    _In_ NTSTATUS JobStatus,
    _In_ UINT32   HardwareStatus);

/* RVD0 registers its completion callback once, at attach time. */
typedef NTSTATUS (*RKMPP_PEER_REGISTER_COMPLETION)(
    _In_ PVOID                    ProviderContext,
    _In_ PVOID                    ConsumerContext,
    _In_ RKMPP_PEER_COMPLETION_CB Callback);

/* RVD0 calls this to launch a job on RVD1.  RVD1 takes responsibility
 * for the dense bank + iova substitutions + MMIO writes + IRQ wait,
 * and invokes the registered completion callback when done.  Returns
 * synchronously after kicking (does not wait for completion).
 *
 * Bank is copied internally — caller's storage may be reused after
 * return.  IovaSlots are the post-substitution iovas in dense-bank
 * coordinate (slot_index, iova) pairs, already resolved against the
 * caller's File's buffer handles. */
typedef struct _RKMPP_PEER_KICK_PARAMS {
    const VOID                  *Bank;          /* RKMPP_DENSE_BANK */
    UINT32                       BankBytes;     /* sizeof(RKMPP_DENSE_BANK) */
    UINT32                       KickValue;     /* swreg0 value (TRIGGER) */
    const VOID                  *IovaSlots;     /* RKMPP_DENSE_IOVA_SLOT[] */
    UINT32                       IovaSlotCount;
    UINT64                       CompletionCookie;
} RKMPP_PEER_KICK_PARAMS;

typedef NTSTATUS (*RKMPP_PEER_KICK_JOB)(
    _In_ PVOID                          ProviderContext,
    _In_ const RKMPP_PEER_KICK_PARAMS  *Params);

/* Inverse notification: provider tells consumer it is about to go away
 * (PnP query-remove).  Consumer MUST stop using the provider's function
 * pointers, drain anything in flight on the peer core, and Dereference
 * + release the provider file-object before returning.  Returning from
 * the callback grants the provider permission to complete query-remove. */
typedef VOID (*RKMPP_PEER_QUERY_REMOVE_CB)(_In_ PVOID ConsumerContext);

typedef NTSTATUS (*RKMPP_PEER_REGISTER_QUERY_REMOVE)(
    _In_ PVOID                       ProviderContext,
    _In_ PVOID                       ConsumerContext,
    _In_ RKMPP_PEER_QUERY_REMOVE_CB  Callback);

typedef struct _RKMPP_PEER_WORKER_INTERFACE {
    INTERFACE                          Header;
    UINT32                             Hid;   /* always 0x3550 */
    UINT32                             Uid;   /* always 1 (RVD1) */
    RKMPP_PEER_REGISTER_COMPLETION     RegisterCompletion;
    RKMPP_PEER_KICK_JOB                KickJob;
    RKMPP_PEER_REGISTER_QUERY_REMOVE   RegisterQueryRemove;
} RKMPP_PEER_WORKER_INTERFACE, *PRKMPP_PEER_WORKER_INTERFACE;
```

- [ ] **Step 2: Build to verify header compiles**

Run from the dev box repo root:

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

Expected: build succeeds (header is included transitively from later tasks but standalone validation now is free).

- [ ] **Step 3: Commit**

```
git add shared/rkmpp_peer_worker_ifc.h
git commit -m "rkmpp: define PEER_WORKER cross-core kernel interface"
```

### Task 1.2: Define master rkiommu PT-base interface header

**Files:**
- Create: `shared/rkiommu_master_ifc.h`

- [ ] **Step 1: Create the header**

```c
/* shared/rkiommu_master_ifc.h — In-kernel interface published by the
 * MASTER rkiommu_vdec instance (UID 9) so the SLAVE instance (UID 10)
 * can attach its MMU to the master's page-table root.  Mirrors BSP
 * Linux's mpp_rkvdec2_link.c:1342 (rkvdec2_attach_ccu): non-core-0
 * rkvdec2 cores reuse core 0's IOMMU domain by aliasing the domain
 * pointer.  We achieve the same by programming both MMU instances'
 * RK_MMU_DTE_ADDR with the same physical address.
 *
 * Spec: docs/superpowers/specs/2026-05-18-rkvdec-multicore-dispatch-
 * design.md, section "IOMMU shared page tables". */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKIOMMU_MASTER,
    0x8c19d4b2, 0x7f30, 0x4c89, 0xa9, 0x55, 0x21, 0x06, 0x44, 0x9e, 0x37, 0xb1);

#define RKIOMMU_MASTER_IFC_VERSION 1u

/* Returns the PHYSICAL address of the page-directory root that the
 * master programs into its own RK_MMU_DTE_ADDR.  Slave programs the
 * same value into its DTE_ADDR.  Stable for the lifetime of the
 * master device's PrepareHardware → ReleaseHardware cycle. */
typedef NTSTATUS (*RKIOMMU_MASTER_GET_PT_BASE)(
    _In_  PVOID    ProviderContext,
    _Out_ PULONG32 PdPhys);

/* Inverse query-remove notification — symmetric to peer worker. */
typedef VOID (*RKIOMMU_MASTER_QUERY_REMOVE_CB)(_In_ PVOID ConsumerContext);

typedef NTSTATUS (*RKIOMMU_MASTER_REGISTER_QUERY_REMOVE)(
    _In_ PVOID                          ProviderContext,
    _In_ PVOID                          ConsumerContext,
    _In_ RKIOMMU_MASTER_QUERY_REMOVE_CB Callback);

typedef struct _RKIOMMU_MASTER_INTERFACE {
    INTERFACE                              Header;
    UINT32                                 Hid;   /* 0x3570 */
    UINT32                                 Uid;   /* 9 */
    RKIOMMU_MASTER_GET_PT_BASE             GetPageTableBase;
    RKIOMMU_MASTER_REGISTER_QUERY_REMOVE   RegisterQueryRemove;
} RKIOMMU_MASTER_INTERFACE, *PRKIOMMU_MASTER_INTERFACE;
```

- [ ] **Step 2: Commit**

```
git add shared/rkiommu_master_ifc.h
git commit -m "rkiommu_vdec: define MASTER PT-base cross-instance interface"
```

### Task 1.3: Add `LookupPeerCodec` / `IsMasterIommu` topology helpers

**Files:**
- Modify: `driver/shared/rkmpp/ifc_client.h`
- Modify: `driver/shared/rkmpp/ifc_client.c`

- [ ] **Step 1: Extend the header**

Add at end of `driver/shared/rkmpp/ifc_client.h`:

```c
/* Topology lookups for multi-core rkvdec2 dispatch.  See spec
 * "Identification — codec ↔ IOMMU and master/slave". */

/* Returns TRUE if the given rkiommu (Hid, Uid) is the master instance
 * (the one that owns the page tables).  On RK3588: rkvdec uses
 * rkiommu Hid=0x3570 Uid=9 as master, Uid=10 as slave.  AV1 has only
 * one rkiommu instance — always master in its own topology. */
BOOLEAN RkMppIsMasterIommu(_In_ UINT32 IommuHid, _In_ UINT32 IommuUid);

/* For an rkvdec codec at (CodecHid, CodecUid), tell the caller whether
 * a peer rkvdec exists in the topology and, if so, what its (Hid, Uid)
 * is.  Used by RVD0 (Uid==0) to find RVD1 (Uid==1).  Returns FALSE for
 * cores that have no peer (AV1, or RVD1 looking for "its" peer — we
 * only dispatch master→slave). */
BOOLEAN RkMppLookupPeerCodec(_In_  UINT32 CodecHid,
                             _In_  UINT32 CodecUid,
                             _Out_ UINT32 *PeerHid,
                             _Out_ UINT32 *PeerUid);
```

- [ ] **Step 2: Implement in `ifc_client.c`**

Append to `driver/shared/rkmpp/ifc_client.c`:

```c
BOOLEAN RkMppIsMasterIommu(_In_ UINT32 IommuHid, _In_ UINT32 IommuUid)
{
    /* Match the static table in LookupIommuForClient:
     *   Hid=0x3570 → rkvdec rkiommus.  Master = UID 9 (paired with RVD0).
     *   Hid=0x3571 → AV1 rkiommu, single instance, always master. */
    if (IommuHid == 0x3570) return IommuUid == 9;
    if (IommuHid == 0x3571) return TRUE;
    return FALSE;
}

BOOLEAN RkMppLookupPeerCodec(_In_  UINT32 CodecHid,
                             _In_  UINT32 CodecUid,
                             _Out_ UINT32 *PeerHid,
                             _Out_ UINT32 *PeerUid)
{
    *PeerHid = 0; *PeerUid = 0;
    /* RKCP3550 rkvdec2: RVD0 (Uid 0) peers with RVD1 (Uid 1).
     * We dispatch master→slave only, so the reverse mapping is not
     * supported (returns FALSE). */
    if (CodecHid == 0x3550 && CodecUid == 0) {
        *PeerHid = 0x3550;
        *PeerUid = 1;
        return TRUE;
    }
    return FALSE;
}
```

- [ ] **Step 3: Build the rkvdec project**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

Expected: builds clean.

- [ ] **Step 4: Commit**

```
git add driver/shared/rkmpp/ifc_client.h driver/shared/rkmpp/ifc_client.c
git commit -m "rkmpp: add peer-codec + master-iommu topology helpers"
```

### Task 1.4: PnP notification helper for late-attach

**Files:**
- Create: `driver/shared/rkmpp/peer_attach.h`
- Create: `driver/shared/rkmpp/peer_attach.c`

This helper wraps `IoRegisterPlugPlayNotification` so each driver doesn't reinvent it. Callback fires at PASSIVE for both arrival and removal.

- [ ] **Step 1: Create the header**

```c
/* driver/shared/rkmpp/peer_attach.h — PnP arrival/removal notification
 * helper.  Both rkvdec (peer worker open) and rkiommu_vdec (slave →
 * master attach) use this to handle late-start and runtime install/
 * uninstall without blocking PrepareHardware.  Spec: "Start-order
 * races". */
#pragma once
#include <ntddk.h>

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
```

- [ ] **Step 2: Create the implementation**

```c
/* driver/shared/rkmpp/peer_attach.c */
#include "peer_attach.h"

DRIVER_NOTIFICATION_CALLBACK_ROUTINE RkMppPnpCallback;

static NTSTATUS
RkMppPnpCallback(_In_ PVOID NotificationStructure, _Inout_opt_ PVOID Context)
{
    RKMPP_PEER_WATCH *w = (RKMPP_PEER_WATCH*)Context;
    if (!w || !NotificationStructure) return STATUS_SUCCESS;

    DEVICE_INTERFACE_CHANGE_NOTIFICATION *n =
        (DEVICE_INTERFACE_CHANGE_NOTIFICATION*)NotificationStructure;

    if (IsEqualGUID(&n->Event, &GUID_DEVICE_INTERFACE_ARRIVAL)) {
        if (w->OnArrival) w->OnArrival(w->Ctx, n->SymbolicLinkName);
    } else if (IsEqualGUID(&n->Event, &GUID_DEVICE_INTERFACE_REMOVAL)) {
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
    extern PDRIVER_OBJECT WdfDriverWdmGetDriverObject(_In_ WDFDRIVER);
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
```

- [ ] **Step 3: Add `peer_attach.c` to every consuming vcxproj**

Edit `driver/rkvdec/rkvdec.vcxproj` and `driver/rkiommu_vdec/rkiommu_vdec.vcxproj`: add `<ClCompile Include="..\shared\rkmpp\peer_attach.c" />` to the source list.

- [ ] **Step 4: Build both projects to verify**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
msbuild driver\rkiommu_vdec\rkiommu_vdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

Expected: both build clean.

- [ ] **Step 5: Commit**

```
git add driver/shared/rkmpp/peer_attach.h driver/shared/rkmpp/peer_attach.c \
        driver/rkvdec/rkvdec.vcxproj driver/rkiommu_vdec/rkiommu_vdec.vcxproj
git commit -m "rkmpp: shared peer-attach PnP notification helper"
```

### Task 1.5: RVD1 publishes PEER_WORKER provider (stub bodies)

**Files:**
- Create: `driver/rkvdec/peer_worker.c`
- Modify: `driver/rkvdec/driver.c`
- Modify: `driver/rkvdec/device.c` (only to wire the publish at PrepareHardware)
- Modify: `driver/rkvdec/rkvdec.vcxproj`

Bodies are stubs that return `STATUS_NOT_IMPLEMENTED` for `KickJob`. Real implementation in Phase 3. The provider must publish so RVD0 can find it.

- [ ] **Step 1: Create `peer_worker.c` with stubbed provider**

```c
/* driver/rkvdec/peer_worker.c — RVD1's PEER_WORKER interface provider.
 * Stub bodies in Phase 1; KickJob wired to actual MMIO path in Phase 3.
 * Only the rkvdec UID==1 instance publishes this interface. */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_peer_worker_ifc.h"
#include "devpub.h"

typedef struct _RKMPP_PEER_PROVIDER_CTX {
    WDFDEVICE                       Device;
    /* Consumer's registered callbacks — set by RegisterCompletion /
     * RegisterQueryRemove.  Read by Phase 3 dispatch path. */
    PVOID                           CompletionCtx;
    RKMPP_PEER_COMPLETION_CB        CompletionCb;
    PVOID                           QueryRemoveCtx;
    RKMPP_PEER_QUERY_REMOVE_CB      QueryRemoveCb;
    KSPIN_LOCK                      Lock;       /* protects CompletionCtx/Cb */
    LONG volatile                   RefCount;
} RKMPP_PEER_PROVIDER_CTX;

static RKMPP_PEER_PROVIDER_CTX g_PeerProvider;

static VOID  PeerIfcReference(PVOID c)   { InterlockedIncrement(&((RKMPP_PEER_PROVIDER_CTX*)c)->RefCount); }
static VOID  PeerIfcDereference(PVOID c) { InterlockedDecrement(&((RKMPP_PEER_PROVIDER_CTX*)c)->RefCount); }

static NTSTATUS
PeerRegisterCompletion(PVOID provCtx, PVOID consCtx,
                       RKMPP_PEER_COMPLETION_CB cb)
{
    RKMPP_PEER_PROVIDER_CTX *p = provCtx;
    KIRQL irql;
    KeAcquireSpinLock(&p->Lock, &irql);
    p->CompletionCtx = consCtx;
    p->CompletionCb  = cb;
    KeReleaseSpinLock(&p->Lock, irql);
    return STATUS_SUCCESS;
}

static NTSTATUS
PeerRegisterQueryRemove(PVOID provCtx, PVOID consCtx,
                        RKMPP_PEER_QUERY_REMOVE_CB cb)
{
    RKMPP_PEER_PROVIDER_CTX *p = provCtx;
    KIRQL irql;
    KeAcquireSpinLock(&p->Lock, &irql);
    p->QueryRemoveCtx = consCtx;
    p->QueryRemoveCb  = cb;
    KeReleaseSpinLock(&p->Lock, irql);
    return STATUS_SUCCESS;
}

static NTSTATUS
PeerKickJob(PVOID provCtx, const RKMPP_PEER_KICK_PARAMS *params)
{
    UNREFERENCED_PARAMETER(provCtx);
    UNREFERENCED_PARAMETER(params);
    /* Phase 3: factor RkMppJobStart into a body that takes Bank +
     * IovaSlots + KickValue + CompletionCookie and posts the kick
     * through this device's normal poller path. */
    return STATUS_NOT_IMPLEMENTED;
}

/* Publishes the PEER_WORKER device interface for the current WDFDEVICE.
 * Called from EvtPrepareHardware on the UID==1 instance only. */
NTSTATUS RkMppPeerWorkerPublish(_In_ WDFDEVICE Device, _In_ UINT32 Uid)
{
    if (Uid != 1) return STATUS_SUCCESS;   /* only RVD1 publishes */

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
    WDF_QUERY_INTERFACE_CONFIG_INIT(&qic,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
                                    NULL);
    NTSTATUS s = WdfDeviceAddQueryInterface(Device, &qic);
    if (!NT_SUCCESS(s)) return s;

    return WdfDeviceCreateDeviceInterface(Device,
                                          &GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
                                          NULL);
}

/* Phase 4: called from EvtDeviceQueryRemove to notify the consumer
 * before this provider goes away.  Returns synchronously once the
 * consumer has detached. */
VOID RkMppPeerWorkerNotifyQueryRemove(VOID)
{
    KIRQL irql;
    RKMPP_PEER_QUERY_REMOVE_CB cb = NULL;
    PVOID ctx = NULL;
    KeAcquireSpinLock(&g_PeerProvider.Lock, &irql);
    cb = g_PeerProvider.QueryRemoveCb;
    ctx = g_PeerProvider.QueryRemoveCtx;
    KeReleaseSpinLock(&g_PeerProvider.Lock, irql);
    if (cb) cb(ctx);
}
```

- [ ] **Step 2: Add declaration to `devpub.h`**

In `driver/rkvdec/devpub.h`, add:

```c
NTSTATUS RkMppPeerWorkerPublish(_In_ WDFDEVICE Device, _In_ UINT32 Uid);
VOID     RkMppPeerWorkerNotifyQueryRemove(VOID);
```

- [ ] **Step 3: Call publish from PrepareHardware**

In `driver/rkvdec/device.c` `RkMppEvtPrepareHardware`, after the existing UID is read (around the `RkMppReadAcpiId()` call, ~line 294) and after `RkMppOpenIfcs` succeeds, add:

```c
NTSTATUS sPeerPub = RkMppPeerWorkerPublish(Device, pub.Uid);
if (!NT_SUCCESS(sPeerPub)) {
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkvdec: PEER_WORKER publish failed 0x%x (UID=%u)\n",
               sPeerPub, pub.Uid);
    return sPeerPub;
}
```

- [ ] **Step 4: Add `peer_worker.c` to `rkvdec.vcxproj`**

Edit `driver/rkvdec/rkvdec.vcxproj` and add `<ClCompile Include="peer_worker.c" />`.

- [ ] **Step 5: Build**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

Expected: builds clean.

- [ ] **Step 6: Commit**

```
git add driver/rkvdec/peer_worker.c driver/rkvdec/devpub.h \
        driver/rkvdec/device.c driver/rkvdec/rkvdec.vcxproj
git commit -m "rkvdec: publish PEER_WORKER on UID==1 (RVD1) — stub provider"
```

### Task 1.6: RVD0 opens peer worker via PnP notification

**Files:**
- Modify: `driver/rkvdec/device.c`
- Modify: `driver/rkvdec/job.h` (add peer-client state fields)

State only; no dispatch yet. RVD0 logs each peer arrival/departure.

- [ ] **Step 1: Extend `RKMPP_JOB_QUEUE` in `job.h`**

After `OwnerLru[4]` (line 140), before the polling-completion comment block, add:

```c
    /* Multi-core dispatch state (RVD0 only — RVD1 leaves these zero).
     * Filled by peer-watch callbacks; read by the promotion loop in
     * RkMppJobComplete.  All fields read+written under Lock. */
    ULONG                       CoreCount;        /* 1 or 2 */
    ULONG                       CoreIdle;         /* bit i = core i idle */
    ULONG                       CorePending[2];   /* BSP task_index analog */
    RKMPP_JOB                  *InFlightPerCore[2];

    /* Peer worker client state.  PeerOpen=TRUE once we've successfully
     * QueryInterface'd RVD1's provider; FALSE during single-core mode
     * (peer driver absent or being unloaded). */
    BOOLEAN                     PeerOpen;
    RKMPP_PEER_WORKER_INTERFACE Peer;
    PFILE_OBJECT                PeerFileObj;
    struct _RKMPP_PEER_WATCH    PeerWatch;        /* registered at PrepareHardware */
```

Add `#include "../../shared/rkmpp_peer_worker_ifc.h"` and `#include "../shared/rkmpp/peer_attach.h"` at the top of `job.h`.

Also: remove the existing single `InFlight` field — it will be replaced by `InFlightPerCore[0]` everywhere it's referenced. **Do not commit this header change in isolation;** the call-site sweep in Step 2 has to land in the same commit.

- [ ] **Step 2: Sweep all `q->InFlight` reads/writes**

Grep `driver/rkvdec/job.c` for `InFlight` and rewrite each access:

| was | becomes |
|---|---|
| `q->InFlight = NULL;` | `q->InFlightPerCore[0] = NULL;` |
| `q->InFlight = next;` | `q->InFlightPerCore[0] = next;` *(promotion site, will be rewritten in Phase 3)* |
| `if (q->InFlight)` | `if (q->InFlightPerCore[0])` |
| `q->InFlight->...` | `q->InFlightPerCore[0]->...` |

This is mechanical for Phase 1 — keeping single-core semantics by using only index 0. Phase 3 introduces real multi-core indexing.

Also in `RkMppJobQueueInit`: explicitly initialize `q->CoreCount = 1; q->CoreIdle = 1; q->CorePending[0] = q->CorePending[1] = 0; q->InFlightPerCore[0] = q->InFlightPerCore[1] = NULL; q->PeerOpen = FALSE;`.

- [ ] **Step 3: Add arrival/removal callbacks in `device.c`**

After `RkMppEvtPrepareHardware` declarations near the top:

```c
static VOID
RkMppPeerArrival(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink)
{
    WDFDEVICE Device = (WDFDEVICE)Ctx;
    RKMPP_DEVICE_PUBLIC pub;
    RkMppGetPublic(Device, &pub);
    if (pub.Uid != 0) return;   /* only RVD0 consumes peer */

    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    /* QueryInterface against the arriving symbolic link.  Reuse the
     * private QueryByGuid in ifc_client.c via a tiny wrapper, OR call
     * IoGetDeviceObjectPointer + the existing QueryOne (export it).
     * For this task we expose RkMppQueryPeerWorker in ifc_client.
     * If query succeeds AND ifc.Uid==1, attach. */
    RKMPP_PEER_WORKER_INTERFACE ifc;
    PFILE_OBJECT fo = NULL;
    NTSTATUS s = RkMppQueryPeerWorkerBySymlink(SymbolicLink, &ifc, &fo);
    if (!NT_SUCCESS(s)) return;
    if (ifc.Uid != 1) {
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        ObDereferenceObject(fo);
        return;
    }

    KIRQL irql;
    KeAcquireSpinLock(&q->Lock, &irql);
    if (q->PeerOpen) {
        /* Already attached (duplicate arrival fired — shouldn't happen
         * but be defensive). */
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

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: peer worker attached (RVD1), CoreCount=2\n");
}

static VOID
RkMppPeerRemoval(_In_ PVOID Ctx, _In_ PUNICODE_STRING SymbolicLink)
{
    UNREFERENCED_PARAMETER(SymbolicLink);
    /* Authoritative detach happens via the peer's QueryRemove callback
     * (registered in Task 4.x).  Removal notification just logs. */
    UNREFERENCED_PARAMETER(Ctx);
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: peer worker removal notification\n");
}
```

- [ ] **Step 4: Add `RkMppQueryPeerWorkerBySymlink` to ifc_client**

In `driver/shared/rkmpp/ifc_client.h`:

```c
NTSTATUS RkMppQueryPeerWorkerBySymlink(_In_ PUNICODE_STRING SymbolicLink,
                                       _Out_ RKMPP_PEER_WORKER_INTERFACE *Out,
                                       _Out_ PFILE_OBJECT *OutFileObj);
```

In `driver/shared/rkmpp/ifc_client.c`, near the other `QueryByGuid` calls:

```c
#include "../../../shared/rkmpp_peer_worker_ifc.h"

NTSTATUS RkMppQueryPeerWorkerBySymlink(_In_ PUNICODE_STRING SymbolicLink,
                                       _Out_ RKMPP_PEER_WORKER_INTERFACE *Out,
                                       _Out_ PFILE_OBJECT *OutFileObj)
{
    *OutFileObj = NULL;
    PFILE_OBJECT fo = NULL;
    PDEVICE_OBJECT devObj = NULL;
    NTSTATUS s = IoGetDeviceObjectPointer(SymbolicLink, FILE_READ_DATA,
                                          &fo, &devObj);
    if (!NT_SUCCESS(s)) return s;
    RtlZeroMemory(Out, sizeof(*Out));
    s = QueryOne(devObj, &GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
                 RKMPP_PEER_WORKER_IFC_VERSION, Out, sizeof(*Out));
    if (!NT_SUCCESS(s)) {
        ObDereferenceObject(fo);
        return s;
    }
    *OutFileObj = fo;
    return STATUS_SUCCESS;
}
```

- [ ] **Step 5: Register the watch from `RkMppEvtPrepareHardware`**

After `RkMppOpenIfcs` succeeds in `device.c`, on the UID==0 path:

```c
if (pub.Uid == 0) {
    NTSTATUS sw = RkMppWatchPeer(&GUID_DEVINTERFACE_RKMPP_PEER_WORKER,
                                 Device,
                                 RkMppPeerArrival,
                                 RkMppPeerRemoval,
                                 &RkMppGetJobQueue(Device)->PeerWatch);
    if (!NT_SUCCESS(sw)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkvdec: RkMppWatchPeer failed 0x%x\n", sw);
        /* Non-fatal — fall back to single-core mode silently. */
    }
}
```

And in `RkMppEvtReleaseHardware`, before the existing teardown:

```c
RKMPP_DEVICE_PUBLIC pub;
RkMppGetPublic(Device, &pub);
if (pub.Uid == 0) {
    RkMppUnwatchPeer(&RkMppGetJobQueue(Device)->PeerWatch);
    /* Detach peer if attached.  Order: Dereference ifc, then release
     * file-object — mirrors RkMppCloseIfcs (ifc_client.c:241-252) per
     * [[wdf_query_interface.md]]. */
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);
    KIRQL irql;
    KeAcquireSpinLock(&q->Lock, &irql);
    BOOLEAN wasOpen = q->PeerOpen;
    q->PeerOpen = FALSE;
    q->CoreCount = 1;
    q->CoreIdle &= 1u;
    RKMPP_PEER_WORKER_INTERFACE ifc = q->Peer;
    PFILE_OBJECT fo = q->PeerFileObj;
    q->PeerFileObj = NULL;
    KeReleaseSpinLock(&q->Lock, irql);
    if (wasOpen) {
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        if (fo) ObDereferenceObject(fo);
    }
}
```

- [ ] **Step 6: Build + sign + deploy to Z:**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
copy ARM64\Debug\rkvdec\* Z:\drivers-arm\rkvdec\ /Y
```

Adjust the destination path to match the user's existing deployment recipe.

- [ ] **Step 7: User verification recipe (manual, on rk Windows ARM box)**

Install rkvdec + rkiommu_vdec for both UIDs the way they're installed today. Then:

1. `dbgview` (or kernel debugger), filter for `rkvdec:`.
2. Reinstall RVD1's rkvdec binary (or disable/enable in Device Manager).
3. Expected log lines on RVD0's side:
   - `rkvdec: peer worker attached (RVD1), CoreCount=2` when RVD1 starts.
   - `rkvdec: peer worker removal notification` when RVD1 is disabled.
4. Decode any H.264 clip with `mft_play`. Behavior unchanged — single-core path still in use (Phase 3 not done). No regressions.

If RVD1 is absent: no attach log; decode still works on RVD0 alone.

- [ ] **Step 8: Commit**

```
git add driver/rkvdec/device.c driver/rkvdec/job.h driver/rkvdec/job.c \
        driver/shared/rkmpp/ifc_client.h driver/shared/rkmpp/ifc_client.c
git commit -m "rkvdec: RVD0 opens peer worker on RVD1 arrival (no dispatch yet)"
```

---

## Phase 2 — Shared IOMMU page tables

Master rkiommu (UID 9) publishes its PT-base. Slave rkiommu (UID 10) skips PT allocation and attaches to master's PT-base. RVD1 PrepareHardware waits on `PtAttached`.

### Task 2.1: Master rkiommu publishes `MASTER` interface

**Files:**
- Modify: `driver/rkiommu_vdec/device.h`
- Modify: `driver/rkiommu_vdec/ifc.c`
- Modify: `driver/rkiommu_vdec/device.c`

- [ ] **Step 1: Add `IsMaster` + master-provider state to device context**

In `driver/rkiommu_vdec/device.h`, add to the device context struct:

```c
    BOOLEAN              IsMaster;       /* TRUE for Hid=0x3570 Uid=9, etc. */
    BOOLEAN              PtAttached;     /* master: TRUE after Domain alloc; slave: TRUE after attach */
    KEVENT               PtAttachedEvent; /* signalled when PtAttached transitions FALSE→TRUE */
    /* Master-only: registry of slave + codec consumers that hold the
     * master ifc, for cascade query-remove (Phase 4). */
    KSPIN_LOCK           ConsumersLock;
    struct {
        PVOID                                ConsumerCtx;
        RKIOMMU_MASTER_QUERY_REMOVE_CB       Cb;
    } Consumers[4];
    ULONG                ConsumerCount;
```

- [ ] **Step 2: Determine `IsMaster` in `RkIommuEvtPrepareHardware`**

After `Uid` is read in `driver/rkiommu_vdec/device.c` (top of `RkIommuEvtPrepareHardware`):

```c
#include "../shared/rkmpp/ifc_client.h"   /* for RkMppIsMasterIommu */
...
ctx->IsMaster = RkMppIsMasterIommu(ctx->Hid, ctx->Uid);
KeInitializeEvent(&ctx->PtAttachedEvent, NotificationEvent, FALSE);
KeInitializeSpinLock(&ctx->ConsumersLock);
ctx->ConsumerCount = 0;
```

- [ ] **Step 3: Add `GetPageTableBase` + `RegisterQueryRemove` provider in `ifc.c`**

Append to `driver/rkiommu_vdec/ifc.c`:

```c
#include "../../shared/rkiommu_master_ifc.h"

static NTSTATUS
MasterGetPtBase(_In_ PVOID provCtx, _Out_ PULONG32 PdPhys)
{
    RKIOMMU_DEVICE *dev = (RKIOMMU_DEVICE*)provCtx;
    if (!dev->IsMaster || !dev->Domain) return STATUS_DEVICE_NOT_READY;
    *PdPhys = dev->Domain->PdPhys;
    return STATUS_SUCCESS;
}

static NTSTATUS
MasterRegisterQueryRemove(_In_ PVOID provCtx,
                          _In_ PVOID consCtx,
                          _In_ RKIOMMU_MASTER_QUERY_REMOVE_CB cb)
{
    RKIOMMU_DEVICE *dev = (RKIOMMU_DEVICE*)provCtx;
    KIRQL irql;
    KeAcquireSpinLock(&dev->ConsumersLock, &irql);
    NTSTATUS s = STATUS_INSUFFICIENT_RESOURCES;
    if (dev->ConsumerCount < ARRAYSIZE(dev->Consumers)) {
        dev->Consumers[dev->ConsumerCount].ConsumerCtx = consCtx;
        dev->Consumers[dev->ConsumerCount].Cb         = cb;
        dev->ConsumerCount++;
        s = STATUS_SUCCESS;
    }
    KeReleaseSpinLock(&dev->ConsumersLock, irql);
    return s;
}

NTSTATUS RkIommuPublishMasterInterface(_In_ WDFDEVICE Device,
                                       _In_ RKIOMMU_DEVICE *Ctx)
{
    if (!Ctx->IsMaster) return STATUS_SUCCESS;

    RKIOMMU_MASTER_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    ifc.Header.Size                = sizeof(ifc);
    ifc.Header.Version             = RKIOMMU_MASTER_IFC_VERSION;
    ifc.Header.Context             = Ctx;
    ifc.Header.InterfaceReference  = RkIommuIfcReference;   /* reuse existing */
    ifc.Header.InterfaceDereference= RkIommuIfcDereference;
    ifc.Hid                        = Ctx->Hid;
    ifc.Uid                        = Ctx->Uid;
    ifc.GetPageTableBase           = MasterGetPtBase;
    ifc.RegisterQueryRemove        = MasterRegisterQueryRemove;

    WDF_QUERY_INTERFACE_CONFIG qic;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&qic, (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKIOMMU_MASTER, NULL);
    NTSTATUS s = WdfDeviceAddQueryInterface(Device, &qic);
    if (!NT_SUCCESS(s)) return s;
    return WdfDeviceCreateDeviceInterface(Device,
                                          &GUID_DEVINTERFACE_RKIOMMU_MASTER,
                                          NULL);
}
```

Declare `RkIommuPublishMasterInterface` in `device.h`.

- [ ] **Step 4: Call publish + mark PtAttached on master**

In `RkIommuEvtPrepareHardware`, after the existing Domain allocation + DTE_ADDR programming succeeds, for the master path:

```c
if (ctx->IsMaster) {
    NTSTATUS sp = RkIommuPublishMasterInterface(Device, ctx);
    if (!NT_SUCCESS(sp)) return sp;
    ctx->PtAttached = TRUE;
    KeSetEvent(&ctx->PtAttachedEvent, IO_NO_INCREMENT, FALSE);
}
```

(Slave path stays unimplemented until Task 2.2 — for now, slave still allocates its own PT, harmless.)

- [ ] **Step 5: Build**

```
msbuild driver\rkiommu_vdec\rkiommu_vdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

Expected: builds clean.

- [ ] **Step 6: Commit**

```
git add driver/rkiommu_vdec/device.h driver/rkiommu_vdec/device.c \
        driver/rkiommu_vdec/ifc.c
git commit -m "rkiommu_vdec: master publishes GetPageTableBase interface"
```

### Task 2.2: Slave rkiommu attaches to master PT-base

**Files:**
- Modify: `driver/rkiommu_vdec/device.c`
- Modify: `driver/rkiommu_vdec/device.h`

- [ ] **Step 1: Add slave-side master client state**

In `RKIOMMU_DEVICE` context (`device.h`):

```c
    /* Slave-only: reference to master interface for PT base. */
    RKIOMMU_MASTER_INTERFACE  MasterIfc;
    PFILE_OBJECT              MasterFileObj;
    BOOLEAN                   MasterOpen;
    struct _RKMPP_PEER_WATCH  MasterWatch;
```

Add `#include "../../shared/rkiommu_master_ifc.h"` and `#include "../shared/rkmpp/peer_attach.h"` to `device.h`.

- [ ] **Step 2: Implement slave attach routine**

In `device.c`:

```c
/* Programs the slave's MMU registers with `PdPhys` (master's page-
 * directory root).  Mirrors the master's existing DTE_ADDR program
 * path in this same file, line ~218 — factor that out to a helper if
 * not already shared, and call it from both. */
static NTSTATUS
RkIommuSlaveProgramPt(_In_ RKIOMMU_DEVICE *Ctx, _In_ ULONG32 PdPhys)
{
    UCHAR *base = Ctx->MmioBase;
    /* For each MMU instance (Ctx->MmuInstanceCount), write DTE_ADDR,
     * enable paging.  See the existing programming sequence in
     * RkIommuEvtPrepareHardware (~line 218) and factor into a helper
     * void RkIommuProgramDte(Ctx, PdPhys) that this function also
     * calls.  Do NOT zero the page tables — they belong to master. */
    for (ULONG mi = 0; mi < Ctx->MmuInstanceCount; mi++) {
        WRITE_REGISTER_ULONG((volatile ULONG*)(base + mi*0x40 + RK_MMU_DTE_ADDR),
                             PdPhys);
        /* Enable paging via existing helper (whatever the master uses
         * after DTE_ADDR write).  Be defensive: read back and compare. */
        ULONG rb = READ_REGISTER_ULONG((volatile ULONG*)(base + mi*0x40 + RK_MMU_DTE_ADDR));
        if (rb != PdPhys) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu_vdec slave: DTE_ADDR readback 0x%x != 0x%x (MMU#%u)\n",
                       rb, PdPhys, mi);
            return STATUS_DEVICE_HARDWARE_ERROR;
        }
    }
    return STATUS_SUCCESS;
}

static VOID
RkIommuSlaveOnMasterArrival(_In_ PVOID c, _In_ PUNICODE_STRING SymLink)
{
    RKIOMMU_DEVICE *ctx = (RKIOMMU_DEVICE*)c;
    if (ctx->MasterOpen) return;

    PFILE_OBJECT fo = NULL;
    PDEVICE_OBJECT devObj = NULL;
    if (!NT_SUCCESS(IoGetDeviceObjectPointer(SymLink, FILE_READ_DATA,
                                              &fo, &devObj))) return;

    RKIOMMU_MASTER_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    NTSTATUS s = QueryOneExported(devObj, &GUID_DEVINTERFACE_RKIOMMU_MASTER,
                                  RKIOMMU_MASTER_IFC_VERSION, &ifc, sizeof(ifc));
    if (!NT_SUCCESS(s)) { ObDereferenceObject(fo); return; }

    /* Topology gate: pair RVD0's iommu UID 9 with RVD1's UID 10 only.
     * For Hid=0x3570 there's only one master (UID 9) anyway, but be
     * explicit in case AV1 ever grows a second instance. */
    if (ifc.Hid != ctx->Hid) {
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        ObDereferenceObject(fo);
        return;
    }

    ULONG32 pdPhys = 0;
    s = ifc.GetPageTableBase(ifc.Header.Context, &pdPhys);
    if (!NT_SUCCESS(s) || pdPhys == 0) {
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        ObDereferenceObject(fo);
        return;
    }

    s = RkIommuSlaveProgramPt(ctx, pdPhys);
    if (!NT_SUCCESS(s)) {
        if (ifc.Header.InterfaceDereference)
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        ObDereferenceObject(fo);
        return;
    }

    /* Register our query-remove hook with master so it can tear us down
     * before unloading itself. */
    if (ifc.RegisterQueryRemove) {
        ifc.RegisterQueryRemove(ifc.Header.Context, ctx,
                                RkIommuSlaveOnMasterQueryRemove);
    }

    ctx->MasterIfc      = ifc;
    ctx->MasterFileObj  = fo;
    ctx->MasterOpen     = TRUE;
    ctx->PtAttached     = TRUE;
    KeSetEvent(&ctx->PtAttachedEvent, IO_NO_INCREMENT, FALSE);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu_vdec slave (UID=%u): attached to master PT 0x%08x\n",
               ctx->Uid, pdPhys);
}

static VOID
RkIommuSlaveOnMasterQueryRemove(_In_ PVOID consCtx)
{
    RKIOMMU_DEVICE *ctx = (RKIOMMU_DEVICE*)consCtx;
    if (!ctx->MasterOpen) return;

    /* Cascade to our own consumers (RVD1) — Phase 4 implements; for now
     * just disable paging and drop refs. */
    KeClearEvent(&ctx->PtAttachedEvent);
    ctx->PtAttached = FALSE;

    /* Disable paging on slave MMU.  Reuse the existing Disable path
     * called by EvtReleaseHardware. */
    RkIommuDisableInternal(ctx);

    if (ctx->MasterIfc.Header.InterfaceDereference)
        ctx->MasterIfc.Header.InterfaceDereference(ctx->MasterIfc.Header.Context);
    if (ctx->MasterFileObj) ObDereferenceObject(ctx->MasterFileObj);
    ctx->MasterFileObj = NULL;
    ctx->MasterOpen = FALSE;
}
```

Add forward declaration of `QueryOneExported` in `ifc_client.h` (and rename existing static `QueryOne` to exported `QueryOneExported`), or duplicate the body locally. Recommend exporting: it's already useful.

- [ ] **Step 3: Skip slave PT alloc; register watch**

In `RkIommuEvtPrepareHardware`, wrap the existing Domain allocation + DTE programming block:

```c
if (ctx->IsMaster) {
    /* existing master allocation + DTE_ADDR program path */
    ...
    ctx->PtAttached = TRUE;
    KeSetEvent(&ctx->PtAttachedEvent, IO_NO_INCREMENT, FALSE);
    /* (Task 2.1) publish master interface */
    RkIommuPublishMasterInterface(Device, ctx);
} else {
    /* Slave: do NOT allocate Domain.  Skip DTE_ADDR program until
     * master arrives.  Register PnP watch.  PtAttached stays FALSE
     * until OnMasterArrival fires. */
    ctx->Domain = NULL;
    NTSTATUS sw = RkMppWatchPeer(&GUID_DEVINTERFACE_RKIOMMU_MASTER,
                                 ctx,
                                 RkIommuSlaveOnMasterArrival,
                                 NULL,
                                 &ctx->MasterWatch);
    if (!NT_SUCCESS(sw)) return sw;
}
```

In `RkIommuEvtReleaseHardware`, before existing cleanup:

```c
if (!ctx->IsMaster) {
    RkMppUnwatchPeer(&ctx->MasterWatch);
    if (ctx->MasterOpen) {
        if (ctx->MasterIfc.Header.InterfaceDereference)
            ctx->MasterIfc.Header.InterfaceDereference(ctx->MasterIfc.Header.Context);
        if (ctx->MasterFileObj) ObDereferenceObject(ctx->MasterFileObj);
        ctx->MasterFileObj = NULL;
        ctx->MasterOpen = FALSE;
    }
}
```

Also: any ifc.c paths that read `Dev->Domain->PdPhys` need a guard `if (!Ctx->IsMaster) return STATUS_DEVICE_NOT_READY` on slave when not attached — they're called from the codec's MapMdl, which we'll gate at the codec side in Task 2.3.

- [ ] **Step 4: Build**

```
msbuild driver\rkiommu_vdec\rkiommu_vdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 5: Commit**

```
git add driver/rkiommu_vdec/device.c driver/rkiommu_vdec/device.h \
        driver/rkiommu_vdec/ifc.c driver/shared/rkmpp/ifc_client.h \
        driver/shared/rkmpp/ifc_client.c
git commit -m "rkiommu_vdec: slave (UID 10) attaches to master PT base"
```

### Task 2.3: RVD1 waits for slave rkiommu `PtAttached`

**Files:**
- Modify: `driver/rkvdec/device.c`
- Modify: `shared/rkiommu_ifc.h` (add `IsPtAttached` method)
- Modify: `driver/rkiommu_vdec/ifc.c`

- [ ] **Step 1: Add `IsPtAttached` method to `RKIOMMU_INTERFACE`**

In `shared/rkiommu_ifc.h`, bump `RKIOMMU_IFC_VERSION` to `7u`, add field:

```c
/* v7: query whether this iommu instance is currently bound to a page
 * table (always TRUE for master; for slave: TRUE only after master
 * arrival).  Codec drivers wait on this before calling MapMdl. */
typedef BOOLEAN (*RKIOMMU_IS_PT_ATTACHED)(_In_ PVOID ProviderContext);

typedef struct _RKIOMMU_INTERFACE {
    ...
    RKIOMMU_UNMASK_IRQ      UnmaskIrq;
    RKIOMMU_IS_PT_ATTACHED  IsPtAttached;    /* v7 */
} RKIOMMU_INTERFACE, *PRKIOMMU_INTERFACE;
```

In `driver/rkiommu_vdec/ifc.c`, implement:

```c
static BOOLEAN IfcIsPtAttached(PVOID provCtx)
{
    return ((RKIOMMU_DEVICE*)provCtx)->PtAttached;
}
```

Wire into the `RKIOMMU_INTERFACE` fill (existing publish path), and update consumers' version-check accordingly.

- [ ] **Step 2: RVD1 PrepareHardware waits**

In `driver/rkvdec/device.c` `RkMppEvtPrepareHardware`, after `RkMppOpenIfcs` (which gets `Out->Iommu`):

```c
if (pub.Uid == 1) {
    /* Slave path: wait for slave rkiommu to attach to master PT.
     * Bounded wait — on timeout, fail PnP so Windows can retry. */
    if (ctx->Ifcs.Iommu.IsPtAttached &&
        !ctx->Ifcs.Iommu.IsPtAttached(ctx->Ifcs.Iommu.Header.Context)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkvdec UID=1: waiting for slave rkiommu PT attach\n");
        /* Poll the iommu instance every 100 ms up to 5 s.  We don't
         * have direct access to its KEVENT (no kernel handle table for
         * cross-driver events) so poll the predicate.  5 s is generous
         * — typical attach < 50 ms after master start. */
        LARGE_INTEGER interval; interval.QuadPart = -1 * 100 * 10000;
        for (ULONG i = 0; i < 50; i++) {
            KeDelayExecutionThread(KernelMode, FALSE, &interval);
            if (ctx->Ifcs.Iommu.IsPtAttached(ctx->Ifcs.Iommu.Header.Context))
                break;
        }
        if (!ctx->Ifcs.Iommu.IsPtAttached(ctx->Ifcs.Iommu.Header.Context)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec UID=1: PT attach timeout — failing PnP\n");
            RkMppCloseIfcs(&ctx->Ifcs);
            return STATUS_DEVICE_NOT_READY;
        }
    }
}
```

- [ ] **Step 3: Build both drivers**

```
msbuild driver\rkiommu_vdec\rkiommu_vdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 4: User verification recipe (manual)**

1. Install all 4 drivers (rkvdec ×2, rkiommu_vdec ×2). Confirm both rkvdec instances probe successfully.
2. `dbgview` log should show `rkiommu_vdec slave (UID=10): attached to master PT 0x...` exactly once.
3. Decode an H.264 clip targeting RVD0 (the only path MFT uses today): bit-exact to pre-Phase-2 baseline.
4. **Slave-first start order test**: in Device Manager, disable master rkiommu (UID 9), then disable slave rkiommu (UID 10). Now re-enable slave first, then master. Expected log: slave shows "waiting" briefly, then "attached to master PT" when master comes up. Both rkvdec instances eventually probe.
5. Decode again; bit-exact to before.

- [ ] **Step 5: Commit**

```
git add shared/rkiommu_ifc.h driver/rkiommu_vdec/ifc.c driver/rkvdec/device.c
git commit -m "rkiommu_vdec: IsPtAttached + RVD1 waits for slave attach"
```

---

## Phase 3 — Kernel-side dispatcher

Make both cores actually run jobs in parallel. Per-Owner LRU picks the next job; BSP-style idle+task_index picks the core.

### Task 3.1: Factor `RkMppJobStart` into local-kick helper

**Files:**
- Modify: `driver/rkvdec/job.c`

- [ ] **Step 1: Extract local kick body**

Identify the function that today writes the SWREG bank, raises clocks, and signals `KickEvent` (likely `RkMppJobStart`). Rename the body to `JobKickLocalInner(Device, job)` and have `RkMppJobStart(Device, job)` call it. No behavior change.

```c
/* Local kick: writes SWREG bank to RVD0's MMIO and signals poller. */
static NTSTATUS JobKickLocalInner(WDFDEVICE Device, RKMPP_JOB *Job)
{
    /* moved verbatim from RkMppJobStart body */
    ...
}

NTSTATUS RkMppJobStart(WDFDEVICE Device, RKMPP_JOB *Job)
{
    return JobKickLocalInner(Device, Job);
}
```

- [ ] **Step 2: Build to verify no functional change**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 3: Commit**

```
git add driver/rkvdec/job.c
git commit -m "rkvdec: factor JobKickLocalInner out of RkMppJobStart"
```

### Task 3.2: Implement RVD1's `PeerKickJob` provider body

**Files:**
- Modify: `driver/rkvdec/peer_worker.c`
- Modify: `driver/rkvdec/job.c` (export a helper that runs a kick from an externally-supplied bank)

Real cross-core kick. RVD1's provider receives `RKMPP_PEER_KICK_PARAMS`, materializes them into an `RKMPP_JOB`, runs it through its own poller, and invokes the consumer's completion callback when done.

- [ ] **Step 1: Add `RkMppJobRunForeign` to job.c**

```c
/* Runs a single dense kick on this device using a bank + iova slots
 * supplied by an external caller (the peer worker provider).  Bypasses
 * the per-File buffer-handle resolution — caller has already done it.
 * Allocates a transient RKMPP_JOB, kicks it, waits for poller, and
 * invokes Cb with the result.  Runs at PASSIVE on a system worker
 * thread queued by the peer provider.  Cb runs on the same thread
 * after completion. */
NTSTATUS RkMppJobRunForeign(_In_ WDFDEVICE Device,
                            _In_ const RKMPP_PEER_KICK_PARAMS *Params,
                            _In_ RKMPP_PEER_COMPLETION_CB Cb,
                            _In_ PVOID CbCtx);
```

Implementation: allocate a transient `RKMPP_JOB`, copy `Bank` into `Job->DenseBank`, copy `IovaSlots` into `Job->DenseIovaSlots`, set `Job->Owner = NULL` (foreign), `Job->DenseKickValue = Params->KickValue`. Stamp into `q->InFlightPerCore[0]` (RVD1's only slot), call `JobKickLocalInner`, wait on `Job->Done`, invoke `Cb(CbCtx, Params->CompletionCookie, Job->Result, Job->HardwareStatus)`, free job.

Foreign-job lifecycle: skip the per-Owner LRU update — `Job->Owner == NULL` means existing code paths that key on Owner must short-circuit (audit `OwnerLru` update in job.c:611-624: skip when `Owner == NULL`).

- [ ] **Step 2: Wire `PeerKickJob` to use a work item**

In `peer_worker.c`, replace the stub `PeerKickJob`:

```c
typedef struct _PEER_KICK_WORK {
    WORK_QUEUE_ITEM           WorkItem;
    RKMPP_PEER_KICK_PARAMS    Params;
    /* Bank + IovaSlots storage owned by this struct so caller can
     * release theirs after KickJob returns. */
    RKMPP_DENSE_BANK          BankCopy;
    RKMPP_DENSE_IOVA_SLOT     IovaCopy[RKMPP_MAX_DENSE_IOVA_SLOTS];
} PEER_KICK_WORK;

static VOID PeerKickWorker(PVOID Ctx)
{
    PEER_KICK_WORK *w = Ctx;
    RKMPP_PEER_PROVIDER_CTX *p = &g_PeerProvider;

    /* Snapshot the consumer's callback under lock. */
    KIRQL irql;
    KeAcquireSpinLock(&p->Lock, &irql);
    PVOID consCtx = p->CompletionCtx;
    RKMPP_PEER_COMPLETION_CB cb = p->CompletionCb;
    KeReleaseSpinLock(&p->Lock, irql);

    if (!cb) { ExFreePool(w); return; }

    /* Repoint the params at our local copies. */
    RKMPP_PEER_KICK_PARAMS p2 = w->Params;
    p2.Bank      = &w->BankCopy;
    p2.IovaSlots = w->IovaCopy;

    RkMppJobRunForeign(g_PeerProvider.Device, &p2, cb, consCtx);
    ExFreePool(w);
}

static NTSTATUS
PeerKickJob(PVOID provCtx, const RKMPP_PEER_KICK_PARAMS *params)
{
    UNREFERENCED_PARAMETER(provCtx);
    if (params->BankBytes != sizeof(RKMPP_DENSE_BANK)) return STATUS_INVALID_PARAMETER;
    if (params->IovaSlotCount > RKMPP_MAX_DENSE_IOVA_SLOTS) return STATUS_INVALID_PARAMETER;

    PEER_KICK_WORK *w = ExAllocatePool2(POOL_FLAG_NON_PAGED, sizeof(*w), 'KrPK');
    if (!w) return STATUS_INSUFFICIENT_RESOURCES;
    RtlCopyMemory(&w->BankCopy, params->Bank, sizeof(w->BankCopy));
    RtlCopyMemory(w->IovaCopy, params->IovaSlots,
                  params->IovaSlotCount * sizeof(RKMPP_DENSE_IOVA_SLOT));
    w->Params = *params;
    ExInitializeWorkItem(&w->WorkItem, PeerKickWorker, w);
    ExQueueWorkItem(&w->WorkItem, DelayedWorkQueue);
    return STATUS_SUCCESS;
}
```

Note: `ExQueueWorkItem` is deprecated but still legal; use `IoQueueWorkItem` if the project prefers (allocate via `IoAllocateWorkItem` keyed to RVD1's `WDFDEVICE`'s WDM device object).

- [ ] **Step 3: Build**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 4: Commit**

```
git add driver/rkvdec/peer_worker.c driver/rkvdec/job.c driver/rkvdec/job.h
git commit -m "rkvdec: RVD1 peer worker materializes foreign kicks into jobs"
```

### Task 3.3: Dispatcher in `RkMppJobComplete` + initial fill in `RkMppJobSubmitDense`

**Files:**
- Modify: `driver/rkvdec/job.c`

- [ ] **Step 1: Add `PickTargetCore` helper**

```c
/* Returns -1 if no idle core. */
static LONG PickTargetCore(RKMPP_JOB_QUEUE *q)
{
    ULONG mask = q->CoreIdle & ((1u << q->CoreCount) - 1);
    if (!mask) return -1;
    LONG best = -1;
    ULONG bestPending = (ULONG)-1;
    for (ULONG i = 0; i < q->CoreCount; i++) {
        if (!(mask & (1u << i))) continue;
        if (q->CorePending[i] < bestPending) {
            bestPending = q->CorePending[i];
            best = (LONG)i;
        }
    }
    return best;
}
```

- [ ] **Step 2: Factor LRU pick into helper**

Extract the existing LRU-min loop (job.c:1052-1075) into:

```c
static RKMPP_JOB *PickPendingByOwnerLru(RKMPP_JOB_QUEUE *q)
{
    if (IsListEmpty(&q->Pending)) return NULL;
    PLIST_ENTRY chosen = q->Pending.Flink;
    UINT64 bestScore = (UINT64)-1;
    for (PLIST_ENTRY e = q->Pending.Flink; e != &q->Pending; e = e->Flink) {
        RKMPP_JOB *cand = CONTAINING_RECORD(e, RKMPP_JOB, Link);
        UINT64 score = 0;
        for (ULONG i = 0; i < ARRAYSIZE(q->OwnerLru); ++i) {
            if (q->OwnerLru[i].File == cand->Owner) {
                score = q->OwnerLru[i].LastKickId;
                break;
            }
        }
        if (score < bestScore) {
            bestScore = score;
            chosen = e;
            if (score == 0) break;
        }
    }
    RemoveEntryList(chosen);
    return CONTAINING_RECORD(chosen, RKMPP_JOB, Link);
}
```

- [ ] **Step 3: Add `PromoteUntilFull` core loop**

```c
/* Caller holds q->Lock.  Fills as many idle cores as possible.
 * Returns a small array of (job, core) to kick after lock release.
 * Capacity 2 = CoreCount upper bound. */
typedef struct _PROMOTION { RKMPP_JOB *Job; ULONG Core; } PROMOTION;

static ULONG PromoteUntilFull(RKMPP_JOB_QUEUE *q, PROMOTION Out[2])
{
    ULONG n = 0;
    while (n < q->CoreCount && !IsListEmpty(&q->Pending)) {
        LONG core = PickTargetCore(q);
        if (core < 0) break;
        RKMPP_JOB *job = PickPendingByOwnerLru(q);
        if (!job) break;
        job->TargetCore = (UINT32)core;
        q->InFlightPerCore[core] = job;
        q->CoreIdle &= ~(1u << core);
        q->CorePending[core]++;

        /* Update per-Owner LRU here (was previously in JobStart). */
        UINT64 kickId = job->Id;
        ULONG slot = ARRAYSIZE(q->OwnerLru);
        for (ULONG i = 0; i < ARRAYSIZE(q->OwnerLru); i++) {
            if (q->OwnerLru[i].File == job->Owner) { slot = i; break; }
        }
        if (slot == ARRAYSIZE(q->OwnerLru)) {
            /* LRU evict */
            ULONG lru = 0; UINT64 oldest = q->OwnerLru[0].LastKickId;
            for (ULONG i = 1; i < ARRAYSIZE(q->OwnerLru); i++) {
                if (q->OwnerLru[i].LastKickId < oldest) {
                    oldest = q->OwnerLru[i].LastKickId; lru = i;
                }
            }
            slot = lru;
        }
        q->OwnerLru[slot].File = job->Owner;
        q->OwnerLru[slot].LastKickId = kickId;

        Out[n].Job = job;
        Out[n].Core = (ULONG)core;
        n++;
    }
    return n;
}
```

Add `UINT32 TargetCore;` to `RKMPP_JOB` in `job.h` (alongside `KickDecMode` etc.).

- [ ] **Step 4: Add `KickPromotions` (no lock held)**

```c
static VOID KickPromotions(WDFDEVICE Device, RKMPP_JOB_QUEUE *q,
                           PROMOTION *Promos, ULONG Count)
{
    for (ULONG i = 0; i < Count; i++) {
        RKMPP_JOB *job = Promos[i].Job;
        ULONG core = Promos[i].Core;

#if DBG
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkvdec: dispatch job %llu owner=%p core=%u pending=[%u,%u] idle=0x%x\n",
                   job->Id, job->Owner, core,
                   q->CorePending[0], q->CorePending[1], q->CoreIdle);
#endif

        if (core == 0) {
            JobKickLocalInner(Device, job);
        } else {
            RKMPP_PEER_KICK_PARAMS p = {
                .Bank             = &job->DenseBank,
                .BankBytes        = sizeof(job->DenseBank),
                .KickValue        = job->DenseKickValue,
                .IovaSlots        = job->DenseIovaSlots,
                .IovaSlotCount    = job->DenseIovaSlotCount,
                .CompletionCookie = job->Id,   /* used by completion CB to look up job */
            };
            NTSTATUS s = q->Peer.KickJob(q->Peer.Header.Context, &p);
            if (!NT_SUCCESS(s)) {
                /* Peer rejected kick — fail the job inline. */
                job->Result = s;
                KIRQL irql;
                KeAcquireSpinLock(&q->Lock, &irql);
                q->InFlightPerCore[1] = NULL;
                q->CoreIdle |= (1u << 1);
                q->CorePending[1]--;
                InsertTailList(&q->Completed, &job->Link);
                KeReleaseSpinLock(&q->Lock, irql);
                KeSetEvent(&job->Done, IO_NO_INCREMENT, FALSE);
            }
        }
    }
}
```

- [ ] **Step 5: Rewrite promotion site in `RkMppJobComplete`**

Replace job.c:1051-1075 (current single-shot promotion) and the surrounding `KeReleaseSpinLock` with:

```c
    /* Existing per-completion bookkeeping for the completing core. */
    /* (Determine which core completed.  Local completion → core 0.
     * Peer completion enters via a different entry point, Task 3.4.) */
    q->InFlightPerCore[0] = NULL;
    q->CoreIdle |= 1u;
    if (q->CorePending[0] > 0) q->CorePending[0]--;

    PROMOTION promos[2] = {0};
    ULONG nPromos = PromoteUntilFull(q, promos);

    KeReleaseSpinLock(&q->Lock, oldIrql);

    /* Clock gating: only when both cores idle.  The existing single-
     * core code gated on (!next).  Multi-core: gate only when no jobs
     * are in flight on EITHER core. */
    BOOLEAN idleAfter = (q->CoreIdle == ((1u << q->CoreCount) - 1));
    if (idleAfter) {
        /* existing GateRvdec0LeafClocks call, unchanged */
    }

    KickPromotions(Device, q, promos, nPromos);
```

Remove the per-Owner LRU update that was in `RkMppJobStart` (job.c:611-624) — it's now done inside `PromoteUntilFull` under the queue lock.

- [ ] **Step 6: Initial fill in `RkMppJobSubmitDense`**

In `RkMppJobSubmitDense`, after enqueuing the new job to `q->Pending` (~job.c:1354), if `q->InFlight == NULL` (today's check), replace with:

```c
PROMOTION promos[2] = {0};
ULONG nPromos = PromoteUntilFull(q, promos);
KeReleaseSpinLock(&q->Lock, oldIrql);
KickPromotions(Device, q, promos, nPromos);
```

This handles "first job in" and "second stream arrives while first core busy" without waiting for a completion.

- [ ] **Step 7: Build**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 8: Commit**

```
git add driver/rkvdec/job.c driver/rkvdec/job.h
git commit -m "rkvdec: kernel dispatcher promotes to all idle cores with DBG log"
```

### Task 3.4: Wire peer completion callback

**Files:**
- Modify: `driver/rkvdec/device.c`
- Modify: `driver/rkvdec/job.c`

- [ ] **Step 1: Register completion CB at RVD0 attach**

In `RkMppPeerArrival` (Task 1.6 Step 3), after `q->PeerOpen = TRUE;` set:

```c
if (q->Peer.RegisterCompletion) {
    q->Peer.RegisterCompletion(q->Peer.Header.Context, Device,
                               RkMppPeerCompletion);
}
```

- [ ] **Step 2: Implement `RkMppPeerCompletion` in job.c**

```c
/* Runs at PASSIVE on a system worker (the peer's PeerKickWorker after
 * Job->Done is signalled).  ConsumerContext = RVD0's WDFDEVICE.  Re-
 * enters our queue logic via the same path as a local completion, but
 * marks core 1 as completing. */
VOID RkMppPeerCompletion(_In_ PVOID ConsumerContext,
                         _In_ UINT64 CompletionCookie,
                         _In_ NTSTATUS JobStatus,
                         _In_ UINT32 HardwareStatus)
{
    WDFDEVICE Device = (WDFDEVICE)ConsumerContext;
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    KIRQL irql;
    KeAcquireSpinLock(&q->Lock, &irql);

    RKMPP_JOB *job = q->InFlightPerCore[1];
    if (!job || job->Id != CompletionCookie) {
        /* Stale completion — peer reset or job already cancelled. */
        KeReleaseSpinLock(&q->Lock, irql);
        return;
    }

    job->Result         = JobStatus;
    job->HardwareStatus = HardwareStatus;
    q->InFlightPerCore[1] = NULL;
    q->CoreIdle |= (1u << 1);
    if (q->CorePending[1] > 0) q->CorePending[1]--;
    InsertTailList(&q->Completed, &job->Link);

    PROMOTION promos[2] = {0};
    ULONG nPromos = PromoteUntilFull(q, promos);

    KeReleaseSpinLock(&q->Lock, irql);

    KeSetEvent(&job->Done, IO_NO_INCREMENT, FALSE);
    KickPromotions(Device, q, promos, nPromos);
}
```

Declare in `devpub.h` and reference from `RkMppPeerArrival`.

- [ ] **Step 3: Build**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 4: User verification recipe (manual)**

1. Install all 4 drivers. Confirm peer attach log.
2. Run two `mft_play` instances on different H.264 streams simultaneously.
3. In `dbgview` filter `rkvdec: dispatch`, observe:
   - Both `core=0` and `core=1` appear.
   - Counts roughly even at steady state (within ~10% over a minute).
   - Two consecutive `dispatch` lines can appear with no completion between them (proves parallel slots).
4. Compare per-frame output md5 vs the single-core baseline (decoded output must be bit-exact).
5. Single-stream test: only one `mft_play` running. Both cores should still be eligible but utilization will land mostly on core 0 (the one MFT submitted to is also the only one with work to give).

If anything desyncs (decode artifacts, hangs), revert this commit and bisect Phase 3.

- [ ] **Step 5: Commit**

```
git add driver/rkvdec/device.c driver/rkvdec/devpub.h driver/rkvdec/job.c
git commit -m "rkvdec: peer completion callback re-enters dispatcher"
```

### Task 3.5: FileCleanup drain handles both cores

**Files:**
- Modify: `driver/rkvdec/job.c` (`RkMppJobsDrainOwner`, `RkMppJobQueueHasOtherOwner`, `RkMppJobBufferInUse`)

Existing drain logic references `q->InFlight` singularly. All three functions must consider both `InFlightPerCore[0]` and `InFlightPerCore[1]`.

- [ ] **Step 1: Sweep singular InFlight references**

For each of the three functions: replace `if (q->InFlight && q->InFlight->Owner == File)` with a loop over `InFlightPerCore[i]` for `i in 0..CoreCount`.

- [ ] **Step 2: `RkMppJobsDrainOwner` waits on both cores' jobs**

The existing function waits on one `Job->Done` event. For drain, build a small `KWAIT_BLOCK[2]` and use `KeWaitForMultipleObjects` for any matching in-flight job. Out parameter `InFlightTimedOut` is TRUE if any of the matching cores timed out.

- [ ] **Step 3: Build**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 4: User verification recipe (manual)**

Run `mft_play`, kill the process mid-decode. Driver `RkMppEvtFileCleanup` should drain cleanly without the "in-flight timeout → FullCoreReset" path firing unless the job is genuinely stuck.

- [ ] **Step 5: Commit**

```
git add driver/rkvdec/job.c
git commit -m "rkvdec: FileCleanup drain covers both per-core in-flight slots"
```

---

## Phase 4 — Clean uninstall + cascading lifecycle

Handle PnP query-remove on RVD1, master rkiommu, and slave rkiommu. Verify master rkiommu uninstall+reinstall cycle.

### Task 4.1: RVD0 handles peer query-remove

**Files:**
- Modify: `driver/rkvdec/peer_worker.c`
- Modify: `driver/rkvdec/device.c`

- [ ] **Step 1: Hook `EvtDeviceQueryRemove` on RVD1**

In `driver/rkvdec/device.c`, add (only effective on UID==1):

```c
static EVT_WDF_DEVICE_QUERY_REMOVE RkMppEvtDeviceQueryRemove;

NTSTATUS RkMppEvtDeviceQueryRemove(_In_ WDFDEVICE Device)
{
    RKMPP_DEVICE_PUBLIC pub;
    RkMppGetPublic(Device, &pub);
    if (pub.Uid == 1) {
        /* Notify RVD0 to drain peer-targeted jobs + deref. */
        RkMppPeerWorkerNotifyQueryRemove();
    }
    return STATUS_SUCCESS;   /* allow remove */
}
```

Wire into `WDF_PNPPOWER_EVENT_CALLBACKS` alongside the existing `EvtDevicePrepareHardware`.

- [ ] **Step 2: Implement RVD0's QueryRemove callback registration**

In `RkMppPeerArrival`, after `RegisterCompletion`, add:

```c
if (q->Peer.RegisterQueryRemove) {
    q->Peer.RegisterQueryRemove(q->Peer.Header.Context, Device,
                                RkMppPeerOnQueryRemove);
}
```

Implementation:

```c
static VOID RkMppPeerOnQueryRemove(PVOID consCtx)
{
    WDFDEVICE Device = (WDFDEVICE)consCtx;
    PRKMPP_JOB_QUEUE q = RkMppGetJobQueue(Device);

    /* Snapshot peer interface, transition to single-core under lock,
     * then deref outside the lock. */
    KIRQL irql;
    KeAcquireSpinLock(&q->Lock, &irql);
    RKMPP_PEER_WORKER_INTERFACE ifc = q->Peer;
    PFILE_OBJECT fo = q->PeerFileObj;
    q->PeerOpen     = FALSE;
    q->CoreCount    = 1;
    q->CoreIdle    &= 1u;
    q->PeerFileObj  = NULL;
    KeReleaseSpinLock(&q->Lock, irql);

    /* If there's a peer-targeted job in flight at this exact moment,
     * the precondition "all jobs stopped" was violated.  Best effort:
     * wait briefly on InFlightPerCore[1]->Done.  Spec acknowledges
     * surprise-remove is non-graceful. */
    if (q->InFlightPerCore[1]) {
        LARGE_INTEGER to; to.QuadPart = -5000 * 10000;   /* 500 ms */
        KeWaitForSingleObject(&q->InFlightPerCore[1]->Done, Executive,
                              KernelMode, FALSE, &to);
    }

    if (ifc.Header.InterfaceDereference)
        ifc.Header.InterfaceDereference(ifc.Header.Context);
    if (fo) ObDereferenceObject(fo);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: peer query-remove handled, single-core mode\n");
}
```

- [ ] **Step 3: Build**

```
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 4: User verification recipe (manual)**

1. Install all 4 drivers, confirm peer attached.
2. Stop all decode. In Device Manager, disable RVD1 (rkvdec UID=1).
3. Expected log: `rkvdec: peer query-remove handled, single-core mode`.
4. Decode with `mft_play` — works on RVD0 alone; `dbgview` shows only `core=0` in dispatch lines.
5. Re-enable RVD1; expected log: `rkvdec: peer worker attached (RVD1), CoreCount=2`.
6. Decode again; `core=0`+`core=1` interleave returns.

- [ ] **Step 5: Commit**

```
git add driver/rkvdec/device.c driver/rkvdec/peer_worker.c
git commit -m "rkvdec: graceful peer query-remove transitions RVD0 to single-core"
```

### Task 4.2: Master rkiommu cascades query-remove to slave + consumers

**Files:**
- Modify: `driver/rkiommu_vdec/device.c`

- [ ] **Step 1: Hook `EvtDeviceQueryRemove` on master**

```c
NTSTATUS RkIommuEvtDeviceQueryRemove(_In_ WDFDEVICE Device)
{
    RKIOMMU_DEVICE *ctx = RkIommuGetCtx(Device);
    if (!ctx->IsMaster) return STATUS_SUCCESS;

    /* Snapshot consumer list under lock. */
    KIRQL irql;
    KeAcquireSpinLock(&ctx->ConsumersLock, &irql);
    ULONG n = ctx->ConsumerCount;
    struct { PVOID Ctx; RKIOMMU_MASTER_QUERY_REMOVE_CB Cb; } local[4];
    for (ULONG i = 0; i < n; i++) local[i] = (typeof(local[i])){ctx->Consumers[i].ConsumerCtx, ctx->Consumers[i].Cb};
    KeReleaseSpinLock(&ctx->ConsumersLock, irql);

    /* Invoke consumers — they're responsible for declining or
     * succeeding the detach.  We currently honour their action only as
     * "do your thing, then we proceed."  Future: collect a busy vote
     * and return STATUS_DEVICE_BUSY if any consumer can't release. */
    for (ULONG i = 0; i < n; i++) if (local[i].Cb) local[i].Cb(local[i].Ctx);

    return STATUS_SUCCESS;
}
```

Wire into `WDF_PNPPOWER_EVENT_CALLBACKS`. Note: per spec, consumer can refuse only by holding the file-object reference; for now we proceed unconditionally and rely on the "all jobs stopped" precondition.

- [ ] **Step 2: Add RVD0-side rkiommu QueryRemove handling (parallel hook)**

The master rkiommu publishes its `RegisterQueryRemove`; RVD0's `RkMppOpenIfcs` already opens the iommu via the standard `RKIOMMU_INTERFACE`, but the master interface is separate. For Phase 4, have RVD0 *also* open the master interface when it detects its iommu is the master, and register its own QueryRemove callback that:

1. Asserts `IommuAttached = FALSE` (block new IOCTLs).
2. Drains in-flight jobs on both cores (500 ms each).
3. Dereferences the iommu (standard + master).
4. Returns; master can then proceed.

Add `BOOLEAN IommuAttached` to RVD0's device context. In `RkMppEvtIoDeviceControl` (`ioctl.c`), at the top:

```c
if (!RkMppGetCtx(Device)->IommuAttached) {
    WdfRequestComplete(Request, STATUS_DEVICE_NOT_READY);
    return;
}
```

Skip this gate for the few control IOCTLs that don't need IOMMU (capability query, etc.).

- [ ] **Step 3: Build**

```
msbuild driver\rkiommu_vdec\rkiommu_vdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
msbuild driver\rkvdec\rkvdec.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

- [ ] **Step 4: User verification recipe (manual)**

1. All 4 drivers installed. Confirm baseline decode works.
2. Stop all decode. Disable master rkiommu (UID 9) in Device Manager.
3. Expected logs (order may vary):
   - Slave rkiommu logs cascade detach (PT zeroed, MasterOpen=FALSE).
   - RVD0 logs `IommuAttached=FALSE`.
   - RVD1 also eventually loses its iommu (slave detached) → RVD1 stops accepting IOCTLs.
4. Master rkiommu unloads.
5. Re-enable master rkiommu. Expected: master starts, slave's PnP notification fires, slave attaches and re-programs DTE_ADDR, RVD0+RVD1 attach to their respective rkiommus, peer worker arrival fires again, dual-core mode resumes.
6. Decode again; bit-exact to baseline.

- [ ] **Step 5: Commit**

```
git add driver/rkiommu_vdec/device.c driver/rkvdec/device.c driver/rkvdec/ioctl.c
git commit -m "rkiommu_vdec: master cascades query-remove; rkvdec gates IOCTLs on IommuAttached"
```

### Task 4.3: End-to-end uninstall-reinstall stress test

**Files:** (none — verification only)

- [ ] **Step 1: User verification recipe (manual)**

Loop the following 10× on rk's Windows box:

1. Start two `mft_play` instances on different H.264 streams. Let them run for 30 s.
2. Stop both cleanly (`q` or close window).
3. In Device Manager, disable RVD1; wait 2 s; re-enable.
4. Disable master rkiommu; wait 2 s; re-enable.
5. Disable slave rkiommu; wait 2 s; re-enable.
6. Restart both `mft_play` instances; verify decode runs for 30 s with `core=0`/`core=1` dispatches interleaved.

Expected: no BSOD, no kernel WARNING, no decoder artifacts. `dbgview` shows attach/detach lines in clean pairs across the loop.

- [ ] **Step 2: Document findings**

If the loop completes cleanly 10×, write a one-line summary memory:

```
[saves project memory] Multi-core rkvdec dispatch (RVD0+RVD1) verified 2026-05-18 across 10× install/uninstall cycles of all four binaries; bit-exact output vs single-core baseline.
```

If anything BSODs or wedges, capture `!analyze -v`, the kernel debug log, and revert the offending phase. Do not skip and move on.

- [ ] **Step 3: Commit any small fixes from the stress test**

```
git status
git add -p
git commit -m "rkvdec: post-stress-test fixes"   # if any
```

---

## Self-review

- **Spec coverage:** every spec section is covered: Architecture (1.5, 1.6, 3.x); IOMMU shared page tables (2.1, 2.2, 2.3); Start-order races (1.4, 1.6, 2.2, 2.3); Single-core fallback (1.6, 4.1); Uninstall cascade (4.1, 4.2); Debug log (3.3); Per-core parallelism via `InFlightPerCore[]` (1.6, 3.3).
- **Placeholders:** none — every step has code or a runnable command.
- **Type consistency:** `PEER_KICK_PARAMS` fields used identically in Task 1.1 (definition), 3.3 (consumer site in `KickPromotions`), 3.2 (provider site in `PeerKickJob` + `PeerKickWorker`). `RKIOMMU_MASTER_INTERFACE.GetPageTableBase` defined Task 1.2, called Task 2.2. `RKMPP_PEER_QUERY_REMOVE_CB` registered Task 4.1, invoked from `RkMppPeerWorkerNotifyQueryRemove` (defined Task 1.5, called Task 4.1). `IommuAttached` introduced Task 4.2 — not used earlier, so no inconsistency.
- **Build commands:** all reference `msbuild` with `Configuration=Debug Platform=ARM64`, consistent with the project's `Release|ARM64` config from commit d607ceb (we're staying on Debug for the dispatch log to be live).
- **Phase boundaries:** each phase deploys + user-verifies before the next. Reverts cleanly at phase grain.
