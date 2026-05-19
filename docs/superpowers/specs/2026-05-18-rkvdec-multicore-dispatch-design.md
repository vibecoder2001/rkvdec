# rkvdec2 Multi-Core Dispatch (RVD0 + RVD1)

Date: 2026-05-18
Status: design approved, pending implementation plan

## Goal

Dispatch H.264 / H.265 jobs across both RKCP3550 rkvdec2 cores (RVD0 + RVD1) on RK3588, while:

- preserving the existing per-Owner LRU fairness
- keeping the per-codec binary split intact (no return to shared state through CCU)
- degrading gracefully to single-core when only one core's driver is installed
- supporting clean uninstall of either core's driver when no jobs are active

## Non-goals

- AV1 (rkav1d is single-instance, no peer); design only covers rkvdec2.
- Encoder / JPEG / IEP cores.
- _DSD-based ACPI topology discovery (deferred; current hardcoded UID table in `LookupIommuForClient` is reused).

## Background

Today (post-commit 506a51f), each core runs an independent rkvdec.sys instance bound to one rkvdec2 device (UID 0 or 1) and one rkiommu_vdec instance (UID 9 or 10). Each instance has its own `RKMPP_JOB_QUEUE` with per-Owner LRU (`OwnerLru[4]` in `job.h:137-140`). MFT's `OpenDevice` (`mft/engine/backend_windows.cpp:40-80`) picks the first matching `GUID_DEVINTERFACE_RKMPP` device and uses it for the session — File-handle sticky to a single core, no cross-core balancing.

The Rockchip BSP solves the same problem with a master/worker split: `mpp_service` registers core 0 (selected by DT alias `of_alias_get_id(np, "rkvdec")`, rkvdec2.c:1579) as the dispatcher. `rkvdec2_get_idle_core` (link.c:1818) picks the least-loaded core per task (idle bitmask + `task_index` tiebreak). Non-master cores reuse master's IOMMU domain (`cur_info->domain = ccu_info->domain`, link.c:1342) so a single page table is visible to both MMUs.

## Architecture

```
MFT ──► RVD0 (master)              RVD1 (worker)
         ├─ user device interface   ├─ no user device interface
         ├─ RKMPP_JOB_QUEUE         ├─ exports PeerWorker iface:
         │   ├─ OwnerLru[]          │     • KickJob(swregs, callback ctx)
         │   └─ CoreIdle bitmap     │     • cancellation hook
         ├─ own MMIO/UID=0          └─ ISR → DPC → completion cb into RVD0
         ├─ peer ref ──────────────►
         └─ falls back to self-only if peer ref absent

rkiommu_vdec (master, UID 9)        rkiommu_vdec (slave, UID 10)
  ├─ owns page tables                ├─ skips PT allocation in PrepareHardware
  ├─ exports PtBase via               ├─ query-interface → master, reads PtBase
  │   query iface to slave            └─ programs own MMU MMIO with master's PT
  └─ unchanged map/unmap path
```

### Per-codec binary inventory (unchanged)

`rkvdec.sys` × 2 (one per UID 0/1), `rkiommu_vdec.sys` × 2 (one per UID 9/10). No new binaries.

### Identification — codec ↔ IOMMU and master/slave

Reuse the hardcoded table in `driver/shared/rkmpp/ifc_client.c:116-128`:

| rkvdec UID | rkiommu UID | role |
|---|---|---|
| 0 | 9 | master |
| 1 | 10 | slave |

`LookupIommuForClient(codecHid=0x3550, codecUid)` keeps its current mapping. A new `LookupPeerCodec(localCodecUid)` returns the *other* rkvdec's device interface GUID + instance ID for the dispatcher to open. A new `IsMasterIommu(localIommuUid)` returns `localIommuUid == 9`.

Refactor to `_DSD`-driven discovery (parsing `interrupt-names` for `irq_rkvdec{0,1}_mmu`) is explicitly deferred.

## Dispatcher

### State (additions to `RKMPP_JOB_QUEUE`)

```c
// job.h additions
ULONG          CoreCount;          // 1 or 2
ULONG          CoreIdle;           // bit 0 = self idle, bit 1 = peer idle
ULONG          CorePending[2];     // approximation of BSP task_index
RKMPP_JOB     *InFlightPerCore[2]; // replaces single InFlight slot
PEER_WORKER_IFC Peer;              // zeroed if single-core
BOOLEAN        PeerOpen;
```

The existing single `q->InFlight` slot is replaced by `InFlightPerCore[]`. This is the change that makes **both cores actually run jobs in parallel** rather than one core sitting idle while the other holds the slot. All `q->InFlight` reads (completion match-up, FileCleanup drain, etc.) become `InFlightPerCore[coreId]` lookups; the completing core is identified by which interrupt fired or which `CompletionCallback` invoked.

`PEER_WORKER_IFC` mirrors the rkiommu query-interface contract already established (`ifc_client.h:8-18`): file-object held to pin worker binary, `InterfaceReference`/`Dereference` lifecycle, BSOD 0xCE rules from `[[wdf_query_interface.md]]`.

### Promotion (modify `RkMppJobComplete`, job.c:1034-1074)

The current code promotes **exactly one job per completion** into the single `q->InFlight` slot. This must change: after a completion, if there are pending jobs AND idle cores, the dispatcher must promote **as many as fit** — otherwise on the steady state with two streams and two cores, one core will sit idle whenever its completion arrives before the other's.

Promotion loop (under queue lock):

```
while (!IsListEmpty(&q->Pending) && (q->CoreIdle & ((1u << CoreCount) - 1)) != 0) {
    next  = PickPendingByOwnerLru(q);   // existing logic, factored out
    core  = PickTargetCore(q);           // BSP-style idle + task_index
    RemoveEntryList(&next->Link);
    next->TargetCore = core;
    q->InFlightPerCore[core] = next;
    q->CoreIdle &= ~(1u << core);
    q->CorePending[core]++;
    EnqueueKick(next, core);             // local: defer to after lock release
}
```

`EnqueueKick` collects the (job, core) pairs to a local stack array; after `KeReleaseSpinLock` we walk it and call `JobKickLocal` or `JobKickPeer` per entry. Kicks must run outside the queue lock — `JobKickPeer` crosses a driver boundary and can take an arbitrary path through the peer's WDF infrastructure; holding our spinlock across that is a deadlock vector.

`PickTargetCore` (BSP-style):
- Candidates = cores where bit `i` is set in `CoreIdle`.
- Pick the candidate with the lowest `CorePending[i]`. Ties → lowest index.
- Returns `-1` only if no idle core (the `while` guard prevents this in practice).

Initial fill: same loop runs from `RkMppJobSubmitDense` after enqueue, so the very first job (or two, when both cores are idle) gets kicked immediately rather than waiting for a completion event that will never come.

### Kick (split `RkMppJobStart`, job.c)

Factor the existing MMIO-write body into `JobKickLocal(ctx, job)`. Add `JobKickPeer(queue, job)` which:

1. Captures the packed SWREG region + dense reg-list.
2. Calls `queue->Peer.KickJob(swregs, ownerKey, completionCtx)`.
3. Returns immediately; RVD1's ISR-DPC will call back.

Both `JobKickLocal` and `JobKickPeer` emit a debug-only trace at dispatch:

```c
#if DBG
DbgPrintEx(DPFLTR_DEFAULT_ID, DPFLTR_INFO_LEVEL,
           "rkvdec: dispatch job %llu owner=%p core=%u pending=[%u,%u] idle=0x%x\n",
           job->Id, job->Owner, job->TargetCore,
           q->CorePending[0], q->CorePending[1], q->CoreIdle);
#endif
```

Gated by `#if DBG` so retail builds get nothing — per [[dbgprintex_per_kick_costs.md]] this kind of multi-arg per-kick `DbgPrintEx` dominated AV1 driver overhead, so it must not ship in release. The release-build gating from commit d607ceb (Release|ARM64 config) applies.

### Completion

- Local completion stays in RVD0's existing ISR → DPC → `RkMppJobComplete`.
- Peer completion enters via `PeerWorker.CompletionCallback(jobId, status, regs)`. The callback is registered at RVD0 PrepareHardware; it runs at DISPATCH (the peer's DPC), and serializes via the same `WdfSpinLock` already protecting the queue.
- Both paths: set `CoreIdle` bit, decrement `CorePending[TargetCore]`, run the existing per-Owner LRU promotion (`RkMppJobComplete` body).

### Clock / reset gating

`job.c:1099-1114` already gates leaf clocks by `pub.Uid`. With dispatch:

- Local jobs: `pub.Uid == 0` (master self).
- Peer jobs: the peer's own JobKick path runs in RVD1's address space and calls `GateRvdec1LeafClocks` from there. CCU is unchanged.

The peer ISR's wedge detection (existing) escalates to `FullCoreReset1`; doesn't touch RVD0. Per `[[rkvdec2_reset_escalation.md]]`, soft errors stay narrow.

## IOMMU shared page tables

### Master rkiommu_vdec (UID 9)

Unchanged. Allocates page tables in PrepareHardware. New query-interface export `GetPageTableBase()` returns the physical address of the L1 page-table root.

### Slave rkiommu_vdec (UID 10)

At PrepareHardware:

1. Read own UID, see it's 10. Call new `IsMasterIommu` → false.
2. Lookup master rkiommu via existing topology (just like rkvdec looks up its rkiommu — same `IfcClient` pattern, but client = rkiommu peer).
3. Query master for `PageTableBase`.
4. Skip own page-table allocation. Skip own `IovaStartBitmap` allocation (the bitmap lives on master; allocations are bounded there per `[[iommu_unmap_overunmaps_peer.md]]`).
5. Program own MMU registers with master's PT base. Enable normally.

Map/Unmap IOCTLs targeting slave: forward to master (or block — see Open Questions). MFT only opens RVD0, which routes its `MapMdl` to its bound rkiommu (UID 9, master); slave never receives map IOCTLs from MFT in practice.

### Why this is safe

- Linux BSP does exactly this; both MMU instances pointing at one page-table root is hardware-supported.
- `[[rkvdec_iommu_shares_codec_bus_clocks.md]]` already tells us we don't per-kick gate the IOMMU bus clocks, so the slave's MMU stays addressable across both cores' kicks.
- `[[rkmpp_dma_cache_coherency.md]]` is unaffected (same MmCached buffers, same KeFlushIoBuffers).

## Start-order races

Windows PnP gives no ordering guarantee between independent ACPI devices. All of these orderings are legal and must work:

| order | how it must resolve |
|---|---|
| 9, 10, 0, 1 (canonical) | each Prepare succeeds on first try |
| 10, 9, 0, 1 (slave-first) | slave waits for master |
| 9, 0, 10, 1 (RVD1 stack late) | RVD0 starts single-core, joins peer later |
| 9, 0 only (RVD1 stack never appears) | permanent single-core (the fallback case) |

Resolution rule: **every consumer that depends on a peer registers a `IoRegisterPlugPlayNotification` on the peer's device interface class, with `EventCategoryDeviceInterfaceChange`.** PrepareHardware never blocks on a peer being present. The notification callback runs at PASSIVE and does the late-attach work.

### Slave rkiommu_vdec (UID 10) when master (UID 9) is not yet started

1. PrepareHardware succeeds. Page-table allocation is skipped (slave never allocates). MMU MMIO is **not** programmed yet; the IOMMU is held in a disabled state.
2. A flag `Ctx->PtAttached = FALSE`. All IOCTLs that require a working IOMMU return `STATUS_DEVICE_NOT_READY`. Map/Unmap forwarded-to-master rejection (Open Question #1) is unchanged.
3. `IoRegisterPlugPlayNotification` for `GUID_DEVINTERFACE_RKIOMMU_VDEC`. Callback filters on master (`LookupIommuForClient`-style — we know master's identifier from the same hardcoded table).
4. When master arrives: open peer interface, read `GetPageTableBase`, program slave MMU registers, set `PtAttached = TRUE`. Signal a kernel event on the device context that any waiter (slave rkvdec PrepareHardware path) can pulse on.
5. Slave rkvdec (RVD1) PrepareHardware queries its rkiommu for `IsPtAttached()`. If false, wait on the event with a timeout (e.g. 5 seconds — PnP-friendly, not infinite). On timeout, fail PrepareHardware → PnP retries. On signal, continue.

### Master rkvdec (RVD0) when peer (RVD1) is not yet started

Symmetric. RVD0 PrepareHardware succeeds in single-core mode (`PeerOpen = FALSE`, `CoreCount = 1`). Registers PnP notification on `GUID_DEVINTERFACE_RKMPP` filtered by HID/UID matching RVD1. When the notification fires:

1. Acquire queue lock.
2. Open peer worker interface.
3. Set `CoreCount = 2`, mark `CoreIdle |= 0b10`, zero `CorePending[1]`.
4. Release lock. Next promotion naturally includes core 1.

No in-flight job is disturbed. The transition is monotonic single-core → dual-core under lock.

### Peer disappears at runtime (uninstall path)

Already covered in "Clean uninstall." The PnP `EvtDeviceQueryRemove` on RVD1 invokes the inverse callback to RVD0, which transitions back from dual-core to single-core under the same queue lock, drains peer-targeted jobs, derefs the interface.

### Order assumptions we are NOT making

- **No assumption that UID 9 < UID 10 in start order.** The numeric ordering is firmware-author preference, not enforced.
- **No assumption that rkiommu starts before rkvdec.** Already handled by the existing `OpenIfcs` retry / `IoRegisterPlugPlayNotification` patterns in `ifc_client.c` (RVD↔rkiommu race today); peer-rkiommu race uses the same mechanism.
- **No assumption that PrepareHardware runs once.** PnP can `ReleaseHardware` + `PrepareHardware` again (rebalance, surprise-remove recovery). Each entry must re-evaluate peer presence.

## Single-core fallback

Three cases, all transparent:

1. **RVD1 driver not installed**: `OpenPeerCodec` in RVD0 PrepareHardware fails (no matching device interface). `PeerOpen = FALSE`, `CoreCount = 1`. Dispatcher only picks self.
2. **Slave rkiommu_vdec (UID 10) not installed**: RVD1's PrepareHardware fails (`LookupIommuForClient(rkvdecUid=1)` finds no rkiommu device); RVD1 doesn't come up; case (1) applies.
3. **Master rkiommu_vdec (UID 9) not installed**: RVD0 PrepareHardware fails (existing behavior). Decode is unavailable on both cores. Acceptable — no master = no page tables = no decode.

MFT requires zero changes. It still opens "first matching rkvdec," which is RVD0.

## Clean uninstall (preconditions: all jobs stopped)

### RVD1 driver uninstall

1. PnP issues query-remove on RVD1.
2. RVD1's `EvtDeviceQueryRemove` calls back into RVD0 via the inverse hook registered at master-side `OpenPeerCodec`: `RVD0::PrepareForPeerUnload()`.
3. RVD0 sets `CoreCount = 1`, clears peer-related bits in `CoreIdle`. Any newly-promoted job picks self.
4. RVD0 drains in-flight jobs on RVD1 (existing `RkMppJobsDrainOwner` semantics, scoped to TargetCore == 1; 500 ms timeout).
5. RVD0 dereferences `Peer` interface and releases peer file-object reference. Ordering matches `RkMppCloseIfcs` (`ifc_client.c:241-252`): `InterfaceDereference` before `ObDereferenceObject`. Violating this hits BSOD 0xCE per `[[wdf_query_interface.md]]`.
6. RVD1 completes query-remove, unloads.
7. Slave rkiommu_vdec (UID 10) — if also being uninstalled — runs analogous query-remove: notifies master rkiommu via inverse interface, slave dereferences master's PT-base interface, unloads. Master keeps page tables (still serves RVD0).

### RVD0 driver uninstall

1. Existing FileCleanup drain semantics handle any open Files (`RkMppEvtFileCleanup`, device.c:553-702).
2. New: before `RkMppCloseIfcs`, run peer-detach: drain peer-targeted jobs, deref `Peer`, release peer file-object.
3. Existing IOMMU disable → CCU DropCluster ordering (device.c:417-475) unchanged.

RVD1 may remain loaded after RVD0 unloads (it's a worker with no user surface). Acceptable but pointless; admin will likely uninstall both.

### Master rkiommu_vdec uninstall while slave is loaded

The file-object references on master form a tree: RVD0 holds master directly; slave rkiommu holds master (for `GetPageTableBase`); RVD1 holds slave. A naive "slave must be uninstalled first" rule pushes the cleanup burden onto the admin and is fragile.

**Cascade rule, driven from PnP query-remove on master:**

1. Master rkiommu (UID 9) receives `EvtDeviceQueryRemove`. It enumerates its registered consumers (one slot for RVD0, one for slave-rkiommu) via the inverse callbacks they registered at attach time.
2. Each consumer's inverse callback runs:
   - **RVD0**: precondition "all jobs stopped" — assert no jobs in flight on either core, no open Files (`WdfDeviceEnqueueRequest` returns busy if any). Set `IommuAttached = FALSE`. Dereference master, release file-object. RVD0 returns to a non-decoding state; it stays loaded.
   - **Slave rkiommu (UID 10)**: cascade query-remove to *its* consumers (RVD1, same protocol — RVD1 must have no in-flight). Set `Ctx->PtAttached = FALSE`, dereference master, release file-object. Slave's MMU is disabled.
3. If any consumer cannot release (in-flight jobs that the precondition violates), master fails query-remove with `STATUS_DEVICE_BUSY`. Admin sees a normal "device in use" error rather than the system wedging.
4. With all consumers detached, master tears down page tables and unloads.

This makes "uninstall master rkiommu" idempotently safe **iff all jobs are stopped** — which is the design's stated precondition. No admin choreography required.

### Master rkiommu_vdec reinstall after uninstall

When master rkiommu starts again, the same `IoRegisterPlugPlayNotification` callbacks fire that handled the canonical start-order races:

1. New master allocates fresh page tables. New PT-base address — **any cached IOVA from before is invalid**.
2. Slave's notification callback fires → opens new master, programs MMU with new PT base, signals `PtAttached` event.
3. RVD0's rkiommu notification callback fires → opens new master, marks `IommuAttached = TRUE`. RVD0 becomes able to decode again.
4. RVD1's rkiommu (slave) notification fires when slave signals `PtAttached` → RVD1 becomes able to decode.
5. RVD0's peer notification re-fires when RVD1 transitions to operational; dual-core mode resumes under queue lock.

**Stale-mapping risk for MFT-side state across the uninstall/reinstall:**

If an MFT process held an open File on RVD0 across the master uninstall: that File was force-closed when RVD0 returned to non-decoding state (per the "all jobs stopped" precondition — the admin actually did stop them, so there were no Files). On reinstall, MFT re-opens via the device interface arrival notification it (or the runtime) gets, and re-maps buffers from scratch. No buffer survives the master rkiommu lifecycle — page tables were torn down with the master, so cached IOVA in any user-mode handle is meaningless and must not be reused. RVD0 enforces this by failing any IOCTL that arrives with `IommuAttached = FALSE`, which forces MFT into its open-from-fresh path.

**Surprise-remove (master rkiommu driver yanked while jobs in flight):**

Violates the "all jobs stopped" precondition. RVD0/slave-rkiommu will see I/O failures from the dereferenced interface; the safest path is to drain via timeout (RVD0's existing `RkMppJobsDrainOwner` 500 ms), force `FullCoreReset` on both cores, then accept loss of in-flight frames. This path exists but is not graceful — document as "do not surprise-remove rkiommu drivers." The same caveat already applies to surprise-removing rkiommu today.

## Open questions

These are deferred to implementation, not blocking the design:

1. **Slave Map/Unmap IOCTL handling.** Today MFT doesn't issue them to RVD1 (it opens RVD0 only). Decision: reject in slave with `STATUS_NOT_SUPPORTED`, document, revisit if a future codec needs it.
2. **Cancellation across cores.** If RVD0 cancels a job already kicked to RVD1, peer interface needs a `CancelKick(jobId)` that aborts at next safe point. Likely just wait for completion + drop result.
3. **Counter coherency.** `CorePending[1]` is updated by master under queue lock from both local promotion path and peer completion callback (also under queue lock). No additional sync needed.
4. **Stress: peer wedge during master kick.** Per `[[rkvdec2_reset_escalation.md]]` FullCoreReset1 is narrow; doesn't disturb master. Need a test in `linux_mpp_decode` harness ([[linux_multistream_baseline.md]]) cross-checked against this Windows path.

## Testing

- **Single-core regression**: uninstall rkvdec UID=1 binary, run existing H.264/H.265 streams. Must match current per-core perf.
- **Dual-stream balance**: two `mft_play` instances, observe per-core kick count via existing perf counters; expect ~50/50 with idle+task_index selector. Verify via debug-build dispatch log that both `core=0` and `core=1` appear interleaved (not a long run on one core followed by a long run on the other).
- **True parallelism**: dual stream + `dbgview` on debug build; observe overlapping `dispatch job ... core=0` and `dispatch job ... core=1` lines where the second dispatch happens before the first's matching completion log. If completions and dispatches strictly alternate per core, the per-core slot regression has snuck back in.
- **Uninstall sequence**: stop all decode, uninstall RVD1; verify RVD0 continues to decode. Reinstall RVD1; verify dispatcher resumes balancing.
- **Peer wedge isolation**: inject FullCoreReset1 mid-stream; master stream must continue unaffected.
- **Cross-check vs BSP**: `linux_mpp_decode` ([[linux_mpp_decode_state.md]]) two-stream run on same Linux core_count=2 setup; compare kick distribution.

## Implementation phases

1. **Phase 1 — peer worker plumbing**: add `PEER_WORKER_IFC` to rkvdec, `OpenPeerCodec`/`ClosePeerCodec` in RVD0 PrepareHardware/ReleaseHardware (no-ops when single-core). No dispatch yet.
2. **Phase 2 — shared page table**: add `GetPageTableBase` query iface to master rkiommu_vdec, slave-mode PrepareHardware in rkiommu_vdec (skip PT alloc, attach to master). Test: RVD0 + RVD1 both decode independently (no dispatch) with shared PT.
3. **Phase 3 — kernel dispatch**: add `CoreIdle` / `CorePending` to queue, BSP-style `PickTargetCore`, `JobKickPeer`, peer completion callback. Per-Owner LRU on top.
4. **Phase 4 — uninstall hooks**: query-remove callbacks on RVD1 and slave rkiommu_vdec; drain-peer path in RVD0.

Each phase is independently testable and revertible.

## References

- BSP: `mpp_rkvdec2.c:1557-1701` (probe), `mpp_rkvdec2_link.c:1314-1361` (`rkvdec2_attach_ccu`), `mpp_rkvdec2_link.c:1818-1853` (`rkvdec2_get_idle_core`).
- Driver today: `driver/rkvdec/job.h:97-150` (queue + OwnerLru), `driver/rkvdec/job.c:611-624` + `1034-1074` (LRU update/promote), `driver/rkvdec/device.c:553-702` (FileCleanup), `driver/shared/rkmpp/ifc_client.c:116-128` + `241-252` (topology + deref order).
- Memory: [[ccu_leaf_clock_per_instance.md]], [[wdf_query_interface.md]], [[iommu_unmap_overunmaps_peer.md]], [[rkvdec2_reset_escalation.md]], [[driver_split_verified.md]], [[linux_rkvdec_source_refs.md]].
