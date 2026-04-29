# Windows rkmpp.sys vs Linux BSP mpp_service — Scheduler Comparison

Captured 2026-05-17 after Linux multi-stream experiment showed clean
concurrent H.264+VP9 on a single rkvdec core (zero resets, fair AU-level
interleaving). Goal: identify what the Windows driver does differently that
causes the open multi-stream wedge.

## Linux experimental result (baseline this doc compares against)

Core1 disabled via `echo 1 > /proc/mpp_service/rkvdec-core1/disable_work`,
both streams forced onto core0:

| Run | Wall | Notes |
|---|---|---|
| 1× 720p H.264 (200f) | 3.0s | baseline |
| 1× 720p VP9 (200f) | 3.0s | baseline |
| 2× 720p H.264 concurrent | 5.5s / 5.5s | 1.83× per-stream slowdown |
| 1× 1080p H.264 + 1× 720p H.264 | 8.8s / 5.5s | 1.31× / 1.83× |
| 1× 720p H.264 + 1× 720p VP9 | 5.45s / 5.46s | 1.82× — fair to within 10ms |

`dmesg | grep -ciE 'reset|timeout|fault|err|abort'` = **0** in every run.
The BSP scheduler interleaves H.264↔VP9 register banks per-AU on the same
core without ever asserting `FullCoreReset` or hitting a timeout.

## Architecture comparison

### Linux

- ONE global `mpp_taskqueue` services BOTH rkvdec cores
  (`mpp_rkvdec2_link.c:1818 rkvdec2_get_idle_core`).
- Worker pulls FIFO from `queue->pending_list`, then picks any idle core
  by `core_idle` bitmap, tie-breaking by least lifetime `task_index`
  (round-robin / least-loaded).
- Cores share the queue; sessions don't bind to a core.
- `task_capacity > 1` supported via hard-CCU link list — multiple kicks
  primed at the device before INT_RDY (`mpp_rkvdec2_link.c:566`).
- Codec switching = per-AU register write. No driver action.

### Windows (`driver/rkvdec/job.c` + `job.h`)

- ONE `RKMPP_JOB_QUEUE` **per WDFDEVICE** (= per core).
- User-mode handle binds a session to a core at `CreateFile` time
  (`\Device\rkvdec0` vs `\Device\rkvdec1`).
- `Pending` list is per-device FIFO with caps `MAX_PENDING_JOBS` total and
  `MAX_PENDING_JOBS_PER_FILE` (`job.c:1564`).
- `InFlight` is singular (`job.h:110`). On completion, `RemoveHeadList(&q->Pending)`
  promotes the next (`job.c:1296`).
- One ISR/poller thread per device.

## Key differences

| Aspect | Linux | Windows |
|---|---|---|
| Core selection | Per-task by kernel, idle-aware | Fixed at file-open; no rebalance |
| Cross-file fairness | Global FIFO, kernel interleaves AUs | Per-device FIFO — File A's N jobs all kick before File B's first |
| HW queue depth | `task_capacity` can be >1 (link list) | Always 1 (`InFlight == NULL` gates promotion) |
| Codec switch cost | Zero — just write the next reg list | Logs `kick-switch` (`job.c:869`); on error escalates to `FullCoreReset` (`job.c:899`) |
| Inter-kick clock gate | BSP `clk_off → clk_on` per device, no IOMMU touch | Per-kick Gate/Ungate of leaf clocks (`job.c:1303–1338`) plus narrow CoreReset on error plus wide FullCoreReset+IOMMU Disable/Enable on hang |
| Cross-stream contention | Smoothly multiplexed | One File's jobs serialize behind the other's |

## Specific Windows bug surfaces

### 1. No work-stealing
mpv-A opens `\\Device\\rkvdec0` and starts a 4K decode; mpv-B opening
`\\Device\\rkvdec0` queues behind A even when RVD1 sits idle. Linux's
`rkvdec2_get_idle_core` would land B's first AU on RVD1 immediately.

### 2. Singular `InFlight` → zero pipelining
The codec is idle between INT_RDY and the next register write. Linux
hard-CCU keeps `task_capacity` slots primed.

### 3. Mid-stream FullCoreReset is destructive to the peer
`job.c:907–918` documents the rationale ("safe because we're at start-of-
kick, codec is idle, dense bank rewrites everything"). But the IOMMU
Disable+Enable window (~10µs paging-OFF, per
`iommu_reattach_mid_peer_dma_unsafe.md`) can fault peer DMA.
Linux's `mpp_dev_reset` is gated on `reset_request > 0` and runs only
when `!rkvdec2_core_working(queue)` — i.e. both cores idle
(`mpp_rkvdec2_link.c:1888`). Ours runs whenever `NeedsFullReset` is set
for THIS device, regardless of peer state.

### 4. Per-kick clock gate adds CDC settle time
`job.c:1327–1337` runs `GateRvdecNLeafClocks` between every job. Linux's
BSP only gates at end-of-task (`clk_off` in `mpp_power_off`), not between
back-to-back kicks of the same session.

### 5. `MAX_PENDING_JOBS_PER_FILE` cap returns `STATUS_DEVICE_BUSY`
`job.c:1564–1572` — sustained backpressure under load surfaces a "queue
full" error to user-mode. Linux's pending list is uncapped.

## Cheapest wins (ordered by impact)

1. **Cross-core queue.** One `RKMPP_JOB_QUEUE` shared between rkvdec0 +
   rkvdec1; `RkMppJobStart` picks the device with `InFlight == NULL`.
   Mirrors Linux's `cores[]` model. Likely fixes the wedge in
   `rkvdec_dual_decode_wedge_recovery_open.md`.
2. **Drop per-kick gate/ungate** for back-to-back kicks of the same
   device unless cross-mode or end-of-stream. Matches BSP.
3. **Gate FullCoreReset on `!RkMppJobQueueHasOtherOwner`** (helper exists
   at `job.c:385`). If a peer is mid-DMA, defer to narrow CoreReset or
   wait. Matches `!rkvdec2_core_working(queue)`.
4. **Allow `InFlight` to be a small queue** (e.g. 2 deep) — submit job
   N+1's bank before job N's INT_RDY. Same pattern as `task_capacity > 1`
   in `mpp_rkvdec2_link.c`.

Item 1 alone would convert "open device 0 → bound to RVD0 forever" to
BSP-style any-idle-core, which the Linux experiment demonstrated handles
concurrent streams with zero resets.

## Follow-up: cached-buffer multistream on Linux

The Windows driver uses MmCached + KeFlushIoBuffers — so we re-ran the
Linux experiment with cached buffers to confirm scheduler behavior is
independent of cache mode (it is) and that the cached path is itself
stable.

**Setup**:
- Kernel: `echo 1 > /sys/module/rk_vcodec/parameters/force_clean_invalidate`
  — kernel-side post-decode `mpp_dma_buf_sync(DMA_BIDIRECTIONAL, for_cpu=true)`
  over every imported region (`mpp_rkvdec2.c:519`).
- Heap: switched harness from `/dev/dma_heap/system-uncached` to
  `/dev/dma_heap/system-dma32` (cached + DMA32, world-rw; plain `system`
  is `0600 root:root` on rk).
- Harness: added `DMA_BUF_IOCTL_SYNC` plumbing to `backend_linux.cpp`,
  gated on `MPP_FORCE_CACHED=1` env var. Pre-submit:
  `END | RW` (flushes CPU-written bitstream/PPS/RPS/scaling). Post-poll:
  `START | READ` (kept for symmetry; kernel `force_clean_invalidate=1`
  already covers this side).
- Without the user-space pre-decode flush, every kick fails — the kernel
  knob only handles post-decode. First attempt without sync ioctls
  produced 104 resets across a single concurrent run.

**Results** (core1 disabled, both streams on core0, 200 frames each):

| Run | Cached wall | Uncached wall |
|---|---|---|
| 1× H.264 720p | 3.55s | 3.0s |
| 1× VP9 720p | 3.17s | 3.0s |
| 2× H.264 720p concurrent | 5.55s / 6.11s | 5.6s / 5.6s |
| H.264 + VP9 concurrent | 5.95s / 5.95s | 5.45s / 5.46s |

- Zero faults/resets/timeouts in dmesg across all cached runs.
- Output YUVs bit-size-exact (276,480,000 B per 720p 200f stream).
- Cached path adds ~15% wall-clock overhead from per-submit sync ioctls
  (n_fds ≈ 30 fd-walks each direction); a real driver would only flush
  the buffers actually CPU-touched.

**Conclusion**: same scheduler behavior as uncached. The cached path is
not the variable for the multistream wedge — the Windows driver and the
Linux harness now both use a clean+invalidate scheme and Linux still
multiplexes two concurrent streams on one core without a single reset.
The scheduler differences enumerated above remain the working hypothesis.

**State restored after experiment**:
- `force_clean_invalidate = 0`
- `rkvdec-core1 disable_work = 0`
- `backend_linux.cpp` sync code retained but gated on `MPP_FORCE_CACHED=1`
  (no-op when env var unset)
- `mpp_svc.c` reverted to `system-uncached` heap
