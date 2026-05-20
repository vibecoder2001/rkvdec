# Code Review — rkvdec codebase (2026-05-19)

Four parallel reviewers covered: codec kernel drivers, IOMMU kernel drivers, MFT user-mode DLL, and bitstream parsers/regbuilders. This document consolidates findings; full per-surface reports follow the summary.

---

## Top-line assessment

The codebase is sophisticated and shows hard-won familiarity with RK3588 silicon quirks and Windows-kernel landmines. Comment quality is generally excellent on the "why." It is **production-track for the current admin-only IOCTL surface**, but **not yet ready for non-admin user-mode exposure** that the MFT will eventually bring. Total: **~15 Critical, ~50 Important, ~30 Minor** issues across the four surfaces.

## Critical issues — must fix before non-admin exposure

### Kernel — codec drivers
1. `RkMppJobsDrainOwner` (rkvdec/job.c:361) leaks job pointer on timeout → permanent core wedge or UAF when IRQ finally arrives.
2. `RkMppJobWait` timeout path leaks `RKMPP_JOB` (~5 KB) per abandoned wait — non-paged-pool DoS once IOCTL is non-admin.
3. `RkMppPeerCompletion` stale-cookie + `RkMppJobRunForeign` infinite `KeWaitForSingleObject` → peer worker stuck forever.
4. `KickPromotions` → `JobKickLocalInner` → `RkMppJobComplete` → `PromoteUntilFull` → `KickPromotions` unbounded recursion. Zero-kick submissions from user-mode → kernel stack overflow bugcheck.

### Kernel — IOMMU
5. `UnmapMdl` / `RkIommuFreeIova` lack upper-bound on user-supplied `Iova`. `Iova >= 0x1_0000_0000` → 1 GiB OOB bitmap access → kernel heap corruption.
6. Slave `OnMasterArrival` PnP callback can race `ReleaseHardware` with no state lock → UAF on slave context during driver unload.
7. `MapMdl` PA-overflow rollback misses `RkIommuFreeIova` when `i == 0` → IOVA-space exhaustion DoS.

### User-mode — MFT DLL
8. `IMFMediaBuffer::Lock` returns `cur` that's trusted unchecked at decoder_mft.cpp:1336 — `cur > max` writes past locked region.
9. No upper bound on `width × height` from `MF_MT_FRAME_SIZE`. decoder_mft.cpp:606 integer overflow on `width_ * height_ * 3 / 2`. NV12 D3D11 texture with odd height is invalid; current `SetInputType` only rejects zero.
10. decoder_mft.cpp:1130 holds `lock_` across synchronous `DeviceIoControl(WAIT_JOB, 1000)` — MF host deadlock under load.
11. Bitstream dump (`mft_dump.flag`) writes to CWD with no sanitization — content leak in unusual hosting contexts.

### Parsers — untrusted bitstream
12. parser_glue_h265.cpp:732 — `slice_segment_address` bit-width derived from `pic_width_in_ctbs * pic_height_in_ctbs` with no upper bound; uint32 multiply wraps → reads attacker-controlled bit count.
13. parser_glue.cpp:392 — `pic_width_in_mbs_minus1` is `br_ue()` (uint32) cast to uint16 silently; truncated value flows into regbuilder → codec MMIO programmed with attacker-controlled dimensions kernel doesn't re-validate.
14. AV1 film-grain `num_uv` write at av1_parser.cpp:1264 is one past array end (verify dav1d struct size).
15. Exp-Golomb / leb128 / uvlc all silently clamp on excessive zeros and return 0 → parser continues with poisoned state, regbuilder hands corrupt values to kernel.

## Cross-cutting themes

**A. The trust-boundary story isn't enforced.** Parsers are the only layer that validates bitstream-derived sizes. The kernel iova-substitution path does *not* re-validate dimensions, tile counts, or ref counts. Single fix with highest leverage: a `RkmppValidateResolution(w, h)` gate at every regbuilder entry, plus explicit error returns from clamped Exp-Golomb / leb128 / uvlc (currently silent-return-0).

**B. ~50% code duplication between paired drivers.** rkvdec ↔ rkav1d job machinery (~1500 lines), rkiommu_vdec ↔ rkiommu_av1d (UnmapMdl/AllocIova/ReadAcpiId), three near-identical bitreaders (H.264/H.265/AV1), three near-identical D3D11 upload paths in `decoder_mft.cpp`. The IOMMU bitmap-OOB issue (#5) and the rkvdec leak issues (#1-#3) will each need to be fixed in multiple places.

**C. Dead/misleading code.** `driver/shared/iommu/fault.c` (unreferenced, references nonexistent struct field), `topology.c` in both rkiommu drivers (functions never called), vestigial `RkMppPollerThread` per device, stale "Phase 2 stub" comments. Delete or wire up.

**D. Magic numbers throughout.** `0xF0` for INT-error mask hardcoded in rkvdec/job.c instead of `g_ops.IntErrorMask` (already wrong for AV1 — its mask is 0x7e000). Register byte-offsets `0x208`/`0x288`/`0x2D0` as bare literals. `RKMPP_HID_*` constants exist as 0x3550/0x3560 literals across three files.

**E. Static parser state blocks the just-landed multicore dispatch.** parser_glue_h265.cpp:978-979 has `static int32_t s_prev_poc_*` — concurrent H.265 streams will clobber each other.

**F. `DbgPrintEx` in hot paths** despite the `dbgprintex_per_kick_costs` memory entry — many sites in `driver/rkvdec/job.c`, `rkav1d/job.c`, and both rkiommu `device.c` files still log per-kick or per-PnP-cycle in release builds.

**G. Diagnostic gating is ad-hoc.** Sentinel files, ~15 distinct `RKMPP_*` env vars, hardcoded `if (logged) ...` statics. Standardize on env-var-with-cached-bool.

**H. Stale build artifacts in working tree.** `build_out*.txt`, `build_output.txt`, `build_all_check.bat`, plus the bizarrely-named `C\357\200\272Usersvibecoderrkvdecbuild_out.txt`. `.gitignore` them.

## Recommended priority order

1. **Critical-block-non-admin-exposure** (issues 1–15). Most are surgical fixes; the design is sound.
2. **Resolution/Exp-Golomb gates** (parser cross-cutting A) — single highest-leverage security fix.
3. **Lift duplicated job/queue + IOMMU helpers to `driver/shared/`** before more codecs land.
4. **Delete dead code** (`shared/iommu/fault.c`, both `topology.c`, `RkMppPollerThread`).
5. **Hot-path `DbgPrintEx` cleanup** + magic-number constants.
6. **Static-parser-state → per-result-struct** to unblock concurrent multi-stream H.265.

---

# Per-surface reports

## 1. Codec kernel drivers (rkvdec / rkav1d / rkmpp_ccu / shared)

### Strengths

- **Defense-in-depth on ARM64 cache hygiene** is well-engineered: `driver/shared/rkmpp/bufpool.c:310` zero-then-flush of fresh pages, classification-driven per-kick clean/invalidate (`driver/rkvdec/job.c:907-916, 1343-1366`, `driver/rkav1d/job.c:1043-1063`), and the VP9 `reg162` vs `reg172` disambiguation (`job.c:1667-1686`). The comments explaining WHY (dirty alias snoop, HW writeback vs CPU writes) are excellent.
- **Per-File LRU + per-File pending caps** (`driver/rkvdec/job.c:1697-1731`, `driver/rkav1d/job.c:1100-1125`) cleanly prevent one open handle from starving peers.
- **PnP-cascade query-remove handling** (`driver/rkvdec/device.c:760-783, 866-905`) plus `RkMppDetachPeer` (`device.c:265-307`) with documented Dereference→FileObject ordering is exactly the right pattern.
- **Hi-word-mask CRU writes** (`driver/rkmpp_ccu/ccu.c:267-273`) and the carefully-justified inclusion/exclusion of bus-clock bits from per-kick gate masks (`ccu.c:641-724`) reflect hard-won knowledge.
- **MMIO bounds checks BEFORE any MMIO write** in the dense-bank path (`driver/rkvdec/job.c:884-905`) and AV1 path (`driver/rkav1d/job.c:760-780`) — refuses to half-program the codec.
- `RkMppJobQueueQuiesce`/`Resume` (`driver/rkav1d/job.c:396-455`) snapshot of `Done` event under lock is the correct pattern.
- Parallel-dispatch IOCTL queue + reentrancy-discipline comment (`ioctl.c:27-44`) shows the locking design has been thought through.

### Critical

**C1. `RkMppJobsDrainOwner` leaks the in-flight job pointer indefinitely on timeout**
- `driver/rkvdec/job.c:361-376`, also `driver/rkav1d/job.c:315-327`
- On WAIT timeout the code nulls `OutputFrameMdl`/`ColmvCurMdl` and deliberately leaks the `RKMPP_JOB` struct. The comment says "the next decode session triggers a core reset which unblocks the wedged poll" — but `JobKickLocalInner` does not re-issue a kick for the wedged job, and `q->InFlightPerCore[c]` still points at this job. The next `SubmitDense` cannot promote a Pending job to `InFlightPerCore[c]` because the slot is still occupied.
- **Fix:** On drain timeout, force a `FullCoreReset0/1` and wait synchronously for `&job->Done` once more before declaring the slot dead. Always reclaim the slot before returning.

**C2. `RkMppJobWait` "InFlight" timeout path leaks job memory**
- `driver/rkvdec/job.c:1970-1982`, `driver/rkav1d/job.c:1362-1367`
- When the caller's `WAIT_JOB` times out and the job is still InFlight, the function returns `STATUS_TIMEOUT` without freeing. If the user-mode client gives up on a timed-out job and never re-waits, the job sits on `q->Completed` forever — every timed-out, abandoned wait leaks one `RKMPP_JOB` (~5 KB plus tag).
- **Fix:** Mark the job with `OrphanOnComplete=TRUE` when the WAIT timeout returns leaving InFlight; in `RkMppJobComplete`, if the flag is set, free immediately instead of moving to Completed.

**C3. `RkMppPeerCompletion` may double-free a peer job on stale callbacks**
- `driver/rkvdec/job.c:1424-1444`
- When the peer reset path on RVD0 fires and the RVD1 side later delivers a "stale" completion, this function logs and returns — but `InFlightPerCore[1]` still points at the actual peer-side job allocated by `RkMppJobRunForeign`, which expects to free its job on return from KeWait. If RVD1's completion never matches, `RkMppJobRunForeign` blocks forever (line 1033: `KeWaitForSingleObject(&job->Done, ..., NULL)` — no timeout).
- **Fix:** Wrap the wait in `RkMppJobRunForeign` with a bounded timeout matched to the codec watchdog, and on timeout reclaim the slot and fail the job.

**C4. `JobKickLocalInner` calls `RkMppJobComplete` while still holding the dispatcher's logical chain — re-entrancy hazard**
- `driver/rkvdec/job.c:895, 903, 952`
- `JobKickLocalInner` is invoked from `KickPromotions` (line 1196), which runs outside the queue lock. But its failure paths and zero-kick-value path (line 952) call `RkMppJobComplete`, which acquires the queue lock and re-runs `PromoteUntilFull`, which again calls `KickPromotions`, which again may call `JobKickLocalInner`. There is no recursion bound.
- **Impact:** Kernel-stack overflow reachable from user-mode by submitting a queue of dense jobs with `KickValue = 0`. STACK_OVERFLOW bugcheck.
- **Fix:** Defer the synchronous-complete to a work item, or iterate in a loop in `KickPromotions` instead of recursing.

### Important

**I1. `RkMppEvtIoDeviceControl` `StructSize` not upper-bound checked** — `driver/rkvdec/ioctl.c:103-106`. Either drop `StructSize` from the wire format or actually use it to detect new-caller-old-driver and reject with `STATUS_REVISION_MISMATCH`.

**I2. Global cookie counter never invalidated on PnP cycle** — `driver/shared/rkmpp/bufpool.c:41`. Per-file lookup makes it safe today; add comment documenting why.

**I3. `RkMppBufFreeOne` races process-exit** — `driver/shared/rkmpp/bufpool.c:142-181`. `KeStackAttachProcess` to a zombie EPROCESS can attach to torn-down page tables. Use `PsIsProcessExiting` check before attaching.

**I4. `RkMppPmuPowerOff` does not poll for power-off complete** — `driver/rkmpp_ccu/pmu.c:241-265`. A fast Drop→Raise cycle may issue PowerOn before previous PowerOff was acknowledged, leaving the PMU FSM in an inconsistent half-state. Add a `!PmuDomainIsOn(D)` poll.

**I5. `RKMPP_MAX_MMIO_WINDOWS = 1` but array machinery written for N>1** — `driver/rkvdec/device.c:29, 49`. Dead code.

**I6. `JobKickLocalInner` calls `iommu->Disable()` then `Enable()` around FullCoreReset while a peer File may have buffers mapped** — `driver/rkvdec/job.c:728-748`. Per memory `iommu_reattach_mid_peer_dma_unsafe`, the ~10µs paging-OFF window risks peer-IOMMU faults. Serialize FullCoreReset with all bufpool ops on this iommu instance.

**I7. `RkMppEvtPrepareHardware` delays up to 1 second in a sleep loop** — `driver/rkvdec/device.c:925-932`. Consider event-driven notification from master rkiommu's PT-attach completion.

**I8. `RkMppJobComplete` uses raw mask `0xF0u` instead of `g_ops.IntErrorMask`** — `driver/rkvdec/job.c:1297, 1452`, `driver/rkav1d/job.c:893`. AV1 error mask is 0x7e000 — currently uses irrelevant low bits.

**I9. Error count never decrements** — `driver/rkvdec/job.c:1335, 1455`. Session-cumulative; document intent.

**I10. Cookie 0 should be rejected at IOCTL boundary** — `driver/rkvdec/ioctl.c:127-146`.

**I11. Per-file buffer count and byte caps interact awkwardly** — `driver/shared/rkmpp/bufpool.c:44-47`. Document or lower one.

**I12. `RkMppEvtFileCleanup` does PD power-cycle with MaskIrq cross-rkiommu** — `driver/rkvdec/device.c:1166-1190`. May hit `wdf_interrupt_disable_cross_device` BSOD.

**I13. Stale build artifacts committed** — `build_out.txt`, `build_out2.txt`, etc. `.gitignore` them.

**I14. `RkMppPollerThread` vestigial but allocates a referenced PETHREAD per device** — `driver/rkvdec/job.c:463-477`.

### Minor

- **M1.** `job.c` files are 2008 / 1393 lines — past maintainability threshold. Split.
- **M2.** `StructSize` upper bound — see I1.
- **M3.** Duplicated drain/wait/peek/submit logic between rkvdec and rkav1d (~80% same).
- **M4.** Magic register-byte-offsets in `RkMppJobSubmitDense`.
- **M5.** `RkMppOnIommuFault` overwrites most recent fault without latching.
- **M6.** AV1 `job.c:803-805` allocates ~4.5 KB on kick-path stack.
- **M7.** `DbgPrintEx` directly used in hot paths instead of `RKMPP_LOG_*` macros.
- **M8.** Stale "Phase 2 stub" comments.
- **M9.** `g_PeerProvider` global; should be in WDFDEVICE context.
- **M10.** `RkMppCcuFullCoreReset0` lock-ordering comment missing.
- **M11.** PowerOff failure count never tracked.
- **M12.** ACPI HID parse duplicated in 3 places.
- **M13.** `RkMppJobsDrainOwner` `InitializeListHead(&toFree)` re-init pattern.

---

## 2. IOMMU kernel drivers

### Strengths

- `pgtable.c:41` `hi.QuadPart = 0xffffffff` correctly constrains PD/PT pages to the low 4 GiB.
- `rkiommu_vdec/ifc.c:213-230` — UnmapMdl's bounded walk using `IovaStartBitmap` is well-commented.
- `rkiommu_vdec/device.c:215` — IovaStartBitmap boundary marking of the RCB-SRAM reservation at `0xFFF00000`.
- `rkiommu_vdec/device.c:686-704` — ReleaseHardware ordering: scrub consumer registration *before* releasing the master file object.
- `rkiommu_vdec/device.c:712-722` — Comment + correctness around NOT touching MMIO in ReleaseHardware (PD gating WHEA).
- `rkiommu_vdec/device.c:200-206 / 280-288` — STALL-bracketed enable sequence with explicit `KeStallExecutionProcessor(50)`.
- `pgtable.c:241-273` — Top-down IOVA allocator matches BSP allocation strategy.

### Critical

**1. Unbounded IOVA in UnmapMdl reads/writes the IOVA bitmap OOB**
- `driver/rkiommu_vdec/ifc.c:207-216`, `driver/rkiommu_av1d/ifc.c:144-151`, `driver/shared/iommu/pgtable.c:280-296` (`RkIommuFreeIova`)
- `startPage = (ULONG)(Iova >> 12)` is used to index `IovaStartBitmap[startPage / 64]` with no upper bound. A caller-supplied `Iova >= 0x1_0000_0000` sets `startPage` near `0xFFFFFFFF` and reads array index `0x3FFFFFF` — a 1 GiB OOB read.
- **Fix:** Add explicit bound: `if (Iova >= ((ULONG64)RK_IOMMU_IOVA_PAGES << 12)) return STATUS_INVALID_PARAMETER;`

**2. Slave OnMasterArrival callback can race ReleaseHardware → UAF on slave context**
- `driver/rkiommu_vdec/device.c:380-448` vs `:679-704`
- PnP arrival callbacks fire on a worker thread and can be delivered concurrently with `RkMppUnwatchPeer`. No lock guards `slave->MasterOpen` / `Domain` transitions across the two paths.
- **Fix:** Wrap MasterOpen / Domain transitions in a state lock; gate the arrival handler on a `ctx->Tearing` flag.

**3. Inconsistent rollback in MapMdl on AV1D path leaks IOVA**
- `driver/rkiommu_av1d/ifc.c:92-100`, also `rkiommu_vdec/ifc.c:135-144`
- If `i == 0` and the very first page is over-4GiB, `baseIova` is allocated but never freed.
- **Fix:** Move `RkIommuFreeIova` outside the `if (i > 0)` check in both PA-overflow rollback branches.

### Important

**4. `RkIommuDomainDestroy` IRQL annotation wrong** — `pgtable.h:191-192`. Annotated `DISPATCH_LEVEL` but calls `MmFreeContiguousMemory` (PASSIVE only).

**5. No memory barrier between PTE writes and ZAP_CACHE MMIO write** — `pgtable.c:367`. Add `KeMemoryBarrier()` at end of `RkIommuMapAt`.

**6. Master domain freed in ReleaseHardware while slave may still hold a ShadowDomain** — `driver/rkiommu_vdec/device.c:478-513` vs `:729-732`. Cascade callback contract needs explicit quiesce.

**7. Heavy duplication between rkiommu_vdec and rkiommu_av1d; dead `shared/iommu/fault.c`** — fault.c references nonexistent field `ctx->IsAv1d`. Delete or refactor to share.

**8. `RkIommuLookupBinding` exists in both `topology.c` but has no callers** — dead, misleading. Delete `topology.{c,h}` from both drivers.

**9. `WdfInterruptCreate` failure silently ignored** — `rkiommu_av1d/device.c:240-249`. For AV1D, treat InterruptCreate failure as fatal.

**10. `Snapshot` returns uninitialized fields for AV1D** — `rkiommu_av1d/ifc.c:215-216`. Zero the entire snapshot at function entry.

**11. `RkIommuReadAcpiId` only matches `RKCP35`* prefix** — `rkiommu_av1d/device.c:35-39`. Tight coupling to a single SoC.

**12. `g_listInitialized` is a non-atomic `BOOLEAN` flag for global init** — `rkiommu_vdec/device.c:32, 750-754`. Fragile pattern; use `KeInitializeSpinLock` from `DriverEntry`.

### Minor

- **13.** AllocIova lock contract not enforced with assert.
- **14.** `Page0Scratch` allocated in struct but never populated.
- **15.** `KeSetEvent` from PnP notification callback without `Wait==FALSE` rationale.
- **16.** `DbgPrintEx` peppered across PrepareHardware and IRQ paths.
- **17.** Magic number `0x40` for per-MMU stride repeated 10+ times.
- **18.** `ConsumerCount` array fixed at 4 entries.
- **19.** `MasterUnregisterFn` stored as `PVOID` to avoid header inclusion.
- **20.** `RkIommuForceReset` sets `PagingEnabled = FALSE` only on NT_SUCCESS path.

---

## 3. MFT user-mode DLL

### Strengths

- `mft/avcc_to_annexb.cpp:31-65` — input validation is careful: every length read bounds-checked, zero-length NAL rejected, output capacity verified before each `memcpy`.
- `mft/dll/decoder_mft.cpp:154-174` — `IUnknown` plumbing is correct; `Release()` reads post-decrement into local before potentially deleting.
- `mft/dll/decoder_mft.cpp:372-388` — throwaway `DecoderMFT tmp(kind_)` for extradata parse is tidy (with caveats — see #7).
- `mft/dll/decoder_mft.cpp:2109-2147` — epoch-tag + last-pts monotonic guard is a thoughtful belt-and-suspenders for surviving FLUSH races.
- `mft/engine/decode_engine.cpp:1374-1416` — `DecodeEngine_Flush` releases per-entry holds in the right order, bumps DPB epoch instead of nuking consumer-held slots.
- `mft/engine/decode_engine_vp9.cpp:436-453` — failure-cascade firebreak (invalidate `prob_ctx_valid[fcx]`, arm `wait_for_keyframe`).

### Critical

**1. `UpdateSubresource` row-count math when `height_` is odd** — `mft/dll/decoder_mft.cpp:1487-1495, 1746-1751`. D3D11 rejects odd NV12 heights with `E_INVALIDARG`. Fix: in `SetInputType`, `if ((w | h) & 1) return MF_E_INVALIDMEDIATYPE;`

**2. Three copies of the same NV12/P010 D3D11 upload, no overflow check** — `mft/dll/decoder_mft.cpp:1486-1492, 1745-1751, 1986-1995`. `width_ * height_ * 3u / 2u * bytes_per_sample` at `decoder_mft.cpp:606` overflows silently for `>= ~16k×16k`. Clamp to 16384×16384.

**3. `lock_` held across synchronous `DeviceIoControl(WAIT_JOB, 1000)`** — `mft/dll/decoder_mft.cpp:1130`. MF worker pool blocks for up to 1 second; topology deadlock under load.

**4. `IMFMediaBuffer::Lock` returns `cur` trusted unchecked** — `mft/dll/decoder_mft.cpp:1336-1338` (and parallel sites at 1164-1167, 1219-1223). `cur > max` writes past locked region. Fix: `if (cur > max) { buf->Unlock(); buf->Release(); return MF_E_INVALID_STREAM_DATA; }`

**5. `DumpAuIfActive` writes to CWD with no path sanitisation** — `mft/dll/decoder_mft.cpp:1067-1125`. Leaks DRM-decrypted content if upstream chain delivers any. Gate behind env var, require absolute path.

**6. `ReleaseD3DManager` doesn't handle `MF_E_DXGI_NEW_VIDEO_DEVICE`** — `mft/dll/decoder_mft.cpp:130-135`. On DXGI device reset, we'd operate with stale `d3d_device_` pointer.

**7. `tmp.ParseAvcCExtradata` churns global DLL lock and touches global state** — `mft/dll/decoder_mft.cpp:372-388`. Factor parse into a free helper.

### Important

**8.** `EnsureAttributes` ignores `SetUINT32` failures — `decoder_mft.cpp:138-144`.
**9.** `GetAttributes` lifetime comment needed — `decoder_mft.cpp:228-235`.
**10.** Decode errors silently swallowed with `decode_errors_++; return S_OK` — `decoder_mft.cpp:1393-1394`.
**11.** No upper cap on AU size (`au.size()`). Cap at 16 MB.
**12.** Engine teardown outside lock — `decoder_mft.cpp:103-120`.
**13.** AV1 sequence-header parse mishandles `obu_extension_flag=1` — `decoder_mft.cpp:484`.
**14.** `thread_local` globals for per-Submit state in AV1 engine — `decode_engine_av1.cpp:205-208`.
**15.** Hardcoded 8-subframe cap for VP9 superframe split without validation — `decoder_mft.cpp:1230-1232`.
**16.** VP9 path double-increments `decode_errors_` — `decoder_mft.cpp:1278-1282`.
**17.** `bump_lowest` O(N²) comment needed — `decode_engine.cpp:1118-1138`.
**18.** Linux `DECODE_ENV_GET` macro is GCC-specific — `decode_engine.cpp:21-25`.
**19.** `e && e[0]=="1"[0]` confused expression — `backend_linux.cpp:77`.
**20.** `DecodeEngine_Submit` no upper bound on `reorder_q.size()` — `decode_engine.cpp:1226-1239`.
**21.** `find_start_code` boundary handling dense — `au_iter.cpp:11-19`.
**22.** `dump_checked_` latch never re-evaluates sentinel — `decoder_mft.cpp:1067-1106`.
**23.** Hardcoded `caps.Hid == 0x3560` magic — `decode_engine_av1.cpp:107-143`.

### Minor

- **24.** `decoder_mft.cpp` is 2272 lines; ProcessOutput alone ~800.
- **25.** `static bool logged_av1 = false` cross-codec staleness.
- **26.** `RKMPP_TIMING` env-var probe only reads 8 bytes.
- **27.** Registration return values ignored — `registration.cpp:43-46`.
- **28.** Type registration comment for AV1/VP9 needed — `registration.cpp:88`.
- **29.** `DetectNalFraming` unreliable; document — `avcc_to_annexb.cpp:69-84`.
- **30.** `Av1DumpBuffer` calls `getenv()` per-kick — `decode_engine_av1.cpp:30-92`.
- **31.** `std::fprintf(stderr, ...)` from DLL — `decoder_mft.cpp:1457-1466`.
- **32.** `DpbHoldReason` unscoped enum — `dpb.h:67-73`.

---

## 4. Bitstream parsers and HW register builders

### Strengths

- **VP9 bool decoder bounds** — `vp9_bool_decoder.cpp:90-93` clamps refill near EOF.
- **VP9 superframe split** — `vp9_parser.cpp:948-998` all bounds checked.
- **VP9 H.264 compressed-header length clamping** — `vp9_parser.cpp:907-918`.
- **H.265 STRPS bounds** — `parser_glue_h265.cpp:376-378` validates `num_neg + num_pos` against `H265_MAX_REFS`.
- **AV1 leb128 overflow check** — `av1_parser.cpp:140` rejects > UINT32_MAX before truncation.
- **AV1 num-points caps** — `av1_parser.cpp:1229, 1244`.
- **AV1 dim-inherit slot validation** — `av1_parser.cpp:471`.
- **H.265 scaling-list `delta` underflow guard** — `parser_glue_h265.cpp:225`.
- **VP9 aarch64-miscompile workaround** — `vp9_parser.cpp:633-640` is exemplary.
- **H.265 packed PPS sub-layer index clamp** — `h265_packed_tables.cpp:198-200`.

### Critical

**1. AV1 film-grain `num_uv` OOB write** — `mft/av1_parser.cpp:1254-1264`. Line 1264 writes `fgd.ar_coeffs_uv[pl][num_uv] = 0;` (one past end if array sized 25). Verify dav1d struct size; assert/clamp `num_uv < DAV1D_AR_COEFFS_UV_SIZE - 1`.

**2. `slice_segment_address` width derived from product can wrap** — `mft/parser_glue_h265.cpp:732-734`. `pic_width_in_ctbs_y * pic_height_in_ctbs_y` is uint32 multiply that wraps silently. Cap `pic_width_in_luma_samples` and `pic_height_in_luma_samples` at 16384 each.

**3. H.265 STRPS slot index inconsistency** — `mft/h265_packed_tables.cpp:404-440`. Parser allows `num_neg + num_pos == 16`, packer caps at 15. Document or align.

**4. AV1 `refidx` used as DPB slot index** — `mft/regbuilder_av1.cpp:472-548`. Defense-in-depth: validate `refidx >= 0` before indexing at line 751.

### Important

**5. VP9 `read_frame_size_with_refs` silently leaves width/height unset** — `vp9_parser.cpp:189-210`. `found = true` set even when slot invalid. Downstream `vp9_scale_factor` dies on `cur_dim=0`.

**6. AV1 `frame_offs[h->refidx[0]] = INT_MIN` ordering hazard** — `av1_parser.cpp:734-790`. Fallback to slot 0 even if invalid.

**7. AV1 `col_start_sb[]` sentinel may exceed array size** — `av1_parser.cpp:867-896`. Verify dav1d `col_start_sb[]` is sized `DAV1D_MAX_TILE_COLS + 1`.

**8. AV1 `tiling.update` upper bound** — `av1_parser.cpp:899`. Use unsigned multiply.

**9. `br_uniform` assert is debug-only** — `av1_parser.cpp:158-167`. Replace with release-safe guard.

**10. H.264 Exp-Golomb silent clamp on >32 zeros** — `mft/parser_glue.cpp:73-79`. Propagate error.

**11. H.264 width/height not bounded** — `mft/parser_glue.cpp:392-393`. `(uint16_t)br_ue(br)` silent truncation; adversarial SPS with `pic_width_in_mbs_minus1 = 0x10000` stored as 0.

**12. `frame_dims` overflow on attacker-controlled SPS dimensions** — `regbuilder_h264.cpp:51-58`, `regbuilder_h265.cpp:106-113`. Gate accepted resolution.

**13. VP9 width × pitch overflow** — `regbuilder_vp9.cpp:84-86`. Use 64-bit intermediate.

**14. H.265 tile column count truncated to uint8** — `h265_packed_tables.cpp:283-292`, `parser_glue_h265.cpp:653-654`. Reject `num_tile_columns_minus1 >= 19`.

**15. VPS HRD body parse abandons reader on `nh != 0`** — `parser_glue_h265.cpp:444-471`. Trap waiting for future maintainers.

**16. VP9 superframe `bytes_per_sz * num_frames * sz` collusion** — `vp9_parser.cpp:976-988`. Safe today; flag with comment.

**17. H.264 `frame_num_wrap` cast safety** — `h264_packed_tables.cpp:209-211`. Bounded in practice; defense-in-depth.

**18. H.265 POC high-bit extraction is 1 bit not 4** — `regbuilder_h265.cpp:466`. Pending design risk for long-running streams.

**19. Scaling-list `delta *= mat_step` overflow** — `parser_glue_h265.cpp:225`. Compare before multiply.

**20. Static parser POC state blocks concurrent streams** — `parser_glue_h265.cpp:978-979`. Move to `H265ParseResult` struct.

### Minor

- **21.** `rb24` zero-pads on short buffer; verify error stickiness.
- **22.** `BitPacker` 99% identical between H.264 and H.265 — extract.
- **23.** `BitReader` near-identical across codecs — extract.
- **24.** AV1 ref-9-bit-mask `0x1ff` repeated 21 times.
- **25.** AV1 GM `br_subexp * (1 << shift)` UB on negative.
- **26.** VP9 `show_existing_frame || header_size == 0` short-circuit.
- **27.** H.265 NAL header from RBSP[0..1] — practically safe; document.
- **28.** AV1 GMV matrix value caps.

### Cross-cutting

- **Shared bitreader**: three near-identical BitReaders + VP9 bool decoder + two packed-table BitPackers (~250 lines of duplicated logic). Each copy has slightly different EOF handling. Consolidate to `mft/bitio.h`.
- **Resolution validation should be centralized**: single helper `RkmppValidateResolution(w, h)` at every regbuilder entry catches issues #11, #12, #13, and AV1/H.265 equivalents in one place.
- **Exp-Golomb / leb128 / uvlc all silently clamp on excessive leading-zero runs**: all return 0. Convert to single explicit-error path.
- **VP9 / AV1 DPB-slot index validation**: spec-required `frame_offset`/`order_hint_bits` consistency check absent.
- **`assert()` in security-sensitive code paths**: compiled out in release. Replace with explicit error returns.
- **Static parser state**: blocks safe concurrent parsing per multi-core dispatch.
