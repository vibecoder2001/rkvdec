# Phase 3b — Codec MMIO bring-up, then FFmpeg parser, register-list builder, first H.264 decode

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decode a single I-frame from a known H.264 .mp4 through the rkmpp stack on RK3588 and verify the YUV output matches an FFmpeg software-decode reference. Acceptance gate: `tests/harness/rkmpp_decode --in test.mp4 --out test.yuv` exits 0 and the produced YUV matches the FFmpeg reference within 0 mismatches.

**Architecture:** Phase 3a confirmed the kernel/user data path on real hardware but exposed a single hard blocker — **codec and IOMMU MMIO at `0xFDC38xxx` SError on first access** despite the cluster appearing brought up (PMU mapped, clocks ungated by UEFI, CRU readback shows zeros). Reading the REVISION register, writing the IOMMU's DTE_ADDR, and writing CRU `SOFTRST_CON(40)` all bugcheck the box. Phase 3b therefore opens with a focused investigation to make codec MMIO reachable, and only after that does the H.264 parser/decoder work proceed.

The original Phase 3b plan (parser → register builder → first decode) is preserved as Tasks 4–13. Tasks 1–3 are new prerequisites added based on Phase 3a hardware findings.

**Tech Stack:** Same as before. Adds: FFmpeg (vendored subset, LGPL), V4L2-stateless H.264 control struct headers, BSP rkvdec2 H.264 register definitions, possibly UEFI/PE-shell tooling for the bring-up investigation.

**Reference:**
- Spec: `docs/superpowers/specs/2026-04-28-rkvdec-mft-h264-design.md` §4 (MFT) and §6 (data flow)
- Phase 3a tag: `phase3a-effective`
- Project memory: `rk3588_acpi_codec_topology.md`, `rk3588_gated_mmio_serror.md`, `linux_rkvdec_source_refs.md`
- BSP: `drivers/video/rockchip/mpp/mpp_rkvdec2_h264.c` — H.264-specific register programming on rkv-decoder-v2
- FFmpeg upstream: master branch of git.ffmpeg.org/ffmpeg.git, `libavcodec/h264dec.c` and adjacent
- V4L2 stateless H.264 controls: Linux UAPI `linux/v4l2-controls.h` (no kernel needed; just headers)
- Reference clip: `tests/data/i_frame_only_8x8.h264` — a synthetic 8×8 I-frame-only clip

## What Phase 3a left behind

When you start Phase 3b you'll find rkmpp.sys and rkiommu.sys carrying these temporary "skip" workarounds — all of them are because codec/IOMMU MMIO SErrors:

- `rkmpp/device.c`: REVISION read, DeassertCoreReset call, RegisterFaultHandler call all behind `#if 0` / SKIPPED comments.
- `rkmpp.inx`: matches only `ACPI\RKCP3550` (not the other 7 codec HIDs) for single-device debug.
- `rkmpp/device.c::PrepareHardware`: returns `STATUS_NOT_SUPPORTED` for any `_UID != 0`.
- `rkiommu/ifc.c::MapMdl`: `RkIommuEnable` call removed; ZAP_CACHE skipped. Page tables built in RAM only.
- `rkiommu/device.c`: `WdfInterruptCreate` for the IOMMU IRQ behind `#if 0`.
- `rkmpp_ccu/pmu.c`: `RKMPP_PMU_DISABLE_POWER_GATING = 1` — `RkMppPmuPowerOn/Off` are no-ops.
- `RKMPP_STEP` macros + 5ms `KeDelayExecutionThread` calls scattered through `PrepareHardware` and `RaiseCluster`.

**Task 1 below removes the trace/delay scaffolding.** Tasks 2–3 are the actual codec-MMIO investigation. Tasks 4+ then re-enable the skipped paths one at a time as the bring-up sequence solidifies.

---

## Tasks

### 1. Strip Phase 3a debug scaffolding

Mechanical cleanup, no hardware testing required. Remove:

- The `RKMPP_STEP` macro from `rkmpp/device.c` and all call sites.
- `RkCcuStep` from `rkmpp_ccu/ccu.c` and call sites.
- All `KeDelayExecutionThread` calls inside `PrepareHardware` and `RaiseCluster`.
- The `_UID != 0` early-return gate in `rkmpp/device.c`.
- The single-HID restriction in `rkmpp.inx` (restore the full RKCP3510..RKCP3560 list).

**Keep** the `#if 0` blocks around the SKIPPED codec MMIO touches — those are load-bearing for the box not bugchecking. They get re-enabled in Tasks 4–6 once Task 2 lands.

Build, deploy, confirm `rkmpp_smoke` still passes 100 iterations on hardware. That's the regression gate for this cleanup.

### 2. Derive bring-up from the linux-rockchip DTS, decide firmware-vs-driver split

**Important context:** we author this firmware's ACPI ourselves. The DSDT does not currently have `_PR0`/`_ON`/`_OFF` methods for the codec devices because they haven't been written yet — the firmware does no automatic PD bring-up. The only authoritative spec for what bring-up needs to happen is the **linux-rockchip device tree** (the DTS files in `arch/arm64/boot/dts/rockchip/rk3588*.dtsi`). The Linux drivers consume those phandles via `clk_get`, `reset_control_get`, `dev_pm_domain_attach`; we have to do the equivalent.

**Step 1: enumerate everything the rkvdec nodes claim from the DTS.**

For each of `rkvdec0`, `rkvdec1`, `rkvdec_ccu`, `rkvdec0_mmu`, `rkvdec1_mmu` (matching the ACPI HID/UID layout in `rk3588_acpi_codec_topology.md`):

- `clocks = <...>;` — list every clock phandle. Each resolves to a `cru` node + numeric ID. Translate via `dt-bindings/clock/rk3588-cru.h` to a `(CON register, bit)` pair using `RK3588_CLKGATE_CON(n)` macros.
- `resets = <...>;` — same exercise for `RK3588_SOFTRST_CON(n)` bits via `dt-bindings/reset/rockchip,rk3588-cru.h`.
- `power-domains = <...>;` — phandle into `power-controller` (the PMU). Translate the symbolic ID (e.g. `RK3588_PD_RKVDEC0`) to whatever the **owning firmware ACPI source** says it should mean. Since we author the firmware, we get to pick. Match the BSP `pm_domains.c` `DOMAIN_RK3588(...)` macro arguments for the rk3588 variant we're targeting; pick the variant whose PMU offsets match what UEFI dumps (Step 2).
- Any `assigned-clocks` / `assigned-clock-parents` — initial mux/divider settings.
- `iommus = <&rkvdec0_mmu>;` etc. — translation domain wiring (already covered by our rkiommu topology table).

Capture the result in a new memory file `phase3b_codec_bringup_table.md` with one row per (HID/UID, sequence-step, register, bits, polarity).

**Step 2: confirm with a UEFI-shell PMU snapshot.**

Boot UEFI shell, dump `0xFD8D0000..0xFD8D1000` (or whichever range covers the relevant PMU registers per Step 1) to a file while the firmware's boot logo is on screen — the codec PD must be on for the logo. Compare against the same dump taken before any codec activity. The diff is the "what UEFI itself programs to bring the codec up" answer, and it sanity-checks the offsets we got from the DTS.

**Step 3: decide where the bring-up code lives.**

Two options, pick one for Phase 3b:

- **Driver-side (recommended for v1).** `rkmpp_ccu.sys` does the bring-up directly — replace the current PMU no-op with the real sequence derived in Step 1, plus restore the CRU writes (with corrected offsets). Faster iteration: rebuild + redeploy + smoke without touching firmware. We're already structured for this (`pmu.c`, `ccu.c`).
- **Firmware-side (the eventual right answer).** Author `Method (_PR0)` on RKCP3550 / RKCP3540 / etc. that returns a `PowerResource`, with `_ON`/`_OFF` methods that program the PMU/CRU via `OperationRegion (SystemMemory, 0xFD7C0000, ...)`. acpi.sys then invokes `_ON` automatically before IRP_MN_START_DEVICE. The Windows driver becomes much smaller — no PMU map, no system CRU map, no in-driver clock framework substitute. This is the standard ACPI mechanism for exactly this problem.

For Phase 3b we go driver-side first — we want to iterate the bring-up sequence empirically against hardware, and rebuilding the Windows driver is faster than rebuilding firmware. Once the sequence is proven correct, a follow-up phase (3c?) ports the bring-up into ACPI `_PR0`/`_ON`/`_OFF` methods and removes the corresponding driver code.

**Acceptance gate for Task 2:** with the DTS-derived sequence applied in `RkMppCcuRaiseCluster`, a subsequent `READ_REGISTER_ULONG(0xFDC38100)` returns a non-zero hardware-id value. Capture the value in `docs/hw-observations.md`.

Until Task 2 lands, **don't proceed past it.** The remaining tasks all depend on codec MMIO working.

### 3. Restore the skipped paths and validate

Once Task 2 lands a working bring-up sequence, re-enable in order, hardware-testing each:

- a. PMU PowerOn/Off in `pmu.c` (`RKMPP_PMU_DISABLE_POWER_GATING = 0`). Smoke must still exit 0.
- b. REVISION read in `rkmpp/device.c::PrepareHardware`. Capture the rev value to `docs/hw-observations.md`.
- c. DeassertCoreReset call (now safe because cluster is genuinely up).
- d. `RkIommuEnable` in `rkiommu/ifc.c::MapMdl` (with the BSP flags from Phase 3a-6). Smoke must still exit 0 and IOMMU INT_STATUS reads as 0.
- e. RegisterFaultHandler call in `rkmpp/device.c`.
- f. `WdfInterruptCreate` in `rkiommu/device.c`.

Each of these has been independently shown to bugcheck on the current firmware; they restore one at a time so the regression source is obvious if any single one breaks the smoke.

### 4. Add buffer-handle substitution to SUBMIT_JOB

Extend `RKMPP_REG_WRITE` with an optional `BufferHandle` + `IovaOffset` field. When set, the driver overwrites `Value` with `iova-of-handle + IovaOffset` at submit time. Test: golden-vector translation in user mode + a kernel test that submits a write with a substituted iova and observes the rewritten value via a peek IOCTL.

### 5. Vendor FFmpeg H.264 parser as a separate DLL

Bring in `libavcodec/h264_parser.c`, `h264_ps.c`, `h264_slice.c`, `h264_sei.c`, `h264data.c`, plus the minimum `libavutil` they pull in. Build with MSVC ARM64 as `ffmpeg_h264.dll`, LGPL-compliant, no codec backends, no demuxers. Static-link only `libavutil` minimal subset.

### 6. Vendor V4L2-stateless H.264 control headers

Copy `v4l2-controls.h` H.264 sections + the `v4l2_ctrl_h264_*` struct definitions into `external/v4l2-h264-controls/` under their original GPL-2 with note. Add a thin C++ wrapper for use from `mft/` code.

### 7. `mft/parser_glue.cpp`

Call FFmpeg's parser on input bytes, walk the output to populate the V4L2 control structs (`v4l2_ctrl_h264_sps`, `_pps`, `_scaling_matrix`, `_slice_params`, `_decode_params`, `_pred_weights`). Golden-vector unit tests using known H.264 streams.

### 8. `mft/regbuilder_h264.cpp`

Read the V4L2 control structs, produce a `RKMPP_REG_WRITE[]` for rkv-decoder-v2's H.264 register layout. The register layout is in `mpp_rkvdec2_h264.c` (BSP). Golden-vector tests: known control struct → expected register payload (captured offline by patching the BSP driver to dump its register writes).

### 9. `mft/dpb.cpp`

DPB manager: pick output frame buffers, resolve reference-frame buffer handles per slice, hand them to the register builder. Golden tests for DPB ordering.

### 10. `mft/buffer.cpp`

Wrap an `RKMPP_ALLOC_BUFFER_OUT` as an `IMFMediaBuffer` with `Lock()` returning the pre-mapped user VA. (Not yet wired to MF — Phase 6 — but the wrapper itself is testable.)

### 11. Replace `RkMppJobStart` stub with real kick

Write each register in the supplied list to `MmioBase + offset`, then write the kick register (`RKVDEC_REG_RUN` or equivalent — confirm in BSP). The ISR/DPC path (re-enabled in Task 3) handles completion. Software-completion DPC removed.

### 12. First-decode test harness

New tool `tests/harness/rkmpp_decode` that:
- Reads an H.264 .mp4 / Annex-B byte stream from disk.
- Splits NALs, runs `parser_glue` over them.
- Runs `dpb` + `regbuilder_h264` to produce a register list per slice.
- Allocates output + reference frame buffers via `IOCTL_RKMPP_ALLOC_BUFFER`.
- Submits one register-list job per slice via `IOCTL_RKMPP_SUBMIT_JOB`.
- On WAIT, reads back the output frame buffer and writes YUV to disk.
- Compares against an FFmpeg software-decode reference; exits 0 on bit-exact match.

### 13. First I-frame acceptance

Run `rkmpp_decode` against `i_frame_only_8x8.h264` on RK3588 hardware. Expected: exit 0, bit-exact YUV match against the FFmpeg reference produced by `ffmpeg -i input.mp4 -vf scale=8:8 -bf 0 -g 1 -c:v libx264 -f rawvideo -pix_fmt nv12 ref.yuv`.

### 14. Single-GOP acceptance

Generate a 16-frame GOP-1 sequence (one I-frame followed by 15 P-frames). Run end-to-end, verify hash match against FFmpeg software reference.

### 15. Bring up RVD1 in parallel

Currently we only deassert RVD0's reset. Extend the CCU ifc to take a core index, deassert RVD1, and submit jobs to RVD1 in addition to RVD0. Acceptance: 200-iteration smoke alternating between RVD0 and RVD1.

### Files affected

```
external/ffmpeg-h264/               # vendored FFmpeg subset
  libavcodec/...
  libavutil/...
  CMakeLists.txt
  LICENSE                           # LGPL-2.1
external/v4l2-h264-controls/
  v4l2-controls.h                   # vendored
  v4l2_h264_controls.hpp            # C++ wrapper
shared/
  rkmpp_ioctl.h                     # MODIFY — add BufferHandle field to RKMPP_REG_WRITE
mft/                                # NEW — user-mode helpers, used by harness now,
                                    # by the IMFTransform later.
  parser_glue.cpp / .h
  regbuilder_h264.cpp / .h
  dpb.cpp / .h
  buffer.cpp / .h
  CMakeLists.txt
driver/rkmpp/
  job.c                             # MODIFY — replace stub with real kick
  ioctl.c                           # MODIFY — perform iova substitution at submit
tests/data/
  i_frame_only_8x8.h264             # NEW
  i_frame_only_8x8.ref.yuv          # NEW (FFmpeg-produced reference)
  gop1_16f_8x8.h264                 # NEW
  gop1_16f_8x8.ref.yuv              # NEW
tests/harness/rkmpp_decode/
  CMakeLists.txt
  main.cpp
  decode_engine.cpp / .h            # ties parser_glue + regbuilder + IOCTLs
```

### What's deferred to Phase 4+

- Media Foundation Transform shell (`IMFTransform` impl in `mft/transform.cpp`) — Phase 4.
- MFT registration (`MFTRegister`) — Phase 4.
- Films & TV integration test — Phase 4.
- HEVC, VP9, AV1 — Phase 5+.
- Async MFT, D3D11 zero-copy — Phase 6+.

### Acceptance signal

A single command on RK3588:
```cmd
rkmpp_decode --in tests\data\i_frame_only_8x8.h264 --out out.yuv
fc /b out.yuv tests\data\i_frame_only_8x8.ref.yuv
echo exit=%ERRORLEVEL%
```

Expected: `FC: no differences encountered` and `exit=0`. That's the first end-to-end hardware decode through our stack on Windows.
