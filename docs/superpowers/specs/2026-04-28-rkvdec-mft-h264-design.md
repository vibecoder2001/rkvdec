# rkvdec H.264 MFT for Windows on RK3588 — Design

Date: 2026-04-28
Status: Approved for planning

## 1. Goal & scope (v1)

Hardware-accelerated H.264 decoder for Windows on ARM64 / RK3588 that plugs into Media Foundation as a decoder MFT. Acceptance gate for v1: Films & TV plays an 8-bit H.264 .mp4 from disk start to finish on RK3588 hardware without artifacts.

Explicit non-goals for v1, but the architecture must leave seams for them:

- HEVC, VP9, AV1.
- Zero-copy D3D11 output for browser / compositor consumption.
- Async hardware MFT registration (sync MFT in v1).

## 2. High-level architecture

Four components, shipped together. The kernel side mirrors the firmware's ACPI device decomposition (see RKCP35xx HID map) — one Windows class driver per device class, not one per codec.

```
+---------------------- user mode -----------------------+
|   rkvdec_mft.dll  (COM, IMFTransform)                  |
|   ├─ H.264 parser        (vendored from FFmpeg)        |
|   ├─ DPB / refframe mgmt                               |
|   ├─ Register-list builder  (rkv-decoder-v2 H.264)     |
|   └─ Buffer pool client                                |
+---------------------|----------------------------------+
                      |  IOCTLs (job submit, buf alloc)
+---------------------V----------------------------------+
|   rkmpp.sys   (KMDF, codec class driver)               |
|   matches HIDs RKCP3510..RKCP3560 (codec cores)        |
|   ├─ Per-instance state by HID + _UID                  |
|   ├─ MMIO / IRQ                                        |
|   ├─ Job queue + completion DPC                        |
|   ├─ Buffer pool (DMA-coherent common buffers)         |
|   └─ In-kernel calls UP to ccu, DOWN to iommu          |
+-----|---------------------------------|----------------+
      |                                 |
      v                                 v
+-------------------+         +--------------------------+
| rkmpp_ccu.sys     |         | rkiommu.sys              |
| matches RKCP3501  |         | matches RKCP3570 + 3571  |
|        ..RKCP3503 |         | one instance per _UID    |
| clocks/reset/pwr  |         | exposes MapMdl(client,   |
| per cluster       |         |   mdl, role) -> iova     |
+-------------------+         +--------------------------+
```

Key invariants:

- **rkmpp.sys is codec-class-agnostic.** It accepts a register-list + buffer-handle-list job descriptor and reports completion. Adding HEVC / VP9 / AV1 later is a new user-mode parser and register-list builder, with no kernel change beyond declaring the additional codec capability for the relevant HID/_UID.
- **IOMMU programming lives in rkiommu.sys, not in the codec driver.** rkmpp.sys obtains an iova for a buffer by calling rkiommu.sys via an in-kernel device interface; it never touches the IOMMU MMIO directly. This matches the ACPI topology (RKCP3570/3571 are independent devices) and lets a future AV1 path reuse rkiommu.sys with `_UID = A1MU`.
- **Clocks, resets, and power for the rkv-decoder-v2 cluster live in rkmpp_ccu.sys (RKCP3503).** rkmpp.sys raises/drops the cluster via an in-kernel CCU interface around job batches.
- **v1 only wires the rkv-decoder-v2 path:** RKCP3550 (RVD0, _UID = 0), its CCU RKCP3503 (RDCC), and its IOMMU instance under RKCP3570 (_UID matching RD0M). Other matching HIDs on the same INFs may load and probe but stay inert.
- **MPSV (RKCP3500, mpp-service) is out of scope for v1.** On Linux mpp-service multiplexes userspace requests across cores; on Windows that role is filled by Media Foundation + a single device handle to rkmpp.sys, so we do not need an mpp-service driver to ship H.264. Revisit when adding multi-core load balancing.

## 3. Kernel drivers

### 3.1 rkmpp.sys — codec class driver

KMDF, ARM64. INF matches HIDs RKCP3510..RKCP3560. Per device instance, the driver reads HID + `_UID` and selects a feature profile; in v1 only the (RKCP3550, _UID=0) profile (rkv-decoder-v2 RVD0, H.264 capable) does anything beyond minimal probe.

Hardware resources owned per instance:

- The codec core's MMIO range (from ACPI `_CRS`).
- The codec core's IRQ (from ACPI `_CRS`).

Hardware resources NOT owned:

- IOMMU MMIO — owned by `rkiommu.sys`. rkmpp.sys obtains an iova by calling rkiommu via an in-kernel device interface, identifying itself by its own ACPI handle so rkiommu picks the right IOMMU instance.
- Clocks, resets, power domains — owned by `rkmpp_ccu.sys`. rkmpp.sys raises the cluster around a job batch and drops it on idle via an in-kernel CCU interface.

IOCTL surface (minimal, codec-class-agnostic):

- `IOCTL_RKMPP_GET_CAPS` — returns a per-instance descriptor: HID, _UID, hardware revision word read from MMIO, and a supported-codec bitmap. v1 sets only the H.264 bit, and only on the RVD0 instance.
- `IOCTL_RKMPP_ALLOC_BUFFER` — input: size and usage tag (`BITSTREAM_INPUT`, `REFERENCE_FRAME`, `OUTPUT_FRAME`). Output: a kernel buffer handle plus a section the caller maps with `MapViewOfFile`. The buffer is IOMMU-mapped once at allocation (rkmpp asks rkiommu) and stays mapped until freed.
- `IOCTL_RKMPP_FREE_BUFFER`.
- `IOCTL_RKMPP_SUBMIT_JOB` — METHOD_BUFFERED descriptor pointing at a register-list payload plus an array of `(buffer_handle, role)` entries. Returns a job id.
- `IOCTL_RKMPP_WAIT_JOB` — cancel-safe blocking wait, returns hardware status word and timing.

Buffer model is hybrid: bitstream input may be either a transient "lock these user pages, IOMMU-map for this job, unmap on completion" path or a pooled buffer; output and reference frames must be pooled.

Concurrency: per-instance single in-flight job, FIFO queue. ISR raises a DPC that marks the current job complete and pulls the next. Cross-instance parallelism (RVD0 + RVD1) is supported by the design but not exercised in v1.

Power: rkmpp.sys asks rkmpp_ccu.sys to raise the cluster on first queued job and drop it after an idle timeout.

Failure modes:

- Hardware timeout: ask rkmpp_ccu.sys to assert the core's reset line, fail the in-flight job with `STATUS_DEVICE_HUNG`, keep driver alive and queue running.
- IOMMU page fault: rkiommu.sys reports the faulting iova up to its registered client (rkmpp.sys); rkmpp treats it like a timeout and records the iova in engineering telemetry.
- Process exit: buffers and pending jobs owned by the file object are torn down by `EvtFileCleanup`, which calls rkiommu to release iova mappings.

### 3.2 rkmpp_ccu.sys — codec cluster control unit driver

KMDF, ARM64. INF matches RKCP3501 (vpu-jpege CCU), RKCP3502 (rkv-encoder-v2 CCU), RKCP3503 (rkv-decoder-v2 CCU). v1 only meaningfully drives RKCP3503.

Owns the cluster's CCU MMIO range. Exposes an in-kernel device interface (`GUID_DEVINTERFACE_RKMPP_CCU`) with these operations, addressed by client ACPI handle so a single CCU instance can serve multiple cores in its cluster:

- `RaiseCluster(client)` — ungate clocks, deassert resets, raise power domain. Idempotent / refcounted.
- `DropCluster(client)` — refcounted release.
- `AssertCoreReset(client)` / `DeassertCoreReset(client)` — per-core reset for hang recovery.

In v1, RKCP3550's RVD0 is the only client that exercises the RKCP3503 instance.

### 3.3 rkiommu.sys — IOMMU class driver

KMDF, ARM64. INF matches RKCP3570 (rockchip iommu-v2, with up to 11 _UID instances covering all the per-block IOMMUs) and RKCP3571 (rockchip iommu-av1d). Per device instance, identifies itself by HID + _UID and owns the corresponding IOMMU's MMIO + IRQ.

Exposes an in-kernel device interface (`GUID_DEVINTERFACE_RKIOMMU`) with:

- `MapMdl(client, mdl, role) -> iova` — pin pages, program IOMMU page table entries, return a device-visible iova range.
- `UnmapMdl(client, iova)`.
- `RegisterFaultHandler(client, callback)` — IOMMU fault DPC dispatches to the registered client (rkmpp.sys).

The IOMMU client is identified by its ACPI handle; rkiommu maintains a static mapping at boot from "client ACPI handle → which IOMMU _UID serves it" by walking the ACPI namespace once at start. For v1 the only entry that matters is RVD0 → RD0M.

## 4. User-mode MFT — rkvdec_mft.dll

Implements `IMFTransform`. Registered with `MFTRegister` under `MFT_CATEGORY_VIDEO_DECODER` with `MFT_ENUM_FLAG_HARDWARE` so MF treats it as a hardware decoder. Not tied to a DXVA device — no WDDM video device exists on RK3588 in v1.

MFT registration is **synchronous** in v1. Async hardware-MFT registration is a v2 option for real-time pipelines.

Media types:

- Input: `MFVideoFormat_H264` and `MFVideoFormat_H264_ES`. Accepts `MF_MT_MPEG_SEQUENCE_HEADER` if present, otherwise extracts SPS/PPS from in-band Annex-B.
- Output (v1): `MFVideoFormat_NV12`, 8-bit 4:2:0, frame size derived from SPS. NV15 / P010 added when HEVC Main10 lands.

Per-input-sample pipeline:

1. Annex-B / AVCC NAL split.
2. Vendored FFmpeg H.264 parser updates SPS / PPS state, parses slice headers, and produces a set of `v4l2_ctrl_h264_*`-shaped control structures per slice (`decode_params`, `sps`, `pps`, `scaling_matrix`, `slice_params`, `pred_weights`).
3. DPB manager picks an output frame buffer from the pool and resolves reference-frame buffer handles per slice.
4. Register-list builder converts the control structures into the rkvdec2 H.264 register payload. This is the rkvdec-specific code; the V4L2-request hwaccel is the reference.
5. `IOCTL_RKMPP_SUBMIT_JOB` → `IOCTL_RKMPP_WAIT_JOB`.
6. Wrap the output buffer's mapped view in a custom `IMFMediaBuffer` whose `Lock()` returns the already-mapped CPU pointer (system-memory consumer in v1) and whose internals carry the kernel buffer handle (the seam for D3D11 wrapping later).

Vendoring of the FFmpeg parser: brought in under `external/ffmpeg-h264/`, stripped to the parser, refs, and hwaccel control structures. **Shipped as a separate dynamically-linked .dll** to keep LGPL compliance simple.

## 5. Repo layout

```
rkvdec/
├─ docs/
│  └─ superpowers/
│     ├─ specs/                 # design docs (this file lives here)
│     └─ plans/                 # phase implementation plans
├─ driver/
│  ├─ rkmpp/                    # codec class driver (rkmpp.sys)
│  │  ├─ device.c, queue.c, power.c, job.c, ioctl.c
│  │  ├─ profile_rvdec_v2.c     # per-HID profile for RKCP3550
│  │  └─ rkmpp.inx
│  ├─ rkmpp_ccu/                # CCU driver (rkmpp_ccu.sys)
│  │  ├─ ccu.c, ifc.c
│  │  └─ rkmpp_ccu.inx
│  └─ rkiommu/                  # IOMMU driver (rkiommu.sys)
│     ├─ pgtable.c, fault.c, ifc.c, topology.c
│     └─ rkiommu.inx
├─ mft/                         # user-mode MFT (rkvdec_mft.dll)
│  ├─ transform.cpp             # IMFTransform impl
│  ├─ parser_glue.cpp           # FFmpeg parser → rkvdec controls
│  ├─ regbuilder_h264.cpp       # rkv-decoder-v2 H.264 register list
│  ├─ dpb.cpp
│  ├─ buffer.cpp                # IMFMediaBuffer wrapping kernel handles
│  └─ register.cpp              # MFTRegister entry points
├─ shared/                      # IOCTL codes, register-payload structs,
│  │                            #   in-kernel ifc GUIDs and structs
│  ├─ rkmpp_ioctl.h
│  ├─ rkmpp_ccu_ifc.h
│  └─ rkiommu_ifc.h
├─ external/ffmpeg-h264/        # vendored parser, built as separate .dll
├─ tests/
│  ├─ harness/                  # CLI dev-loop tool (decode .mp4 → YUV/hash)
│  └─ conformance/              # JM/Allegro H.264 vectors + reference hashes
└─ README.md
```

## 6. Data flow — one frame, happy path

1. MF hands the MFT an H.264 access unit as `IMFSample`.
2. MFT NAL-splits, parser updates state, produces per-slice control structs.
3. MFT acquires an output frame buffer from the pool (pre-allocated by rkmpp.sys; iova was set up at allocation time by rkiommu.sys).
4. MFT builds the register payload and calls `IOCTL_RKMPP_SUBMIT_JOB(payload, [bitstream_buf, output_buf, ref_buf_0..N])`.
5. rkmpp.sys asks rkmpp_ccu.sys to raise the cluster (refcounted, usually a no-op after the first job), writes registers, and kicks the core. ISR on completion raises a DPC that marks the job done.
6. MFT's `IOCTL_RKMPP_WAIT_JOB` returns; MFT wraps the output buffer's mapped view as an `IMFSample` and returns it through `ProcessOutput`.

## 7. Error handling

- Parser errors / corrupt bitstream: log, drop the access unit, do not kill the pipeline.
- Hardware timeout: rkmpp.sys asks rkmpp_ccu.sys to assert the core's reset line, deasserts, and returns `STATUS_DEVICE_HUNG`. MFT marks the current frame as a decode error, recovers at the next IDR, signals `MF_E_TRANSFORM_NEED_MORE_INPUT` upward.
- IOMMU fault: rkiommu.sys's IRQ DPC dispatches to the registered fault handler in rkmpp.sys, which fails the in-flight job exactly as for a timeout. Engineering telemetry includes the faulting iova reported by rkiommu.
- Buffer-pool exhaustion: MFT blocks on `ProcessOutput` rather than over-allocating, matching Media Foundation backpressure conventions.

## 8. Testing strategy

- Unit: `parser_glue` and `regbuilder_h264` tested against golden vectors — `(bitstream, expected control structs)` and `(control structs, expected register payload)`.
- Conformance: the standard JM / Allegro H.264 conformance suite, decoded through the real driver on physical RK3588 hardware, MD5 compared against reference YUV with zero mismatch tolerance.
- Integration A — primary dev loop: `tests/harness` decodes .mp4 → YUV, frame-hash compare against an FFmpeg software reference.
- Integration B — v1 acceptance: scripted Films & TV launch on RK3588 with on-device pixel capture of N seconds of a known clip, compared against a reference render.
- Stress: seek storms, mid-stream resolution changes, error injection (truncated NALs, missing references).

## 9. Build, signing, deploy

- Drivers: WDK 10 / KMDF / ARM64. All three (rkmpp, rkmpp_ccu, rkiommu) test-signed for development and attestation-signed for distribution alongside the existing Rockchip-Windows-Drivers package, matching that project's signing posture. Loading order is enforced by `Needs`/`Includes` directives in the INFs so rkiommu and rkmpp_ccu probe before rkmpp's first I/O.
- MFT: MSVC ARM64, C++17. Self-registers via `regsvr32` or the installer. The driver INF also drops the COM registration entries for the MFT.

## 10. Future seams — designed for, not built

- **D3D11 output:** the kernel buffer handle already round-trips through the custom `IMFMediaBuffer`. A future WDDM- or IddCx-style GPU driver wraps the same buffer as a `ID3D11Texture2D` via a shared NT handle. No MFT API change required.
- **HEVC / VP9:** new files under `mft/regbuilder_*.cpp` and new vendored parsers. The same RKCP3550 RVD0/RVD1 cores already match rkmpp.sys, so the only kernel work is enabling the codec bit in that profile.
- **AV1:** RKCP3560 (AV1D) is a different codec core but already matches rkmpp.sys's INF. Its IOMMU is RKCP3571 (A1MU), already matching rkiommu.sys's INF. So adding AV1 is a new MFT + new register-list builder + a new rkmpp profile entry; no new driver project.
- **Async MFT:** swap the MFT registration flag and add an event-driven completion thread riding on `WAIT_JOB`. Internal pipeline unchanged.

## 11. Open questions / risks

- rkv-decoder-v2 H.264 register layout on RK3588 is not officially documented for Windows; primary reference is the Linux mainline `rockchip-vpu` / `rkvdec2` driver. Risk: undocumented errata between SoC revisions. Mitigation: gate code paths on the revision word read at probe and surface it through `IOCTL_RKMPP_GET_CAPS`.
- Windows ARM64 KMDF maturity for non-Microsoft platform drivers: covered by the Rockchip-Windows-Drivers repo's track record, but the codec subsystem is the first place in the project where three drivers must coordinate via in-kernel device interfaces (rkmpp ↔ rkmpp_ccu, rkmpp ↔ rkiommu) with strict load ordering. Mitigation: bring up the three drivers as a unit in Phase 1 and make the in-kernel interfaces a tight, tested seam.
- ACPI topology assumption: rkiommu's "client ACPI handle → IOMMU _UID" mapping is built by walking the namespace at start. If firmware ever exposes the relationship explicitly (e.g., via an `_DSD` reference or an IORT-like table), prefer that. Until then, hardcode the topology table for RK3588 in `rkiommu/topology.c`.
- LGPL boundary: shipping the FFmpeg parser as a separate .dll keeps this clean; no static-link audit needed.
