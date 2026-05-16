# VP9 decode support — MFT + rkvdec driver

Status: design, not yet implemented.
Author: brainstorming session 2026-05-15.

## 1. Scope

Add VP9 Profile 0 (8-bit 4:2:0) hardware-accelerated decode through a
new Media Foundation Transform that submits jobs to the existing
`rkvdec` kernel driver via the dense-bank IOCTL path.

Out of scope (tracked as follow-ups, separate specs):

- Profile 2 (10-bit). NV15 -> P010 unpack, same shape as H.264/H.265
  10-bit.
- Profile 1 / 3 (4:4:4 chroma). Hardware HAL never validated this in
  the BSP; no business case yet.
- WebM demux inside `mft_play`. Bring-up uses IVF; container plumbing
  is a separable change.
- VP9 FATE conformance pass.

Green-light success criterion: bit-exact NV12 output vs ffmpeg on a
720p VP9 IVF clip via `mft_decode`.

## 2. Architecture

VP9 reuses the existing RKVDEC2 (vdpu34x) hardware and therefore:

- Lives inside the existing `rkvdec` driver binary. No new INF, no new
  IOMMU pair-driver, no new device split.
- Uses `IOCTL_RKMPP_SUBMIT_DENSE_JOB` unchanged. VP9 only touches
  SWREGs that already fall inside covered bank ranges (see
  `shared/rkmpp_ioctl.h`: common 8..32, codec_params 64..112,
  common_addr 128..142, codec_addr 160..199, highpoc 200..204,
  stat 256..277).
- Uses the same MMIO geometry as H.264/H.265: SWREG base, INT_STATUS
  offset, DEC_E kick register, PERF_WORKING_CNT for polled drain.

User-mode does all the codec-specific work:

```
mft/
  vp9_parser.{cpp,h}          uncompressed + compressed header parser
  vp9_default_probs.h         generated initial probability tables
  vp9_dpb.{cpp,h}             8-slot ref map + show_existing_frame
  regbuilder_vp9.{cpp,h}      Vp9PicParams -> RKMPP_DENSE_BANK
  engine/decode_engine_vp9.{cpp,h}    DecodeEngine vp9 path
  dll/                        new COM CLSID + MFT registration
```

The MFT advertises:

- Input subtype: `MFVideoFormat_VP90`.
- Output subtype: `MFVideoFormat_NV12` (Profile 0).
- Container hint: IVF for bring-up; MP4/WebM later.

## 3. Parser

`vp9_parser` is hand-written against the VP9 bitstream specification
(WebM project, Jan 2017). No upstream code is imported; licensing
matches the rest of the tree.

Stage 1 -- uncompressed header (bit-reader, spec section 6.2):

- frame marker, profile, show_existing_frame (early-out: no kick),
  frame_type, show_frame, error_resilient_mode.
- Color config (bit_depth, color_space, color_range, subsampling).
- Frame size + render size; or frame size from ref for inter frames.
- ref_frame_idx[3] + ref_frame_sign_bias.
- allow_high_precision_mv, interp_filter, refresh_frame_context,
  frame_parallel_decoding_mode, frame_context_idx.
- Loop filter params (level, sharpness, mode/ref deltas).
- Quantization (base_qindex + deltas + segment overrides).
- Segmentation (enable, update_map, update_data, feature data).
- Tile config (tile_cols log2, tile_rows log2).
- header_size.

Stage 2 -- compressed header via VP9 boolean decoder (~150 lines):

- read_tx_mode, tx_mode_probs (if tx_mode == ALLOW_4_TO_32).
- read_coef_probs (per-tx-size flag + prob_diff_update of 1728 coef
  probs).
- read_skip_prob, read_inter_mode_probs, read_interp_filter_probs,
  read_is_inter_probs, frame_reference_mode + probs, single/comp
  ref probs, y_mode_probs, partition_probs, mv_probs.

Stage 2 output: a flat `Vp9ProbUpdates` struct containing only the
deltas the bitstream actually carried. The regbuilder applies these
to the per-context prob buffer.

Stage 3 -- emit `Vp9PicParams` (internal struct, DXVA-shaped but not
dependent on Windows DXVA headers): frame_refs[3].Index7Bits,
ref_frame_map[8].Index7Bits, frame_context_idx, profile, bit_depth,
txmode, interp_filter, segmentation arrays, loop-filter arrays,
ref_frame_sign_bias, refresh_frame_flags, allow_high_precision_mv,
tile_cols/rows.

## 4. DPB

`vp9_dpb` owns 8 reference-frame slots:

```
struct Vp9DpbSlot {
    bool       valid;
    BufferRef  frame;      // user-mode pinned NV12 buffer handle
    BufferRef  colmv;      // per-frame colmv buffer
    uint32_t   width, height;        // coded dims
    uint8_t    bit_depth;
};
```

Operations:

- `update(refresh_frame_flags, cur)`: write `cur` into every slot
  whose bit is set in `refresh_frame_flags` (spec section 8.2).
- `show_existing_frame(idx)`: return slot[idx].frame for emission
  without triggering an HW kick.
- Ref-frame **scaling** support: each ref records its coded dims so
  the regbuilder can program reg104..107 (ref widths/heights + fixed
  point scaling factors `(ref_w << 14) / cur_w`).

Probability contexts: `frame_context_idx` selects one of 4 per-session
probability buffers. The regbuilder keeps 4 user-allocated prob
buffers per MFT instance and routes each kick to the correct one;
`prob_save_en` is mirrored from `refresh_frame_context` so hardware
writes updated probs back into the same buffer.

## 5. Regbuilder

`regbuilder_vp9` translates `Vp9PicParams` + DPB state into a
`RKMPP_SUBMIT_DENSE_JOB_IN`. Mirrors `hal_vp9d_vdpu34x_gen_regs` from
the BSP field-for-field. Bank fills (idx ranges as defined in
`shared/rkmpp_ioctl.h`):

- common[8..32]: stream length, mode = VP9, error config. dec_e at
  idx 10 is left zero -- the kernel writes the kick value last via
  `KickValue = 0x1`.
- codec_params[64..112]: reg64 frame-flags, reg65 cur_poc, reg95/96/97
  last/golden/altref poc, reg98 col_ref_poc, reg100 segid_ref_poc,
  reg103 prob/intra/txmode/refresh/prob_save flags, reg104..107 ref
  widths/heights + scaling factors, loop-filter / quant /
  segmentation deltas.
- common_addr[128..142]: stream base, decout base, error info base,
  rcb bases.
- codec_addr[160..199]: ref[0..7] frame bases, ref[0..7] colmv bases,
  reg167 stream tail, reg168/169 segid_last/segid_cur,
  reg170 prob_default, reg171 prob_base.
- highpoc[200..204]: per VP9 the BSP writes the four ref POCs here.
- stat[256..277]: zero-filled at submit time; read back through
  PEEK or kernel-side stat capture for diagnostics.

Iova substitution slots (worst case ~32): stream, decout, 8 ref
frames, 8 ref colmvs, current colmv, segid_last, segid_cur,
prob_default, prob_base, rcb (up to 10). Well inside the 64-slot
cap.

Probability-buffer fill (not part of the dense bank, written into
the user-pinned prob MDL before submit):

- On keyframe or intra_only: write defaults from
  `vp9_default_probs.h`.
- Otherwise: walk parsed `Vp9ProbUpdates` and patch deltas at the
  byte offsets the hardware expects. Layout is captured from BSP
  shim runs (see section 7).

## 6. Kernel driver changes

Deliberately small:

1. `driver/rkvdec/profile.c`: add `RKMPP_CODEC_VP9` to
   `SupportedCodecs` surfaced via `IOCTL_RKMPP_QUERY_INFO`.
2. `driver/rkvdec/ioctl.c`: confirm `RkMppDenseIsAddressReg` already
   covers reg167..171 (all are in codec_addr[160..199]) -- no code
   change expected, just an audit.
3. `driver/rkvdec/job.c`: no change. `RKMPP_CODEC_OPS` is unchanged
   (same SWREG base, INT_STATUS, kick register, drain register).
4. No new IOCTLs.
5. No FullCoreReset changes (VP9 uses the same reset bundle as
   H.264/H.265 on this core).
6. INF (`rkvdec.inx`): no change required; the device is already
   enumerated.

The driver split (rkvdec / rkav1d / rkiommu_vdec / rkiommu_av1d)
stays as is. VP9 rides on rkvdec + rkiommu_vdec.

## 7. Bring-up plan

Each step is independently verifiable before the next.

1. **Parser + DXVA dump (host-only).** Extend the existing
   `parser_dump`-style harness to dump per-AU `Vp9PicParams` and
   `Vp9ProbUpdates`. Cross-check against ffmpeg `vp9_parser`
   debug output on a 720p IVF. No hardware required.
2. **Regbuilder vs BSP shim diff.** Capture an rkmpp VP9 run on
   `rk` under the BSP shim on the same 720p IVF. Diff our
   dense-bank output against the shim's register trace per-kick.
   Iterate until bit-exact. Uses the same harness pattern as
   `winreplay_diff_harness`.
3. **First hardware kick.** Load rkvdec on the Windows ARM64
   target, submit a single keyframe-only IVF via `mft_decode`.
   Confirm DEC_RDY, inspect NV12 output. Highest-likelihood
   first-time failures: reg104/105 scaling math, prob_default
   mis-fill, segid buffer not zeroed on first frame.
4. **Inter frames.** Extend to a short GOP IVF. Then
   show_existing_frame cases. Hash each emitted NV12 frame and
   diff against ffmpeg.
5. **Bit-exact 720p IVF vs ffmpeg via mft_decode.** Green-light
   criterion.
6. **mft_play integration.** Register the MFT, verify playback
   of an IVF in mft_play. (WebM demux is a separate follow-up.)

Test streams under `tests/data/vp9/`:

- keyframe-only 720p (~10 frames)
- IPPP 720p GOP
- GOP with `show_existing_frame`
- segmentation_enabled=1
- loop-filter-stressed (high deltas)
- 1080p sanity

## 8. Risks

Ranked by likelihood of biting us during bring-up.

- **Compressed-header bool-decoder bugs (medium).** Small errors
  propagate silently. Mitigation: unit-test the decoder against a
  captured prob_update payload from ffmpeg before any HW work.
- **Probability-buffer byte layout (medium).** BSP
  `hal_vp9d_prob_flag_delta` writes specific offsets that aren't
  well documented. Mitigation: capture the BSP prob buffer
  post-init via the shim and byte-diff during step 2.
- **Reference scaling reg math (low-medium).** reg104..107 encode
  fixed-point ratios; off-by-one produces green tearing.
  Mitigation: shim diff catches it.
- **Multi-tile column entry points (low for target).** 720p
  typically has tile_cols=1, so step 5 doesn't exercise this.
  Multi-tile is on the FATE follow-up.
- **show_existing_frame integration in MFT (low).** No HW kick;
  just re-emit the cached DPB slot frame.

## 9. What this design does not change

- No new driver binary.
- No new IOMMU pair-driver.
- No new IOCTLs.
- No firmware / ACPI changes.
- No CCU clock-rate work (RVD0/RVD1 already at BSP rates per
  `b705bfa`).
- No kernel-side buffer allocator (all prob/segid/colmv buffers are
  user-allocated MDLs reused from the existing buffer-handle pool).

## 10. Related work / references

- BSP HAL: `mpp/hal/rkdec/vp9d/hal_vp9d_vdpu34x.c` (1106 lines,
  authoritative for register field semantics, treat as reference
  only -- do not copy code).
- BSP codec layer: `mpp/codec/dec/vp9/vp9d.c` (parser shape; do
  not import -- hand-write against spec).
- Existing dense-bank consumer: `mft/regbuilder_h265.cpp`.
- DecodeEngine vtable: `mft/engine/decode_engine.{cpp,h}`.
- IOCTL surface: `shared/rkmpp_ioctl.h`.
