# regbuilder_av1.cpp vs MPP hal_av1d_vdpu.c — gap analysis

Side-by-side audit of `mft/regbuilder_av1.cpp` against
`mpp/hal/vpu/av1d/hal_av1d_vdpu.c` from the rockchip-linux/mpp `develop`
branch HEAD (commit `c2c1ee50`, 2026-03).  This is the same MPP that
produced the `tests/data/av1/av1capture/` dumps — a known-working
reference, even though it post-dates the deployed librockchip-mpp 1.5.0
(which had since-rewritten parser code for licensing reasons).

Status legend: ✅ implemented · 🟡 partial · ❌ missing · ⚪ skipped (n/a
for our scope).

## Coverage of MPP's per-area set functions

| MPP function (line) | Our coverage | Notes |
|---|---|---|
| `vdpu_av1d_set_global_model` (1421) | 🟡 reg-side only | We don't build the global-model **content buffer** at all, only place gm_mode bits in swreg184-188/257/262.  reg83 DMA handle missing. |
| `vdpu_av1d_set_tile_info_mem` (1512) | ❌ missing | Builds the ~640-byte packed uncompressed-header into the tile_info buffer.  Hardware reads this every frame.  This is the single biggest blocker — captured per-frame as `tests/data/av1/av1capture/tile_info_N.txt`.|
| `vdpu_av1d_set_tile_info_regs` (1463) | 🟡 partial | We don't write reg167 (tile_base DMA). |
| `vdpu_av1d_set_reference_frames` (768) | ✅ done | Per-ref dims, scales, sign-bias, ref_frames count.  Matches BSP capture. |
| `vdpu_av1d_set_segmentation` (1221) | 🟡 seg0 only | seg1..7 delta1/2/3 + global_mv (swreg21-27) skeleton'd; only segmented streams hit this path. |
| `vdpu_av1d_set_loopfilter` (1385) | ✅ done | Filter levels + ref/mode deltas + filtering_dis. |
| `vdpu_av1d_set_picture_dimensions` (1203) | 🟡 dims only | Picture width/height present.  Superres + step calc partial — see Batch 16. |
| `vdpu_av1d_set_cdef` (1602) | ✅ done | CDEF strengths via swreg53/263/264. |
| `vdpu_av1d_set_lr` (1634) | ✅ done | swreg18 (type) + swreg19 (unit_size) match BSP. |
| `vdpu_av1d_set_fgs` (1681) | ❌ missing | Builds the film-grain LUT into `film_grain_mem` and writes reg95.  Streams without grain skip this — our 720p test stream has none. |
| `vdpu_av1d_set_prob` (749) | ❌ missing | Initialises the CDF probability tables and writes reg173 (in) / reg171 (out).  Required every frame regardless of features. |
| `vdpu_av1d_setup_tile_bufs` (1810) | ❌ missing | Allocates per-output-slot scratch buffers (colmv, segment_read).  Drives reg81 (segment_read) and others. |
| `vdpu_av1d_filtermem_alloc` (152) | ❌ missing | Single shared "filter_mem" buffer used by reg85/87/89/91/93/96/97/179/183 (cdef_colbuf, cdef_left_colbuf, superres_colbuf, lr_colbuf, lr_left_colbuf, dec_vert_filt, dec_bsd_ctrl).  All 9 of these regs need filter_mem's iova or codec AXI-faults. |

## Per-register gaps in inline gen_regs section (MPP lines 1845-2228)

| swreg | Field | MPP code | Ours | Gap |
|---|---|---|---|---|
| 5 | `sw_strm_start_bit` | `(frame_tag_size & 0xf) * 8` (2119) | 0 | Need OBU bit offset. |
| 6 | `sw_stream_len` | `MPP_ALIGN(strm_len, 128)` | aligned | ✅ |
| 9 | `sw_context_update_tile_id` | computed (1479) | 0 | Need to compute. |
| 65/99 | output Y/CbCr base | shared FD, reg99 +y_stride offset | reg65 only | reg99 needs +y_stride offset, reg133 needs +mv_offset. |
| 133 | dec_out_dbase | shared FD, +mv_offset | not written | Same buffer, offset = y_stride + uv_stride + 64. |
| 167 | tile_base | tile_info FD | not written | DMA gap. |
| 168 | stream_base_msb | 0 | not written | Plain 0 write needed. |
| 169 | stream_base_lsb | bitstream FD, +(frame_tag_size & ~0xf) | not written | DMA gap. |
| 175/177 | mc_sync_curr/left base | tile_buf FD | not written | DMA gap (when tile_buf in use). |
| 258 | strm_buffer_len | `MPP_ALIGN(strm_len, 128)` | ✅ | ✅ |
| 259 | strm_start_offset | 0 | not written | Plain 0 write. |
| 266 | sw_error_conceal_e | 0 | not written | Plain 0 write. |
| 326/328 | pp_out_lu_base/ch_base | output FD (reg328 +y_stride) | not written | PP output DMA. |
| 503-507 | AFBC fbc_en path | conditional | ❌ | Skip for NV12 non-FBC. |

## Critical missing functions to implement

In priority order for first decode of `av1_720p.ivf`:

### 1. `vdpu_av1d_set_tile_info_mem` (the uncompressed-header builder)

Builds ~640 bytes into `tile_info` (RK_U8 buffer).  Format: packed
big-endian bitstream describing per-tile geometry, segmentation map
references, CDEF index per superblock, etc.  Reverse-engineerable from
`tests/data/av1/av1capture/tile_info_N.txt` (hex u32 per line) by
diffing against frame headers.

### 2. `vdpu_av1d_set_prob`

Initialises CDF probability tables (~70 KiB structure of `AV1CDFs`).
Reset to defaults from `default_av1_cdfs.h` for keyframes;
inter frames use either default or copy from `prob_tbl_out_base` of
the primary reference frame.  Captured as `prob_tbl_N.txt`.

### 3. `vdpu_av1d_set_global_model`

Packs warped-motion matrices for each ref into `global_model` buffer.
Identity for keyframes.  Captured as `global_mode_N.txt`.

### 4. Conditional DMA address emission (filter_mem, tile_out_buf, refs)

Beyond the four authoritative DMA writes we already do
(output_y/output_c/tile_info/bitstream), the codec hits:

- `filter_mem` at reg 85, 87, 89, 91, 93, 96/97, 179, 183
- `tile_out_buf` (per-output-slot) at reg 81, 133
- per-ref Y/C/D bases at reg 67-79, 101-113, 135-147 (when ref active)

These all share a few backing buffers; emit only when relevant feature
is enabled (CDEF, LR, segmentation, refs).

### 5. `swreg5.sw_strm_start_bit` + `swreg9.sw_context_update_tile_id`

Two scalar fields easy to fill once we have the OBU layout offsets
from the parser.

## Less critical / out of scope for first decode

- **AFBC/FBC output path** (swreg503 etc.): we output NV12, fbc_en=0.
- **swreg190/224** (dec_out_t{y,c}base for FBC): fbc_en=0.
- **Segmentation regs 21-27 seg1..7 delta1/2/3 + global_mv**: only for
  streams with segmentation enabled.
- **swreg258/259 strm offsets when frame_tag_size != 0**: 720p test
  has frame_tag_size=0 in IVF.

## How to use this doc

Each missing function above has a captured target at
`tests/data/av1/av1capture/`:

- `tile_info_N.txt` ↔ `vdpu_av1d_set_tile_info_mem`
- `global_mode_N.txt` ↔ `vdpu_av1d_set_global_model`
- `prob_tbl_N.txt` ↔ `vdpu_av1d_set_prob` (input CDF for kick N)
- `prob_tbl_out_N.txt` ↔ post-decode CDF (don't need to build, codec writes)
- `film_grain_mem_N.txt` ↔ `vdpu_av1d_set_fgs` (when grain present)
- `reg_N_in.txt` ↔ flat reg dump for cross-checking everything else

Implement → diff against capture per kick → bit-exact win.

Reference path on rk box: `/home/vibecoder/linux-rockchip/drivers/video/rockchip/mpp/`
(kernel side) and `/tmp/mpp-src/mpp/hal/vpu/av1d/` (userspace HAL we
patched + built).  Local mirror: `C:/Users/vibecoder/mpp/` on develop
branch.
