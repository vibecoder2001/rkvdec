/* mft/parser_glue_h265.h — minimal H.265 (HEVC) NAL → struct parser.
 *
 * Sibling of parser_glue.h.  Hand-rolled rather than vendoring FFmpeg
 * so the rest of the HEVC pipeline (regbuilder, DPB, hardware kick) can
 * land first.  Field set is what the rkvdec2 vdpu34x HEVC regbuilder
 * needs (cf. Vdpu34xRegH265d_t / hal_h265d_vdpu34x_gen_regs in the BSP).
 *
 * Scope:
 *   - Annex-B NAL framing + RBSP emulation-prevention unescape
 *   - Exp-Golomb bit reader (shared with H.264 path)
 *   - VPS / SPS / PPS / slice_segment_header parse
 *   - Short-term RPS table (SPS-side and slice-side)
 *   - Scaling list parse (when present in SPS or PPS)
 *
 * Out of scope (deferred):
 *   - SEI parsing (skipped — not needed for decode)
 *   - CABAC entropy (rkvdec hardware does CABAC itself)
 *   - Long-term ref management beyond capturing num_long_term_*  — our
 *     test streams (multi.h265, first_idr.h265) use STRPS only
 *   - Multi-layer / SHVC / MV-HEVC
 *   - PPS range / screen-content extensions
 *   - Tile column/row width tables (we capture tiles_enabled_flag and
 *     num_tile_{columns,rows} but the test streams are tile-free)
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    H265_PARSE_OK            = 0,
    H265_PARSE_NEED_MORE     = 1,  /* not enough data — VPS/SPS/PPS only */
    H265_PARSE_INVALID       = 2,  /* malformed bitstream */
    H265_PARSE_UNSUPPORTED   = 3,  /* feature outside our scope */
} H265ParseStatus;

/* ---- Limits (mirroring rockchip-linux mpp h265d_ctx.h) -------------- */
#define H265_MAX_VPS                16
#define H265_MAX_SPS                32
#define H265_MAX_PPS                64
#define H265_MAX_SUB_LAYERS          7
#define H265_MAX_REFS               16   /* per RPS list */
#define H265_MAX_SHORT_TERM_RPS     64
#define H265_MAX_DPB_SIZE           16
#define H265_MAX_LONG_TERM_REFS     32

/* Slice-type codes (H.265 7.4.7.1, table 7-7). */
#define H265_SLICE_TYPE_B           0
#define H265_SLICE_TYPE_P           1
#define H265_SLICE_TYPE_I           2

/* NAL unit type ranges (H.265 7.4.2.2, table 7-1). */
#define H265_NAL_TRAIL_N             0
#define H265_NAL_TRAIL_R             1
#define H265_NAL_TSA_N               2
#define H265_NAL_TSA_R               3
#define H265_NAL_STSA_N              4
#define H265_NAL_STSA_R              5
#define H265_NAL_RADL_N              6
#define H265_NAL_RADL_R              7
#define H265_NAL_RASL_N              8
#define H265_NAL_RASL_R              9
#define H265_NAL_BLA_W_LP           16
#define H265_NAL_BLA_W_RADL         17
#define H265_NAL_BLA_N_LP           18
#define H265_NAL_IDR_W_RADL         19
#define H265_NAL_IDR_N_LP           20
#define H265_NAL_CRA_NUT            21
#define H265_NAL_VPS                32
#define H265_NAL_SPS                33
#define H265_NAL_PPS                34
#define H265_NAL_AUD                35
#define H265_NAL_PREFIX_SEI         39
#define H265_NAL_SUFFIX_SEI         40

/* ---- Profile-tier-level (subset) ------------------------------------ */
typedef struct H265Ptl {
    uint8_t  profile_space;       /* 2 bits */
    uint8_t  tier_flag;           /* 1 bit  */
    uint8_t  profile_idc;         /* 5 bits — 1=Main, 2=Main10, 3=MainStill, 4=RangeExt */
    uint8_t  profile_compatibility_flag[32];
    uint8_t  progressive_source_flag;
    uint8_t  interlaced_source_flag;
    uint8_t  non_packed_constraint_flag;
    uint8_t  frame_only_constraint_flag;
    uint8_t  level_idc;           /* sub_layer ptl ignored */
} H265Ptl;

/* ---- Short-term reference picture set (7.4.7.1) -------------------- */
typedef struct H265ShortTermRPS {
    uint8_t  inter_ref_pic_set_prediction_flag;
    uint8_t  num_negative_pics;
    uint8_t  num_positive_pics;
    uint8_t  num_delta_pocs;      /* num_negative + num_positive */
    int16_t  delta_poc[H265_MAX_REFS];          /* signed; sorted: neg ascending then pos ascending */
    uint8_t  used_by_curr_pic_flag[H265_MAX_REFS];
} H265ShortTermRPS;

/* ---- Scaling-list payload (7.4.5) ---------------------------------- *
 * Four sizes: 4x4 (sz_id=0, 6 lists), 8x8 (1, 6), 16x16 (2, 6), 32x32
 * (3, 2 — Y/Cb default; 4:4:4 path adds the rest).  Stored flat: the
 * regbuilder packs these into the rkvdec2 scanlist buffer. */
typedef struct H265ScalingList {
    uint8_t  scaling_list_4x4    [6][16];
    uint8_t  scaling_list_8x8    [6][64];
    uint8_t  scaling_list_16x16  [6][64];
    uint8_t  scaling_list_32x32  [6][64];   /* only [0],[3] are spec-mandatory */
    uint8_t  scaling_list_dc_16x16[6];      /* sz_id=2 DC coeff */
    uint8_t  scaling_list_dc_32x32[6];      /* sz_id=3 DC coeff */
} H265ScalingList;

/* ---- VPS (7.3.2.1) ------------------------------------------------- */
typedef struct H265Vps {
    uint8_t  valid;
    uint8_t  vps_id;                              /* 4 bits */
    uint8_t  vps_max_layers_minus1;               /* 6 bits */
    uint8_t  vps_max_sub_layers_minus1;           /* 3 bits */
    uint8_t  vps_temporal_id_nesting_flag;
    H265Ptl  ptl;
    uint8_t  vps_sub_layer_ordering_info_present_flag;
    uint8_t  vps_max_dec_pic_buffering_minus1[H265_MAX_SUB_LAYERS];
    uint8_t  vps_max_num_reorder_pics        [H265_MAX_SUB_LAYERS];
    uint32_t vps_max_latency_increase_plus1  [H265_MAX_SUB_LAYERS];
} H265Vps;

/* ---- SPS (7.3.2.2) ------------------------------------------------- */
typedef struct H265Sps {
    uint8_t  valid;
    uint8_t  vps_id;
    uint8_t  sps_max_sub_layers_minus1;
    uint8_t  sps_temporal_id_nesting_flag;
    H265Ptl  ptl;

    uint8_t  sps_id;
    uint8_t  chroma_format_idc;
    uint8_t  separate_colour_plane_flag;

    uint32_t pic_width_in_luma_samples;
    uint32_t pic_height_in_luma_samples;

    uint8_t  conformance_window_flag;
    uint32_t conf_win_left_offset;
    uint32_t conf_win_right_offset;
    uint32_t conf_win_top_offset;
    uint32_t conf_win_bottom_offset;

    uint8_t  bit_depth_luma_minus8;
    uint8_t  bit_depth_chroma_minus8;
    uint8_t  log2_max_pic_order_cnt_lsb_minus4;

    uint8_t  sps_sub_layer_ordering_info_present_flag;
    uint8_t  sps_max_dec_pic_buffering_minus1[H265_MAX_SUB_LAYERS];
    uint8_t  sps_max_num_reorder_pics        [H265_MAX_SUB_LAYERS];
    uint32_t sps_max_latency_increase_plus1  [H265_MAX_SUB_LAYERS];

    uint8_t  log2_min_luma_coding_block_size_minus3;
    uint8_t  log2_diff_max_min_luma_coding_block_size;
    uint8_t  log2_min_luma_transform_block_size_minus2;
    uint8_t  log2_diff_max_min_luma_transform_block_size;
    uint8_t  max_transform_hierarchy_depth_inter;
    uint8_t  max_transform_hierarchy_depth_intra;

    uint8_t  scaling_list_enabled_flag;
    uint8_t  sps_scaling_list_data_present_flag;
    H265ScalingList scaling_list;       /* valid when sps_scaling_list_data_present_flag */

    uint8_t  amp_enabled_flag;
    uint8_t  sample_adaptive_offset_enabled_flag;

    uint8_t  pcm_enabled_flag;
    uint8_t  pcm_sample_bit_depth_luma_minus1;
    uint8_t  pcm_sample_bit_depth_chroma_minus1;
    uint8_t  log2_min_pcm_luma_coding_block_size_minus3;
    uint8_t  log2_diff_max_min_pcm_luma_coding_block_size;
    uint8_t  pcm_loop_filter_disabled_flag;

    uint8_t  num_short_term_ref_pic_sets;
    H265ShortTermRPS st_rps[H265_MAX_SHORT_TERM_RPS];

    uint8_t  long_term_ref_pics_present_flag;
    uint8_t  num_long_term_ref_pics_sps;
    uint16_t lt_ref_pic_poc_lsb_sps[H265_MAX_LONG_TERM_REFS];
    uint8_t  used_by_curr_pic_lt_sps_flag[H265_MAX_LONG_TERM_REFS];

    uint8_t  sps_temporal_mvp_enabled_flag;
    uint8_t  strong_intra_smoothing_enabled_flag;

    /* Derived (7.4.7) — useful for the regbuilder. */
    uint8_t  ctb_log2_size_y;        /* CtbLog2SizeY */
    uint8_t  ctb_size_y;             /* CtbSizeY = 1 << ctb_log2_size_y */
    uint32_t pic_width_in_ctbs_y;
    uint32_t pic_height_in_ctbs_y;
} H265Sps;

/* ---- PPS (7.3.2.3) ------------------------------------------------- */
typedef struct H265Pps {
    uint8_t  valid;
    uint8_t  pps_id;
    uint8_t  sps_id;

    uint8_t  dependent_slice_segments_enabled_flag;
    uint8_t  output_flag_present_flag;
    uint8_t  num_extra_slice_header_bits;
    uint8_t  sign_data_hiding_enabled_flag;
    uint8_t  cabac_init_present_flag;
    uint8_t  num_ref_idx_l0_default_active_minus1;
    uint8_t  num_ref_idx_l1_default_active_minus1;
    int8_t   init_qp_minus26;
    uint8_t  constrained_intra_pred_flag;
    uint8_t  transform_skip_enabled_flag;
    uint8_t  cu_qp_delta_enabled_flag;
    uint8_t  diff_cu_qp_delta_depth;
    int8_t   pps_cb_qp_offset;
    int8_t   pps_cr_qp_offset;
    uint8_t  pps_slice_chroma_qp_offsets_present_flag;
    uint8_t  weighted_pred_flag;
    uint8_t  weighted_bipred_flag;
    uint8_t  transquant_bypass_enabled_flag;
    uint8_t  tiles_enabled_flag;
    uint8_t  entropy_coding_sync_enabled_flag;

    /* Tiles (only num_tile_{columns,rows} captured — no width/height
     * tables; our test streams are tile-free.  Regbuilder author should
     * extend this when tile streams ship). */
    uint8_t  num_tile_columns_minus1;
    uint8_t  num_tile_rows_minus1;
    uint8_t  uniform_spacing_flag;
    uint8_t  loop_filter_across_tiles_enabled_flag;

    uint8_t  pps_loop_filter_across_slices_enabled_flag;
    uint8_t  deblocking_filter_control_present_flag;
    uint8_t  deblocking_filter_override_enabled_flag;
    uint8_t  pps_deblocking_filter_disabled_flag;
    int8_t   pps_beta_offset_div2;
    int8_t   pps_tc_offset_div2;
    uint8_t  pps_scaling_list_data_present_flag;
    H265ScalingList scaling_list;       /* valid when pps_scaling_list_data_present_flag */
    uint8_t  lists_modification_present_flag;
    uint8_t  log2_parallel_merge_level_minus2;
    uint8_t  slice_segment_header_extension_present_flag;
} H265Pps;

/* ---- Slice segment header (7.3.6) ---------------------------------- */
typedef struct H265SliceHeader {
    uint8_t  first_slice_segment_in_pic_flag;
    uint8_t  no_output_of_prior_pics_flag;
    uint8_t  slice_pic_parameter_set_id;
    uint8_t  dependent_slice_segment_flag;
    uint32_t slice_segment_address;
    uint8_t  slice_type;                          /* H265_SLICE_TYPE_* */
    uint8_t  pic_output_flag;
    uint8_t  colour_plane_id;
    uint16_t slice_pic_order_cnt_lsb;

    uint8_t  short_term_ref_pic_set_sps_flag;
    uint8_t  short_term_ref_pic_set_idx;
    H265ShortTermRPS st_rps_slice;                /* valid if !sps_flag */

    uint8_t  num_long_term_sps;
    uint8_t  num_long_term_pics;

    uint8_t  slice_temporal_mvp_enabled_flag;
    uint8_t  slice_sao_luma_flag;
    uint8_t  slice_sao_chroma_flag;

    uint8_t  num_ref_idx_active_override_flag;
    uint8_t  num_ref_idx_l0_active_minus1;
    uint8_t  num_ref_idx_l1_active_minus1;

    uint8_t  mvd_l1_zero_flag;
    uint8_t  cabac_init_flag;
    uint8_t  collocated_from_l0_flag;
    uint8_t  collocated_ref_idx;
    uint8_t  five_minus_max_num_merge_cand;

    int8_t   slice_qp_delta;
    int8_t   slice_cb_qp_offset;
    int8_t   slice_cr_qp_offset;

    uint8_t  deblocking_filter_override_flag;
    uint8_t  slice_deblocking_filter_disabled_flag;
    int8_t   slice_beta_offset_div2;
    int8_t   slice_tc_offset_div2;
    uint8_t  slice_loop_filter_across_slices_enabled_flag;

    uint32_t num_entry_point_offsets;
    /* entry_point_offset_minus1[] not stored — our test streams have 0. */

    /* Bit-position diagnostics for the regbuilder. */
    uint32_t header_bit_size;            /* bits consumed by slice header */
    uint32_t short_term_ref_pic_set_size;/* bits consumed by inline STRPS */
} H265SliceHeader;

/* ---- Aggregate parse result --------------------------------------- *
 * Persistent: vps[], sps[], pps[] (indexed by id) survive across
 * H265ParseAccessUnit calls so later AUs can reference them.  Reset
 * per-AU: slice header + decode flags + slice_data pointer.
 *
 * Mirrors the H.264 H264ParseResult sticky pattern. */
typedef struct H265ParseResult {
    H265Vps          vps[H265_MAX_VPS];
    H265Sps          sps[H265_MAX_SPS];
    H265Pps          pps[H265_MAX_PPS];

    H265SliceHeader  slice;
    uint8_t          has_slice;

    /* Active set for THIS access unit. */
    int8_t           active_vps_id;     /* -1 if none parsed yet */
    int8_t           active_sps_id;
    int8_t           active_pps_id;

    /* Has-flags for "what arrived in *this* AU". */
    uint8_t          got_vps_in_au;
    uint8_t          got_sps_in_au;
    uint8_t          got_pps_in_au;

    /* NAL unit type of the slice (or 0 if no slice). */
    uint8_t          slice_nal_unit_type;
    uint8_t          nal_ref_flag;       /* derived: !is RASL/RADL/SLNR */
    uint8_t          is_idr;             /* type == IDR_W_RADL or IDR_N_LP */
    uint8_t          is_irap;            /* BLA / IDR / CRA */

    /* Computed POC for the current picture.  Held as 32-bit signed; the
     * regbuilder splits this into reg65 (low 32) + reg204 (highbit). */
    int32_t          poc;

    /* Slice data (post-emulation-prevention RBSP, after the slice header).
     * Lifetime: valid until the next H265ParseAccessUnit call. */
    const uint8_t   *slice_data;
    size_t           slice_data_size;

    /* Per-result POC continuity state (H.265 8.3.1 PrevTid0Pic).
     * Previously two `static int32_t` locals inside H265ParseAccessUnit
     * — fine for a single-stream parser but a hazard once the just-
     * landed multi-core dispatch lets two H.265 sessions parse in
     * parallel from different threads (each clobbered the other's
     * MSB/LSB across the unprotected statics, producing nonsense POCs
     * after the first wrap).  Per-instance now.  Reset by
     * H265ParseResultInit. */
    int32_t          prev_poc_msb_tid0;
    int32_t          prev_poc_lsb_tid0;
} H265ParseResult;

/* Initialise a result struct (zero everything, mark all PS slots empty).
 * Call once before the first H265ParseAccessUnit; the parser keeps
 * VPS/SPS/PPS state sticky across subsequent calls. */
void H265ParseResultInit(H265ParseResult *out);

/* Parse one access unit's worth of NAL units.  `buf` is Annex-B framed.
 * `scratch` is used for the RBSP unescape step — pass at least 2x len.
 * Returns OK if a slice header was parsed; NEED_MORE if only param sets
 * were seen. */
H265ParseStatus H265ParseAccessUnit(const uint8_t *buf, size_t len,
                                    uint8_t *scratch, size_t scratch_size,
                                    H265ParseResult *out);

#ifdef __cplusplus
}
#endif
