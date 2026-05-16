// mft/vp9_types.h
#pragma once
#include <cstdint>
#include <array>

namespace vp9 {

constexpr int kNumRefFrames = 8;
constexpr int kRefsPerFrame = 3;        // last, golden, altref
constexpr int kMaxSegments  = 8;
constexpr int kSegLvlMax    = 4;        // ALT_Q, ALT_LF, REF_FRAME, SKIP
constexpr int kMaxModeLfDeltas = 2;
constexpr int kMaxRefLfDeltas  = 4;

struct SegmentParams {
    uint8_t enabled;
    uint8_t update_map;
    uint8_t temporal_update;
    uint8_t update_data;
    uint8_t abs_delta;
    uint8_t tree_probs[7];
    uint8_t pred_probs[3];
    int16_t feature_data[kMaxSegments][kSegLvlMax]; // ALT_Q, ALT_LF (signed); REF, SKIP (binary)
    uint8_t feature_mask[kMaxSegments];             // bit i = feature i enabled
};

struct LoopFilterParams {
    uint8_t level;
    uint8_t sharpness;
    uint8_t mode_ref_delta_enabled;
    uint8_t mode_ref_delta_update;
    int8_t  ref_deltas[kMaxRefLfDeltas];   // INTRA, LAST, GOLDEN, ALTREF
    int8_t  mode_deltas[kMaxModeLfDeltas]; // ZERO, NEW
};

struct RefFrameDesc {
    uint8_t index;          // ref_frame_map[] slot (0..7); 0xFF = invalid
    uint8_t sign_bias;
};

struct PicParams {
    uint8_t  profile;
    uint8_t  bit_depth;          // 8 for Profile 0
    uint8_t  color_space;
    uint8_t  color_range;
    uint8_t  subsampling_x, subsampling_y;

    uint8_t  frame_type;         // 0 = KEY, 1 = INTER
    uint8_t  show_frame;
    uint8_t  show_existing_frame;
    uint8_t  show_existing_frame_idx;   // valid only if show_existing_frame
    uint8_t  error_resilient_mode;
    uint8_t  intra_only;
    uint8_t  allow_high_precision_mv;
    uint8_t  refresh_frame_context;
    uint8_t  frame_parallel_decoding_mode;
    uint8_t  reset_frame_context;
    uint8_t  frame_context_idx;         // 0..3

    uint32_t width, height;
    uint32_t render_width, render_height;

    // Refs: ref_frame_map[8] holds DPB slot indices; frame_refs[3] is the
    // per-frame {last, golden, altref} pick by name. Values 0..7 index
    // into ref_frame_map; 0xFF = unused.
    RefFrameDesc ref_frame_map[kNumRefFrames];
    RefFrameDesc frame_refs[kRefsPerFrame];

    uint8_t  refresh_frame_flags;       // bit i = update DPB slot i with this frame

    uint8_t  interp_filter;             // 0..3 fixed, 4 = SWITCHABLE
    uint8_t  txmode;                    // 0..3 fixed, 4 = TX_MODE_SELECT
    uint8_t  reference_mode;            // 0=single, 1=compound, 2=hybrid

    LoopFilterParams lf;
    SegmentParams    seg;

    uint8_t  base_qindex;
    int8_t   y_dc_delta_q;
    int8_t   uv_dc_delta_q;
    int8_t   uv_ac_delta_q;
    uint8_t  lossless;

    uint8_t  tile_cols_log2;
    uint8_t  tile_rows_log2;

    uint32_t header_size;               // compressed header size in bytes
    uint32_t uncompressed_header_size;  // for bitstream offset to compressed header
    uint32_t frame_size;                // total payload (incl. headers)
};

struct ProbUpdates {
    // Sparse — only fields the compressed header actually wrote.
    // Each "_present" flag means the regbuilder should patch the byte
    // offset in the prob buffer; "_value" carries the new value.
    uint8_t tx_mode_present;
    uint8_t tx_mode;

    // TX-size probability updates (§6.3.2). tx_probs_present is set when
    // tx_mode == TX_MODE_SELECT and at least one set was decoded.
    uint8_t tx_probs_present;
    uint8_t tx_size_8x8[2][1];    // [ctx][0]
    uint8_t tx_size_16x16[2][2];  // [ctx][0..1]
    uint8_t tx_size_32x32[2][3];  // [ctx][0..2]

    // Coef probs: 4 tx sizes × 2 (intra/inter) × 2 (Y/UV) × 6 × 6 × 3.
    // Store as a packed bitmap of changed entries + values; regbuilder
    // walks the bitmap.
    uint8_t coef_present[4]; // per tx size
    // Compact representation; populated by parser, consumed by
    // regbuilder. See vp9_parser.cpp for exact layout.
    uint8_t coef_values[4 /*tx*/][2 /*ref*/][2 /*plane*/][6][6][3];
    uint8_t coef_changed[4][2][2][6][6][3];

    // Parallel "_flag" arrays mirror each prob group; the parser sets
    // a flag byte to 1 only when *that specific context's* prob update
    // bit fired in the compressed header.  The hardware reads these
    // 1-bit flags from the prob buffer to decide which deltas to
    // apply, so we MUST track per-context (not just per-section).

    uint8_t skip_present;
    uint8_t skip[3];
    uint8_t skip_flag[3];

    uint8_t tx_size_8x8_flag[2][1];
    uint8_t tx_size_16x16_flag[2][2];
    uint8_t tx_size_32x32_flag[2][3];

    uint8_t inter_mode_present;
    uint8_t inter_mode[7][3];
    uint8_t inter_mode_flag[7][3];

    uint8_t interp_filter_present;
    uint8_t interp_filter[4][2];
    uint8_t interp_filter_flag[4][2];

    uint8_t is_inter_present;
    uint8_t is_inter[4];
    uint8_t is_inter_flag[4];

    uint8_t ref_mode_present;     // single vs compound vs hybrid
    uint8_t reference_mode;       // when set, was selected by compressed header
    uint8_t comp_mode_present;
    uint8_t comp_mode[5];
    uint8_t comp_mode_flag[5];
    uint8_t single_ref_present;
    uint8_t single_ref[5][2];
    uint8_t single_ref_flag[5][2];
    uint8_t comp_ref_present;
    uint8_t comp_ref[5];
    uint8_t comp_ref_flag[5];

    uint8_t y_mode_present;
    uint8_t y_mode[4][9];
    uint8_t y_mode_flag[4][9];

    uint8_t partition_present;
    uint8_t partition[16][3];
    uint8_t partition_flag[16][3];

    uint8_t mv_present;
    uint8_t mv_joints[3];
    uint8_t mv_joints_flag[3];
    uint8_t mv_sign[2];
    uint8_t mv_sign_flag[2];
    uint8_t mv_classes[2][10];
    uint8_t mv_classes_flag[2][10];
    uint8_t mv_class0[2];
    uint8_t mv_class0_flag[2];
    uint8_t mv_bits[2][10];
    uint8_t mv_bits_flag[2][10];
    uint8_t mv_class0_fp[2][2][3];
    uint8_t mv_class0_fp_flag[2][2][3];
    uint8_t mv_fp[2][3];
    uint8_t mv_fp_flag[2][3];
    uint8_t mv_class0_hp[2];
    uint8_t mv_class0_hp_flag[2];
    uint8_t mv_hp[2];
    uint8_t mv_hp_flag[2];
};

} // namespace vp9
