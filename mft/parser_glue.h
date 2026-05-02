/* mft/parser_glue.h — minimal H.264 NAL → V4L2 control-struct parser.
 *
 * Phase 3b "first I-frame decode" target. Hand-rolled rather than
 * vendoring FFmpeg, so we can ship the rest of the pipeline (regbuilder,
 * DPB, hardware kick) faster. When real-world streams hit edge cases we
 * miss, swap in libavcodec's parser per the original Phase 3b plan.
 *
 * Scope:
 *   - Annex-B NAL framing + RBSP emulation-prevention unescape
 *   - Exp-Golomb bit reader
 *   - SPS / PPS / slice-header / scaling-list parse for Baseline +
 *     Main + (constrained) High profile
 *   - Output is filled `v4l2_ctrl_h264_*` structs the regbuilder
 *     consumes
 *
 * Out of scope:
 *   - SEI parsing (skipped — not needed for decode)
 *   - CABAC entropy (not needed; rkvdec hardware does CABAC itself)
 *   - Long-term ref management (single I-frame stream doesn't use it)
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#include "../external/v4l2-h264-controls/v4l2-h264-controls.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    H264_PARSE_OK            = 0,
    H264_PARSE_NEED_MORE     = 1,  /* not enough data */
    H264_PARSE_INVALID       = 2,  /* malformed bitstream */
    H264_PARSE_UNSUPPORTED   = 3,  /* feature outside our scope */
} H264ParseStatus;

/* H.264 dec_ref_pic_marking memory_management_control_operation (spec 7.4.3.3).
 * One per MMCO op in the slice header.  op == 0 marks end-of-list (never
 * stored).  Only the fields relevant to a given op are populated. */
typedef struct H264Mmco {
    uint8_t  op;                                 /* 1..6 */
    uint32_t difference_of_pic_nums_minus1;      /* ops 1, 3 */
    uint32_t long_term_pic_num;                  /* op 2 */
    uint32_t long_term_frame_idx;                /* ops 3, 6 */
    uint32_t max_long_term_frame_idx_plus1;      /* op 4 */
} H264Mmco;

#define H264_MAX_MMCO_OPS 16

/* H.264 ref_pic_list_modification op (spec 7.3.3.1, 8.2.4.3). */
typedef struct H264RplmOp {
    uint8_t  op;     /* 0=ST backward, 1=ST forward, 2=LT */
    uint32_t value;  /* abs_diff_pic_num_minus1 (op 0/1) OR long_term_pic_num (op 2) */
} H264RplmOp;

/* The result of parsing one access unit's worth of NAL units.  Caller
 * zero-inits before each call.  has_* flags indicate which structs the
 * parser populated. */
typedef struct H264ParseResult {
    /* Parsed control structs (only the has_* ones are valid). */
    struct v4l2_ctrl_h264_sps              sps;
    struct v4l2_ctrl_h264_pps              pps;
    struct v4l2_ctrl_h264_scaling_matrix   scaling_matrix;
    struct v4l2_ctrl_h264_pred_weights     pred_weights;
    struct v4l2_ctrl_h264_slice_params     slice;
    struct v4l2_ctrl_h264_decode_params    decode;

    uint8_t has_sps;
    uint8_t has_pps;
    uint8_t has_scaling_matrix;
    uint8_t has_pred_weights;
    uint8_t has_slice;

    /* Slice data (post emulation-prevention unescape, RBSP form, after
     * the slice header).  Pointer is into a caller-owned scratch buffer
     * supplied via H264ParseAccessUnit; valid until the next parse. */
    const uint8_t *slice_data;
    size_t         slice_data_size;

    /* Persistent POC-type-0 state (spec 8.2.1.1).  Updated each call.
     * Reset on IDR. */
    int32_t        prev_pic_order_cnt_msb;
    int32_t        prev_pic_order_cnt_lsb;
    /* Persistent POC-type-1 state (8.2.1.2): frame_num_offset. */
    int32_t        prev_frame_num;
    int32_t        prev_frame_num_offset;

    /* dec_ref_pic_marking surface (spec 7.3.3.1, 8.2.5). */
    uint8_t        adaptive_ref_pic_marking_mode_flag; /* 1 if MMCO list present */
    uint8_t        idr_no_output_of_prior_pics_flag;   /* IDR only */
    uint8_t        idr_long_term_reference_flag;       /* IDR only — store as LT idx 0 */
    uint8_t        n_mmco;                             /* count of valid mmco[] entries */
    H264Mmco       mmco[H264_MAX_MMCO_OPS];

    /* ref_pic_list_modification surface (spec 7.3.3.1, 8.2.4.3).  The
     * rkvdec H.264 path consumes a pre-built RefPicList[3][32] from
     * rps_base (it doesn't re-derive from slice-header bits like HEVC
     * HW_RPS does), so any per-slice list modification must be applied
     * by the DPB layer before regbuilder pack. */
    uint8_t        ref_pic_list_modification_flag_l0;
    uint8_t        ref_pic_list_modification_flag_l1;
    uint8_t        n_rplm_l0;
    uint8_t        n_rplm_l1;
    /* op encoding: 0 = short-term backward, 1 = short-term forward,
     * 2 = long-term, 3 = end-of-list (never stored). */
    H264RplmOp     rplm_l0[32];
    H264RplmOp     rplm_l1[32];
} H264ParseResult;

/* Parse one access unit (typically an IDR-only buffer for the first
 * decode test).  `buf` is Annex-B-framed (start codes 0x000001 /
 * 0x00000001).  The parser uses `scratch` of `scratch_size` bytes for
 * RBSP unescape — pass at least 2x len.
 *
 * Result fields are valid until the next H264ParseAccessUnit call. */
H264ParseStatus H264ParseAccessUnit(const uint8_t *buf, size_t len,
                                    uint8_t *scratch, size_t scratch_size,
                                    H264ParseResult *out);

#ifdef __cplusplus
}
#endif
