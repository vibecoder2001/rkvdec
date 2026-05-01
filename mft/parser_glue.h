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
