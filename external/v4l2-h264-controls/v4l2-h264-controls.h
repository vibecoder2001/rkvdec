/* v4l2-h264-controls.h — H.264 stateless decoder control structures.
 *
 * Cherry-picked from linux/include/uapi/linux/v4l2-controls.h on master
 * (commit class: kernel >= v6.0; ABI is stable).  Linux uAPI is licensed
 * under GPL-2 with the well-known kernel-syscall-headers exemption that
 * lets userspace consumers redistribute these struct definitions without
 * inheriting GPL-2.  Field names match Linux exactly so the BSP register
 * builder (mpp_rkvdec2_h264.c) will be a near-mechanical port later.
 *
 * Types remap from Linux's __u8/__s32/etc. to <stdint.h> for MSVC.
 *
 * SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause)
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SPS ----------------------------------------------------------- */

#define V4L2_H264_SPS_CONSTRAINT_SET0_FLAG                  0x01
#define V4L2_H264_SPS_CONSTRAINT_SET1_FLAG                  0x02
#define V4L2_H264_SPS_CONSTRAINT_SET2_FLAG                  0x04
#define V4L2_H264_SPS_CONSTRAINT_SET3_FLAG                  0x08
#define V4L2_H264_SPS_CONSTRAINT_SET4_FLAG                  0x10
#define V4L2_H264_SPS_CONSTRAINT_SET5_FLAG                  0x20

#define V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE            0x01
#define V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS  0x02
#define V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO      0x04
#define V4L2_H264_SPS_FLAG_GAPS_IN_FRAME_NUM_VALUE_ALLOWED  0x08
#define V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY                   0x10
#define V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD          0x20
#define V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE             0x40

struct v4l2_ctrl_h264_sps {
    uint8_t  profile_idc;
    uint8_t  constraint_set_flags;
    uint8_t  level_idc;
    uint8_t  seq_parameter_set_id;
    uint8_t  chroma_format_idc;
    uint8_t  bit_depth_luma_minus8;
    uint8_t  bit_depth_chroma_minus8;
    uint8_t  log2_max_frame_num_minus4;
    uint8_t  pic_order_cnt_type;
    uint8_t  log2_max_pic_order_cnt_lsb_minus4;
    uint8_t  max_num_ref_frames;
    uint8_t  num_ref_frames_in_pic_order_cnt_cycle;
    int32_t  offset_for_ref_frame[255];
    int32_t  offset_for_non_ref_pic;
    int32_t  offset_for_top_to_bottom_field;
    uint16_t pic_width_in_mbs_minus1;
    uint16_t pic_height_in_map_units_minus1;
    uint32_t flags;
};

/* ---- PPS ----------------------------------------------------------- */

#define V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE                  0x0001
#define V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT 0x0002
#define V4L2_H264_PPS_FLAG_WEIGHTED_PRED                        0x0004
#define V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT    0x0008
#define V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED               0x0010
#define V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT            0x0020
#define V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE                   0x0040
#define V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT               0x0080

struct v4l2_ctrl_h264_pps {
    uint8_t pic_parameter_set_id;
    uint8_t seq_parameter_set_id;
    uint8_t num_slice_groups_minus1;
    uint8_t num_ref_idx_l0_default_active_minus1;
    uint8_t num_ref_idx_l1_default_active_minus1;
    uint8_t weighted_bipred_idc;
    int8_t  pic_init_qp_minus26;
    int8_t  pic_init_qs_minus26;
    int8_t  chroma_qp_index_offset;
    int8_t  second_chroma_qp_index_offset;
    uint16_t flags;
};

/* ---- Scaling matrix ----------------------------------------------- */

struct v4l2_ctrl_h264_scaling_matrix {
    uint8_t scaling_list_4x4[6][16];
    uint8_t scaling_list_8x8[6][64];
};

/* ---- Predictive weights ------------------------------------------- */

struct v4l2_h264_weight_factors {
    int16_t luma_weight[32];
    int16_t luma_offset[32];
    int16_t chroma_weight[32][2];
    int16_t chroma_offset[32][2];
};

struct v4l2_ctrl_h264_pred_weights {
    uint16_t luma_log2_weight_denom;
    uint16_t chroma_log2_weight_denom;
    struct v4l2_h264_weight_factors weight_factors[2];
};

/* ---- Slice params ------------------------------------------------- */

#define V4L2_H264_SLICE_TYPE_P  0
#define V4L2_H264_SLICE_TYPE_B  1
#define V4L2_H264_SLICE_TYPE_I  2
#define V4L2_H264_SLICE_TYPE_SP 3
#define V4L2_H264_SLICE_TYPE_SI 4

#define V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED 0x01
#define V4L2_H264_SLICE_FLAG_SP_FOR_SWITCH          0x02

#define V4L2_H264_TOP_FIELD_REF                     0x1
#define V4L2_H264_BOTTOM_FIELD_REF                  0x2
#define V4L2_H264_FRAME_REF                         0x3

struct v4l2_h264_reference {
    uint8_t fields;   /* combo of V4L2_H264_*_FIELD_REF */
    uint8_t index;    /* DPB entry index */
};

struct v4l2_ctrl_h264_slice_params {
    uint32_t header_bit_size;
    uint32_t first_mb_in_slice;
    uint8_t  slice_type;
    uint8_t  colour_plane_id;
    uint8_t  redundant_pic_cnt;
    uint8_t  cabac_init_idc;
    int8_t   slice_qp_delta;
    int8_t   slice_qs_delta;
    uint8_t  disable_deblocking_filter_idc;
    int8_t   slice_alpha_c0_offset_div2;
    int8_t   slice_beta_offset_div2;
    uint8_t  num_ref_idx_l0_active_minus1;
    uint8_t  num_ref_idx_l1_active_minus1;
    struct v4l2_h264_reference ref_pic_list0[32];
    struct v4l2_h264_reference ref_pic_list1[32];
    uint32_t flags;
};

/* ---- DPB entry ---------------------------------------------------- */

#define V4L2_H264_DPB_ENTRY_FLAG_VALID         0x01
#define V4L2_H264_DPB_ENTRY_FLAG_ACTIVE        0x02
#define V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM     0x04
#define V4L2_H264_DPB_ENTRY_FLAG_FIELD         0x08

struct v4l2_h264_dpb_entry {
    uint64_t reference_ts;
    uint32_t pic_num;
    uint16_t frame_num;
    uint8_t  fields;
    uint8_t  flags;
    int32_t  top_field_order_cnt;
    int32_t  bottom_field_order_cnt;
};

/* ---- Decode params ------------------------------------------------ */

#define V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC      0x01
#define V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC    0x02
#define V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD 0x04
#define V4L2_H264_DECODE_PARAM_FLAG_PFRAME       0x08
#define V4L2_H264_DECODE_PARAM_FLAG_BFRAME       0x10

struct v4l2_ctrl_h264_decode_params {
    struct v4l2_h264_dpb_entry dpb[16];
    uint16_t nal_ref_idc;
    uint16_t frame_num;
    int32_t  top_field_order_cnt;
    int32_t  bottom_field_order_cnt;
    uint16_t idr_pic_id;
    uint16_t pic_order_cnt_lsb;
    int32_t  delta_pic_order_cnt_bottom;
    int32_t  delta_pic_order_cnt0;
    int32_t  delta_pic_order_cnt1;
    uint32_t dec_ref_pic_marking_bit_size;
    uint32_t pic_order_cnt_bit_size;
    uint32_t slice_group_change_cycle;
    uint32_t flags;
};

#ifdef __cplusplus
}
#endif
