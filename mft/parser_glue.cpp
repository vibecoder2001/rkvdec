/* mft/parser_glue.cpp — minimal H.264 NAL → V4L2 control-struct parser.
 *
 * See parser_glue.h for scope.  This is hand-rolled so we can ship Phase 3b
 * without dragging in FFmpeg first; replace with libavcodec's h264_parser.c
 * later for real-world stream coverage.
 */

#include "parser_glue.h"
#include <string.h>
#include <assert.h>

namespace {

/* ============================================================ Bit reader */

struct BitReader {
    const uint8_t *data;
    size_t         len;     /* in bytes */
    size_t         bit_pos; /* current absolute bit offset */
};

static void br_init(BitReader *br, const uint8_t *data, size_t len) {
    br->data = data;
    br->len  = len;
    br->bit_pos = 0;
}

static int br_eof(const BitReader *br) {
    return br->bit_pos >= br->len * 8;
}

/* Read up to 32 bits.  Returns 0 on EOF. */
static uint32_t br_u(BitReader *br, int n) {
    uint32_t v = 0;
    while (n > 0) {
        if (br->bit_pos >= br->len * 8) return v;
        size_t byte_idx = br->bit_pos >> 3;
        int    bit_idx  = 7 - (br->bit_pos & 7);
        int    bit      = (br->data[byte_idx] >> bit_idx) & 1;
        v = (v << 1) | (uint32_t)bit;
        br->bit_pos++;
        n--;
    }
    return v;
}

static int br_u1(BitReader *br) { return (int)br_u(br, 1); }

/* Unsigned Exp-Golomb. */
static uint32_t br_ue(BitReader *br) {
    int zeros = 0;
    while (!br_eof(br) && br_u1(br) == 0) zeros++;
    if (zeros >= 32) return 0;          /* malformed; clamp */
    uint32_t suffix = br_u(br, zeros);
    return ((1u << zeros) - 1) + suffix;
}

/* Signed Exp-Golomb. */
static int32_t br_se(BitReader *br) {
    uint32_t k = br_ue(br);
    return (k & 1) ? (int32_t)((k + 1) >> 1) : -(int32_t)(k >> 1);
}

/* ============================================================ Annex-B + RBSP */

/* Find next start code (0x000001 or 0x00000001).  Returns offset of first
 * payload byte (after the start code), or len on EOF.  Sets *sc_start to
 * the start of the start code itself. */
static size_t find_start_code(const uint8_t *buf, size_t len, size_t from,
                              size_t *sc_start) {
    for (size_t i = from; i + 2 < len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0) {
            if (buf[i+2] == 1) {
                *sc_start = i;
                return i + 3;
            }
            if (i + 3 < len && buf[i+2] == 0 && buf[i+3] == 1) {
                *sc_start = i;
                return i + 4;
            }
        }
    }
    *sc_start = len;
    return len;
}

/* Copy NAL payload from `src` into `dst`, dropping 0x03 emulation-prevention
 * bytes (turning EBSP into RBSP).  Returns RBSP byte length, or 0 if dst is
 * too small. */
static size_t ebsp_to_rbsp(const uint8_t *src, size_t src_len,
                           uint8_t *dst, size_t dst_cap) {
    size_t o = 0;
    int    zeros = 0;
    for (size_t i = 0; i < src_len; i++) {
        uint8_t b = src[i];
        if (zeros >= 2 && b == 0x03) {
            zeros = 0;
            continue;   /* drop the emulation prevention byte */
        }
        if (o >= dst_cap) return 0;
        dst[o++] = b;
        zeros = (b == 0) ? zeros + 1 : 0;
    }
    return o;
}

/* ============================================================ Scaling list */

/* H.264 7.3.2.1.1.1 — fills the supplied `list` with scaling-list values
 * (default fallback handled by the caller). */
static void parse_scaling_list(BitReader *br, uint8_t *list, int size,
                               int *use_default) {
    int last_scale = 8, next_scale = 8;
    for (int j = 0; j < size; j++) {
        if (next_scale != 0) {
            int delta_scale = br_se(br);
            next_scale = (last_scale + delta_scale + 256) & 0xff;
            if (j == 0 && next_scale == 0) {
                *use_default = 1;
                /* Spec says: stop reading; let caller fill defaults. */
                return;
            }
        }
        list[j] = (next_scale == 0) ? (uint8_t)last_scale : (uint8_t)next_scale;
        last_scale = list[j];
    }
}

/* ============================================================ SPS */

static H264ParseStatus parse_sps(BitReader *br, struct v4l2_ctrl_h264_sps *sps) {
    memset(sps, 0, sizeof(*sps));
    sps->profile_idc          = (uint8_t)br_u(br, 8);
    sps->constraint_set_flags = (uint8_t)br_u(br, 8);
    sps->level_idc            = (uint8_t)br_u(br, 8);
    sps->seq_parameter_set_id = (uint8_t)br_ue(br);

    /* High-profile / fidelity extensions */
    if (sps->profile_idc == 100 || sps->profile_idc == 110 ||
        sps->profile_idc == 122 || sps->profile_idc == 244 ||
        sps->profile_idc ==  44 || sps->profile_idc ==  83 ||
        sps->profile_idc ==  86 || sps->profile_idc == 118 ||
        sps->profile_idc == 128 || sps->profile_idc == 138 ||
        sps->profile_idc == 139 || sps->profile_idc == 134 ||
        sps->profile_idc == 135) {
        sps->chroma_format_idc = (uint8_t)br_ue(br);
        if (sps->chroma_format_idc == 3) {
            int sep = br_u1(br);
            if (sep) sps->flags |= V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE;
        }
        sps->bit_depth_luma_minus8   = (uint8_t)br_ue(br);
        sps->bit_depth_chroma_minus8 = (uint8_t)br_ue(br);
        if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS;
        int seq_scaling_matrix_present = br_u1(br);
        if (seq_scaling_matrix_present) {
            /* Skip scaling lists at SPS level — we re-parse from PPS or
             * use defaults.  Skipping requires reading the bits to keep
             * the bit position correct. */
            int count = (sps->chroma_format_idc == 3) ? 12 : 8;
            for (int i = 0; i < count; i++) {
                if (br_u1(br)) {
                    uint8_t tmp[64];
                    int defaultUsed = 0;
                    parse_scaling_list(br, tmp, (i < 6) ? 16 : 64, &defaultUsed);
                }
            }
        }
    } else {
        sps->chroma_format_idc       = 1;  /* 4:2:0 default */
        sps->bit_depth_luma_minus8   = 0;
        sps->bit_depth_chroma_minus8 = 0;
    }

    sps->log2_max_frame_num_minus4 = (uint8_t)br_ue(br);
    sps->pic_order_cnt_type        = (uint8_t)br_ue(br);
    if (sps->pic_order_cnt_type == 0) {
        sps->log2_max_pic_order_cnt_lsb_minus4 = (uint8_t)br_ue(br);
    } else if (sps->pic_order_cnt_type == 1) {
        if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO;
        sps->offset_for_non_ref_pic            = br_se(br);
        sps->offset_for_top_to_bottom_field    = br_se(br);
        sps->num_ref_frames_in_pic_order_cnt_cycle = (uint8_t)br_ue(br);
        for (int i = 0; i < sps->num_ref_frames_in_pic_order_cnt_cycle &&
                       i < 255; i++) {
            sps->offset_for_ref_frame[i] = br_se(br);
        }
    }
    sps->max_num_ref_frames = (uint8_t)br_ue(br);
    if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_GAPS_IN_FRAME_NUM_VALUE_ALLOWED;
    sps->pic_width_in_mbs_minus1        = (uint16_t)br_ue(br);
    sps->pic_height_in_map_units_minus1 = (uint16_t)br_ue(br);
    int frame_mbs_only = br_u1(br);
    if (frame_mbs_only) sps->flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;
    if (!frame_mbs_only) {
        if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
    }
    if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;
    /* frame_cropping + VUI parsing skipped — not needed by regbuilder. */

    return H264_PARSE_OK;
}

/* ============================================================ PPS */

static H264ParseStatus parse_pps(BitReader *br, struct v4l2_ctrl_h264_pps *pps,
                                 struct v4l2_ctrl_h264_scaling_matrix *sm,
                                 int *has_scaling) {
    memset(pps, 0, sizeof(*pps));
    pps->pic_parameter_set_id = (uint8_t)br_ue(br);
    pps->seq_parameter_set_id = (uint8_t)br_ue(br);
    if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
    if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT;

    pps->num_slice_groups_minus1 = (uint8_t)br_ue(br);
    if (pps->num_slice_groups_minus1 != 0) {
        /* FMO — not supported.  Skipping the slice_group_map_type bits is
         * non-trivial; flag and bail. */
        return H264_PARSE_UNSUPPORTED;
    }
    pps->num_ref_idx_l0_default_active_minus1 = (uint8_t)br_ue(br);
    pps->num_ref_idx_l1_default_active_minus1 = (uint8_t)br_ue(br);
    if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_WEIGHTED_PRED;
    pps->weighted_bipred_idc        = (uint8_t)br_u(br, 2);
    pps->pic_init_qp_minus26        = (int8_t)br_se(br);
    pps->pic_init_qs_minus26        = (int8_t)br_se(br);
    pps->chroma_qp_index_offset     = (int8_t)br_se(br);
    if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;
    if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED;
    if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT;

    /* Optional fidelity-range extension trailer (only present if more bits). */
    if (!br_eof(br)) {
        if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;
        int pic_scaling_matrix_present = br_u1(br);
        if (pic_scaling_matrix_present) {
            pps->flags |= V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
            *has_scaling = 1;
            memset(sm, 0, sizeof(*sm));
            int count_8x8 = (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) ? 2 : 0;
            for (int i = 0; i < 6 + count_8x8; i++) {
                if (br_u1(br)) {
                    int defaultUsed = 0;
                    if (i < 6) {
                        parse_scaling_list(br, sm->scaling_list_4x4[i], 16,
                                           &defaultUsed);
                    } else {
                        parse_scaling_list(br, sm->scaling_list_8x8[i - 6], 64,
                                           &defaultUsed);
                    }
                }
            }
        }
        if (!br_eof(br)) {
            pps->second_chroma_qp_index_offset = (int8_t)br_se(br);
        } else {
            pps->second_chroma_qp_index_offset = pps->chroma_qp_index_offset;
        }
    } else {
        pps->second_chroma_qp_index_offset = pps->chroma_qp_index_offset;
    }
    return H264_PARSE_OK;
}

/* ============================================================ Slice header */

/* H.264 7.3.3 — parses the slice header.  Skips ref-pic list modification
 * and dec_ref_pic_marking after capturing what regbuilder needs. */
static H264ParseStatus parse_slice_header(
    BitReader *br, uint8_t nal_ref_idc, uint8_t nal_unit_type,
    const struct v4l2_ctrl_h264_sps *sps,
    const struct v4l2_ctrl_h264_pps *pps,
    struct v4l2_ctrl_h264_slice_params *slice,
    struct v4l2_ctrl_h264_decode_params *decode)
{
    memset(slice, 0, sizeof(*slice));
    memset(decode, 0, sizeof(*decode));

    decode->nal_ref_idc = nal_ref_idc;

    slice->first_mb_in_slice = br_ue(br);
    uint32_t st = br_ue(br);
    if (st >= 5) st -= 5;             /* the "all slices in pic same type" range */
    if (st > 4) return H264_PARSE_INVALID;
    slice->slice_type = (uint8_t)st;

    /* (pic_parameter_set_id) — caller already chose the PPS.  Read+discard. */
    (void)br_ue(br);

    if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE) {
        slice->colour_plane_id = (uint8_t)br_u(br, 2);
    }

    int log2_max_frame_num = sps->log2_max_frame_num_minus4 + 4;
    decode->frame_num = (uint16_t)br_u(br, log2_max_frame_num);

    int field_pic = 0;
    if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
        field_pic = br_u1(br);
        if (field_pic) {
            decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
            if (br_u1(br))
                decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;
        }
    }

    if (nal_unit_type == 5) {                                  /* IDR */
        decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;
        decode->idr_pic_id = (uint16_t)br_ue(br);
    }

    if (sps->pic_order_cnt_type == 0) {
        int log2_max_poc_lsb = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
        decode->pic_order_cnt_lsb = (uint16_t)br_u(br, log2_max_poc_lsb);
        if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
            !field_pic) {
            decode->delta_pic_order_cnt_bottom = br_se(br);
        }
    } else if (sps->pic_order_cnt_type == 1 &&
               !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
        decode->delta_pic_order_cnt0 = br_se(br);
        if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
            !field_pic) {
            decode->delta_pic_order_cnt1 = br_se(br);
        }
    }

    if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT) {
        slice->redundant_pic_cnt = (uint8_t)br_ue(br);
    }

    /* P / SP / B specifics — for IDR-only streams these aren't reached. */
    if (st == V4L2_H264_SLICE_TYPE_B) {
        if (br_u1(br)) slice->flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;
    }
    if (st == V4L2_H264_SLICE_TYPE_P || st == V4L2_H264_SLICE_TYPE_SP ||
        st == V4L2_H264_SLICE_TYPE_B) {
        int override = br_u1(br);
        if (override) {
            slice->num_ref_idx_l0_active_minus1 = (uint8_t)br_ue(br);
            if (st == V4L2_H264_SLICE_TYPE_B)
                slice->num_ref_idx_l1_active_minus1 = (uint8_t)br_ue(br);
        } else {
            slice->num_ref_idx_l0_active_minus1 =
                pps->num_ref_idx_l0_default_active_minus1;
            slice->num_ref_idx_l1_active_minus1 =
                pps->num_ref_idx_l1_default_active_minus1;
        }
        /* ref_pic_list_modification — skipping (zero-only for IDR-only). */
    }

    /* pred_weight_table, dec_ref_pic_marking, cabac_init_idc — not yet
     * exercised by IDR-only test stream; leave defaults zeroed. */

    return H264_PARSE_OK;
}

} /* anon namespace */

/* ============================================================ Public entry */

extern "C"
H264ParseStatus H264ParseAccessUnit(const uint8_t *buf, size_t len,
                                    uint8_t *scratch, size_t scratch_size,
                                    H264ParseResult *out)
{
    if (!buf || !out || !scratch) return H264_PARSE_INVALID;
    memset(out, 0, sizeof(*out));

    size_t scratch_off = 0;
    size_t pos = 0;
    while (pos < len) {
        size_t sc_start = 0;
        size_t nal_start = find_start_code(buf, len, pos, &sc_start);
        if (nal_start >= len) break;

        /* Find next start code to bound this NAL. */
        size_t next_sc = 0;
        find_start_code(buf, len, nal_start, &next_sc);
        size_t nal_end = (next_sc > nal_start) ? next_sc : len;

        /* Ensure scratch room. */
        size_t this_size = nal_end - nal_start;
        if (scratch_off + this_size > scratch_size) return H264_PARSE_INVALID;
        uint8_t *rbsp = scratch + scratch_off;
        size_t rbsp_len = ebsp_to_rbsp(buf + nal_start, this_size,
                                       rbsp, scratch_size - scratch_off);
        if (rbsp_len == 0) return H264_PARSE_INVALID;
        scratch_off += rbsp_len;

        uint8_t nal_header   = rbsp[0];
        uint8_t nal_ref_idc  = (nal_header >> 5) & 0x3;
        uint8_t nal_unit_type =  nal_header       & 0x1f;

        BitReader br;
        br_init(&br, rbsp + 1, rbsp_len - 1);

        switch (nal_unit_type) {
        case 7: {
            H264ParseStatus s = parse_sps(&br, &out->sps);
            if (s != H264_PARSE_OK) return s;
            out->has_sps = 1;
            break;
        }
        case 8: {
            int has_scaling = 0;
            H264ParseStatus s = parse_pps(&br, &out->pps, &out->scaling_matrix,
                                          &has_scaling);
            if (s != H264_PARSE_OK) return s;
            out->has_pps = 1;
            if (has_scaling) out->has_scaling_matrix = 1;
            break;
        }
        case 1: case 5: {
            if (!out->has_sps || !out->has_pps) return H264_PARSE_INVALID;
            H264ParseStatus s = parse_slice_header(&br, nal_ref_idc,
                                                   nal_unit_type, &out->sps,
                                                   &out->pps, &out->slice,
                                                   &out->decode);
            if (s != H264_PARSE_OK) return s;
            out->has_slice = 1;
            /* Bit position rounded up to the next byte gives us the start
             * of slice data within the RBSP. */
            size_t header_bits = br.bit_pos;
            out->slice.header_bit_size = (uint32_t)header_bits;
            size_t data_off = (header_bits + 7) >> 3;
            if (data_off > rbsp_len - 1) return H264_PARSE_INVALID;
            out->slice_data      = rbsp + 1 + data_off;
            out->slice_data_size = (rbsp_len - 1) - data_off;
            break;
        }
        default:
            /* Skip SEI / AUD / filler / unknown. */
            break;
        }

        pos = nal_end;
    }

    if (!out->has_slice) {
        return out->has_sps && out->has_pps ? H264_PARSE_NEED_MORE
                                            : H264_PARSE_INVALID;
    }
    return H264_PARSE_OK;
}
