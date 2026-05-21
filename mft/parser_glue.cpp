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
    /* Sticky error flag — set by any read that detected an EOF, an
     * Exp-Golomb leading-zero run >= 32, or other malformed input.
     * Callers should check br_failed(br) after a parse routine and
     * reject the unit; without this, br_ue's silent clamp-to-0 lets
     * the parser sail past malformed inputs with a poisoned-zero
     * value that the regbuilder then trusts.
     * See [[parser_eg_error_propagation]]. */
    bool           failed;
};

static void br_init(BitReader *br, const uint8_t *data, size_t len) {
    br->data = data;
    br->len  = len;
    br->bit_pos = 0;
    br->failed = false;
}

static inline bool br_failed(const BitReader *br) { return br->failed; }

static int br_eof(const BitReader *br) {
    return br->bit_pos >= br->len * 8;
}

/* H.264 more_rbsp_data() per spec 7.2.  Returns true if there is data
 * remaining BEFORE the rbsp_trailing_bits (= a single '1' followed by
 * zero-padding to a byte boundary).  Used to detect the optional
 * fidelity-range trailer in PPS — which is missing for Baseline/Main
 * IDRs but `br_eof` still returns false (the trailing bits are still
 * "data" from its perspective).  Without this check, the parser reads
 * the rbsp_stop_one_bit as transform_8x8_mode_flag=1 and corrupts the
 * decoded output. */
static int br_more_rbsp_data(const BitReader *br) {
    if (br_eof(br)) return 0;
    /* If next bit is 0, we have data: rbsp_trailing always starts with 1. */
    size_t pos = br->bit_pos;
    int next_bit = (br->data[pos >> 3] >> (7 - (pos & 7))) & 1;
    if (next_bit == 0) return 1;
    /* Next bit is 1: check if all remaining bits are zero (= trailer). */
    pos++;
    while (pos < br->len * 8) {
        if ((br->data[pos >> 3] >> (7 - (pos & 7))) & 1) return 1;
        pos++;
    }
    return 0;
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

/* Unsigned Exp-Golomb.  On malformed input (EOF mid-prefix or
 * leading-zero run >= 32), latches br->failed and returns 0 — callers
 * that have already checked failed don't re-set on every return. */
static uint32_t br_ue(BitReader *br) {
    int zeros = 0;
    while (!br_eof(br) && br_u1(br) == 0) zeros++;
    if (br_eof(br) && zeros > 0) { br->failed = true; return 0; }
    /* `zeros > 31` is the safe-by-construction form: `1ull << 32` is UB
     * for `1u <<`, and `1u << 31` produces 0x80000000 then the `-1`
     * wraps — using `1ull << zeros` keeps the math correct up to the
     * full 31-bit suffix.  Review parser Medium #7. */
    if (zeros > 31) { br->failed = true; return 0; }
    uint32_t suffix = br_u(br, zeros);
    return (uint32_t)(((uint64_t)1 << zeros) - 1) + suffix;
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
    /* Bound check `i + 3 <= len` matches au_iter.cpp's `i + 3 <= len`
     * form — `i + 2 < len` skipped the last legal 3-byte start code at
     * offset len-3 (review parser Medium #5). */
    for (size_t i = from; i + 3 <= len; i++) {
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

/* H.264 zig-zag scan tables (spec 8.5.6 / Figure 7-3).  Element j is
 * the raster position of the value at bitstream index j.  Used to
 * convert scaling lists from the bitstream's zig-zag order to V4L2's
 * raster order (also what our codec table layout expects). */
static const uint8_t kH264ZigZag4x4[16] = {
     0,  1,  4,  8,  5,  2,  3,  6,
     9, 12, 13, 10,  7, 11, 14, 15,
};
static const uint8_t kH264ZigZag8x8[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
};

/* H.264 Tables 7-3, 7-4 default scaling lists (raster order). */
static const uint8_t kH264Default4x4Intra[16] = {
     6, 13, 13, 20, 20, 20, 28, 28,
    28, 28, 32, 32, 32, 36, 36, 42,
};
static const uint8_t kH264Default4x4Inter[16] = {
    10, 14, 14, 20, 20, 20, 24, 24,
    24, 24, 27, 27, 27, 30, 30, 34,
};
static const uint8_t kH264Default8x8Intra[64] = {
     6, 10, 13, 16, 18, 23, 25, 27,
    10, 11, 16, 18, 23, 25, 27, 29,
    13, 16, 18, 23, 25, 27, 29, 31,
    16, 18, 23, 25, 27, 29, 31, 33,
    18, 23, 25, 27, 29, 31, 33, 36,
    23, 25, 27, 29, 31, 33, 36, 38,
    25, 27, 29, 31, 33, 36, 38, 40,
    27, 29, 31, 33, 36, 38, 40, 42,
};
static const uint8_t kH264Default8x8Inter[64] = {
     9, 13, 15, 17, 19, 21, 22, 24,
    13, 13, 17, 19, 21, 22, 24, 25,
    15, 17, 19, 21, 22, 24, 25, 27,
    17, 19, 21, 22, 24, 25, 27, 28,
    19, 21, 22, 24, 25, 27, 28, 30,
    21, 22, 24, 25, 27, 28, 30, 32,
    22, 24, 25, 27, 28, 30, 32, 33,
    24, 25, 27, 28, 30, 32, 33, 35,
};

/* H.264 7.3.2.1.1.1 — fills the supplied `list` with scaling-list values
 * (default fallback handled by the caller).  Bitstream values arrive in
 * zig-zag scan order; V4L2 (and our packed-table layout) expects raster
 * order, so each value is written to list[ZigZag[j]]. */
static void parse_scaling_list(BitReader *br, uint8_t *list, int size,
                               int *use_default) {
    const uint8_t *zz = (size == 16) ? kH264ZigZag4x4 : kH264ZigZag8x8;
    int last_scale = 8, next_scale = 8;
    int last_value = 8;  /* track in scan order for the "next_scale==0" rule */
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
        int v = (next_scale == 0) ? last_value : next_scale;
        list[zz[j]] = (uint8_t)v;
        last_scale = (next_scale == 0) ? last_scale : next_scale;
        last_value = v;
    }
}

/* ============================================================ SPS */

/* Parse VUI hrd_parameters (E.1.2) — we don't need any of its fields,
 * just need to advance past them to reach later VUI fields. */
static void parse_hrd_parameters(BitReader *br) {
    uint32_t cpb_cnt_minus1 = br_ue(br);
    if (cpb_cnt_minus1 > 31) cpb_cnt_minus1 = 31;  /* spec cap */
    (void)br_u(br, 4);                              /* bit_rate_scale */
    (void)br_u(br, 4);                              /* cpb_size_scale */
    for (uint32_t i = 0; i <= cpb_cnt_minus1; i++) {
        (void)br_ue(br);                            /* bit_rate_value_minus1[i] */
        (void)br_ue(br);                            /* cpb_size_value_minus1[i] */
        (void)br_u1(br);                            /* cbr_flag[i] */
    }
    (void)br_u(br, 5);                              /* initial_cpb_removal_delay_length_minus1 */
    (void)br_u(br, 5);                              /* cpb_removal_delay_length_minus1 */
    (void)br_u(br, 5);                              /* dpb_output_delay_length_minus1 */
    (void)br_u(br, 5);                              /* time_offset_length */
}

/* Parse VUI parameters (E.1.1).  We're only after
 * `bitstream_restriction.max_num_reorder_frames` — everything else is
 * walked with bit-precise field skips so the bit position lands at the
 * restriction block correctly.  Output goes to *out_*. */
static void parse_vui_parameters(BitReader *br,
                                 uint8_t *out_has_reorder,
                                 uint8_t *out_max_reorder,
                                 H264ParseResult *out_colour = nullptr)
{
    if (out_has_reorder) *out_has_reorder = 0;
    if (out_max_reorder) *out_max_reorder = 0;

    if (br_u1(br)) {                                /* aspect_ratio_info_present */
        uint32_t idc = br_u(br, 8);                 /* aspect_ratio_idc */
        if (idc == 255) {
            (void)br_u(br, 16);                     /* sar_width */
            (void)br_u(br, 16);                     /* sar_height */
        }
    }
    if (br_u1(br)) {                                /* overscan_info_present */
        (void)br_u1(br);                            /* overscan_appropriate_flag */
    }
    if (br_u1(br)) {                                /* video_signal_type_present */
        if (out_colour) out_colour->has_vui_colour = 1;
        (void)br_u(br, 3);                          /* video_format */
        uint8_t full_range = (uint8_t)br_u1(br);    /* video_full_range_flag */
        if (out_colour) out_colour->vui_full_range_flag = full_range;
        if (br_u1(br)) {                            /* colour_description_present */
            uint8_t cp = (uint8_t)br_u(br, 8);
            uint8_t tc = (uint8_t)br_u(br, 8);
            uint8_t mc = (uint8_t)br_u(br, 8);
            if (out_colour) {
                out_colour->has_vui_colour_desc       = 1;
                out_colour->vui_colour_primaries      = cp;
                out_colour->vui_transfer_characteristics = tc;
                out_colour->vui_matrix_coefficients   = mc;
            }
        }
    }
    if (br_u1(br)) {                                /* chroma_loc_info_present */
        (void)br_ue(br);                            /* chroma_sample_loc_type_top_field */
        (void)br_ue(br);                            /* chroma_sample_loc_type_bottom_field */
    }
    if (br_u1(br)) {                                /* timing_info_present */
        (void)br_u(br, 32);                         /* num_units_in_tick */
        (void)br_u(br, 32);                         /* time_scale */
        (void)br_u1(br);                            /* fixed_frame_rate_flag */
    }
    int nal_hrd = br_u1(br);                        /* nal_hrd_parameters_present */
    if (nal_hrd) parse_hrd_parameters(br);
    int vcl_hrd = br_u1(br);                        /* vcl_hrd_parameters_present */
    if (vcl_hrd) parse_hrd_parameters(br);
    if (nal_hrd || vcl_hrd) {
        (void)br_u1(br);                            /* low_delay_hrd_flag */
    }
    (void)br_u1(br);                                /* pic_struct_present_flag */
    if (br_u1(br)) {                                /* bitstream_restriction_flag */
        (void)br_u1(br);                            /* motion_vectors_over_pic_boundaries_flag */
        (void)br_ue(br);                            /* max_bytes_per_pic_denom */
        (void)br_ue(br);                            /* max_bits_per_mb_denom */
        (void)br_ue(br);                            /* log2_max_mv_length_horizontal */
        (void)br_ue(br);                            /* log2_max_mv_length_vertical */
        uint32_t mr = br_ue(br);                    /* max_num_reorder_frames ← target */
        (void)br_ue(br);                            /* max_dec_frame_buffering */
        if (mr > 16) mr = 16;                       /* spec cap is implementation but
                                                     * 16 is the largest H.264 DPB. */
        if (out_has_reorder) *out_has_reorder = 1;
        if (out_max_reorder) *out_max_reorder = (uint8_t)mr;
    }
}

/* parse_sps signature:
 *   sps                — V4L2 control fields (mandatory)
 *   out_has_reorder    — set to 1 if VUI bitstream_restriction parsed
 *   out_max_reorder    — VUI max_num_reorder_frames (when has_reorder=1)
 *   out_sm             — populated with SPS-level scaling lists when the
 *                        stream sets seq_scaling_matrix_present_flag.
 *                        Lists not explicitly provided by the bitstream
 *                        are left zero (caller's job to flat-16 fill).
 *   out_has_sps_sm     — set to 1 iff seq_scaling_matrix_present_flag was 1
 *   any of out_*       — may be nullptr if caller doesn't need that
 *                        signal (full SPS still parses correctly). */
static H264ParseStatus parse_sps(BitReader *br, struct v4l2_ctrl_h264_sps *sps,
                                 uint8_t *out_has_reorder = nullptr,
                                 uint8_t *out_max_reorder = nullptr,
                                 struct v4l2_ctrl_h264_scaling_matrix *out_sm = nullptr,
                                 uint8_t *out_has_sps_sm = nullptr,
                                 H264ParseResult *out_vui_colour = nullptr) {
    if (out_has_reorder) *out_has_reorder = 0;
    if (out_max_reorder) *out_max_reorder = 0;
    if (out_has_sps_sm)  *out_has_sps_sm  = 0;
    if (out_sm)          memset(out_sm, 0, sizeof(*out_sm));
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
            if (out_has_sps_sm) *out_has_sps_sm = 1;
            int count = (sps->chroma_format_idc == 3) ? 12 : 8;
            for (int i = 0; i < count; i++) {
                if (br_u1(br)) {
                    int defaultUsed = 0;
                    if (out_sm && i < 6) {
                        parse_scaling_list(br, out_sm->scaling_list_4x4[i],
                                           16, &defaultUsed);
                    } else if (out_sm && i < 8) {
                        parse_scaling_list(br, out_sm->scaling_list_8x8[i - 6],
                                           64, &defaultUsed);
                    } else {
                        /* chroma_format_idc==3 8x8 lists (i=8..11) — V4L2
                         * scaling_matrix has slots for them; populate when
                         * caller supplies the matrix, else skip. */
                        uint8_t tmp[64];
                        if (out_sm) {
                            parse_scaling_list(br,
                                               out_sm->scaling_list_8x8[i - 6],
                                               64, &defaultUsed);
                        } else {
                            parse_scaling_list(br, tmp, 64, &defaultUsed);
                        }
                    }
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
    {
        /* Bound width/height before silent uint16 truncation.  Without
         * this gate, an SPS with `pic_width_in_mbs_minus1 = 0x10000`
         * truncates to 0, the regbuilder computes a tiny stride yet the
         * codec is told the actual stream is bigger, and the kernel's
         * iova-substitution writes attacker-controlled MMIO values.
         * Reject any resolution beyond hardware capability (8192x8192;
         * mbs are 16 luma samples → 512x512 mbs).  See
         * [[critical_h264_dim_bound]]. */
        uint32_t w_mbs = br_ue(br);
        uint32_t h_mbs = br_ue(br);
        constexpr uint32_t kMaxMbs = 512u - 1u;
        if (w_mbs > kMaxMbs || h_mbs > kMaxMbs) return H264_PARSE_INVALID;
        sps->pic_width_in_mbs_minus1        = (uint16_t)w_mbs;
        sps->pic_height_in_map_units_minus1 = (uint16_t)h_mbs;
    }
    int frame_mbs_only = br_u1(br);
    if (frame_mbs_only) sps->flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;
    if (!frame_mbs_only) {
        if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
    }
    if (br_u1(br)) sps->flags |= V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;

    /* frame_cropping_flag — skip the crop-offset values when present.
     * Cropping doesn't affect regbuilder; we just walk the bits to keep
     * the bit position aligned for downstream VUI parsing. */
    if (br_u1(br)) {
        (void)br_ue(br);                            /* frame_crop_left_offset */
        (void)br_ue(br);                            /* frame_crop_right_offset */
        (void)br_ue(br);                            /* frame_crop_top_offset */
        (void)br_ue(br);                            /* frame_crop_bottom_offset */
    }

    /* VUI parameters — only field the engine actually needs is
     * `bitstream_restriction.max_num_reorder_frames`, which is the
     * proper display-reorder bound used by DecodeEngine_OnDecodeComplete
     * to size the reorder window.  When the VUI is absent (or doesn't
     * include bitstream_restriction), the caller falls back to a
     * conservative bound based on max_num_ref_frames. */
    if (br_u1(br)) {                                /* vui_parameters_present_flag */
        parse_vui_parameters(br, out_has_reorder, out_max_reorder,
                             out_vui_colour);
    }

    /* Surface any latched malformed-bitstream condition from br_ue's
     * silent-clamp path so the caller doesn't trust a poisoned-zero
     * field that flows into the regbuilder. */
    if (br_failed(br)) return H264_PARSE_INVALID;
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

    /* Optional fidelity-range extension trailer (only present if more
     * data exists BEFORE the rbsp_trailing_bits). */
    if (br_more_rbsp_data(br)) {
        if (br_u1(br)) pps->flags |= V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;
        int pic_scaling_matrix_present = br_u1(br);
        if (pic_scaling_matrix_present) {
            pps->flags |= V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
            *has_scaling = 1;
            memset(sm, 0, sizeof(*sm));
            int count_8x8 = (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) ? 2 : 0;
            int n_lists   = 6 + count_8x8;
            uint8_t present[8]    = {0};
            uint8_t use_default[8]= {0};
            for (int i = 0; i < n_lists; i++) {
                present[i] = (uint8_t)br_u1(br);
                if (present[i]) {
                    int du = 0;
                    if (i < 6) {
                        parse_scaling_list(br, sm->scaling_list_4x4[i], 16, &du);
                    } else {
                        parse_scaling_list(br, sm->scaling_list_8x8[i - 6], 64, &du);
                    }
                    use_default[i] = (uint8_t)du;
                }
            }
            /* Resolve missing / use_default lists per H.264 7.4.2.2
             * Table 7-2 fall-back Set A (the no-SPS-scaling case — the
             * SPS-inheritance case isn't yet plumbed; we'd need the
             * has_sps_scaling_matrix flag and the SPS lists here).
             * Set A:
             *   i=0,3,6,7 → default (Intra4, Inter4, Intra8, Inter8)
             *   i=1       → list 0
             *   i=2       → list 1
             *   i=4       → list 3
             *   i=5       → list 4
             * Without this, missing chroma lists (i=2,5) decode with
             * flat-16 instead of the encoder's Cb list, shifting Cr
             * dequant and tinting reds/blues. */
            auto fill_default = [&](int i) {
                if (i == 0)      memcpy(sm->scaling_list_4x4[0], kH264Default4x4Intra, 16);
                else if (i == 3) memcpy(sm->scaling_list_4x4[3], kH264Default4x4Inter, 16);
                else if (i == 6) memcpy(sm->scaling_list_8x8[0], kH264Default8x8Intra, 64);
                else if (i == 7) memcpy(sm->scaling_list_8x8[1], kH264Default8x8Inter, 64);
            };
            auto inherit_4x4 = [&](int dst, int src) {
                memcpy(sm->scaling_list_4x4[dst], sm->scaling_list_4x4[src], 16);
            };
            for (int i = 0; i < n_lists; i++) {
                if (present[i] && !use_default[i]) continue;
                /* Either pic_scaling_list_present_flag[i]==0 (Set A
                 * fall-back) or use_default_scaling_matrix_flag[i]==1
                 * (caller signalled "use the table"). Same outcome. */
                switch (i) {
                case 0: fill_default(0); break;
                case 1: inherit_4x4(1, 0); break;
                case 2: inherit_4x4(2, 1); break;
                case 3: fill_default(3); break;
                case 4: inherit_4x4(4, 3); break;
                case 5: inherit_4x4(5, 4); break;
                case 6: fill_default(6); break;
                case 7: fill_default(7); break;
                default: break;
                }
            }
        }
        if (br_more_rbsp_data(br)) {
            pps->second_chroma_qp_index_offset = (int8_t)br_se(br);
        } else {
            pps->second_chroma_qp_index_offset = pps->chroma_qp_index_offset;
        }
    } else {
        pps->second_chroma_qp_index_offset = pps->chroma_qp_index_offset;
    }
    if (br_failed(br)) return H264_PARSE_INVALID;
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
    struct v4l2_ctrl_h264_decode_params *decode,
    H264ParseResult *out)
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

    /* P / SP / B specifics. */
    if (st == V4L2_H264_SLICE_TYPE_B) {
        if (br_u1(br)) slice->flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;
    }
    if (st == V4L2_H264_SLICE_TYPE_P || st == V4L2_H264_SLICE_TYPE_SP ||
        st == V4L2_H264_SLICE_TYPE_B) {
        int override_flag = br_u1(br);
        if (override_flag) {
            slice->num_ref_idx_l0_active_minus1 = (uint8_t)br_ue(br);
            if (st == V4L2_H264_SLICE_TYPE_B)
                slice->num_ref_idx_l1_active_minus1 = (uint8_t)br_ue(br);
        } else {
            slice->num_ref_idx_l0_active_minus1 =
                pps->num_ref_idx_l0_default_active_minus1;
            slice->num_ref_idx_l1_active_minus1 =
                pps->num_ref_idx_l1_default_active_minus1;
        }

        /* ref_pic_list_modification (H.264 7.3.3.1) — surface to the
         * DPB layer so it can apply spec 8.2.4.3 modifications before
         * packing rps_base.  The rkvdec H.264 path does NOT re-derive
         * the per-slice RefPicList from the slice header bits (unlike
         * HEVC HW_RPS); it consumes whatever we packed into rps_base. */
        if (st != V4L2_H264_SLICE_TYPE_I && st != V4L2_H264_SLICE_TYPE_SI) {
            int mod_l0 = br_u1(br);
            out->ref_pic_list_modification_flag_l0 = (uint8_t)mod_l0;
            if (mod_l0) {
                while (1) {
                    uint32_t mod_op = br_ue(br);
                    if (mod_op == 3) break;
                    uint32_t val = br_ue(br);
                    if (out->n_rplm_l0 < 32) {
                        out->rplm_l0[out->n_rplm_l0].op    = (uint8_t)mod_op;
                        out->rplm_l0[out->n_rplm_l0].value = val;
                        out->n_rplm_l0++;
                    }
                    if (br_eof(br)) break;
                }
            }
            if (st == V4L2_H264_SLICE_TYPE_B) {
                int mod_l1 = br_u1(br);
                out->ref_pic_list_modification_flag_l1 = (uint8_t)mod_l1;
                if (mod_l1) {
                    while (1) {
                        uint32_t mod_op = br_ue(br);
                        if (mod_op == 3) break;
                        uint32_t val = br_ue(br);
                        if (out->n_rplm_l1 < 32) {
                            out->rplm_l1[out->n_rplm_l1].op    = (uint8_t)mod_op;
                            out->rplm_l1[out->n_rplm_l1].value = val;
                            out->n_rplm_l1++;
                        }
                        if (br_eof(br)) break;
                    }
                }
            }
        }
    }

    /* pred_weight_table — walk to keep alignment when present.  Our
     * test stream has weighted_pred=0 and weighted_bipred_idc=0 so
     * this typically isn't triggered. */
    if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) &&
         (st == V4L2_H264_SLICE_TYPE_P || st == V4L2_H264_SLICE_TYPE_SP)) ||
        (pps->weighted_bipred_idc == 1 && st == V4L2_H264_SLICE_TYPE_B)) {
        (void)br_ue(br);                                /* luma_log2_weight_denom */
        if (sps->chroma_format_idc != 0)
            (void)br_ue(br);                            /* chroma_log2_weight_denom */
        for (int list = 0; list < (st == V4L2_H264_SLICE_TYPE_B ? 2 : 1); list++) {
            int n = list == 0 ? slice->num_ref_idx_l0_active_minus1
                              : slice->num_ref_idx_l1_active_minus1;
            for (int i = 0; i <= n; i++) {
                if (br_u1(br)) {                        /* luma_weight_flag */
                    (void)br_se(br); (void)br_se(br);
                }
                if (sps->chroma_format_idc != 0 && br_u1(br)) {
                    (void)br_se(br); (void)br_se(br);
                    (void)br_se(br); (void)br_se(br);
                }
            }
        }
    }

    /* dec_ref_pic_marking (H.264 7.3.3.1) — surface MMCO ops to the
     * DPB layer so non-trivial streams (real-world H.264 with adaptive
     * marking) get correct ref-list maintenance. */
    if (nal_ref_idc != 0) {
        if (nal_unit_type == 5) {
            out->idr_no_output_of_prior_pics_flag = (uint8_t)br_u1(br);
            out->idr_long_term_reference_flag     = (uint8_t)br_u1(br);
        } else {
            int adaptive = br_u1(br);
            out->adaptive_ref_pic_marking_mode_flag = (uint8_t)adaptive;
            if (adaptive) {
                while (1) {
                    uint32_t op = br_ue(br);
                    if (op == 0) break;
                    if (op > 6 || br_eof(br)) break;
                    H264Mmco m = {};
                    m.op = (uint8_t)op;
                    if (op == 1 || op == 3) m.difference_of_pic_nums_minus1 = br_ue(br);
                    if (op == 2)            m.long_term_pic_num             = br_ue(br);
                    if (op == 3 || op == 6) m.long_term_frame_idx           = br_ue(br);
                    if (op == 4)            m.max_long_term_frame_idx_plus1 = br_ue(br);
                    if (out->n_mmco < H264_MAX_MMCO_OPS) {
                        out->mmco[out->n_mmco++] = m;
                    }
                }
            }
        }
    }

    /* Surface any latched malformed-bitstream condition from br_ue/br_se
     * — the rest of the function reads many Exp-Golomb fields and a
     * malformed slice header would otherwise have a clamped-zero
     * mmco/RPLM table flow into the DPB layer.  Review parser Medium #8. */
    if (br_failed(br)) return H264_PARSE_INVALID;
    return H264_PARSE_OK;
}

/* H.264 8.2.1.1 — Picture Order Count derivation for type 0.  Caller
 * provides previous-AU POC state; we update + return TopFieldOrderCnt
 * and BottomFieldOrderCnt for the current pic.  Field coding is out
 * of scope (top == bottom for our streams). */
static void h264_compute_poc_type0(
    const v4l2_ctrl_h264_sps    *sps,
    const v4l2_ctrl_h264_pps    *pps,
    v4l2_ctrl_h264_decode_params *decode,
    int                          is_idr,
    int                          nal_ref_idc,
    int32_t                     *prev_msb_io,
    int32_t                     *prev_lsb_io)
{
    (void)pps;
    int32_t MaxPicOrderCntLsb = 1 << (sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
    int32_t poc_lsb = (int32_t)decode->pic_order_cnt_lsb;
    int32_t prev_msb, prev_lsb;
    if (is_idr) {
        prev_msb = 0;
        prev_lsb = 0;
    } else {
        /* For a "non-reference picture that immediately follows a memory
         * management 5 marking" the prev_msb/lsb are reset, but our
         * MMCO walker doesn't surface that yet — assume normal flow. */
        prev_msb = *prev_msb_io;
        prev_lsb = *prev_lsb_io;
    }
    int32_t poc_msb;
    if ((poc_lsb < prev_lsb) &&
        ((prev_lsb - poc_lsb) >= (MaxPicOrderCntLsb / 2)))
        poc_msb = prev_msb + MaxPicOrderCntLsb;
    else if ((poc_lsb > prev_lsb) &&
             ((poc_lsb - prev_lsb) > (MaxPicOrderCntLsb / 2)))
        poc_msb = prev_msb - MaxPicOrderCntLsb;
    else
        poc_msb = prev_msb;

    int32_t top    = poc_msb + poc_lsb;
    int32_t bottom = top + decode->delta_pic_order_cnt_bottom;

    decode->top_field_order_cnt    = top;
    decode->bottom_field_order_cnt = bottom;

    /* Update prev state — only for reference pictures (nal_ref_idc != 0)
     * per spec 8.2.1.1.  For non-ref pics the previous POC values stay
     * unchanged so the next ref can still derive correctly. */
    if (nal_ref_idc != 0) {
        *prev_msb_io = poc_msb;
        *prev_lsb_io = poc_lsb;
    }
}

/* H.264 8.2.1.2 — POC derivation for type 1 (cycle-based). */
static void h264_compute_poc_type1(
    const v4l2_ctrl_h264_sps    *sps,
    v4l2_ctrl_h264_decode_params *decode,
    int                          is_idr,
    int                          nal_ref_idc,
    int32_t                     *prev_frame_num_io,
    int32_t                     *prev_frame_num_offset_io)
{
    int32_t MaxFrameNum = 1 << (sps->log2_max_frame_num_minus4 + 4);
    int32_t cur_frame_num = (int32_t)decode->frame_num;
    int32_t prev_frame_num = *prev_frame_num_io;
    int32_t prev_offset    = *prev_frame_num_offset_io;
    int32_t frame_num_offset;
    if (is_idr) frame_num_offset = 0;
    else if (prev_frame_num > cur_frame_num)
        frame_num_offset = prev_offset + MaxFrameNum;
    else
        frame_num_offset = prev_offset;

    int32_t abs_frame_num =
        (sps->num_ref_frames_in_pic_order_cnt_cycle != 0)
            ? frame_num_offset + cur_frame_num
            : 0;
    if (nal_ref_idc == 0 && abs_frame_num > 0) abs_frame_num--;

    int32_t expected_pic_order_cnt = 0;
    if (abs_frame_num > 0) {
        int32_t poc_cycle_len = 0;
        for (int i = 0; i < sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
            poc_cycle_len += sps->offset_for_ref_frame[i];
        int32_t pic_order_cnt_cycle_cnt =
            (abs_frame_num - 1) / sps->num_ref_frames_in_pic_order_cnt_cycle;
        int32_t frame_num_in_poc_cycle =
            (abs_frame_num - 1) % sps->num_ref_frames_in_pic_order_cnt_cycle;
        expected_pic_order_cnt = pic_order_cnt_cycle_cnt * poc_cycle_len;
        for (int i = 0; i <= frame_num_in_poc_cycle; i++)
            expected_pic_order_cnt += sps->offset_for_ref_frame[i];
    }
    if (nal_ref_idc == 0)
        expected_pic_order_cnt += sps->offset_for_non_ref_pic;

    int32_t top    = expected_pic_order_cnt + decode->delta_pic_order_cnt0;
    int32_t bottom = top + sps->offset_for_top_to_bottom_field +
                     decode->delta_pic_order_cnt1;

    decode->top_field_order_cnt    = top;
    decode->bottom_field_order_cnt = bottom;

    *prev_frame_num_io        = cur_frame_num;
    *prev_frame_num_offset_io = frame_num_offset;
}

/* H.264 8.2.1.3 — POC derivation for type 2 (decoder-order = display-order). */
static void h264_compute_poc_type2(
    const v4l2_ctrl_h264_sps    *sps,
    v4l2_ctrl_h264_decode_params *decode,
    int                          is_idr,
    int                          nal_ref_idc,
    int32_t                     *prev_frame_num_io,
    int32_t                     *prev_frame_num_offset_io)
{
    int32_t MaxFrameNum = 1 << (sps->log2_max_frame_num_minus4 + 4);
    int32_t cur_frame_num = (int32_t)decode->frame_num;
    int32_t prev_frame_num = *prev_frame_num_io;
    int32_t prev_offset    = *prev_frame_num_offset_io;
    int32_t frame_num_offset;
    if (is_idr) frame_num_offset = 0;
    else if (prev_frame_num > cur_frame_num)
        frame_num_offset = prev_offset + MaxFrameNum;
    else
        frame_num_offset = prev_offset;

    int32_t poc;
    if (is_idr) poc = 0;
    else if (nal_ref_idc == 0)
        poc = 2 * (frame_num_offset + cur_frame_num) - 1;
    else
        poc = 2 * (frame_num_offset + cur_frame_num);

    decode->top_field_order_cnt    = poc;
    decode->bottom_field_order_cnt = poc;

    *prev_frame_num_io        = cur_frame_num;
    *prev_frame_num_offset_io = frame_num_offset;
}

} /* anon namespace */

/* ============================================================ Public entry */

extern "C"
H264ParseStatus H264ParseAccessUnit(const uint8_t *buf, size_t len,
                                    uint8_t *scratch, size_t scratch_size,
                                    H264ParseResult *out)
{
    if (!buf || !out || !scratch) return H264_PARSE_INVALID;
    /* Reset per-AU fields, keep persistent SPS/PPS state across calls.
     * Caller initialises the whole struct once; subsequent calls inherit
     * the most recent SPS/PPS so P/B slices in later AUs can decode. */
    memset(&out->decode, 0, sizeof(out->decode));
    memset(&out->slice,  0, sizeof(out->slice));
    memset(&out->pred_weights, 0, sizeof(out->pred_weights));
    out->has_slice         = 0;
    out->has_pred_weights  = 0;
    /* has_scaling_matrix stays sticky across the PPS that defined it. */
    out->slice_data        = NULL;
    out->slice_data_size   = 0;
    /* Reset dec_ref_pic_marking surface — these are per-AU. */
    out->adaptive_ref_pic_marking_mode_flag = 0;
    out->idr_no_output_of_prior_pics_flag   = 0;
    out->idr_long_term_reference_flag       = 0;
    out->n_mmco                             = 0;
    memset(out->mmco, 0, sizeof(out->mmco));
    /* Reset ref_pic_list_modification surface. */
    out->ref_pic_list_modification_flag_l0 = 0;
    out->ref_pic_list_modification_flag_l1 = 0;
    out->n_rplm_l0                         = 0;
    out->n_rplm_l1                         = 0;
    memset(out->rplm_l0, 0, sizeof(out->rplm_l0));
    memset(out->rplm_l1, 0, sizeof(out->rplm_l1));

    /* Stage prev_* POC state on the stack across the AU loop and commit
     * back to `out` only at the very end on success.  Without this
     * staging, a successfully-parsed slice early in the AU writes new
     * POC state into `out->prev_*`, then a later NAL fails and the
     * caller sees the partially-mutated POC state next call — visible
     * as POC desync after a corrupted middle NAL.  Review parser
     * High #2. */
    int32_t staged_prev_msb         = out->prev_pic_order_cnt_msb;
    int32_t staged_prev_lsb         = out->prev_pic_order_cnt_lsb;
    int32_t staged_prev_frame_num   = out->prev_frame_num;
    int32_t staged_prev_frame_num_offset = out->prev_frame_num_offset;

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
            uint8_t sps_has_sm = 0;
            v4l2_ctrl_h264_scaling_matrix sps_sm{};
            H264ParseStatus s = parse_sps(&br, &out->sps,
                                          &out->has_sps_vui_reorder,
                                          &out->sps_vui_max_num_reorder_frames,
                                          &sps_sm, &sps_has_sm,
                                          /*out_vui_colour=*/out);
            if (s != H264_PARSE_OK) return s;
            out->has_sps = 1;
            /* SPS-level scaling lists become the active matrix until a
             * subsequent PPS overrides them.  Without this, High-profile
             * streams that put scaling lists at SPS-level (common for
             * x264-encoded movies — defined once globally) decode with
             * flat=16 dequant → image structure intact, chroma colors
             * shifted from the encoder's intent. */
            if (sps_has_sm) {
                out->scaling_matrix    = sps_sm;
                out->has_scaling_matrix = 1;
                /* Make sure the codec is told to use the scaling table
                 * even if/when a PPS arrives that doesn't itself set
                 * pic_scaling_matrix_present. */
                if (out->has_pps) {
                    out->pps.flags |= V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
                }
            }
            break;
        }
        case 8: {
            int has_scaling = 0;
            H264ParseStatus s = parse_pps(&br, &out->pps, &out->scaling_matrix,
                                          &has_scaling);
            if (s != H264_PARSE_OK) return s;
            out->has_pps = 1;
            if (has_scaling) {
                out->has_scaling_matrix = 1;
            } else if (out->has_scaling_matrix) {
                /* PPS doesn't override but SPS-level matrix is active —
                 * propagate the flag so the codec uses the scaling table. */
                out->pps.flags |= V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
            }
            break;
        }
        case 1: case 5: {
            if (!out->has_sps || !out->has_pps) return H264_PARSE_INVALID;
            H264ParseStatus s = parse_slice_header(&br, nal_ref_idc,
                                                   nal_unit_type, &out->sps,
                                                   &out->pps, &out->slice,
                                                   &out->decode, out);
            if (s != H264_PARSE_OK) return s;
            out->has_slice = 1;
            /* Compute Picture Order Count per spec 8.2.1.  The regbuilder
             * consumes top/bottom_field_order_cnt for reg65/66, and the
             * DPB packs ref dpb[].top_field_order_cnt for reg67..98.
             * Without this populated, every non-IDR pic carries POC=0 and
             * the codec can't disambiguate refs.  Persistent state lives
             * on out->prev_* (sticky across calls). */
            {
                int is_idr_now = (out->decode.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;
                int nal_ref = nal_ref_idc;
                if (out->sps.pic_order_cnt_type == 0) {
                    h264_compute_poc_type0(&out->sps, &out->pps, &out->decode,
                                           is_idr_now, nal_ref,
                                           &staged_prev_msb,
                                           &staged_prev_lsb);
                } else if (out->sps.pic_order_cnt_type == 1) {
                    h264_compute_poc_type1(&out->sps, &out->decode,
                                           is_idr_now, nal_ref,
                                           &staged_prev_frame_num,
                                           &staged_prev_frame_num_offset);
                } else {
                    h264_compute_poc_type2(&out->sps, &out->decode,
                                           is_idr_now, nal_ref,
                                           &staged_prev_frame_num,
                                           &staged_prev_frame_num_offset);
                }
            }
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
    /* Commit staged POC state — see comment at the top of the AU loop.
     * Review parser High #2. */
    out->prev_pic_order_cnt_msb     = staged_prev_msb;
    out->prev_pic_order_cnt_lsb     = staged_prev_lsb;
    out->prev_frame_num             = staged_prev_frame_num;
    out->prev_frame_num_offset      = staged_prev_frame_num_offset;
    return H264_PARSE_OK;
}
