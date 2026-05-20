/* mft/parser_glue_h265.cpp — minimal H.265 NAL → struct parser.
 *
 * See parser_glue_h265.h for scope.  Mirrors the structure of
 * parser_glue.cpp (H.264 path) closely.  Reimplemented from spec /
 * rockchip-linux mpp h265d source — no upstream code is vendored.
 *
 * Spec references throughout (e.g. "7.3.2.2.1") are to ITU-T H.265
 * (V8 02/2018) which is the version the rkvdec2 hardware targets.
 */

#include "parser_glue_h265.h"
#include <string.h>

namespace {

/* ============================================================ Bit reader *
 * Note: this is structurally identical to the H.264 parser_glue.cpp's
 * BitReader.  Kept inline (in this file's anon namespace) rather than
 * sharing a header — extracting to a third file would touch the H.264
 * code, and the working H.264 parser is the one thing we don't want to
 * break in this task.  The two copies will diverge only if HEVC needs
 * something exotic the H.264 reader doesn't (it doesn't, today). */

struct BitReader {
    const uint8_t *data;
    size_t         len;     /* bytes */
    size_t         bit_pos;
    /* Sticky error flag — set by any read that hit EOF mid-prefix or
     * an Exp-Golomb leading-zero run >= 32.  Callers check
     * br_failed(br) at unit-parse boundaries to reject malformed
     * input rather than silently using clamped-zero values that flow
     * into the regbuilder.  See [[parser_eg_error_propagation]]. */
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

/* H.265 9.2.2 — unsigned Exp-Golomb code (ue(v)).  Identical algorithm
 * to H.264's ue(v); the spec text is shared.  On malformed input
 * (EOF mid-prefix or >= 32 leading zeros) latches br->failed and
 * returns 0 so callers can reject the unit at the parse boundary. */
static uint32_t br_ue(BitReader *br) {
    int zeros = 0;
    while (!br_eof(br) && br_u1(br) == 0) zeros++;
    if (br_eof(br) && zeros > 0) { br->failed = true; return 0; }
    if (zeros >= 32) { br->failed = true; return 0; }
    uint32_t suffix = br_u(br, zeros);
    return ((1u << zeros) - 1) + suffix;
}

/* Signed Exp-Golomb se(v) — H.265 9.2.2. */
static int32_t br_se(BitReader *br) {
    uint32_t k = br_ue(br);
    return (k & 1) ? (int32_t)((k + 1) >> 1) : -(int32_t)(k >> 1);
}

/* ceil(log2(n)) for slice-segment-address bit width (7.4.7.1). */
static int ceil_log2(uint32_t n) {
    int r = 0;
    if (n <= 1) return 0;
    n -= 1;
    while (n) { r++; n >>= 1; }
    return r;
}

/* ============================================================ Annex-B + RBSP */

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

/* H.265 7.3.1.1 — emulation-prevention removal.  Identical to H.264. */
static size_t ebsp_to_rbsp(const uint8_t *src, size_t src_len,
                           uint8_t *dst, size_t dst_cap) {
    size_t o = 0;
    int    zeros = 0;
    for (size_t i = 0; i < src_len; i++) {
        uint8_t b = src[i];
        if (zeros >= 2 && b == 0x03) {
            zeros = 0;
            continue;
        }
        if (o >= dst_cap) return 0;
        dst[o++] = b;
        zeros = (b == 0) ? zeros + 1 : 0;
    }
    return o;
}

/* ============================================================ Profile/tier/level */

/* H.265 7.3.3 — profile_tier_level().  Captures the fields useful for
 * regbuilder gating; sub-layer PTL records are skipped. */
static void parse_ptl(BitReader *br, H265Ptl *ptl, int max_sub_layers_minus1) {
    ptl->profile_space = (uint8_t)br_u(br, 2);
    ptl->tier_flag     = (uint8_t)br_u1(br);
    ptl->profile_idc   = (uint8_t)br_u(br, 5);
    for (int i = 0; i < 32; i++)
        ptl->profile_compatibility_flag[i] = (uint8_t)br_u1(br);
    ptl->progressive_source_flag    = (uint8_t)br_u1(br);
    ptl->interlaced_source_flag     = (uint8_t)br_u1(br);
    ptl->non_packed_constraint_flag = (uint8_t)br_u1(br);
    ptl->frame_only_constraint_flag = (uint8_t)br_u1(br);
    /* general_reserved_zero_43bits + general_inbld_or_reserved_zero_bit
     * = 44 bits total (covers all the constraint flags we don't use). */
    (void)br_u(br, 16);
    (void)br_u(br, 16);
    (void)br_u(br, 12);
    ptl->level_idc = (uint8_t)br_u(br, 8);

    /* Sub-layer profile/level present flags + 2-bit reserved each. */
    uint8_t sub_profile_present[H265_MAX_SUB_LAYERS] = {0};
    uint8_t sub_level_present  [H265_MAX_SUB_LAYERS] = {0};
    for (int i = 0; i < max_sub_layers_minus1; i++) {
        sub_profile_present[i] = (uint8_t)br_u1(br);
        sub_level_present  [i] = (uint8_t)br_u1(br);
    }
    if (max_sub_layers_minus1 > 0) {
        /* reserved_zero_2bits padding to make the present-flag table 8
         * entries wide (spec 7.3.3, "for ( i = maxNumSubLayersMinus1; i < 8;...)"). */
        for (int i = max_sub_layers_minus1; i < 8; i++)
            (void)br_u(br, 2);
    }
    for (int i = 0; i < max_sub_layers_minus1; i++) {
        if (sub_profile_present[i]) {
            /* Skip a full sub-layer PTL: 2+1+5+32+1+1+1+1+44 = 88 bits. */
            (void)br_u(br, 32);
            (void)br_u(br, 32);
            (void)br_u(br, 24);
        }
        if (sub_level_present[i])
            (void)br_u(br, 8);
    }
}

/* ============================================================ Scaling lists *
 * H.265 7.3.4 / 7.4.5 — scaling_list_data().  Hand-rolled; pre-fills
 * defaults (16 for 4x4, the 8x8 default-intra/-inter tables) before
 * applying delta_coeff updates. */

static const uint8_t kDefaultScalingIntra[64] = {
    16, 16, 16, 16, 17, 18, 21, 24,  16, 16, 16, 16, 17, 19, 22, 25,
    16, 16, 17, 18, 20, 22, 25, 29,  16, 16, 18, 21, 24, 27, 31, 36,
    17, 17, 20, 24, 30, 35, 41, 47,  18, 19, 22, 27, 35, 44, 54, 65,
    21, 22, 25, 31, 41, 54, 70, 88,  24, 25, 29, 36, 47, 65, 88, 115,
};
static const uint8_t kDefaultScalingInter[64] = {
    16, 16, 16, 16, 17, 18, 20, 24,  16, 16, 16, 17, 18, 20, 24, 25,
    16, 16, 17, 18, 20, 24, 25, 28,  16, 17, 18, 20, 24, 25, 28, 33,
    17, 18, 20, 24, 25, 28, 33, 41,  18, 20, 24, 25, 28, 33, 41, 54,
    20, 24, 25, 28, 33, 41, 54, 71,  24, 25, 28, 33, 41, 54, 71, 91,
};

/* H.265 6.5.4 — diagonal scan order, used to map raster index → spec
 * coefficient position when filling in scaling lists. */
static const uint8_t kDiagScan4x4[16] = {
    0, 4,  1,  8,   5, 2,  12, 9,
    6, 3,  13, 10,  7, 14, 11, 15,
};
static const uint8_t kDiagScan8x8[64] = {
    0,  8,  1,  16, 9,  2,  24, 17,  10, 3,  32, 25, 18, 11, 4,  40,
    33, 26, 19, 12, 5,  48, 41, 34,  27, 20, 13, 6,  56, 49, 42, 35,
    28, 21, 14, 7,  57, 50, 43, 36,  29, 22, 15, 58, 51, 44, 37, 30,
    23, 59, 52, 45, 38, 31, 60, 53,  46, 39, 61, 54, 47, 62, 55, 63,
};

static void scaling_list_set_default(H265ScalingList *sl) {
    /* sz_id=0: 4x4 — flat 16 (spec 7.4.5 default A). */
    for (int m = 0; m < 6; m++) memset(sl->scaling_list_4x4[m], 16, 16);
    /* sz_id≥1: intra for matrix 0..2, inter for 3..5. */
    for (int m = 0; m < 6; m++) {
        const uint8_t *src = (m < 3) ? kDefaultScalingIntra : kDefaultScalingInter;
        memcpy(sl->scaling_list_8x8  [m], src, 64);
        memcpy(sl->scaling_list_16x16[m], src, 64);
        memcpy(sl->scaling_list_32x32[m], src, 64);
    }
    for (int m = 0; m < 6; m++) {
        sl->scaling_list_dc_16x16[m] = 16;
        sl->scaling_list_dc_32x32[m] = 16;
    }
}

/* H.265 7.3.4 — scaling_list_data(). */
static H265ParseStatus parse_scaling_list_data(BitReader *br, H265ScalingList *sl) {
    scaling_list_set_default(sl);
    for (int sz_id = 0; sz_id < 4; sz_id++) {
        int mat_step = (sz_id == 3) ? 3 : 1;
        for (int mat_id = 0; mat_id < 6; mat_id += mat_step) {
            int pred_flag = br_u1(br);   /* scaling_list_pred_mode_flag */
            if (!pred_flag) {
                /* Copy from another matrix. */
                uint32_t delta = br_ue(br);
                if (delta) {
                    delta *= mat_step;
                    if ((uint32_t)mat_id < delta) return H265_PARSE_INVALID;
                    int from = mat_id - (int)delta;
                    int n = (sz_id == 0) ? 16 : 64;
                    uint8_t *dst, *src;
                    switch (sz_id) {
                    case 0: dst = sl->scaling_list_4x4  [mat_id]; src = sl->scaling_list_4x4  [from]; break;
                    case 1: dst = sl->scaling_list_8x8  [mat_id]; src = sl->scaling_list_8x8  [from]; break;
                    case 2: dst = sl->scaling_list_16x16[mat_id]; src = sl->scaling_list_16x16[from]; break;
                    default:dst = sl->scaling_list_32x32[mat_id]; src = sl->scaling_list_32x32[from]; break;
                    }
                    memcpy(dst, src, n);
                    if (sz_id == 2) sl->scaling_list_dc_16x16[mat_id] = sl->scaling_list_dc_16x16[from];
                    if (sz_id == 3) sl->scaling_list_dc_32x32[mat_id] = sl->scaling_list_dc_32x32[from];
                }
                /* delta == 0 leaves defaults in place. */
            } else {
                int next_coef = 8;
                int coef_count = (sz_id == 0) ? 16 : 64;
                if (sz_id > 1) {
                    int dc_minus8 = br_se(br);
                    if (dc_minus8 < -7 || dc_minus8 > 247) return H265_PARSE_INVALID;
                    next_coef = dc_minus8 + 8;
                    if (sz_id == 2) sl->scaling_list_dc_16x16[mat_id] = (uint8_t)next_coef;
                    else            sl->scaling_list_dc_32x32[mat_id] = (uint8_t)next_coef;
                }
                for (int i = 0; i < coef_count; i++) {
                    int pos = (sz_id == 0) ? kDiagScan4x4[i] : kDiagScan8x8[i];
                    int delta_coef = br_se(br);
                    next_coef = (next_coef + delta_coef + 256) & 0xff;
                    uint8_t *dst;
                    switch (sz_id) {
                    case 0: dst = sl->scaling_list_4x4  [mat_id]; break;
                    case 1: dst = sl->scaling_list_8x8  [mat_id]; break;
                    case 2: dst = sl->scaling_list_16x16[mat_id]; break;
                    default:dst = sl->scaling_list_32x32[mat_id]; break;
                    }
                    dst[pos] = (uint8_t)next_coef;
                }
            }
        }
    }
    return H265_PARSE_OK;
}

/* ============================================================ Short-term RPS *
 * H.265 7.3.7 / 7.4.8 — short_term_ref_pic_set( stRpsIdx ).  Two paths:
 * inter-RPS prediction (delta from a previously parsed STRPS) and
 * direct (num_neg + num_pos delta_pocs).  The slice path uses the same
 * function with stRpsIdx == sps->num_short_term_ref_pic_sets.
 *
 * `rps_list` is the SPS's already-parsed STRPS table (NULL if parsing
 * the first one).  `st_rps_idx` is the position of `out` in the table
 * (or num_short_term_ref_pic_sets if called from the slice header). */
static H265ParseStatus parse_st_rps(BitReader *br, H265ShortTermRPS *out,
                                    const H265ShortTermRPS *rps_list,
                                    uint32_t st_rps_idx,
                                    uint32_t num_short_term_ref_pic_sets) {
    int inter_rps = 0;
    if (st_rps_idx != 0)
        inter_rps = br_u1(br);
    out->inter_ref_pic_set_prediction_flag = (uint8_t)inter_rps;

    if (inter_rps) {
        /* H.265 7.3.7 — derive the predicted RPS from a reference RPS. */
        uint32_t delta_idx_minus1 = 0;
        if (st_rps_idx == num_short_term_ref_pic_sets)
            delta_idx_minus1 = br_ue(br);
        if (delta_idx_minus1 + 1 > st_rps_idx) return H265_PARSE_INVALID;
        uint32_t ref_idx = st_rps_idx - (delta_idx_minus1 + 1);
        int delta_rps_sign = br_u1(br);
        uint32_t abs_delta_rps_minus1 = br_ue(br);
        int delta_rps = (1 - 2 * delta_rps_sign) * (int)(abs_delta_rps_minus1 + 1);

        const H265ShortTermRPS *ref = &rps_list[ref_idx];
        int n_ref = ref->num_delta_pocs;
        if (n_ref > H265_MAX_REFS) return H265_PARSE_INVALID;

        uint8_t used_by_curr_pic[H265_MAX_REFS + 1] = {0};
        uint8_t use_delta       [H265_MAX_REFS + 1] = {0};
        for (int j = 0; j <= n_ref; j++) {
            used_by_curr_pic[j] = (uint8_t)br_u1(br);
            if (!used_by_curr_pic[j])
                use_delta[j] = (uint8_t)br_u1(br);
            else
                use_delta[j] = 1;
        }

        /* Build the predicted delta_poc list per spec 7.4.8 / eq 7-61.
         * Order: positive-from-ref (ascending), zero-delta pivot,
         * negative-from-ref (descending) — then we re-sort into the
         * canonical (negatives ascending, positives ascending) form. */
        int16_t pocs[H265_MAX_REFS + 1];
        uint8_t used[H265_MAX_REFS + 1];
        int n_pos = 0, n_neg = 0;

        /* Positives first (loop over ref's positive entries, descending). */
        for (int j = ref->num_positive_pics - 1; j >= 0; j--) {
            int dPoc = ref->delta_poc[ref->num_negative_pics + j] + delta_rps;
            if (dPoc < 0 && use_delta[ref->num_negative_pics + j]) {
                pocs[n_neg + n_pos] = (int16_t)dPoc;
                used[n_neg + n_pos] = used_by_curr_pic[ref->num_negative_pics + j];
                n_neg++;
            }
        }
        if (delta_rps < 0 && use_delta[n_ref]) {
            pocs[n_neg + n_pos] = (int16_t)delta_rps;
            used[n_neg + n_pos] = used_by_curr_pic[n_ref];
            n_neg++;
        }
        for (int j = 0; j < ref->num_negative_pics; j++) {
            int dPoc = ref->delta_poc[j] + delta_rps;
            if (dPoc < 0 && use_delta[j]) {
                pocs[n_neg + n_pos] = (int16_t)dPoc;
                used[n_neg + n_pos] = used_by_curr_pic[j];
                n_neg++;
            }
        }
        for (int j = ref->num_negative_pics - 1; j >= 0; j--) {
            int dPoc = ref->delta_poc[j] + delta_rps;
            if (dPoc > 0 && use_delta[j]) {
                pocs[n_neg + n_pos] = (int16_t)dPoc;
                used[n_neg + n_pos] = used_by_curr_pic[j];
                n_pos++;
            }
        }
        if (delta_rps > 0 && use_delta[n_ref]) {
            pocs[n_neg + n_pos] = (int16_t)delta_rps;
            used[n_neg + n_pos] = used_by_curr_pic[n_ref];
            n_pos++;
        }
        for (int j = 0; j < ref->num_positive_pics; j++) {
            int dPoc = ref->delta_poc[ref->num_negative_pics + j] + delta_rps;
            if (dPoc > 0 && use_delta[ref->num_negative_pics + j]) {
                pocs[n_neg + n_pos] = (int16_t)dPoc;
                used[n_neg + n_pos] = used_by_curr_pic[ref->num_negative_pics + j];
                n_pos++;
            }
        }

        if (n_neg + n_pos > H265_MAX_REFS) return H265_PARSE_INVALID;
        out->num_negative_pics = (uint8_t)n_neg;
        out->num_positive_pics = (uint8_t)n_pos;
        out->num_delta_pocs    = (uint8_t)(n_neg + n_pos);
        for (int j = 0; j < n_neg + n_pos; j++) {
            out->delta_poc[j]            = pocs[j];
            out->used_by_curr_pic_flag[j]= used[j];
        }
    } else {
        /* Direct: read num_neg, num_pos, then per-entry delta+used. */
        uint32_t num_neg = br_ue(br);
        uint32_t num_pos = br_ue(br);
        if (num_neg > H265_MAX_REFS || num_pos > H265_MAX_REFS ||
            num_neg + num_pos > H265_MAX_REFS)
            return H265_PARSE_INVALID;
        out->num_negative_pics = (uint8_t)num_neg;
        out->num_positive_pics = (uint8_t)num_pos;
        out->num_delta_pocs    = (uint8_t)(num_neg + num_pos);

        int prev = 0;
        for (uint32_t j = 0; j < num_neg; j++) {
            int delta = (int)br_ue(br) + 1;
            int poc = prev - delta;
            out->delta_poc[j] = (int16_t)poc;
            out->used_by_curr_pic_flag[j] = (uint8_t)br_u1(br);
            prev = poc;
        }
        prev = 0;
        for (uint32_t j = 0; j < num_pos; j++) {
            int delta = (int)br_ue(br) + 1;
            int poc = prev + delta;
            out->delta_poc[num_neg + j] = (int16_t)poc;
            out->used_by_curr_pic_flag[num_neg + j] = (uint8_t)br_u1(br);
            prev = poc;
        }
    }
    return H265_PARSE_OK;
}

/* ============================================================ HRD/VUI skip *
 * For VUI we only need to advance past the structure if the SPS
 * encoder included one — none of the fields are consumed by the
 * regbuilder.  Implementing fully would double the file size; instead
 * we skip until rbsp_trailing if vui_parameters_present_flag is set.
 * That works because the SPS extension flags after VUI are also fields
 * we don't care about (range/SCC ext) — we just don't read them. */

/* ============================================================ VPS parser */

/* H.265 7.3.2.1 — video_parameter_set_rbsp(). */
static H265ParseStatus parse_vps(BitReader *br, H265ParseResult *out) {
    uint8_t vps_id = (uint8_t)br_u(br, 4);
    if (vps_id >= H265_MAX_VPS) return H265_PARSE_INVALID;
    H265Vps *vps = &out->vps[vps_id];
    memset(vps, 0, sizeof(*vps));
    vps->vps_id = vps_id;

    /* vps_base_layer_internal_flag, vps_base_layer_available_flag.
     * Plus 2 reserved bits for plain HEVC = 4 bits total. */
    (void)br_u(br, 2);
    vps->vps_max_layers_minus1     = (uint8_t)br_u(br, 6);
    vps->vps_max_sub_layers_minus1 = (uint8_t)br_u(br, 3);
    vps->vps_temporal_id_nesting_flag = (uint8_t)br_u1(br);
    /* vps_reserved_0xffff_16bits */
    (void)br_u(br, 16);

    if (vps->vps_max_sub_layers_minus1 >= H265_MAX_SUB_LAYERS)
        return H265_PARSE_INVALID;

    parse_ptl(br, &vps->ptl, vps->vps_max_sub_layers_minus1);

    vps->vps_sub_layer_ordering_info_present_flag = (uint8_t)br_u1(br);
    int start = vps->vps_sub_layer_ordering_info_present_flag
              ? 0 : vps->vps_max_sub_layers_minus1;
    for (int i = start; i <= vps->vps_max_sub_layers_minus1; i++) {
        vps->vps_max_dec_pic_buffering_minus1[i] = (uint8_t)br_ue(br);
        vps->vps_max_num_reorder_pics        [i] = (uint8_t)br_ue(br);
        vps->vps_max_latency_increase_plus1  [i] = br_ue(br);
    }

    /* vps_max_layer_id (6) + vps_num_layer_sets_minus1 ue, then
     * layer_id_included_flag table — we don't track scalability so
     * skip cleanly; same is true for VPS timing/HRD. */

    uint32_t vps_max_layer_id        = br_u(br, 6);
    uint32_t vps_num_layer_sets_minus1 = br_ue(br);
    for (uint32_t i = 1; i <= vps_num_layer_sets_minus1; i++)
        for (uint32_t j = 0; j <= vps_max_layer_id; j++)
            (void)br_u1(br);

    /* vps_timing_info_present_flag — skip the body if set; we don't
     * need timing/HRD to drive the hardware. */
    int vps_timing_info_present = br_u1(br);
    if (vps_timing_info_present) {
        (void)br_u(br, 32);   /* vps_num_units_in_tick */
        (void)br_u(br, 32);   /* vps_time_scale */
        if (br_u1(br))        /* vps_poc_proportional_to_timing_flag */
            (void)br_ue(br);  /* vps_num_ticks_poc_diff_one_minus1 */
        /* vps_num_hrd_parameters — bail out: HRD bodies are deeply
         * variable-length and we never need them.  Set the rest of
         * the bitreader past EOF safely. */
        uint32_t nh = br_ue(br);
        if (nh) {
            /* Best-effort skip: there is no fixed bit count here so
             * the only safe action is to ignore the rest of the VPS. */
            (void)br_eof(br);
        }
    }

    vps->valid = 1;
    return H265_PARSE_OK;
}

/* ============================================================ SPS parser */

/* H.265 7.3.2.2 — seq_parameter_set_rbsp().  This is the workhorse —
 * the regbuilder consumes most of these fields directly. */
static H265ParseStatus parse_sps(BitReader *br, H265ParseResult *out) {
    H265Sps tmp;
    memset(&tmp, 0, sizeof(tmp));

    tmp.vps_id                    = (uint8_t)br_u(br, 4);
    tmp.sps_max_sub_layers_minus1 = (uint8_t)br_u(br, 3);
    tmp.sps_temporal_id_nesting_flag = (uint8_t)br_u1(br);
    if (tmp.sps_max_sub_layers_minus1 >= H265_MAX_SUB_LAYERS)
        return H265_PARSE_INVALID;

    parse_ptl(br, &tmp.ptl, tmp.sps_max_sub_layers_minus1);

    uint32_t sps_id = br_ue(br);
    if (sps_id >= H265_MAX_SPS) return H265_PARSE_INVALID;
    tmp.sps_id = (uint8_t)sps_id;

    tmp.chroma_format_idc = (uint8_t)br_ue(br);
    if (tmp.chroma_format_idc == 3)
        tmp.separate_colour_plane_flag = (uint8_t)br_u1(br);

    tmp.pic_width_in_luma_samples  = br_ue(br);
    tmp.pic_height_in_luma_samples = br_ue(br);
    /* Bound the dimensions before they feed `pic_width_in_ctbs *
     * pic_height_in_ctbs` (used as a bit-width for slice_segment_address
     * at line ~732).  Without this, an adversarial SPS at 2^31 wraps the
     * uint32 product to a small value and `ceil_log2` reads an
     * attacker-controlled bit count.  rkvdec2 silicon cap is 8192×8192;
     * reject above that.  See [[critical_h265_dim_bound]]. */
    if (tmp.pic_width_in_luma_samples  == 0 ||
        tmp.pic_width_in_luma_samples  > 8192u ||
        tmp.pic_height_in_luma_samples == 0 ||
        tmp.pic_height_in_luma_samples > 8192u) {
        return H265_PARSE_INVALID;
    }

    tmp.conformance_window_flag = (uint8_t)br_u1(br);
    if (tmp.conformance_window_flag) {
        tmp.conf_win_left_offset   = br_ue(br);
        tmp.conf_win_right_offset  = br_ue(br);
        tmp.conf_win_top_offset    = br_ue(br);
        tmp.conf_win_bottom_offset = br_ue(br);
    }

    tmp.bit_depth_luma_minus8   = (uint8_t)br_ue(br);
    tmp.bit_depth_chroma_minus8 = (uint8_t)br_ue(br);
    tmp.log2_max_pic_order_cnt_lsb_minus4 = (uint8_t)br_ue(br);

    tmp.sps_sub_layer_ordering_info_present_flag = (uint8_t)br_u1(br);
    int start = tmp.sps_sub_layer_ordering_info_present_flag
              ? 0 : tmp.sps_max_sub_layers_minus1;
    for (int i = start; i <= tmp.sps_max_sub_layers_minus1; i++) {
        tmp.sps_max_dec_pic_buffering_minus1[i] = (uint8_t)br_ue(br);
        tmp.sps_max_num_reorder_pics        [i] = (uint8_t)br_ue(br);
        tmp.sps_max_latency_increase_plus1  [i] = br_ue(br);
    }
    /* If sub_layer info wasn't per-layer, broadcast layer-N down. */
    if (!tmp.sps_sub_layer_ordering_info_present_flag) {
        for (int i = 0; i < start; i++) {
            tmp.sps_max_dec_pic_buffering_minus1[i] = tmp.sps_max_dec_pic_buffering_minus1[start];
            tmp.sps_max_num_reorder_pics        [i] = tmp.sps_max_num_reorder_pics        [start];
            tmp.sps_max_latency_increase_plus1  [i] = tmp.sps_max_latency_increase_plus1  [start];
        }
    }

    tmp.log2_min_luma_coding_block_size_minus3        = (uint8_t)br_ue(br);
    tmp.log2_diff_max_min_luma_coding_block_size      = (uint8_t)br_ue(br);
    tmp.log2_min_luma_transform_block_size_minus2     = (uint8_t)br_ue(br);
    tmp.log2_diff_max_min_luma_transform_block_size   = (uint8_t)br_ue(br);
    tmp.max_transform_hierarchy_depth_inter           = (uint8_t)br_ue(br);
    tmp.max_transform_hierarchy_depth_intra           = (uint8_t)br_ue(br);

    tmp.scaling_list_enabled_flag = (uint8_t)br_u1(br);
    if (tmp.scaling_list_enabled_flag) {
        tmp.sps_scaling_list_data_present_flag = (uint8_t)br_u1(br);
        if (tmp.sps_scaling_list_data_present_flag) {
            H265ParseStatus s = parse_scaling_list_data(br, &tmp.scaling_list);
            if (s != H265_PARSE_OK) return s;
        } else {
            scaling_list_set_default(&tmp.scaling_list);
        }
    }

    tmp.amp_enabled_flag                       = (uint8_t)br_u1(br);
    tmp.sample_adaptive_offset_enabled_flag    = (uint8_t)br_u1(br);
    tmp.pcm_enabled_flag                       = (uint8_t)br_u1(br);
    if (tmp.pcm_enabled_flag) {
        tmp.pcm_sample_bit_depth_luma_minus1   = (uint8_t)br_u(br, 4);
        tmp.pcm_sample_bit_depth_chroma_minus1 = (uint8_t)br_u(br, 4);
        tmp.log2_min_pcm_luma_coding_block_size_minus3   = (uint8_t)br_ue(br);
        tmp.log2_diff_max_min_pcm_luma_coding_block_size = (uint8_t)br_ue(br);
        tmp.pcm_loop_filter_disabled_flag      = (uint8_t)br_u1(br);
    }

    tmp.num_short_term_ref_pic_sets = (uint8_t)br_ue(br);
    if (tmp.num_short_term_ref_pic_sets > H265_MAX_SHORT_TERM_RPS)
        return H265_PARSE_INVALID;
    for (int i = 0; i < tmp.num_short_term_ref_pic_sets; i++) {
        H265ParseStatus s = parse_st_rps(br, &tmp.st_rps[i],
                                         tmp.st_rps,
                                         (uint32_t)i,
                                         tmp.num_short_term_ref_pic_sets);
        if (s != H265_PARSE_OK) return s;
    }

    tmp.long_term_ref_pics_present_flag = (uint8_t)br_u1(br);
    if (tmp.long_term_ref_pics_present_flag) {
        tmp.num_long_term_ref_pics_sps = (uint8_t)br_ue(br);
        if (tmp.num_long_term_ref_pics_sps > H265_MAX_LONG_TERM_REFS)
            return H265_PARSE_INVALID;
        int log2_lsb = tmp.log2_max_pic_order_cnt_lsb_minus4 + 4;
        for (int i = 0; i < tmp.num_long_term_ref_pics_sps; i++) {
            tmp.lt_ref_pic_poc_lsb_sps   [i] = (uint16_t)br_u(br, log2_lsb);
            tmp.used_by_curr_pic_lt_sps_flag[i] = (uint8_t)br_u1(br);
        }
    }

    tmp.sps_temporal_mvp_enabled_flag       = (uint8_t)br_u1(br);
    tmp.strong_intra_smoothing_enabled_flag = (uint8_t)br_u1(br);

    /* vui_parameters_present_flag, sps_extension_present_flag and the
     * extension blocks follow but we don't consume any of those fields
     * — leave them unread.  The slice path doesn't reach for them. */

    /* Derived quantities the regbuilder wants pre-computed. */
    int log2_min_cb = tmp.log2_min_luma_coding_block_size_minus3 + 3;
    int log2_ctb    = log2_min_cb + tmp.log2_diff_max_min_luma_coding_block_size;
    tmp.ctb_log2_size_y = (uint8_t)log2_ctb;
    tmp.ctb_size_y      = (uint8_t)(1u << log2_ctb);
    if (log2_ctb == 0 || log2_ctb > 6) return H265_PARSE_INVALID;
    uint32_t ctb_unit = 1u << log2_ctb;
    tmp.pic_width_in_ctbs_y  = (tmp.pic_width_in_luma_samples  + ctb_unit - 1) >> log2_ctb;
    tmp.pic_height_in_ctbs_y = (tmp.pic_height_in_luma_samples + ctb_unit - 1) >> log2_ctb;

    /* Surface any latched malformed-bitstream condition from
     * br_ue/br_se before committing the parsed SPS. */
    if (br_failed(br)) return H265_PARSE_INVALID;
    tmp.valid = 1;
    out->sps[tmp.sps_id] = tmp;
    out->active_sps_id   = (int8_t)tmp.sps_id;
    out->got_sps_in_au   = 1;
    return H265_PARSE_OK;
}

/* ============================================================ PPS parser */

/* H.265 7.3.2.3 — pic_parameter_set_rbsp(). */
static H265ParseStatus parse_pps(BitReader *br, H265ParseResult *out) {
    uint32_t pps_id = br_ue(br);
    if (pps_id >= H265_MAX_PPS) return H265_PARSE_INVALID;
    uint32_t sps_id = br_ue(br);
    if (sps_id >= H265_MAX_SPS || !out->sps[sps_id].valid)
        return H265_PARSE_INVALID;

    H265Pps tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.pps_id = (uint8_t)pps_id;
    tmp.sps_id = (uint8_t)sps_id;
    /* Spec defaults that survive when tile fields aren't read. */
    tmp.uniform_spacing_flag                       = 1;
    tmp.loop_filter_across_tiles_enabled_flag      = 1;

    tmp.dependent_slice_segments_enabled_flag = (uint8_t)br_u1(br);
    tmp.output_flag_present_flag              = (uint8_t)br_u1(br);
    tmp.num_extra_slice_header_bits           = (uint8_t)br_u(br, 3);
    tmp.sign_data_hiding_enabled_flag         = (uint8_t)br_u1(br);
    tmp.cabac_init_present_flag               = (uint8_t)br_u1(br);
    tmp.num_ref_idx_l0_default_active_minus1  = (uint8_t)br_ue(br);
    tmp.num_ref_idx_l1_default_active_minus1  = (uint8_t)br_ue(br);
    tmp.init_qp_minus26                       = (int8_t)br_se(br);
    tmp.constrained_intra_pred_flag           = (uint8_t)br_u1(br);
    tmp.transform_skip_enabled_flag           = (uint8_t)br_u1(br);
    tmp.cu_qp_delta_enabled_flag              = (uint8_t)br_u1(br);
    if (tmp.cu_qp_delta_enabled_flag)
        tmp.diff_cu_qp_delta_depth            = (uint8_t)br_ue(br);
    tmp.pps_cb_qp_offset                      = (int8_t)br_se(br);
    tmp.pps_cr_qp_offset                      = (int8_t)br_se(br);
    tmp.pps_slice_chroma_qp_offsets_present_flag = (uint8_t)br_u1(br);
    tmp.weighted_pred_flag                    = (uint8_t)br_u1(br);
    tmp.weighted_bipred_flag                  = (uint8_t)br_u1(br);
    tmp.transquant_bypass_enabled_flag        = (uint8_t)br_u1(br);
    tmp.tiles_enabled_flag                    = (uint8_t)br_u1(br);
    tmp.entropy_coding_sync_enabled_flag      = (uint8_t)br_u1(br);

    if (tmp.tiles_enabled_flag) {
        /* Capture the count + uniform flag.  We don't store the
         * per-column / per-row width tables — the regbuilder author
         * who ships tile support will need to extend H265Pps. */
        tmp.num_tile_columns_minus1 = (uint8_t)br_ue(br);
        tmp.num_tile_rows_minus1    = (uint8_t)br_ue(br);
        tmp.uniform_spacing_flag    = (uint8_t)br_u1(br);
        if (!tmp.uniform_spacing_flag) {
            for (int i = 0; i < tmp.num_tile_columns_minus1; i++)
                (void)br_ue(br);     /* column_width_minus1[i] */
            for (int i = 0; i < tmp.num_tile_rows_minus1; i++)
                (void)br_ue(br);     /* row_height_minus1[i] */
        }
        tmp.loop_filter_across_tiles_enabled_flag = (uint8_t)br_u1(br);
    }

    tmp.pps_loop_filter_across_slices_enabled_flag = (uint8_t)br_u1(br);
    tmp.deblocking_filter_control_present_flag     = (uint8_t)br_u1(br);
    if (tmp.deblocking_filter_control_present_flag) {
        tmp.deblocking_filter_override_enabled_flag = (uint8_t)br_u1(br);
        tmp.pps_deblocking_filter_disabled_flag     = (uint8_t)br_u1(br);
        if (!tmp.pps_deblocking_filter_disabled_flag) {
            tmp.pps_beta_offset_div2 = (int8_t)br_se(br);
            tmp.pps_tc_offset_div2   = (int8_t)br_se(br);
        }
    }

    tmp.pps_scaling_list_data_present_flag = (uint8_t)br_u1(br);
    if (tmp.pps_scaling_list_data_present_flag) {
        H265ParseStatus s = parse_scaling_list_data(br, &tmp.scaling_list);
        if (s != H265_PARSE_OK) return s;
    }

    tmp.lists_modification_present_flag = (uint8_t)br_u1(br);
    tmp.log2_parallel_merge_level_minus2 = (uint8_t)br_ue(br);
    tmp.slice_segment_header_extension_present_flag = (uint8_t)br_u1(br);
    /* pps_extension_flag and the range/SCC extensions follow — we
     * don't consume them. */

    if (br_failed(br)) return H265_PARSE_INVALID;
    tmp.valid = 1;
    out->pps[tmp.pps_id] = tmp;
    out->active_pps_id   = (int8_t)tmp.pps_id;
    out->got_pps_in_au   = 1;
    return H265_PARSE_OK;
}

/* ============================================================ Slice header */

/* H.265 7.3.6.1 — slice_segment_header().  Captures the fields the
 * regbuilder needs and skips the pred-weight-table inline.  The bit
 * position at the end of this function marks the start of slice data. */
static H265ParseStatus parse_slice_header(BitReader *br,
                                          uint8_t nal_unit_type,
                                          H265ParseResult *out) {
    H265SliceHeader *sh = &out->slice;
    memset(sh, 0, sizeof(*sh));

    sh->first_slice_segment_in_pic_flag = (uint8_t)br_u1(br);

    int is_irap = (nal_unit_type >= H265_NAL_BLA_W_LP &&
                   nal_unit_type <= H265_NAL_CRA_NUT);
    int is_idr  = (nal_unit_type == H265_NAL_IDR_W_RADL ||
                   nal_unit_type == H265_NAL_IDR_N_LP);
    out->is_irap = (uint8_t)is_irap;
    out->is_idr  = (uint8_t)is_idr;

    if (is_irap)
        sh->no_output_of_prior_pics_flag = (uint8_t)br_u1(br);

    sh->slice_pic_parameter_set_id = (uint8_t)br_ue(br);
    if (sh->slice_pic_parameter_set_id >= H265_MAX_PPS ||
        !out->pps[sh->slice_pic_parameter_set_id].valid)
        return H265_PARSE_INVALID;
    out->active_pps_id = (int8_t)sh->slice_pic_parameter_set_id;

    const H265Pps *pps = &out->pps[sh->slice_pic_parameter_set_id];
    if (!out->sps[pps->sps_id].valid) return H265_PARSE_INVALID;
    const H265Sps *sps = &out->sps[pps->sps_id];
    out->active_sps_id = (int8_t)pps->sps_id;

    if (!sh->first_slice_segment_in_pic_flag) {
        if (pps->dependent_slice_segments_enabled_flag)
            sh->dependent_slice_segment_flag = (uint8_t)br_u1(br);
        int slice_addr_bits = ceil_log2(sps->pic_width_in_ctbs_y *
                                        sps->pic_height_in_ctbs_y);
        sh->slice_segment_address = br_u(br, slice_addr_bits);
    }

    if (!sh->dependent_slice_segment_flag) {
        for (int i = 0; i < pps->num_extra_slice_header_bits; i++)
            (void)br_u1(br);

        sh->slice_type = (uint8_t)br_ue(br);
        if (sh->slice_type > H265_SLICE_TYPE_I) return H265_PARSE_INVALID;

        if (pps->output_flag_present_flag)
            sh->pic_output_flag = (uint8_t)br_u1(br);
        if (sps->separate_colour_plane_flag)
            sh->colour_plane_id = (uint8_t)br_u(br, 2);

        if (!is_idr) {
            int log2_lsb = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
            sh->slice_pic_order_cnt_lsb = (uint16_t)br_u(br, log2_lsb);
            sh->short_term_ref_pic_set_sps_flag = (uint8_t)br_u1(br);

            uint32_t bit_begin = (uint32_t)br->bit_pos;
            if (!sh->short_term_ref_pic_set_sps_flag) {
                H265ParseStatus s = parse_st_rps(br, &sh->st_rps_slice,
                                                 sps->st_rps,
                                                 sps->num_short_term_ref_pic_sets,
                                                 sps->num_short_term_ref_pic_sets);
                if (s != H265_PARSE_OK) return s;
            } else if (sps->num_short_term_ref_pic_sets > 1) {
                int n = ceil_log2(sps->num_short_term_ref_pic_sets);
                sh->short_term_ref_pic_set_idx = (uint8_t)br_u(br, n);
            }
            sh->short_term_ref_pic_set_size = (uint32_t)br->bit_pos - bit_begin;

            if (sps->long_term_ref_pics_present_flag) {
                if (sps->num_long_term_ref_pics_sps > 0)
                    sh->num_long_term_sps = (uint8_t)br_ue(br);
                sh->num_long_term_pics = (uint8_t)br_ue(br);
                /* Skip per-LT entries — we don't manage LTR.  Bail
                 * out cleanly if the stream actually uses them; for
                 * our test streams the count is always 0. */
                if (sh->num_long_term_sps + sh->num_long_term_pics > 0)
                    return H265_PARSE_UNSUPPORTED;
            }

            if (sps->sps_temporal_mvp_enabled_flag)
                sh->slice_temporal_mvp_enabled_flag = (uint8_t)br_u1(br);
        }

        if (sps->sample_adaptive_offset_enabled_flag) {
            sh->slice_sao_luma_flag = (uint8_t)br_u1(br);
            if (sps->chroma_format_idc != 0)
                sh->slice_sao_chroma_flag = (uint8_t)br_u1(br);
        }

        if (sh->slice_type == H265_SLICE_TYPE_P ||
            sh->slice_type == H265_SLICE_TYPE_B) {
            sh->num_ref_idx_active_override_flag = (uint8_t)br_u1(br);
            if (sh->num_ref_idx_active_override_flag) {
                sh->num_ref_idx_l0_active_minus1 = (uint8_t)br_ue(br);
                if (sh->slice_type == H265_SLICE_TYPE_B)
                    sh->num_ref_idx_l1_active_minus1 = (uint8_t)br_ue(br);
            } else {
                sh->num_ref_idx_l0_active_minus1 = pps->num_ref_idx_l0_default_active_minus1;
                sh->num_ref_idx_l1_active_minus1 = pps->num_ref_idx_l1_default_active_minus1;
            }

            /* Determine number of refs for ref_pic_list_modification gating. */
            int total_refs = 0;
            const H265ShortTermRPS *strps =
                sh->short_term_ref_pic_set_sps_flag
                ? &sps->st_rps[sh->short_term_ref_pic_set_idx]
                : &sh->st_rps_slice;
            for (int i = 0; i < strps->num_delta_pocs; i++)
                if (strps->used_by_curr_pic_flag[i]) total_refs++;
            /* + LT refs (we already bailed if any are present). */

            if (pps->lists_modification_present_flag && total_refs > 1) {
                int rpl_l0 = br_u1(br);
                if (rpl_l0) {
                    int b = ceil_log2((uint32_t)total_refs);
                    for (int i = 0; i <= sh->num_ref_idx_l0_active_minus1; i++)
                        (void)br_u(br, b);
                }
                if (sh->slice_type == H265_SLICE_TYPE_B) {
                    int rpl_l1 = br_u1(br);
                    if (rpl_l1) {
                        int b = ceil_log2((uint32_t)total_refs);
                        for (int i = 0; i <= sh->num_ref_idx_l1_active_minus1; i++)
                            (void)br_u(br, b);
                    }
                }
            }

            if (sh->slice_type == H265_SLICE_TYPE_B)
                sh->mvd_l1_zero_flag = (uint8_t)br_u1(br);
            if (pps->cabac_init_present_flag)
                sh->cabac_init_flag = (uint8_t)br_u1(br);

            if (sh->slice_temporal_mvp_enabled_flag) {
                if (sh->slice_type == H265_SLICE_TYPE_B)
                    sh->collocated_from_l0_flag = (uint8_t)br_u1(br);
                else
                    sh->collocated_from_l0_flag = 1;
                int collocated_list_size =
                    sh->collocated_from_l0_flag
                    ? (sh->num_ref_idx_l0_active_minus1 + 1)
                    : (sh->num_ref_idx_l1_active_minus1 + 1);
                if (collocated_list_size > 1)
                    sh->collocated_ref_idx = (uint8_t)br_ue(br);
            }

            if ((pps->weighted_pred_flag   && sh->slice_type == H265_SLICE_TYPE_P) ||
                (pps->weighted_bipred_flag && sh->slice_type == H265_SLICE_TYPE_B)) {
                /* H.265 7.3.6.3 — pred_weight_table().  We don't expose
                 * the parsed values to the regbuilder (rkvdec hardware
                 * re-parses them from the slice payload), but we MUST
                 * advance past the structure so following fields stay
                 * aligned.  Implementation walks the syntax exactly. */
                int chroma = sps->chroma_format_idc != 0;
                (void)br_ue(br);                  /* luma_log2_weight_denom */
                if (chroma) (void)br_se(br);      /* delta_chroma_log2_weight_denom */
                /* L0 weights. */
                int n0 = sh->num_ref_idx_l0_active_minus1 + 1;
                uint8_t luma_w0  [16] = {0};
                uint8_t chroma_w0[16] = {0};
                for (int i = 0; i < n0; i++) luma_w0[i]   = (uint8_t)br_u1(br);
                if (chroma)
                    for (int i = 0; i < n0; i++) chroma_w0[i] = (uint8_t)br_u1(br);
                for (int i = 0; i < n0; i++) {
                    if (luma_w0[i]) {
                        (void)br_se(br);          /* delta_luma_weight_l0[i] */
                        (void)br_se(br);          /* luma_offset_l0[i] */
                    }
                    if (chroma_w0[i]) {
                        (void)br_se(br); (void)br_se(br);   /* Cb */
                        (void)br_se(br); (void)br_se(br);   /* Cr */
                    }
                }
                if (sh->slice_type == H265_SLICE_TYPE_B) {
                    int n1 = sh->num_ref_idx_l1_active_minus1 + 1;
                    uint8_t luma_w1  [16] = {0};
                    uint8_t chroma_w1[16] = {0};
                    for (int i = 0; i < n1; i++) luma_w1[i]   = (uint8_t)br_u1(br);
                    if (chroma)
                        for (int i = 0; i < n1; i++) chroma_w1[i] = (uint8_t)br_u1(br);
                    for (int i = 0; i < n1; i++) {
                        if (luma_w1[i]) {
                            (void)br_se(br);
                            (void)br_se(br);
                        }
                        if (chroma_w1[i]) {
                            (void)br_se(br); (void)br_se(br);
                            (void)br_se(br); (void)br_se(br);
                        }
                    }
                }
            }

            sh->five_minus_max_num_merge_cand = (uint8_t)br_ue(br);
        }

        sh->slice_qp_delta = (int8_t)br_se(br);
        if (pps->pps_slice_chroma_qp_offsets_present_flag) {
            sh->slice_cb_qp_offset = (int8_t)br_se(br);
            sh->slice_cr_qp_offset = (int8_t)br_se(br);
        }

        if (pps->deblocking_filter_control_present_flag) {
            if (pps->deblocking_filter_override_enabled_flag)
                sh->deblocking_filter_override_flag = (uint8_t)br_u1(br);
            if (sh->deblocking_filter_override_flag) {
                sh->slice_deblocking_filter_disabled_flag = (uint8_t)br_u1(br);
                if (!sh->slice_deblocking_filter_disabled_flag) {
                    sh->slice_beta_offset_div2 = (int8_t)br_se(br);
                    sh->slice_tc_offset_div2   = (int8_t)br_se(br);
                }
            } else {
                sh->slice_deblocking_filter_disabled_flag = pps->pps_deblocking_filter_disabled_flag;
                sh->slice_beta_offset_div2 = pps->pps_beta_offset_div2;
                sh->slice_tc_offset_div2   = pps->pps_tc_offset_div2;
            }
        }

        if (pps->pps_loop_filter_across_slices_enabled_flag &&
            (sh->slice_sao_luma_flag || sh->slice_sao_chroma_flag ||
             !sh->slice_deblocking_filter_disabled_flag)) {
            sh->slice_loop_filter_across_slices_enabled_flag = (uint8_t)br_u1(br);
        }
    }

    if (pps->tiles_enabled_flag || pps->entropy_coding_sync_enabled_flag) {
        sh->num_entry_point_offsets = br_ue(br);
        if (sh->num_entry_point_offsets) {
            uint32_t offset_len_minus1 = br_ue(br);
            for (uint32_t i = 0; i < sh->num_entry_point_offsets; i++)
                (void)br_u(br, (int)(offset_len_minus1 + 1));
        }
    }

    if (pps->slice_segment_header_extension_present_flag) {
        uint32_t length = br_ue(br);
        for (uint32_t i = 0; i < length; i++)
            (void)br_u(br, 8);
    }

    /* byte_alignment(): one '1' bit then zero-pad to byte. */
    (void)br_u1(br);
    while (br->bit_pos & 7) (void)br_u1(br);

    sh->header_bit_size = (uint32_t)br->bit_pos;
    return H265_PARSE_OK;
}

/* ============================================================ POC compute *
 * H.265 8.3.1 — short version.  IDR resets to 0; otherwise we treat
 * the previous decoded pic's POC as the reference and resolve LSB ↔
 * MSB by the standard ±max/2 wrap rule.  For first-decode use a
 * persistent prev_poc_lsb / prev_poc_msb in the result struct.
 *
 * We don't store these in the public struct; keep them in static
 * locals in the entry function below. */

} /* anon namespace */

/* ============================================================ Public entry */

extern "C" {

void H265ParseResultInit(H265ParseResult *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->active_vps_id = -1;
    out->active_sps_id = -1;
    out->active_pps_id = -1;
}

H265ParseStatus H265ParseAccessUnit(const uint8_t *buf, size_t len,
                                    uint8_t *scratch, size_t scratch_size,
                                    H265ParseResult *out)
{
    if (!buf || !out || !scratch) return H265_PARSE_INVALID;

    /* POC continuity carried across calls on the per-instance state in
     * H265ParseResult — concurrent streams keep separate state, where
     * the previous statics let parallel parsers stomp each other. */

    /* Reset per-AU state; keep VPS/SPS/PPS arrays sticky. */
    memset(&out->slice, 0, sizeof(out->slice));
    out->has_slice            = 0;
    out->got_vps_in_au        = 0;
    out->got_sps_in_au        = 0;
    out->got_pps_in_au        = 0;
    out->slice_nal_unit_type  = 0;
    out->nal_ref_flag         = 0;
    out->is_idr               = 0;
    out->is_irap              = 0;
    out->slice_data           = nullptr;
    out->slice_data_size      = 0;

    size_t scratch_off = 0;
    size_t pos = 0;
    while (pos < len) {
        size_t sc_start = 0;
        size_t nal_start = find_start_code(buf, len, pos, &sc_start);
        if (nal_start >= len) break;

        size_t next_sc = 0;
        find_start_code(buf, len, nal_start, &next_sc);
        size_t nal_end = (next_sc > nal_start) ? next_sc : len;

        size_t this_size = nal_end - nal_start;
        if (scratch_off + this_size > scratch_size) return H265_PARSE_INVALID;
        uint8_t *rbsp = scratch + scratch_off;
        size_t rbsp_len = ebsp_to_rbsp(buf + nal_start, this_size,
                                       rbsp, scratch_size - scratch_off);
        if (rbsp_len < 2) return H265_PARSE_INVALID;
        scratch_off += rbsp_len;

        /* H.265 7.3.1.2 — 2-byte NAL header.
         *   bit  0    forbidden_zero_bit
         *   bits 1..6 nal_unit_type
         *   bits 7..12 nuh_layer_id
         *   bits 13..15 nuh_temporal_id_plus1 */
        uint16_t nh = ((uint16_t)rbsp[0] << 8) | rbsp[1];
        uint8_t  nal_unit_type   = (uint8_t)((nh >> 9) & 0x3f);
        uint8_t  nuh_layer_id    = (uint8_t)((nh >> 3) & 0x3f);
        uint8_t  temporal_id_p1  = (uint8_t)( nh       & 0x07);
        (void)nuh_layer_id;

        BitReader br;
        br_init(&br, rbsp + 2, rbsp_len - 2);

        switch (nal_unit_type) {
        case H265_NAL_VPS: {
            H265ParseStatus s = parse_vps(&br, out);
            if (s != H265_PARSE_OK) return s;
            break;
        }
        case H265_NAL_SPS: {
            H265ParseStatus s = parse_sps(&br, out);
            if (s != H265_PARSE_OK) return s;
            break;
        }
        case H265_NAL_PPS: {
            H265ParseStatus s = parse_pps(&br, out);
            if (s != H265_PARSE_OK) return s;
            break;
        }
        case H265_NAL_TRAIL_N: case H265_NAL_TRAIL_R:
        case H265_NAL_TSA_N:   case H265_NAL_TSA_R:
        case H265_NAL_STSA_N:  case H265_NAL_STSA_R:
        case H265_NAL_RADL_N:  case H265_NAL_RADL_R:
        case H265_NAL_RASL_N:  case H265_NAL_RASL_R:
        case H265_NAL_BLA_W_LP: case H265_NAL_BLA_W_RADL: case H265_NAL_BLA_N_LP:
        case H265_NAL_IDR_W_RADL: case H265_NAL_IDR_N_LP:
        case H265_NAL_CRA_NUT: {
            H265ParseStatus s = parse_slice_header(&br, nal_unit_type, out);
            if (s != H265_PARSE_OK) return s;
            out->has_slice = 1;
            out->slice_nal_unit_type = nal_unit_type;
            /* "ref" is anything that's not a sub-layer non-reference
             * (suffix _N).  H.265 table 7-1. */
            int is_sub_layer_nonref =
                (nal_unit_type == H265_NAL_TRAIL_N ||
                 nal_unit_type == H265_NAL_TSA_N   ||
                 nal_unit_type == H265_NAL_STSA_N  ||
                 nal_unit_type == H265_NAL_RADL_N  ||
                 nal_unit_type == H265_NAL_RASL_N);
            out->nal_ref_flag = (uint8_t)!is_sub_layer_nonref;

            /* Compute POC (8.3.1).  IDR resets; otherwise wrap-aware. */
            int32_t poc;
            if (out->is_idr) {
                poc = 0;
                out->prev_poc_msb_tid0 = 0;
                out->prev_poc_lsb_tid0 = 0;
            } else {
                const H265Sps *sps = &out->sps[out->active_sps_id];
                int max_lsb = 1 << (sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
                int32_t lsb = out->slice.slice_pic_order_cnt_lsb;
                int32_t prev_lsb = out->prev_poc_lsb_tid0;
                int32_t prev_msb = out->prev_poc_msb_tid0;
                int32_t msb;
                if (lsb < prev_lsb && (prev_lsb - lsb) >= max_lsb / 2)
                    msb = prev_msb + max_lsb;
                else if (lsb > prev_lsb && (lsb - prev_lsb) > max_lsb / 2)
                    msb = prev_msb - max_lsb;
                else
                    msb = prev_msb;
                poc = msb + lsb;
            }
            out->poc = poc;

            /* Update prev_poc_tid0 only for temporal_id == 0 + non-RASL/RADL/SLNR.
             * temporal_id_plus1 is derived from the NAL header. */
            int temporal_id = (int)temporal_id_p1 - 1;
            int is_radl_rasl_n =
                (nal_unit_type == H265_NAL_RADL_N || nal_unit_type == H265_NAL_RADL_R ||
                 nal_unit_type == H265_NAL_RASL_N || nal_unit_type == H265_NAL_RASL_R ||
                 nal_unit_type == H265_NAL_TRAIL_N || nal_unit_type == H265_NAL_TSA_N ||
                 nal_unit_type == H265_NAL_STSA_N);
            if (temporal_id == 0 && !is_radl_rasl_n) {
                int max_lsb = 0;
                if (out->active_sps_id >= 0) {
                    const H265Sps *sps = &out->sps[out->active_sps_id];
                    max_lsb = 1 << (sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
                }
                if (max_lsb) {
                    out->prev_poc_lsb_tid0 = poc % max_lsb;
                    if (out->prev_poc_lsb_tid0 < 0) out->prev_poc_lsb_tid0 += max_lsb;
                    out->prev_poc_msb_tid0 = poc - out->prev_poc_lsb_tid0;
                }
            }

            /* Slice data: bytes after the byte-aligned slice header. */
            size_t data_off = (br.bit_pos + 7) >> 3;
            if (data_off > rbsp_len - 2) return H265_PARSE_INVALID;
            out->slice_data      = rbsp + 2 + data_off;
            out->slice_data_size = (rbsp_len - 2) - data_off;
            break;
        }
        default:
            /* AUD, SEI, EOS, EOB, filler, reserved: ignore. */
            break;
        }

        pos = nal_end;
    }

    if (!out->has_slice) {
        return (out->got_sps_in_au || out->got_pps_in_au || out->got_vps_in_au)
                 ? H265_PARSE_NEED_MORE
                 : H265_PARSE_INVALID;
    }
    return H265_PARSE_OK;
}

} /* extern "C" */
