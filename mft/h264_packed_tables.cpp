/* mft/h264_packed_tables.cpp — port of rockchip-linux/mpp packed-table
 * builders for rkvdec2 H.264 (vdpu34x).
 *
 * Cross-references:
 *   prepare_spspps          : mpp/hal/rkdec/h264d/hal_h264d_vdpu34x.c:130
 *   prepare_framerps        : mpp/hal/rkdec/h264d/hal_h264d_vdpu34x.c:226
 *   prepare_scanlist        : mpp/hal/rkdec/h264d/hal_h264d_vdpu34x.c:291
 *   vdpu34x_get_rcb_buf_size: mpp/vdpu34x_com.c:57
 *   rkv_cabac_table         : mpp/hal/rkdec/h264d/hal_h264d_com.c:16
 */

#include "h264_packed_tables.h"
#include <string.h>

namespace {

/* ============================================================ BitPacker
 *
 * Mirrors MPP's BitputCtx_t / mpp_put_bits / mpp_put_align: writes are
 * LSB-first into a u64 stream (so when the buffer is treated as raw
 * bytes on a little-endian host, the first byte holds bits 0..7 of the
 * first u64). */

struct BitPacker {
    uint64_t *data;
    size_t    cap_u64;
    size_t    word_idx;
    int       bit_in_word;   /* 0..63 */

    void init(uint64_t *buf, size_t words) {
        data = buf;
        cap_u64 = words;
        word_idx = 0;
        bit_in_word = 0;
        memset(buf, 0, words * sizeof(uint64_t));
    }

    void put(uint64_t value, int nbits) {
        if (nbits == 0) return;
        uint64_t mask = (nbits == 64) ? ~0ULL : ((1ULL << nbits) - 1);
        uint64_t v = value & mask;
        if (word_idx >= cap_u64) return;
        if (bit_in_word + nbits <= 64) {
            data[word_idx] |= v << bit_in_word;
            bit_in_word += nbits;
            if (bit_in_word == 64) { word_idx++; bit_in_word = 0; }
        } else {
            int low_bits = 64 - bit_in_word;
            data[word_idx] |= (v & ((1ULL << low_bits) - 1)) << bit_in_word;
            word_idx++;
            if (word_idx < cap_u64) {
                data[word_idx] |= v >> low_bits;
            }
            bit_in_word = nbits - low_bits;
        }
    }

    /* Pad with zeros to the next multiple of `n_bits` total bits. */
    void align(int n_bits) {
        size_t total = word_idx * 64 + (size_t)bit_in_word;
        size_t aligned = (total + (size_t)n_bits - 1) & ~((size_t)n_bits - 1);
        size_t pad = aligned - total;
        while (pad > 0) {
            int chunk = (pad > 32) ? 32 : (int)pad;
            put(0, chunk);
            pad -= chunk;
        }
    }
};

/* ============================================================ CABAC table */
/* Generated from /tmp/mpp/hal_h264d_com.c lines 16..172. */
#include "h264_cabac_init.inc"

/* ============================================================ Helpers */

static inline int has_flag(uint32_t f, uint32_t bit) { return (f & bit) ? 1 : 0; }

/* DXVA `pp` field equivalents from V4L2 control structs. */
static inline int sps_field_mbs_only(const v4l2_ctrl_h264_sps *s) {
    return has_flag(s->flags, V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY);
}
static inline int sps_mbaff_capable(const v4l2_ctrl_h264_sps *s) {
    return has_flag(s->flags, V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD);
}
static inline int sps_direct_8x8(const v4l2_ctrl_h264_sps *s) {
    return has_flag(s->flags, V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE);
}
static inline int sps_dpoc_always_zero(const v4l2_ctrl_h264_sps *s) {
    return has_flag(s->flags, V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO);
}

} /* anon */

/* ---- CABAC ------------------------------------------------------- */

extern "C"
const uint32_t *H264GetCabacInitTable(size_t *out_word_count) {
    if (out_word_count) *out_word_count = sizeof(kRkH264CabacInit) / sizeof(uint32_t);
    return kRkH264CabacInit;
}

/* ---- SPS+PPS packed unit ----------------------------------------- */

extern "C"
void H264PackSpsPpsUnit(uint8_t out[RKH264_SPSPPS_UNIT_SIZE],
                        const v4l2_ctrl_h264_sps           *sps,
                        const v4l2_ctrl_h264_pps           *pps,
                        const v4l2_h264_dpb_entry          *dpb,
                        int field_pic)
{
    /* 48 bytes = 6 u64 words. */
    uint64_t buf[6];
    BitPacker bp;
    bp.init(buf, 6);

    /* SPS section ---------------------------------------------------- */
    bp.put(0x1FFFu, 13);                                /* sps/profile/cs3 = -1 */
    bp.put(sps->chroma_format_idc, 2);
    bp.put(sps->bit_depth_luma_minus8, 3);
    bp.put(sps->bit_depth_chroma_minus8, 3);
    bp.put(0, 1);                                       /* qpprime_y_zero */
    bp.put(sps->log2_max_frame_num_minus4, 4);
    bp.put(sps->max_num_ref_frames, 5);
    bp.put(sps->pic_order_cnt_type, 2);
    bp.put(sps->log2_max_pic_order_cnt_lsb_minus4, 4);
    bp.put((uint64_t)sps_dpoc_always_zero(sps), 1);
    bp.put((uint64_t)sps->pic_width_in_mbs_minus1 + 1, 12);
    bp.put((uint64_t)sps->pic_height_in_map_units_minus1 + 1, 12);
    bp.put((uint64_t)sps_field_mbs_only(sps), 1);
    /* MbaffFrameFlag = MB_ADAPTIVE_FRAME_FIELD && !field_pic */
    bp.put((uint64_t)(sps_mbaff_capable(sps) && !field_pic), 1);
    bp.put((uint64_t)sps_direct_8x8(sps), 1);

    /* MVC extension — non-MVC streams pin a single-view config. */
    bp.put(1, 1);                                       /* mvc_extension_enable */
    bp.put(1, 2);                                       /* num_views_minus1 + 1 */
    bp.put(0, 10);                                      /* view_id[0] */
    bp.put(0, 10);                                      /* view_id[1] */
    bp.put(0, 1); bp.put(0, 10);                        /* num_anchor_refs_l0 */
    bp.put(0, 1); bp.put(0, 10);                        /* num_anchor_refs_l1 */
    bp.put(0, 1); bp.put(0, 10);                        /* num_non_anchor_refs_l0 */
    bp.put(0, 1); bp.put(0, 10);                        /* num_non_anchor_refs_l1 */
    bp.align(128);

    /* PPS section ---------------------------------------------------- */
    bp.put(0x1FFFu, 13);                                /* pps_id 8 | sps_id 5 = -1 */
    bp.put(has_flag(pps->flags, V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE), 1);
    bp.put(has_flag(pps->flags,
        V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT), 1);
    bp.put(pps->num_ref_idx_l0_default_active_minus1, 5);
    bp.put(pps->num_ref_idx_l1_default_active_minus1, 5);
    bp.put(has_flag(pps->flags, V4L2_H264_PPS_FLAG_WEIGHTED_PRED), 1);
    bp.put(pps->weighted_bipred_idc, 2);
    bp.put((uint8_t)pps->pic_init_qp_minus26, 7);       /* signed → low 7 bits */
    bp.put((uint8_t)pps->pic_init_qs_minus26, 6);
    bp.put((uint8_t)pps->chroma_qp_index_offset, 5);
    bp.put(has_flag(pps->flags,
        V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT), 1);
    bp.put(has_flag(pps->flags, V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED), 1);
    bp.put(has_flag(pps->flags, V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT), 1);
    bp.put(has_flag(pps->flags, V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE), 1);
    bp.put((uint8_t)pps->second_chroma_qp_index_offset, 5);
    bp.put(has_flag(pps->flags, V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT), 1);
    bp.put(0, 32);                                      /* scaling buffer addr placeholder */

    /* DPB tail (always re-emitted, lines 211..221 in source) -------- */
    uint32_t tail = 0;
    for (int i = 0; i < 16; i++) {
        int valid = dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID;
        int lt    = (valid && (dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)) ? 1 : 0;
        tail |= (uint32_t)lt << i;
        /* RefPicLayerIdList[i] is MVC voidx — non-MVC: 0 */
    }
    bp.put(tail, 32);
    bp.align(64);

    memcpy(out, buf, RKH264_SPSPPS_UNIT_SIZE);
}

/* ---- Frame RPS --------------------------------------------------- */

extern "C"
void H264PackFrameRps(uint8_t out[RKH264_RPS_SIZE],
                      uint32_t frame_num,
                      uint32_t log2_max_frame_num_minus4,
                      const v4l2_h264_dpb_entry *dpb,
                      const v4l2_h264_reference  ref_lists[3][32])
{
    /* 384 bytes = 48 u64 words. */
    uint64_t buf[48];
    BitPacker bp;
    bp.init(buf, 48);

    bp.align(128);   /* head pad — no-op at offset 0 */

    uint32_t max_frame_num = 1u << (log2_max_frame_num_minus4 + 4);

    /* frame_num_wrap[0..15] — 16 bits each. */
    for (int i = 0; i < 16; i++) {
        const v4l2_h264_dpb_entry &d = dpb[i];
        int valid = (d.flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) ? 1 : 0;
        uint16_t wrap;
        if (!valid) {
            wrap = 0;
        } else if (d.flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) {
            wrap = (uint16_t)d.frame_num;
        } else {
            wrap = (d.frame_num > frame_num)
                 ? (uint16_t)((int32_t)d.frame_num - (int32_t)max_frame_num)
                 : (uint16_t)d.frame_num;
        }
        bp.put(wrap, 16);
    }

    bp.put(0, 16);                              /* NULL pad */
    bp.put(0, 16);                              /* RefPicLayerIdList bitmap (non-MVC = 0) */

    /* RefPicList[0..2][0..31] — 7 bits each. */
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 32; i++) {
            const v4l2_h264_reference &r = ref_lists[j][i];
            int valid = (r.fields & V4L2_H264_FRAME_REF) ? 1 : 0;
            uint32_t dpb_idx     = valid ? (r.index & 0xF) : 0;
            uint32_t bottom_flag = valid && (r.fields == V4L2_H264_BOTTOM_FIELD_REF) ? 1 : 0;
            uint32_t entry = (dpb_idx | (uint32_t)(valid << 4)) & 0x1F;
            entry |= (bottom_flag & 0x1u) << 5;
            /* voidx (bit 6) — 0 for non-MVC */
            bp.put(entry, 7);
        }
    }
    bp.align(128);

    memcpy(out, buf, RKH264_RPS_SIZE);
}

/* ---- Scaling list ------------------------------------------------ */

extern "C"
void H264PackScalingList(uint8_t out[RKH264_SCALING_LIST_SIZE],
                         const v4l2_ctrl_h264_scaling_matrix *sm,
                         int enable)
{
    /* H.264 default ("flat") scaling lists: all factors = 16.  When the
     * stream has no PPS/SPS scaling matrix, the spec REQUIRES the
     * decoder to use these flat lists — using zeros makes the inverse-
     * quant produce zero residuals, which then causes the CABAC engine
     * to stall mid-frame because the predicted-vs-actual block content
     * diverges from the bitstream's coded MV/coeff stream.
     *
     * Stamp flat-16 first; overlay any explicit per-stream lists below. */
    memset(out, 16, RKH264_SCALING_LIST_SIZE);
    if (!enable || !sm) {
        return;
    }
    size_t off = 0;
    for (int i = 0; i < 6; i++) {                       /* 6 × 4x4 */
        memcpy(out + off, sm->scaling_list_4x4[i], 16);
        off += 16;
    }
    for (int i = 0; i < 2; i++) {                       /* 2 × 8x8 (Intra/Inter Y) */
        memcpy(out + off, sm->scaling_list_8x8[i], 64);
        off += 64;
    }
    /* off now == 6*16 + 2*64 == RKH264_SCALING_LIST_SIZE */
}

/* ---- RCB scratch sizing ----------------------------------------- */

/* RCB types in the BSP enum order is:
 *   0 RCB_INTRA_ROW   coeff 6   (uses width)
 *   1 RCB_TRANSD_ROW  coeff 1   (width)
 *   2 RCB_TRANSD_COL  coeff 1   (height)
 *   3 RCB_STRMD_ROW   coeff 3   (width)
 *   4 RCB_INTER_ROW   coeff 6   (width)
 *   5 RCB_INTER_COL   coeff 3   (height)
 *   6 RCB_DBLK_ROW    coeff 22  (width)
 *   7 RCB_SAO_ROW     coeff 6   (width)
 *   8 RCB_FBC_ROW     coeff 11  (width)
 *   9 RCB_FILT_COL    coeff 67  (height)
 *
 * vdpu34x_get_rcb_buf_size lays them out in a different order:
 *   DBLK_ROW, INTRA_ROW, TRANSD_ROW, STRMD_ROW, INTER_ROW, SAO_ROW,
 *   FBC_ROW, TRANSD_COL, INTER_COL, FILT_COL.
 *
 * We keep the BSP layout order to stay byte-compatible with what the
 * HW expects (the offsets land at the right register slots). */

#define RCB_ALIGN 64u

static inline uint32_t align_up(uint32_t x, uint32_t a) {
    return (x + a - 1u) & ~(a - 1u);
}

extern "C"
uint32_t H264GetRcbBufferSizes(H264RcbInfo info[RKH264_RCB_COUNT],
                               uint32_t width_px, uint32_t height_px)
{
    /* RCB sub-region sizes per the rk3588 BSP device tree
     * `rockchip,rcb-info` property (rk3588s.dtsi rkvdec0 node):
     *   <136 24576>, <137 49152>, <141 90112>, <140 49152>,
     *   <139 180224>, <133 49152>, <134 8192>, <135 4352>,
     *   <138 13056>, <142 291584>
     * These are FIXED sizes for any H.264 frame at any resolution that
     * the rk3588 codec supports (up to its rcb-min-width threshold of
     * 512px).  Our earlier formula (`coeff * frame_dim`) gave roughly
     * 1/6 of these sizes — codec wrote past the end of each sub-region
     * after ~16-25% of frame and faulted the IOMMU.
     *
     * Order in the DT property is meaningful: it's the LAYOUT ORDER
     * within the consolidated RCB buffer (largest first by BSP
     * convention).  We populate info[] by reg_idx so info[i] describes
     * the region for register 133+i. */
    (void)width_px; (void)height_px;

    struct RcbDesc { uint32_t reg_idx; uint32_t size; };
    static const RcbDesc kBspOrder[RKH264_RCB_COUNT] = {
        {136,  24576},  /* STRMD_ROW */
        {137,  49152},  /* INTER_ROW */
        {141,  90112},  /* FBC_ROW   */
        {140,  49152},  /* SAO_ROW   */
        {139, 180224},  /* DBLK_ROW  (largest contiguous) */
        {133,  49152},  /* INTRA_ROW */
        {134,   8192},  /* TRANSD_ROW */
        {135,   4352},  /* TRANSD_COL */
        {138,  13056},  /* INTER_COL */
        {142, 291584},  /* FILT_COL  */
    };

    uint32_t offset = 0;
    for (int i = 0; i < RKH264_RCB_COUNT; i++) {
        uint32_t size = align_up(kBspOrder[i].size, RCB_ALIGN);
        uint32_t slot = kBspOrder[i].reg_idx - 133u;
        info[slot].reg_idx = kBspOrder[i].reg_idx;
        info[slot].offset  = offset;
        info[slot].size    = size;
        offset += size;
    }
    return offset;  /* ~759 KiB total */
}
