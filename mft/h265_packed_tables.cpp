/* mft/h265_packed_tables.cpp — port of rockchip-linux/mpp packed-table
 * builders for rkvdec2 H.265 (vdpu34x v345).
 *
 * Cross-references (rockchip-linux mpp / mpp/hal/rkdec/h265d/...):
 *   hal_h265d_v345_output_pps_packet : hal_h265d_vdpu34x.c:219..438
 *     RK3568/RK3588 actually invoke the v345 packer (see line 1127:
 *     `if (reg_ctx->is_v34x) { hal_h265d_v345_output_pps_packet(...) }`).
 *     v345 emits a 112-byte slot, replicated 64x.  Layout differs from
 *     the older non-v345 packer (line 443) at three points:
 *       - after put(0,7), v345 emits put(sps_max_dec_pic_buffering_minus1,4)
 *         + put(0,3) before align(32,0xf) — non-v345 just aligns;
 *       - PPS-section tile cols/rows are gated by tiles_enabled_flag
 *         (non-v345 always emits num_tile_*_minus1 + 1);
 *       - mode field is 4 bits of zero (non-v345 = 2 bits of 3);
 *       - tail after tile widths/heights is put(0,32) + put(0,70) +
 *         align(64,0xf) (non-v345 just put(0,32) + align(64,0xf)).
 *   hal_h265d_slice_hw_rps          : hal_h265d_com.c:300..360
 *     32 LT entries (32 bits each: 16 lsb + 1 used + 15 pad) followed
 *     by 64 STRPS slots (64 bytes each, 64-bit aligned then padded).
 *   hal_h265d_output_scalinglist_packet
 *                                  : hal_h265d_com.c:687..720
 *   hal_record_scaling_list         : hal_h265d_com.c:84..220
 *     1248-byte scalingfactor0 + 96-byte scalingfactor1 (4x4 rotated)
 *     + 12-byte scalingdc + 4-byte reserved.  Each per-matrix block
 *     gets transposed in-place after the linear copy.
 *
 * Buffer geometry (hal_h265d_vdpu34x.c:88..99):
 *   PPS_SIZE          = 112 * 64 = 7168 bytes (slot=80, replicated 64x)
 *   RPS_ALIGEND_SIZE  = ALIGN(400 * 8, 4K) = 4096 bytes (payload 3200)
 *   SCALING_LIST_SIZE = 81 * 1360 = 110160 bytes (one slot used = 1360)
 *
 * Portions Copyright (c) Rockchip Electronics Co., Ltd., licensed under
 * Apache-2.0 OR MIT — the algorithms below (BitPacker, packet emitters,
 * RCB sizing) are re-implementations of the cited rockchip-linux/mpp
 * routines.  See http://www.apache.org/licenses/LICENSE-2.0.
 * The verbatim CABAC table data is in h265_cabac_init.inc.
 *
 * SPDX-License-Identifier: (BSD-2-Clause-Patent AND Apache-2.0)
 */

#include "h265_packed_tables.h"
#include <string.h>

/* HEVC CABAC init table — verbatim from BSP. */
#include "h265_cabac_init.inc"

extern "C" const uint8_t *H265GetCabacInitTable(size_t *out_byte_count) {
    if (out_byte_count) *out_byte_count = sizeof(kRkH265CabacInit);
    return kRkH265CabacInit;
}

namespace {

/* ============================================================ BitPacker
 *
 * Mirrors MPP's BitputCtx_t / mpp_put_bits / mpp_put_align: writes are
 * LSB-first into a u64 stream — same packer as h264_packed_tables.cpp.
 * `align(n, fill)` pads with `fill` to the next multiple of n bits;
 * the BSP uses fill=0xf in the PPS path (all-ones into reserved bits)
 * and fill=0 in the RPS path. */
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

    /* Advance the bit cursor by `nbits` without writing anything.
     * Caller must guarantee the underlying buffer was zero-initialized
     * (init() does that), so skipping the writes leaves zeros in place. */
    void skip(int nbits) {
        if (nbits == 0) return;
        size_t total = word_idx * 64 + (size_t)bit_in_word + (size_t)nbits;
        word_idx    = total / 64;
        bit_in_word = (int)(total & 63);
        if (word_idx > cap_u64) word_idx = cap_u64;
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

    /* Pad with `fill` bits (1-bit value, replicated) to next n_bits multiple. */
    void align(int n_bits, int fill) {
        size_t total = word_idx * 64 + (size_t)bit_in_word;
        size_t aligned = (total + (size_t)n_bits - 1) & ~((size_t)n_bits - 1);
        size_t pad = aligned - total;
        uint64_t fill_pat = fill ? ~0ULL : 0ULL;
        while (pad > 0) {
            int chunk = (pad > 32) ? 32 : (int)pad;
            put(fill_pat, chunk);
            pad -= chunk;
        }
    }
};

/* ============================================================ Helpers */

static inline int sps_field_bit_depth(uint8_t v8) {
    /* SPS bit_depth_luma_minus8 → bit_depth (8..14). */
    return (int)v8 + 8;
}

static inline uint32_t derive_pic_width_px(const H265Sps *sps) {
    return sps->pic_width_in_luma_samples;
}
static inline uint32_t derive_pic_height_px(const H265Sps *sps) {
    return sps->pic_height_in_luma_samples;
}

} /* anon */

/* ===================================================================== *
 * H265PackPPS
 *
 * Layout (bit-position labels are running totals after each field, to
 * match the BSP comments at hal_h265d_vdpu34x.c:497..504).  Field
 * names/widths are byte-for-byte copies of the BSP mpp_put_bits chain.
 * ===================================================================== */
extern "C"
int H265PackPPS(const H265Vps *vps, const H265Sps *sps, const H265Pps *pps,
                uint8_t *out, size_t out_size)
{
    if (!sps || !pps || !out) return -1;
    if (out_size < RKH265_SPSPPS_UNIT_SIZE) return -1;

    /* The v345 BSP packs into a 14 u64 = 112 byte scratch then memcpys
     * it 64 times into the destination DMA buffer (one row per pps_id
     * slot in the codec's lookup table).  We mirror that. */
    uint64_t pps_packet[14];
    BitPacker bp;
    bp.init(pps_packet, 14);

    /* ---- SPS section ------------------------------------------------ */
    bp.put(vps ? vps->vps_id : 0u, 4);
    bp.put(sps->sps_id,            4);
    bp.put(sps->chroma_format_idc, 2);

    uint32_t width  = derive_pic_width_px(sps);
    uint32_t height = derive_pic_height_px(sps);
    bp.put(width,  16);
    bp.put(height, 16);
    bp.put(sps_field_bit_depth(sps->bit_depth_luma_minus8),   4);
    bp.put(sps_field_bit_depth(sps->bit_depth_chroma_minus8), 4);
    bp.put((uint64_t)sps->log2_max_pic_order_cnt_lsb_minus4 + 4, 5);
    bp.put(sps->log2_diff_max_min_luma_coding_block_size, 2);
    bp.put((uint64_t)sps->log2_min_luma_coding_block_size_minus3 + 3, 3);
    bp.put((uint64_t)sps->log2_min_luma_transform_block_size_minus2 + 2, 3);
    /* zrh comment: 57 bits above */

    bp.put(sps->log2_diff_max_min_luma_transform_block_size, 2);
    bp.put(sps->max_transform_hierarchy_depth_inter, 3);
    bp.put(sps->max_transform_hierarchy_depth_intra, 3);
    bp.put(sps->scaling_list_enabled_flag, 1);
    bp.put(sps->amp_enabled_flag, 1);
    bp.put(sps->sample_adaptive_offset_enabled_flag, 1);
    /* 68 bits above */

    bp.put(sps->pcm_enabled_flag, 1);
    bp.put(sps->pcm_enabled_flag ? (uint64_t)sps->pcm_sample_bit_depth_luma_minus1 + 1 : 0, 4);
    bp.put(sps->pcm_enabled_flag ? (uint64_t)sps->pcm_sample_bit_depth_chroma_minus1 + 1 : 0, 4);
    bp.put(sps->pcm_loop_filter_disabled_flag, 1);
    bp.put(sps->log2_diff_max_min_pcm_luma_coding_block_size, 3);
    bp.put(sps->pcm_enabled_flag ? (uint64_t)sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 : 0, 3);

    bp.put(sps->num_short_term_ref_pic_sets, 7);
    bp.put(sps->long_term_ref_pics_present_flag, 1);
    bp.put(sps->num_long_term_ref_pics_sps, 6);
    bp.put(sps->sps_temporal_mvp_enabled_flag, 1);
    bp.put(sps->strong_intra_smoothing_enabled_flag, 1);
    /* 100 bits above (per BSP comment; actual is 106 — comment is stale) */

    /* v345-specific: after put(0,7) the BSP emits a 4-bit
     * sps_max_dec_pic_buffering_minus1 + 3 zero bits before the align,
     * mapped in by `yandong change` at hal_h265d_vdpu34x.c:286..289.
     * The DXVA `pp.sps_max_dec_pic_buffering_minus1` is a scalar; the
     * BSP fills it from the highest-sublayer entry of the array.  We
     * mirror that by reading the entry indexed by sps_max_sub_layers_minus1. */
    bp.put(0, 7);
    {
        unsigned dpb_idx = sps->sps_max_sub_layers_minus1;
        if (dpb_idx >= H265_MAX_SUB_LAYERS) dpb_idx = H265_MAX_SUB_LAYERS - 1;
        bp.put((uint64_t)sps->sps_max_dec_pic_buffering_minus1[dpb_idx], 4);
    }
    bp.put(0, 3);
    bp.align(32, /*fill=*/1);   /* fill=0xf in BSP — pad to 32 with 1s */

    /* ---- PPS section ------------------------------------------------ */
    bp.put(pps->pps_id, 6);
    bp.put(pps->sps_id, 4);
    bp.put(pps->dependent_slice_segments_enabled_flag, 1);
    bp.put(pps->output_flag_present_flag, 1);
    bp.put(pps->num_extra_slice_header_bits, 13);
    bp.put(pps->sign_data_hiding_enabled_flag, 1);
    bp.put(pps->cabac_init_present_flag, 1);
    bp.put((uint64_t)pps->num_ref_idx_l0_default_active_minus1 + 1, 4);
    bp.put((uint64_t)pps->num_ref_idx_l1_default_active_minus1 + 1, 4);
    bp.put((uint8_t)pps->init_qp_minus26, 7);                /* signed → low 7 bits */
    bp.put(pps->constrained_intra_pred_flag, 1);
    bp.put(pps->transform_skip_enabled_flag, 1);
    bp.put(pps->cu_qp_delta_enabled_flag, 1);

    /* The BSP encodes log2_min_cb + log2_diff_max_min - diff_cu_qp_delta
     * (effectively the log2 of the QG size). */
    int log2_min_cb_size = (int)sps->log2_min_luma_coding_block_size_minus3 + 3;
    int log2_qg = log2_min_cb_size +
                  (int)sps->log2_diff_max_min_luma_coding_block_size -
                  (int)pps->diff_cu_qp_delta_depth;
    if (log2_qg < 0) log2_qg = 0;
    if (log2_qg > 7) log2_qg = 7;
    bp.put((uint64_t)log2_qg, 3);

    bp.put((uint8_t)pps->pps_cb_qp_offset, 5);
    bp.put((uint8_t)pps->pps_cr_qp_offset, 5);
    bp.put(pps->pps_slice_chroma_qp_offsets_present_flag, 1);
    bp.put(pps->weighted_pred_flag, 1);
    bp.put(pps->weighted_bipred_flag, 1);
    bp.put(pps->transquant_bypass_enabled_flag, 1);
    bp.put(pps->tiles_enabled_flag, 1);
    bp.put(pps->entropy_coding_sync_enabled_flag, 1);
    bp.put(pps->pps_loop_filter_across_slices_enabled_flag, 1);
    bp.put(pps->loop_filter_across_tiles_enabled_flag, 1);

    bp.put(pps->deblocking_filter_override_enabled_flag, 1);
    bp.put(pps->pps_deblocking_filter_disabled_flag, 1);
    bp.put((uint8_t)pps->pps_beta_offset_div2, 4);
    bp.put((uint8_t)pps->pps_tc_offset_div2, 4);
    bp.put(pps->lists_modification_present_flag, 1);
    bp.put((uint64_t)pps->log2_parallel_merge_level_minus2 + 2, 3);
    bp.put(pps->slice_segment_header_extension_present_flag, 1);
    bp.put(0, 3);
    /* v345 gates these on tiles_enabled_flag (else emits 0). */
    if (pps->tiles_enabled_flag) {
        bp.put((uint64_t)pps->num_tile_columns_minus1 + 1, 5);
        bp.put((uint64_t)pps->num_tile_rows_minus1 + 1, 5);
    } else {
        bp.put(0, 5);
        bp.put(0, 5);
    }
    bp.put(0, 4);   /* v345: 4-bit mode = 0 (non-v345 packer used 2 bits = 3) */
    bp.align(64, /*fill=*/1);                                  /* BSP fill=0xf */

    /* ---- Tile info (column_width[20] + row_height[22], 12 bits each) -
     *
     * For tile-free streams (our test corpus) the BSP collapses into:
     *   column_width[0] = ceil(width / MaxCUWidth) - 1
     *   row_height[0]   = ceil(height / MaxCUWidth) - 1
     * with the rest zero.  Tile-aware streams fill the per-column /
     * per-row CTB widths from PPS column_width_minus1/row_height_minus1
     * (not yet plumbed in our parser — TODO when tile streams ship). */
    {
        uint16_t column_width[20] = {0};
        uint16_t row_height[22] = {0};

        int log2_diff = (int)sps->log2_diff_max_min_luma_coding_block_size;
        int max_cu_width = 1 << (log2_diff + log2_min_cb_size);
        if (!pps->tiles_enabled_flag) {
            column_width[0] = (uint16_t)((width  + max_cu_width - 1) / max_cu_width);
            row_height[0]   = (uint16_t)((height + max_cu_width - 1) / max_cu_width);
        } else {
            /* Uniform-spacing fallback (parser captures uniform_spacing_flag
             * but not the per-tile width tables; treat as uniform until
             * task 2 plumbs the tables through). */
            int n_cols = (int)pps->num_tile_columns_minus1 + 1;
            int n_rows = (int)pps->num_tile_rows_minus1 + 1;
            int pic_in_ctb_w = (width  + (1 << (log2_min_cb_size + log2_diff)) - 1)
                              / (1 << (log2_min_cb_size + log2_diff));
            int pic_in_ctb_h = (height + (1 << (log2_min_cb_size + log2_diff)) - 1)
                              / (1 << (log2_min_cb_size + log2_diff));
            for (int i = 0; i < n_cols && i < 20; i++)
                column_width[i] = (uint16_t)(((i + 1) * pic_in_ctb_w) / n_cols
                                           - (i       * pic_in_ctb_w) / n_cols);
            for (int i = 0; i < n_rows && i < 22; i++)
                row_height[i] = (uint16_t)(((i + 1) * pic_in_ctb_h) / n_rows
                                          - (i       * pic_in_ctb_h) / n_rows);
        }

        for (int j = 0; j < 20; j++) {
            uint16_t v = column_width[j] ? (uint16_t)(column_width[j] - 1) : 0u;
            bp.put(v, 12);
        }
        for (int j = 0; j < 22; j++) {
            uint16_t v = row_height[j] ? (uint16_t)(row_height[j] - 1) : 0u;
            bp.put(v, 12);
        }
    }

    /* ---- Tail: 32-bit zero (legacy scaling-list-addr slot, kept zero
     * by v345 since it programs reg180 directly), then a 70-bit zero
     * pad, then align(64, fill=1).  See hal_h265d_vdpu34x.c:401..405:
     *   mpp_put_bits(&bp, 0, 32);
     *   mpp_put_bits(&bp, 0, 70);
     *   mpp_put_align(&bp, 64, 0xf);
     *
     * Total fixed-size payload = 14 u64 = 112 bytes / slot. */
    bp.put(0, 32);
    bp.put(0, 32);
    bp.put(0, 32);
    bp.put(0, 6);    /* 32+32+6 = 70 bits */
    bp.align(64, /*fill=*/1);

    /* Replicate the 112-byte slot 64 times — `for (i = 0; i < 64; i++)
     * memcpy(pps_ptr + i * 112, pps_buf, 112);` (hal_h265d_vdpu34x.c:432). */
    const uint8_t *src = reinterpret_cast<const uint8_t *>(pps_packet);
    for (uint32_t i = 0; i < RKH265_SPSPPS_NUM_SLOTS; i++) {
        memcpy(out + i * RKH265_SPSPPS_SLOT_SIZE, src, RKH265_SPSPPS_SLOT_SIZE);
    }
    return (int)RKH265_SPSPPS_UNIT_SIZE;
}

/* ===================================================================== *
 * H265PackRPS — SPS-side LT + STRPS table.
 *
 * Layout (per hal_h265d_slice_hw_rps, hal_h265d_com.c:300..360):
 *
 *   [0 .. 32 entries]
 *       u16  lt_ref_pic_poc_lsb_sps[i]      (16 bits)
 *       u1   used_by_curr_pic_lt_sps_flag[i] (1 bit)
 *       u15  pad                             (15 bits)
 *     → 32 bits per LT entry × 32 entries = 128 bytes
 *
 *   [32 .. 32+64 entries]  one STRPS slot every 64-bit-aligned chunk:
 *       u4   num_negative_pics
 *       u4   num_positive_pics
 *       for j in 0..num_negative_pics-1:
 *           u16 delta_poc_s0[j]
 *           u1  s0_used_flag[j]
 *       for j in 0..num_positive_pics-1:
 *           u16 delta_poc_s1[j]
 *           u1  s1_used_flag[j]
 *       for j in (neg+pos)..14:
 *           u16 zero
 *           u1  zero
 *       align(64, fill=0)
 *       u64  zero (BSP put 0,64 trailer)
 *     → 64 bytes per STRPS slot × 64 slots = 4096 bytes
 *
 * Wait — 128 + 4096 > 3200.  The BSP `fifo_len = 400` u64 = 3200 bytes;
 * the per-STRPS slot actually packs into 48 bytes: 8 (header) + 15*17 =
 * 263 bits, padded to 64 bits = 320 bits = 40 bytes, plus the 64-bit
 * trailer = 48 bytes.  32 LT × 4 bytes = 128, 64 STRPS × 48 = 3072,
 * total = 3200 bytes ✓.  We let the BitPacker handle the alignment.
 * ===================================================================== */
extern "C"
int H265PackRPS(const H265ParseResult *parsed, uint8_t *out, size_t out_size)
{
    if (!parsed || !out) return -1;
    if (out_size < RKH265_RPS_SIZE) return -1;

    /* fifo_len = 400 u64 = 3200 bytes (hal_h265d_com.c:303). */
    uint64_t buf[400];
    BitPacker bp;
    bp.init(buf, 400);

    /* Pull the active SPS — task 4 guarantees active_sps_id is valid by
     * the time the regbuilder calls us.  For safety, fall back to all
     * zeros when no SPS has been parsed yet. */
    const H265Sps *sps = nullptr;
    if (parsed->active_sps_id >= 0 && parsed->active_sps_id < H265_MAX_SPS) {
        if (parsed->sps[parsed->active_sps_id].valid)
            sps = &parsed->sps[parsed->active_sps_id];
    }

    /* ---- 32 LT entries (BSP loops i=0..31 unconditionally) -----------
     * 32 bits per entry; buffer is pre-zeroed.  When LT refs aren't
     * present at all (the common case) skip the whole 1024-bit block. */
    if (sps && sps->long_term_ref_pics_present_flag &&
        sps->num_long_term_ref_pics_sps > 0) {
        for (int i = 0; i < 32; i++) {
            if (i < sps->num_long_term_ref_pics_sps) {
                bp.put(sps->lt_ref_pic_poc_lsb_sps[i],   16);
                bp.put(sps->used_by_curr_pic_lt_sps_flag[i], 1);
                bp.skip(15);
            } else {
                bp.skip(32);
            }
        }
    } else {
        bp.skip(32 * 32);
    }

    /* ---- 64 STRPS slots --------------------------------------------- */
    int n_strps = sps ? (int)sps->num_short_term_ref_pic_sets : 0;
    if (n_strps > H265_MAX_SHORT_TERM_RPS) n_strps = H265_MAX_SHORT_TERM_RPS;
    if (n_strps > 64) n_strps = 64;

    for (int i = 0; i < 64; i++) {
        if (i < n_strps && sps) {
            const H265ShortTermRPS &r = sps->st_rps[i];
            bp.put(r.num_negative_pics, 4);
            bp.put(r.num_positive_pics, 4);

            int neg = r.num_negative_pics;
            int pos = r.num_positive_pics;
            if (neg > 15) neg = 15;
            if (pos > 15 - neg) pos = 15 - neg;

            /* The parser stores delta_poc[] as: [0..neg-1] = negative
             * deltas (DESCENDING POC), [neg..neg+pos-1] = positive
             * deltas.  The BSP wants raw 16-bit signed deltas in the
             * same neg-then-pos order. */
            for (int j = 0; j < neg; j++) {
                bp.put((uint16_t)r.delta_poc[j], 16);
                bp.put(r.used_by_curr_pic_flag[j], 1);
            }
            for (int j = 0; j < pos; j++) {
                bp.put((uint16_t)r.delta_poc[neg + j], 16);
                bp.put(r.used_by_curr_pic_flag[neg + j], 1);
            }
            /* Pad up to 15 entries with zeros. */
            for (int j = neg + pos; j < 15; j++) {
                bp.put(0, 16);
                bp.put(0, 1);
            }
        } else {
            /* Empty slot: 4+4 + 15*17 = 263 bits all zero.  Buffer is
             * pre-zeroed by init(), so just advance the cursor. */
            bp.skip(263);
        }
        bp.align(64, /*fill=*/0);
        /* 64-bit zero trailer — also a skip since buffer is pre-zeroed. */
        bp.skip(64);
    }

    memcpy(out, buf, RKH265_RPS_SIZE);
    return (int)RKH265_RPS_SIZE;
}

/* ===================================================================== *
 * H265PackScalingList — 1360-byte scalingFactor_t blob.
 *
 * Layout (hal_h265d_ctx.h:97):
 *   uint8_t scalingfactor0[1248]  -- 6*16 + 6*64 + 6*64 + 6*64 + pad
 *   uint8_t scalingfactor1[96]    -- 6 x 4x4 transposed (rotated 90deg)
 *   uint8_t scalingdc[12]         -- DC coeffs for sizeID=2 (6) and 3 (6)
 *   uint8_t reserved[4]
 *
 * The BSP source first appends per-sizeId blocks linearly, then
 * post-processes by transposing each 4×4 (sizeId 0) and 8×8 (sizeId 1,
 * 2, 3) matrix in-place ("rotated 90°", per the source comment).
 *
 * Our parser stores the matrices in RASTER order, so we can write
 * directly without the diag-scan unscramble that the BSP does on its
 * DXVA inputs (hal_h265d_com.c:692..706).
 * ===================================================================== */
namespace {

/* Per-sizeId list count (hal_record_scaling_list line 88). */
static const uint32_t kListsPerSize[4] = { 6, 6, 6, 2 };

static void transpose_inplace(uint8_t *p, int dim) {
    uint8_t tmp[64];
    for (int i = 0; i < dim; i++)
        for (int k = 0; k < dim; k++)
            tmp[i * dim + k] = p[k * dim + i];
    memcpy(p, tmp, (size_t)dim * dim);
}

} /* anon */

extern "C"
int H265PackScalingList(const H265Sps *sps, const H265Pps *pps,
                        uint8_t *out, size_t out_size)
{
    if (!out) return -1;
    if (out_size < RKH265_SCALING_LIST_SIZE) return -1;

    /* Pick the active list per BSP rule (hal_h265d_vdpu34x.c:621..627):
     *   - PPS list overrides SPS list when pps_scaling_list_data_present
     *   - else SPS list when sps_scaling_list_data_present
     *   - else default flat-16 (parser stamps these on construction). */
    const H265ScalingList *sl = nullptr;
    if (pps && pps->pps_scaling_list_data_present_flag)
        sl = &pps->scaling_list;
    else if (sps && sps->sps_scaling_list_data_present_flag)
        sl = &sps->scaling_list;
    else if (sps)
        sl = &sps->scaling_list;   /* parser pre-fills with flat-16 */

    memset(out, 0, RKH265_SCALING_LIST_SIZE);
    if (!sl) return (int)RKH265_SCALING_LIST_SIZE;   /* all-zero == flat */

    /* ---- scalingfactor0 (1248 bytes) --------------------------------- *
     *
     * Layout offsets (running):
     *   [   0..  96)  6 × 16 bytes   sizeId=0 (4×4)
     *   [  96.. 480)  6 × 64 bytes   sizeId=1 (8×8)
     *   [ 480.. 864)  6 × 64 bytes   sizeId=2 (16×16, sub-sampled to 8×8)
     *   [ 864..1248)  6 × 64 bytes   sizeId=3 (32×32, sub-sampled; only
     *                 [0],[3] mandatory but BSP zero-fills tail to 384) */
    uint32_t off = 0;
    /* sizeId 0 (4×4) — 6 lists × 16 coeffs each. */
    for (uint32_t m = 0; m < kListsPerSize[0]; m++) {
        memcpy(out + off, sl->scaling_list_4x4[m], 16);
        off += 16;
    }
    /* sizeId 1 (8×8) — 6 lists × 64 coeffs. */
    for (uint32_t m = 0; m < kListsPerSize[1]; m++) {
        memcpy(out + off, sl->scaling_list_8x8[m], 64);
        off += 64;
    }
    /* sizeId 2 (16×16, sub-sampled) — 6 lists × 64 coeffs. */
    for (uint32_t m = 0; m < kListsPerSize[2]; m++) {
        memcpy(out + off, sl->scaling_list_16x16[m], 64);
        off += 64;
    }
    /* sizeId 3 (32×32, sub-sampled) — 2 mandatory lists, padded to 6
     * with 64 bytes zero-fill (BSP loops `for (i=0;i<128;i++) out[n++]=0`
     * for each missing list per hal_h265d_com.c:124..127). */
    for (uint32_t m = 0; m < 2; m++) {
        memcpy(out + off, sl->scaling_list_32x32[m], 64);
        off += 64;
    }
    for (uint32_t m = 2; m < 6; m++) {
        memset(out + off, 0, 64);
        off += 64;
    }
    /* off now == 1248. */

    /* ---- scalingfactor1 (96 bytes) — 6 × 4×4 column-rotated copy ----- *
     *
     * BSP code (hal_h265d_com.c:135..147): write the 4×4 matrix four
     * times "column-major", i.e. each dst row j gets the source's
     * column j: dst[j*4+i] = src[i*4+j].  This is a transpose. */
    for (uint32_t m = 0; m < kListsPerSize[0]; m++) {
        const uint8_t *src = sl->scaling_list_4x4[m];
        uint8_t *dst = out + 1248 + m * 16;
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 4; k++) {
                /* hal_h265d_com.c: temp16[j] = sl[0][m][j], then
                 *   sf1[n++] = temp16[i],
                 *   sf1[n++] = temp16[i+4],
                 *   sf1[n++] = temp16[i+8],
                 *   sf1[n++] = temp16[i+12];
                 * for i in 0..3.  i.e. sf1[i*4+k] = sl[0][m][i + k*4]. */
                dst[i * 4 + k] = src[i + k * 4];
            }
        }
    }

    /* ---- scalingdc (12 bytes) — DC coeffs for sizeId 2 / 3 ---------- *
     *
     * BSP layout (hal_h265d_com.c:149..160):
     *   dc[0..5]   = scalingListDCCoefSizeID2[i]   for i in 0..5
     *   dc[6..11]  = pairs of (sl_dc[1][i], 0, 0)  for i in 0..1
     * → 6 + 2*3 = 12 bytes. */
    uint8_t *dc = out + 1248 + 96;
    for (uint32_t m = 0; m < kListsPerSize[2]; m++)
        dc[m] = sl->scaling_list_dc_16x16[m];
    /* sizeId=3 DC coeffs are followed by two zero-pad bytes each. */
    {
        uint32_t n = 6;
        for (uint32_t m = 0; m < kListsPerSize[3]; m++) {
            dc[n++] = sl->scaling_list_dc_32x32[m];
            dc[n++] = 0;
            dc[n++] = 0;
        }
    }
    /* reserved[4] already zero from memset above. */

    /* ---- Per-matrix transpose pass (BSP "rotated 90°" comment) ------- *
     *
     * The BSP runs a transpose AFTER the linear copies above
     * (hal_h265d_com.c:165..220).  We replicate it bit-for-bit. */

    /* sizeId 0 → 4×4 transpose, scalingfactor0[m * 16]. */
    for (uint32_t m = 0; m < kListsPerSize[0]; m++) {
        transpose_inplace(out + m * 16, 4);
    }
    /* sizeId 1 → 8×8 transpose, scalingfactor0[6*16 + m*64]. */
    for (uint32_t m = 0; m < kListsPerSize[1]; m++) {
        transpose_inplace(out + 96 + m * 64, 8);
    }
    /* sizeId 2 → 8×8 transpose, scalingfactor0[6*16 + 6*64 + m*64]. */
    for (uint32_t m = 0; m < kListsPerSize[2]; m++) {
        transpose_inplace(out + 480 + m * 64, 8);
    }
    /* sizeId 3 → 8×8 transpose, scalingfactor0[6*16 + 6*64 + 6*64 + m*64].
     * BSP loops m=0..5 even though only m=0,1 carry valid data — the
     * tail 4 lists are zero-filled, transpose-of-zero is zero. */
    for (uint32_t m = 0; m < 6; m++) {
        transpose_inplace(out + 864 + m * 64, 8);
    }
    /* scalingfactor1 — 4×4 transpose, m * 16. */
    for (uint32_t m = 0; m < kListsPerSize[0]; m++) {
        transpose_inplace(out + 1248 + m * 16, 4);
    }

    return (int)RKH265_SCALING_LIST_SIZE;
}
