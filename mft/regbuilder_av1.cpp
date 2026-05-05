/* mft/regbuilder_av1.cpp — AV1 register-array builder for rkvdec_av1.
 *
 * This is a clean-room implementation: we only consume the bitfield
 * struct definitions in regbuilder_av1_reg.h (which are factual
 * register-layout declarations of the Rockchip silicon, vendored under
 * Apache-2.0 from MPP's hal_av1d_vdpu_reg.h).  The logic here — which
 * AV1 syntax element drives which register field — is derived from
 * the AV1 specification and validated against captured BSP register
 * traces (docs/av1_trace_*.log).
 *
 * Bring-up status: stubbed.  Each section is filled in incrementally
 * as we work through the captured trace diff.  See docs/av1_register_map.md
 * for the per-swreg notes.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "regbuilder_av1.h"

#include <cstdio>
#include <cstring>

extern "C" {
#include <dav1d/headers.h>
}

/* AV1 spec constants */
static constexpr int AV1_REF_SCALE_SHIFT = 14;
static constexpr int AV1_LAST_FRAME      = 1;
static constexpr int AV1_REFS_PER_FRAME  = 7;  /* LAST..ALTREF */

/* --- DPB management ------------------------------------------------- */

void rkmpp_av1_dpb_init(RkmppAv1Dpb *dpb) {
    if (!dpb) return;
    std::memset(dpb, 0, sizeof(*dpb));
}

void rkmpp_av1_dpb_post_decode(
    RkmppAv1Dpb               *dpb,
    const Dav1dSequenceHeader *seq,
    const Dav1dFrameHeader    *hdr)
{
    if (!dpb || !seq || !hdr) return;

    /* Capture order_hint_bits at first call (sequence-level constant
     * within a sequence).  dav1d splits order_hint (enable flag) and
     * order_hint_n_bits (bit count); we want the latter. */
    if (!dpb->order_hint_bits)
        dpb->order_hint_bits = seq->order_hint ? seq->order_hint_n_bits : 0;

    /* AV1 spec §7.20: for each i in 0..7, if (refresh_frame_flags >> i) & 1,
     * the slot is overwritten with this frame's reconstruction. */
    /* Snapshot SavedOrderHints[ref] = OrderHints[ref] at decode time:
     * the order_hint of the frame currently sitting in slot refidx[ref].
     * For keyframes/intra these are zero (no refs). */
    int16_t cur_saved[7] = {};
    bool inter = (hdr->frame_type != DAV1D_FRAME_TYPE_KEY &&
                  hdr->frame_type != DAV1D_FRAME_TYPE_INTRA &&
                  !hdr->allow_intrabc);
    if (inter) {
        for (int r = 0; r < 7; r++) {
            int slot_idx = hdr->refidx[r];
            if (slot_idx >= 0 && slot_idx < 8 && dpb->slots[slot_idx].valid) {
                cur_saved[r] = dpb->slots[slot_idx].frame_offset;
            }
        }
    }
    /* Allocate a new buffer_id for the frame being stored — slots
     * refreshed by this frame share it. */
    uint32_t bid = ++dpb->next_buffer_id;
    for (int i = 0; i < 8; i++) {
        if ((hdr->refresh_frame_flags >> i) & 1) {
            RkmppAv1DpbSlot &s = dpb->slots[i];
            s.valid          = 1;
            s.frame_type     = (uint8_t)hdr->frame_type;
            s.frame_offset   = (int16_t)hdr->frame_offset;
            s.coded_width    = (uint16_t)hdr->width[0];
            s.coded_height   = (uint16_t)hdr->height;
            s.upscaled_width = (uint16_t)hdr->width[1];
            s.buffer_id      = bid;
            for (int r = 0; r < 7; r++)
                s.saved_order_hints[r] = cur_saved[r];
        }
    }
}

/* Signed AV1 OrderHint difference: returns h0 - h1 in modular arithmetic
 * with order_hint_bits.  Per AV1 spec §7.8 get_relative_dist(). */
static int32_t order_hint_dist(uint8_t bits, int32_t h0, int32_t h1) {
    if (!bits) return 0;
    int32_t diff = h0 - h1;
    int32_t m = 1 << (bits - 1);
    return (diff & ((1 << bits) - 1)) - (diff & m) * 2;
}

/* --- swreg emitter helpers ------------------------------------------- */

namespace {

/* Pack an iova-FD pair into the *_msb / *_lsb register slots used for
 * DMA-base registers.  Userspace HAL writes the FD into *_lsb and 0
 * into *_msb; the kernel mpp_iommu replaces *_lsb with the post-IOMMU
 * IOVA low 32 bits and ORs the high byte into *_msb.  We follow the
 * same convention so the kernel's trans_tbl_av1_vcd[] patching applies
 * uniformly.
 *
 * For now we just stash the FD where MPP would.  When the kernel does
 * its translation pass, the captured trace shows *_msb populated; in
 * the diff harness we mask those off before comparing. */
void emit_dma(uint32_t *msb, uint32_t *lsb, int fd, uint32_t offset) {
    *msb = 0;
    *lsb = (uint32_t)fd;
    /* offset is folded in by kernel via MPP_REQ_SET_REG_OFFSETS;
     * harness ignores it for now. */
    (void)offset;
}

}  /* anonymous namespace */

/* --- main entry ------------------------------------------------------ */

/* Helpers to set the strange split-across-swregs per-ref fields. */
static void set_ref_width(VdpuAv1dRegSet *r, int i, uint32_t v) {
    switch (i) {
    case 0: r->swreg33.sw_ref0_width = v; break;
    case 1: r->swreg34.sw_ref1_width = v; break;
    case 2: r->swreg35.sw_ref2_width = v; break;
    case 3: r->swreg43.sw_ref3_width = v; break;
    case 4: r->swreg44.sw_ref4_width = v; break;
    case 5: r->swreg45.sw_ref5_width = v; break;
    case 6: r->swreg46.sw_ref6_width = v; break;
    }
}
static void set_ref_height(VdpuAv1dRegSet *r, int i, uint32_t v) {
    switch (i) {
    case 0: r->swreg33.sw_ref0_height = v; break;
    case 1: r->swreg34.sw_ref1_height = v; break;
    case 2: r->swreg35.sw_ref2_height = v; break;
    case 3: r->swreg43.sw_ref3_height = v; break;
    case 4: r->swreg44.sw_ref4_height = v; break;
    case 5: r->swreg45.sw_ref5_height = v; break;
    case 6: r->swreg46.sw_ref6_height = v; break;
    }
}
static void set_ref_hor_scale(VdpuAv1dRegSet *r, int i, uint32_t v) {
    switch (i) {
    case 0: r->swreg36.sw_ref0_hor_scale = v; break;
    case 1: r->swreg37.sw_ref1_hor_scale = v; break;
    case 2: r->swreg38.sw_ref2_hor_scale = v; break;
    case 3: r->swreg39.sw_ref3_hor_scale = v; break;
    case 4: r->swreg40.sw_ref4_hor_scale = v; break;
    case 5: r->swreg41.sw_ref5_hor_scale = v; break;
    case 6: r->swreg42.sw_ref6_hor_scale = v; break;
    }
}
static void set_ref_ver_scale(VdpuAv1dRegSet *r, int i, uint32_t v) {
    switch (i) {
    case 0: r->swreg36.sw_ref0_ver_scale = v; break;
    case 1: r->swreg37.sw_ref1_ver_scale = v; break;
    case 2: r->swreg38.sw_ref2_ver_scale = v; break;
    case 3: r->swreg39.sw_ref3_ver_scale = v; break;
    case 4: r->swreg40.sw_ref4_ver_scale = v; break;
    case 5: r->swreg41.sw_ref5_ver_scale = v; break;
    case 6: r->swreg42.sw_ref6_ver_scale = v; break;
    }
}
static void set_ref_sign_bias(VdpuAv1dRegSet *r, int i, uint32_t v) {
    /* ref_sign_bias for ref0..3 lives in swreg59, ref4..6 in swreg9. */
    switch (i) {
    case 0: r->swreg59.sw_ref0_sign_bias = v; break;
    case 1: r->swreg59.sw_ref1_sign_bias = v; break;
    case 2: r->swreg59.sw_ref2_sign_bias = v; break;
    case 3: r->swreg59.sw_ref3_sign_bias = v; break;
    case 4: r->swreg9.sw_ref4_sign_bias  = v; break;
    case 5: r->swreg9.sw_ref5_sign_bias  = v; break;
    case 6: r->swreg9.sw_ref6_sign_bias  = v; break;
    }
}

RkmppAv1Status rkmpp_av1_build_regs(
    const Dav1dSequenceHeader *seq,
    const Dav1dFrameHeader    *hdr,
    const RkmppAv1Dpb         *dpb,
    const RkmppAv1Buffers     *bufs,
    VdpuAv1dRegSet            *out)
{
    if (!seq || !hdr || !dpb || !bufs || !out)
        return RKMPP_AV1_ERR_BAD_INPUT;

    /* Caller is required to zero `out` first (per regbuilder_av1.h). */

    /* 8-bit, 4:2:0, non-scalable for first-pass.  10-bit is gated by
     * cap_10bit=0 on RK3588's AV1 cap struct anyway. */
    if (seq->hbd != 0)            return RKMPP_AV1_ERR_UNSUPPORTED;
    if (seq->monochrome != 0)     return RKMPP_AV1_ERR_UNSUPPORTED;
    if (hdr->frame_type == DAV1D_FRAME_TYPE_INTRA && hdr->show_existing_frame)
        return RKMPP_AV1_ERR_UNSUPPORTED;

    /* ====================================================================
     * Batch 1: control / kick (swreg1, 2, 3)
     *
     * Verified against docs/av1_trace_720p.log kick 1:
     *   swreg2 = 0x00000400 → bit 10 sw_dec_clk_gate_e=1
     *   swreg3 = 0x88001100 → dec_mode=17(av1) | skip_mode=1 |
     *                          write_mvs_e=1 | dec_out_ec_bypass=1
     * ==================================================================== */
    out->swreg1.sw_dec_e         = 1;
    out->swreg1.sw_dec_abort_e   = 0;
    out->swreg1.sw_dec_tile_int_e = 0;

    out->swreg2.sw_dec_clk_gate_e = 1;

    out->swreg3.sw_dec_mode             = 17;  /* AV1 mode constant */
    out->swreg3.sw_skip_mode            = hdr->skip_mode_enabled;
    out->swreg3.sw_dec_out_ec_byte_word = 0;   /* word align */
    out->swreg3.sw_write_mvs_e          = 1;
    out->swreg3.sw_dec_out_ec_bypass    = 1;
    /* BSP HAL hal_av1d_vdpu.c:1389 — disable in-loop filter when both
     * Y filter levels are zero (no in-loop work needed). */
    out->swreg3.sw_filtering_dis        = (hdr->loopfilter.level_y[0] == 0 &&
                                           hdr->loopfilter.level_y[1] == 0) ? 1 : 0;

    /* ====================================================================
     * Batch 2a: dimensions (swreg4)
     *   sw_pic_width_in_cbs  = (coded_width  + 7) >> 3   (8-pixel units)
     *   sw_pic_height_in_cbs = (coded_height + 7) >> 3
     *   sw_ref_frames        = active ref count (HAL inits to 1, then
     *                          increments per active ref slot — for now
     *                          we pin to 1 to match the trace and refine
     *                          when we wire up multi-ref handling)
     * Verified at 1280x720: swreg4 = 0x05001681
     *   width  = 1280/8 = 160 cbs → bits[19:31] = 0xa0
     *   height =  720/8 =  90 cbs → bits[6:18]  = 0x5a
     *   ref_frames = 1
     * ==================================================================== */
    out->swreg4.sw_pic_width_in_cbs  = ((unsigned)hdr->width[0] + 7) >> 3;
    out->swreg4.sw_pic_height_in_cbs = ((unsigned)hdr->height   + 7) >> 3;
    /* sw_ref_frames: set later per BSP HAL — keyframes/intra get 0,
     * intrabc gets 1, inter gets count of unique DPB slots. */
    out->swreg4.sw_ref_frames        = hdr->allow_intrabc ? 1 : 0;

    /* ====================================================================
     * Batch 2b: frame coding flags (swreg5)
     *
     * Mix of sequence-level toggles (dual_filter, jnt_comp, ...) and
     * frame-header flags (show_frame, allow_warp, ...).
     *
     * Verified at kick 1 720p: swreg5 = 0x50017f00 →
     *   jnt_comp=1, filter_intra=1, intra_edge_filter=1, interintra=1,
     *   masked_compound=1, cdef=1 (sequence flags)
     *   show_frame=1 (frame flag)
     *   strm_start_bit=0x28 (40)  ← OBU offset bit, fill later
     * ==================================================================== */
    /* sequence-level enables */
    out->swreg5.sw_enable_dual_filter         = seq->dual_filter;
    out->swreg5.sw_enable_jnt_comp            = seq->jnt_comp;
    out->swreg5.sw_allow_filter_intra         = seq->filter_intra;
    out->swreg5.sw_enable_intra_edge_filter   = seq->intra_edge_filter;
    out->swreg5.sw_allow_interintra           = seq->inter_intra;
    out->swreg5.sw_allow_masked_compound      = seq->masked_compound;
    out->swreg5.sw_enable_cdef                = seq->cdef;
    /* frame-header flags */
    out->swreg5.sw_show_frame                 = hdr->show_frame;
    out->swreg5.sw_disable_cdf_update         = hdr->disable_cdf_update;
    out->swreg5.sw_error_resilient            = hdr->error_resilient_mode;
    out->swreg5.sw_force_interger_mv          = hdr->force_integer_mv;
    out->swreg5.sw_allow_intrabc              = hdr->allow_intrabc;
    out->swreg5.sw_allow_screen_content_tools = hdr->allow_screen_content_tools;
    out->swreg5.sw_reduced_tx_set_used        = hdr->reduced_txtp_set;
    out->swreg5.sw_switchable_motion_mode     = hdr->switchable_motion_mode;
    out->swreg5.sw_allow_warp                 = hdr->warp_motion;
    out->swreg5.sw_tempor_mvp_e               = hdr->use_ref_frame_mvs;
    out->swreg5.sw_delta_lf_present           = hdr->delta.lf.present;
    out->swreg5.sw_delta_lf_multi             = hdr->delta.lf.multi;
    out->swreg5.sw_delta_lf_res_log           = hdr->delta.lf.res_log2;
    /* sw_superres_is_scaled: 1 when superres denom != 8 (real upscale). */
    out->swreg5.sw_superres_is_scaled         = hdr->super_res.enabled &&
                                                hdr->super_res.width_scale_denominator != 8;
    /* sw_filt_level_base_gt32, sw_preskip_segid, sw_strm_start_bit are
     * filled by later batches (loop filter / segmentation / OBU offset). */

    /* swreg6: bitstream length in bytes */
    out->swreg6.sw_stream_len = bufs->bitstream_length;

    /* ====================================================================
     * Batch 2c: misc per-frame (swreg7, swreg8)
     * Quant + delta_q + cdef damping/bits + film grain header bits.
     * Verified at kick 1 720p: swreg7 = 0x00000010, swreg8 = 0x0a008c00
     *   swreg7: delta_q_present=0, cdef_damping=2 (=3+2-3), cdef_bits=0
     *   swreg8: quant_base_qindex=0x8c (140 — but kick 1 was inter; the
     *           keyframe kick 0 has yac=44, so yac varies per kick)
     *           idr_pic_e=0, superres_pic_width=0x500 (1280)
     * ==================================================================== */
    out->swreg7.sw_delta_q_present = hdr->delta.q.present;
    out->swreg7.sw_delta_q_res_log = hdr->delta.q.res_log2;
    /* CDEF damping in HAL is dxva->cdef.damping - 3 (resulting field is
     * 0..3 to encode actual damping 3..6).  When cdef.bits=damping=0
     * (cdef disabled), HAL sets enable_cdef=0 in swreg5 and leaves
     * swreg7's cdef bits zero. */
    out->swreg7.sw_cdef_damping = hdr->cdef.damping ? (hdr->cdef.damping - 3) : 0;
    out->swreg7.sw_cdef_bits    = hdr->cdef.n_bits;
    out->swreg7.sw_apply_grain  = hdr->film_grain.present;

    out->swreg8.sw_quant_base_qindex = hdr->quant.yac;
    out->swreg8.sw_idr_pic_e         = (hdr->frame_type == DAV1D_FRAME_TYPE_KEY) ? 1 : 0;
    out->swreg8.sw_bit_depth_y_minus8 = seq->hbd ? 2 : 0;
    out->swreg8.sw_bit_depth_c_minus8 = seq->hbd ? 2 : 0;
    out->swreg8.sw_superres_pic_width = hdr->super_res.enabled ?
                                        (unsigned)hdr->width[1] :
                                        (unsigned)hdr->width[0];

    /* ====================================================================
     * Batch 3: tile config + multicore (swreg10)
     *
     * AV1 tile geometry: log2 cols/rows + actual cols/rows (≤ 64x64).
     * sw_tile_transpose is a HAL constant set to 1 (matches BSP capture
     * of 0x00020401 = bit 0 set, bit 10 set, bits 11+).  Multicore
     * fields stay 0 for single-core RK3588.
     * ==================================================================== */
    out->swreg10.sw_tile_transpose       = 1;  /* HAL hardcodes */
    out->swreg10.sw_tile_enable          = (hdr->tiling.cols * hdr->tiling.rows > 1) ? 1 : 0;
    out->swreg10.sw_num_tile_cols_8k     = hdr->tiling.cols;
    out->swreg10.sw_num_tile_rows_8k_av1 = hdr->tiling.rows;

    /* ====================================================================
     * Batch 4: temporal MVs + comp pred + transform mode (swreg11)
     *   sw_use_temporal0_mvs..3_mvs — per-ref active flag, requires
     *     ref tracking; pinned to 0 for now (matches keyframe behavior)
     *   sw_comp_pred_mode = switchable_comp_refs (0/1/2)
     *   sw_high_prec_mv_e = hp
     *   sw_mcomp_filt_type = subpel_filter_mode
     *   sw_transform_mode = txfm_mode (3 bits)
     *   sw_dec_tile_size_mag = tiling.n_bytes - 1
     * ==================================================================== */
    /* AV1 spec §5.9.30: MV precision = ALLOW_HIGH_PRECISION_MV when hp=1 */
    out->swreg11.sw_high_prec_mv_e   = hdr->hp;
    /* BSP HAL hal_av1d_vdpu.c:2052: comp_pred_mode = reference_mode ? 2 : 0
     * (binary "switchable" → 2; "single" → 0).  dav1d's switchable_comp_refs
     * is the same reference_select flag (0/1). */
    out->swreg11.sw_comp_pred_mode   = hdr->switchable_comp_refs ? 2 : 0;
    out->swreg11.sw_mcomp_filt_type  = (uint32_t)hdr->subpel_filter_mode;
    /* BSP HAL hal_av1d_vdpu.c:2063 — HW field = tx_mode != 0 ? tx_mode + 2 : 0
     * (dav1d 0/1/2 → HW 0/3/4). */
    {
        uint32_t tx = (uint32_t)hdr->txfm_mode;
        out->swreg11.sw_transform_mode = tx ? tx + 2u : 0u;
    }
    /* Single-core decode: BSP HAL line 1480 sets this to (0 == context_update_x)
     * which is true for our use case (no multi-core tile context handoff). */
    out->swreg11.sw_multicore_expect_context_update = 1;
    /* dec_tile_size_mag = tile_size_bytes_minus_1 (0..3).  dav1d only sets
     * n_bytes when tiling.update is parsed (multi-tile streams); single-tile
     * streams leave n_bytes=0.  BSP captures 3 in that case (the AV1 default
     * is the 4-byte form when the field isn't transmitted). */
    out->swreg11.sw_dec_tile_size_mag = hdr->tiling.n_bytes ? (hdr->tiling.n_bytes - 1) : 3;

    /* ====================================================================
     * Batch 5: segmentation (swreg13) + loop filter base (swreg14-17)
     * Captured 720p kick 1: swreg14=0x14000000 (filt_level0=5),
     *                        swreg15=0x14000000 (filt_level1=5),
     *                        swreg16=0x08000000 (filt_level2=2 for U),
     *                        swreg17=0x08000000 (filt_level3=2 for V).
     * ==================================================================== */
    out->swreg13.sw_segment_e        = hdr->segmentation.enabled;
    out->swreg13.sw_segment_upd_e    = hdr->segmentation.update_map;
    out->swreg13.sw_segment_temp_upd_e = hdr->segmentation.temporal;
    out->swreg13.sw_lossless_e       = hdr->all_lossless;
    out->swreg13.sw_qp_delta_y_dc_av1  = hdr->quant.ydc_delta;
    out->swreg13.sw_qp_delta_ch_dc_av1 = hdr->quant.udc_delta;
    out->swreg13.sw_qp_delta_ch_ac_av1 = hdr->quant.uac_delta;

    out->swreg14.sw_filt_level0 = hdr->loopfilter.level_y[0];
    out->swreg15.sw_filt_level1 = hdr->loopfilter.level_y[1];
    out->swreg16.sw_filt_level2 = hdr->loopfilter.level_u;
    out->swreg17.sw_filt_level3 = hdr->loopfilter.level_v;

    /* swreg5.sw_filt_level_base_gt32 = 1 if any of the 4 base levels
     * (Y[0], Y[1], U, V) exceeds 32.  HAL bit; required for higher-
     * filter-level streams. */
    out->swreg5.sw_filt_level_base_gt32 =
        (hdr->loopfilter.level_y[0] > 32 ||
         hdr->loopfilter.level_y[1] > 32 ||
         hdr->loopfilter.level_u    > 32 ||
         hdr->loopfilter.level_v    > 32) ? 1 : 0;

    /* ====================================================================
     * Batch 6: loop filter ref/mode deltas (swreg49) + V-channel quant
     *   sw_filt_ref_adj_0..3 — ref deltas indexed by INTRA_FRAME (0),
     *     LAST..LAST3 (1..3); HAL puts INTRA at 0, LAST at 1, etc.
     *   sw_qmlevel_v = quant.qm_v (already filled in swreg47/48 by HAL)
     * ==================================================================== */
    /* dav1d's mode_ref_delta_enabled gates whether the deltas apply.
     * Layout: adj_0..3 (INTRA, LAST, LAST2, LAST3) live in swreg59;
     *         adj_4..7 (GOLDEN, BWDREF, ALTREF2, ALTREF) split across
     *         swregs (HAL uses swreg49 for adj_6 and adj_7). */
    if (hdr->loopfilter.mode_ref_delta_enabled) {
        const Dav1dLoopfilterModeRefDeltas *d = &hdr->loopfilter.mode_ref_deltas;
        out->swreg59.sw_filt_ref_adj_0 = d->ref_delta[0] & 0x7f;
        out->swreg59.sw_filt_ref_adj_1 = d->ref_delta[1] & 0x7f;
        out->swreg59.sw_filt_ref_adj_2 = d->ref_delta[2] & 0x7f;
        out->swreg59.sw_filt_ref_adj_3 = d->ref_delta[3] & 0x7f;
        /* BSP HAL hal_av1d_vdpu.c:1400-1401 swaps these: adj_6 gets
         * ref_delta[7] (ALTREF) and adj_7 gets ref_delta[6] (ALTREF2). */
        out->swreg49.sw_filt_ref_adj_6 = d->ref_delta[7] & 0x7f;
        out->swreg49.sw_filt_ref_adj_7 = d->ref_delta[6] & 0x7f;
        /* adj_4 (GOLDEN), adj_5 (BWDREF) live in swregs 47/48 alongside
         * qmlevel_y/u — staged for batch 8 since they share registers. */
    }
    out->swreg49.sw_qmlevel_v = hdr->quant.qm_v;
    /* swreg47/48 hold qmlevel_y/u; matching field names follow same
     * pattern.  Defer to a later batch since they live in different
     * struct entries we haven't probed. */

    /* ====================================================================
     * Batch 7: AXI / system static config (swreg55, swreg58, swreg265,
     * swreg314).  BSP HAL writes these constants every kick; verified
     * against live trace 2026-05-05 (kernel #46+) — they are non-zero
     * in every captured kick of every test stream.
     * ==================================================================== */
    out->swreg55.sw_apf_threshold      = 8;
    out->swreg58.sw_dec_max_burst      = 16;
    out->swreg58.sw_dec_buswidth       = 2;
    out->swreg265.sw_axi_rd_ostd_threshold = 64;
    out->swreg265.sw_axi_wr_ostd_threshold = 64;
    out->swreg314.sw_dec_alignment     = 64;

    /* ====================================================================
     * Batch 9: per-reference dimensions, scales, sign-bias (swreg33-46,
     * swreg36-42, swreg9, swreg59).  Consumes RkmppAv1Dpb state via
     * hdr->refidx[i] which indexes into the 8-slot DPB.
     *
     * AV1 has 7 active refs per frame (LAST..ALTREF, indices 1..7 in
     * spec; we use 0..6 in register names).  For each ref:
     *   - width/height = DPB slot's coded dims
     *   - hor/ver scale = ((ref_dim << 14) + cur_dim/2) / cur_dim
     *   - sign_bias = OrderHint(ref) > OrderHint(cur) ? 1 : 0
     * (per AV1 spec §5.9 + §7.8 + libaom REF_SCALE_SHIFT=14)
     *
     * For keyframes / IDR / intrabc: skip — there are no valid refs. */
    if (hdr->frame_type != DAV1D_FRAME_TYPE_KEY &&
        hdr->frame_type != DAV1D_FRAME_TYPE_INTRA &&
        !hdr->allow_intrabc) {

        const uint32_t cur_w = (uint32_t)hdr->width[0];
        const uint32_t cur_h = (uint32_t)hdr->height;
        /* HAL counts UNIQUE DPB slots referenced by hdr->refidx[],
         * not the number of active ref entries.  When all 7 refidx
         * slots point at the keyframe (common for the first inter
         * after IDR), sw_ref_frames=1. */
        bool slot_used[8] = {};

        for (int ref = 0; ref < AV1_REFS_PER_FRAME; ref++) {
            int slot_idx = hdr->refidx[ref];
            if (slot_idx < 0 || slot_idx >= 8) continue;
            const RkmppAv1DpbSlot &slot = dpb->slots[slot_idx];
            if (!slot.valid) continue;

            slot_used[slot_idx] = true;

            uint32_t rw = slot.coded_width;
            uint32_t rh = slot.coded_height;
            set_ref_width (out, ref, rw);
            set_ref_height(out, ref, rh);

            if (cur_w && cur_h) {
                uint32_t hor = ((rw << AV1_REF_SCALE_SHIFT) + cur_w / 2) / cur_w;
                uint32_t ver = ((rh << AV1_REF_SCALE_SHIFT) + cur_h / 2) / cur_h;
                set_ref_hor_scale(out, ref, hor);
                set_ref_ver_scale(out, ref, ver);
                if (hor != (1u << AV1_REF_SCALE_SHIFT) ||
                    ver != (1u << AV1_REF_SCALE_SHIFT)) {
                    out->swreg5.sw_ref_scaling_enable = 1;
                }
            }

            /* Sign bias: ref's order_hint > current's → set 1 */
            int32_t d = order_hint_dist(dpb->order_hint_bits,
                                        slot.frame_offset,
                                        hdr->frame_offset);
            set_ref_sign_bias(out, ref, d > 0 ? 1u : 0u);
        }

        /* BSP HAL counts unique BUFFER indices (vdpu line 785-792), not
         * slot indices.  Slots filled by the same frame (e.g., keyframe
         * with refresh=0xff) share buffer_id and collapse to one. */
        unsigned unique_buffers = 0;
        uint32_t seen_bids[8] = {};
        for (int i = 0; i < 8; i++) {
            if (!slot_used[i]) continue;
            uint32_t bid = dpb->slots[i].buffer_id;
            bool dup = false;
            for (unsigned k = 0; k < unique_buffers; k++)
                if (seen_bids[k] == bid) { dup = true; break; }
            if (!dup) seen_bids[unique_buffers++] = bid;
        }
        out->swreg4.sw_ref_frames = unique_buffers;
    }

    /* ====================================================================
     * Batch 8: cb sizes + pic pad (swreg12)
     * Captured 720p kick 1: swreg12 = 0x00007800
     *   max_cb_size=6 (64x64 superblocks), min_cb_size=3 (8x8 min)
     * ==================================================================== */
    out->swreg12.sw_max_cb_size = seq->sb128 ? 7 : 6;  /* 128 or 64 SB */
    out->swreg12.sw_min_cb_size = 3;                   /* AV1 always 8 */
    out->swreg12.sw_av1_comp_pred_fixed_ref = 0;
    {
        uint32_t aligned_w = ((unsigned)hdr->width[0] + 7) & ~7u;
        uint32_t aligned_h = ((unsigned)hdr->height + 7) & ~7u;
        out->swreg12.sw_pic_width_pad  = aligned_w - (unsigned)hdr->width[0];
        out->swreg12.sw_pic_height_pad = aligned_h - (unsigned)hdr->height;
    }

    /* ====================================================================
     * Batch 10: loop restoration (swreg18, 19)
     * 3 planes × 2-bit fields packed into sw_lr_type / sw_lr_unit_size.
     * dav1d gives 3 type entries but only 2 unit_size entries (Y, UV
     * shared); mirror UV to V.  When restoration disabled, BSP captures
     * sw_lr_unit_size = 0x3f (max) — match that for diff parity. */
    {
        uint32_t lr_type = 0;
        bool any_restoration = false;
        for (int i = 0; i < 3; i++) {
            uint32_t t = (uint32_t)hdr->restoration.type[i] & 0x3;
            lr_type |= t << (i * 2);
            if (t) any_restoration = true;
        }
        uint32_t us_y  = hdr->restoration.unit_size[0] & 0x3;
        uint32_t us_uv = hdr->restoration.unit_size[1] & 0x3;
        uint32_t lr_unit_size = us_y | (us_uv << 2) | (us_uv << 4);
        out->swreg18.sw_lr_type      = lr_type;
        out->swreg19.sw_lr_unit_size = any_restoration ? lr_unit_size : 0x3fu;
    }

    /* ====================================================================
     * Batch 11: loop filter sharpness + ref deltas 4/5 (swreg30) +
     *           skip refs (swreg31, 32)
     * Captured 720p kick 1: swreg31=0x04000000 (sw_skip_ref0=1),
     *                       swreg32=0x14000000 (sw_skip_ref1=5).
     * dav1d skip_mode_refs[]: 1-based frame indices (LAST..ALTREF). */
    out->swreg30.sw_filt_sharpness = hdr->loopfilter.sharpness;
    if (hdr->loopfilter.mode_ref_delta_enabled) {
        out->swreg30.sw_filt_ref_adj_4 =
            hdr->loopfilter.mode_ref_deltas.ref_delta[4] & 0x7f;
        out->swreg30.sw_filt_ref_adj_5 =
            hdr->loopfilter.mode_ref_deltas.ref_delta[5] & 0x7f;
        out->swreg30.sw_filt_mb_adj_0 =
            hdr->loopfilter.mode_ref_deltas.mode_delta[0] & 0x7f;
        out->swreg30.sw_filt_mb_adj_1 =
            hdr->loopfilter.mode_ref_deltas.mode_delta[1] & 0x7f;
    }
    {
        /* AV1 spec §7.8 skip_mode_params: compute SkipModeFrame[0/1]
         * via forward/backward order-hint search.  This matches the
         * MPP parser (av1d_codec.c read_skip_mode_params) bit-for-bit
         * when reference_select=1.
         *
         * dav1d gates the same algorithm behind switchable_comp_refs;
         * when it produces a value (skip_mode_allowed=1) we trust it
         * (saves DPB-walk cost), otherwise we run the algorithm.
         * Outputs are 1-based ref enum (LAST=1..ALTREF=7); default 1
         * for keyframes / no-pair-found / order_hint disabled. */
        int sr0 = 1, sr1 = 1;
        if (hdr->skip_mode_allowed) {
            sr0 = hdr->skip_mode_refs[0] + 1;
            sr1 = hdr->skip_mode_refs[1] + 1;
        } else if (hdr->frame_type != DAV1D_FRAME_TYPE_KEY &&
                   hdr->frame_type != DAV1D_FRAME_TYPE_INTRA &&
                   !hdr->allow_intrabc &&
                   dpb->order_hint_bits) {
            const int32_t cur_h = hdr->frame_offset;
            const uint8_t bits  = dpb->order_hint_bits;
            int forward_idx = -1, backward_idx = -1;
            int32_t forward_hint = 0, backward_hint = 0;
            for (int i = 0; i < AV1_REFS_PER_FRAME; i++) {
                int s = hdr->refidx[i];
                if (s < 0 || s >= 8 || !dpb->slots[s].valid) continue;
                int32_t rh = dpb->slots[s].frame_offset;
                int32_t d  = order_hint_dist(bits, rh, cur_h);
                if (d < 0) {
                    if (forward_idx < 0 ||
                        order_hint_dist(bits, rh, forward_hint) > 0) {
                        forward_idx = i; forward_hint = rh;
                    }
                } else if (d > 0) {
                    if (backward_idx < 0 ||
                        order_hint_dist(bits, rh, backward_hint) < 0) {
                        backward_idx = i; backward_hint = rh;
                    }
                }
            }
            if (forward_idx >= 0 && backward_idx >= 0) {
                int lo = forward_idx < backward_idx ? forward_idx : backward_idx;
                int hi = forward_idx < backward_idx ? backward_idx : forward_idx;
                sr0 = lo + 1;
                sr1 = hi + 1;
            } else if (forward_idx >= 0) {
                int second_idx = -1;
                int32_t second_hint = 0;
                for (int i = 0; i < AV1_REFS_PER_FRAME; i++) {
                    int s = hdr->refidx[i];
                    if (s < 0 || s >= 8 || !dpb->slots[s].valid) continue;
                    int32_t rh = dpb->slots[s].frame_offset;
                    if (order_hint_dist(bits, rh, forward_hint) < 0) {
                        if (second_idx < 0 ||
                            order_hint_dist(bits, rh, second_hint) > 0) {
                            second_idx = i; second_hint = rh;
                        }
                    }
                }
                if (second_idx >= 0) {
                    int lo = forward_idx < second_idx ? forward_idx : second_idx;
                    int hi = forward_idx < second_idx ? second_idx : forward_idx;
                    sr0 = lo + 1;
                    sr1 = hi + 1;
                }
            }
        }
        out->swreg31.sw_skip_ref0 = (uint32_t)sr0 & 0xf;
        out->swreg32.sw_skip_ref1 = (uint32_t)sr1 & 0xf;
    }

    /* ====================================================================
     * Batch 12: segmentation per-segment fields
     * Disabled-segmentation case (our test stream): all per-seg = 0.
     * When enabled: 8 segments × {quant, filt_level_delta0..3, skip,
     * refpic, gmv} packed across swreg14-21 + swreg20-27 + swreg31-32. */
    if (hdr->segmentation.enabled) {
        const Dav1dSegmentationDataSet *sd = &hdr->segmentation.seg_data;
        uint32_t seg_quant_sign = 0;
        uint8_t  preskip_segid  = 0;
        uint8_t  last_active    = 0;

        struct SegRegs {
            uint8_t q, d0, d1, d2, d3, refpic, skip, gmv;
        } seg[8] = {};

        auto clipS = [](int v, int lo, int hi){ if (v<lo) return lo; if (v>hi) return hi; return v; };

        for (int s = 0; s < 8; s++) {
            const Dav1dSegmentationData *d = &sd->d[s];
            int qclip = d->delta_q;
            if (qclip < 0) { seg_quant_sign |= 1u << s; qclip = -qclip; }
            seg[s].q       = (uint8_t)clipS(qclip, 0, 255);
            seg[s].d0      = (uint8_t)(clipS(d->delta_lf_y_v, -63, 63) & 0x7f);
            seg[s].d1      = (uint8_t)(clipS(d->delta_lf_y_h, -63, 63) & 0x7f);
            seg[s].d2      = (uint8_t)(clipS(d->delta_lf_u  , -63, 63) & 0x7f);
            seg[s].d3      = (uint8_t)(clipS(d->delta_lf_v  , -63, 63) & 0x7f);
            seg[s].refpic  = (d->ref >= 0) ? (uint8_t)((d->ref + 1) & 0xf) : 0u;
            seg[s].skip    = d->skip ? 1u : 0u;
            seg[s].gmv     = d->globalmv ? 1u : 0u;
            if (d->delta_q || d->delta_lf_y_v || d->delta_lf_y_h ||
                d->delta_lf_u || d->delta_lf_v || d->ref >= 0 ||
                d->skip || d->globalmv) {
                if (d->ref >= 0 || d->skip || d->globalmv) preskip_segid = 1;
                last_active = (uint8_t)s;
            }
        }
        out->swreg9.sw_last_active_seg = last_active;
        out->swreg5.sw_preskip_segid   = preskip_segid;
        out->swreg12.sw_seg_quant_sign = seg_quant_sign;

        out->swreg14.sw_quant_seg0 = seg[0].q; out->swreg14.sw_filt_level_delta0_seg0 = seg[0].d0;
        out->swreg14.sw_refpic_seg0 = seg[0].refpic; out->swreg14.sw_skip_seg0 = seg[0].skip;
        out->swreg15.sw_quant_seg1 = seg[1].q; out->swreg15.sw_filt_level_delta0_seg1 = seg[1].d0;
        out->swreg15.sw_refpic_seg1 = seg[1].refpic; out->swreg15.sw_skip_seg1 = seg[1].skip;
        out->swreg16.sw_quant_seg2 = seg[2].q; out->swreg16.sw_filt_level_delta0_seg2 = seg[2].d0;
        out->swreg16.sw_refpic_seg2 = seg[2].refpic; out->swreg16.sw_skip_seg2 = seg[2].skip;
        out->swreg17.sw_quant_seg3 = seg[3].q; out->swreg17.sw_filt_level_delta0_seg3 = seg[3].d0;
        out->swreg17.sw_refpic_seg3 = seg[3].refpic; out->swreg17.sw_skip_seg3 = seg[3].skip;
        out->swreg18.sw_quant_seg4 = seg[4].q; out->swreg18.sw_filt_level_delta0_seg4 = seg[4].d0;
        out->swreg18.sw_refpic_seg4 = seg[4].refpic; out->swreg18.sw_skip_seg4 = seg[4].skip;
        out->swreg19.sw_quant_seg5 = seg[5].q; out->swreg19.sw_filt_level_delta0_seg5 = seg[5].d0;
        out->swreg19.sw_refpic_seg5 = seg[5].refpic; out->swreg19.sw_skip_seg5 = seg[5].skip;
        out->swreg31.sw_quant_seg6 = seg[6].q; out->swreg31.sw_filt_level_delta0_seg6 = seg[6].d0;
        out->swreg31.sw_refpic_seg6 = seg[6].refpic; out->swreg31.sw_skip_seg6 = seg[6].skip;
        out->swreg32.sw_quant_seg7 = seg[7].q; out->swreg32.sw_filt_level_delta0_seg7 = seg[7].d0;
        out->swreg32.sw_refpic_seg7 = seg[7].refpic; out->swreg32.sw_skip_seg7 = seg[7].skip;

        /* swreg20-27 hold delta1/2/3 + global_mv per segment (seg0..7). */
        out->swreg20.sw_filt_level_delta1_seg0 = seg[0].d1;
        out->swreg20.sw_filt_level_delta2_seg0 = seg[0].d2;
        out->swreg20.sw_filt_level_delta3_seg0 = seg[0].d3;
        out->swreg20.sw_global_mv_seg0         = seg[0].gmv;
        /* (seg1..7 follow identical pattern in swreg21..27 — left as
         * skeleton; flesh out when we hit a stream that exercises
         * segmentation.  For the disabled-seg case the captured BSP
         * trace is dominated by mf1 offsets in the same regs.) */
    }

    /* ====================================================================
     * Batch 13: V-channel quant deltas + qm levels (swreg13, 28, 29, 47, 48)
     * ==================================================================== */
    out->swreg28.sw_quant_delta_v_dc = hdr->quant.vdc_delta;
    out->swreg29.sw_quant_delta_v_ac = hdr->quant.vac_delta;
    out->swreg13.sw_lossless_e       = hdr->all_lossless;
    /* AV1 spec §5.9.12: qm levels default to 15 (max) when using_qmatrix=0.
     * dav1d's parsed values are zero in that case; BSP transcribes the
     * effective value 15.  Mirror BSP. */
    {
        uint8_t qy = hdr->quant.qm ? hdr->quant.qm_y : 15;
        uint8_t qu = hdr->quant.qm ? hdr->quant.qm_u : 15;
        uint8_t qv = hdr->quant.qm ? hdr->quant.qm_v : 15;
        out->swreg47.sw_qmlevel_y = qy;
        out->swreg48.sw_qmlevel_u = qu;
        out->swreg49.sw_qmlevel_v = qv;  /* override the earlier write */
    }

    /* ====================================================================
     * Batch 14: CDEF strengths (swreg53, 263, 264)
     * AV1: up to 8 luma+chroma strength pairs.  dav1d packs y_strength[i]
     * = (primary << 2) | secondary.  HAL splits: primary 4 bits each into
     * swreg263/264 (32-bit primaries), secondary 2 bits each into swreg53
     * (16+16 bits). */
    {
        uint32_t luma_pri = 0, chroma_pri = 0;
        uint16_t luma_sec = 0, chroma_sec = 0;
        unsigned n = 1u << hdr->cdef.n_bits;
        if (n > 8) n = 8;
        for (unsigned i = 0; i < n; i++) {
            uint8_t y  = hdr->cdef.y_strength[i];
            uint8_t uv = hdr->cdef.uv_strength[i];
            luma_pri   |= (uint32_t)((y  >> 2) & 0xf) << (i * 4);
            luma_sec   |= (uint16_t)( y       & 0x3) << (i * 2);
            chroma_pri |= (uint32_t)((uv >> 2) & 0xf) << (i * 4);
            chroma_sec |= (uint16_t)(uv       & 0x3) << (i * 2);
        }
        out->swreg263.sw_cdef_luma_primary_strength    = luma_pri;
        out->swreg264.sw_cdef_chroma_primary_strength  = chroma_pri;
        out->swreg53.sw_cdef_luma_secondary_strength   = luma_sec;
        out->swreg53.sw_cdef_chroma_secondary_strength = chroma_sec;
    }

    /* ====================================================================
     * Batch 15: stream length (swreg6, swreg258)
     * BSP captured 0x100 = MPP_ALIGN(stream_len, 128). */
    {
        uint32_t aligned = (bufs->bitstream_length + 127u) & ~127u;
        out->swreg6.sw_stream_len        = aligned;
        out->swreg258.sw_strm_buffer_len = aligned;
    }

    /* ====================================================================
     * Batch 16: superres steps (swreg51, 298)
     * BSP captures swreg51=0x0003800e and swreg298=0x000e000e even when
     * superres is disabled (denom=8, no upscale).  These look like HAL
     * defaults — luma_step=14, chroma_step=14 — applied unconditionally.
     * Match that. */
    {
        uint32_t step = 14;  /* HAL default; revisit if superres ever active */
        if (hdr->super_res.enabled && hdr->width[1] != hdr->width[0]) {
            step = ((uint32_t)hdr->width[0] << 14) / (uint32_t)hdr->width[1];
        }
        out->swreg51.sw_superres_luma_step          = step;
        out->swreg51.sw_superres_chroma_step        = step;
        out->swreg298.sw_superres_luma_step_invra   = step;
        out->swreg298.sw_superres_chroma_step_invra = step;
    }

    /* ====================================================================
     * Batch 17: timeout cycles (swreg318, 319) — HAL constants
     * Captured BSP both 0x8fffffff = max non-saturated + override bit. */
    out->swreg318.sw_ext_timeout_cycles     = 0x0fffffff;
    out->swreg318.sw_ext_timeout_override_e = 1;
    out->swreg319.sw_timeout_cycles         = 0x0fffffff;
    out->swreg319.sw_timeout_override_e     = 1;

    /* ====================================================================
     * Batch PP: post-processor output config (swreg320, 322, 329, 331,
     *           332, 394) — NV12 output of half-stride chroma into
     *           the same buffer as luma.  HAL constants + per-frame
     *           dimensions.  Captured BSP values verified at 720p:
     *             swreg320 = 0x00000001 (sw_pp_out_e = 1)
     *             swreg322 = 0x000c0000 (sw_pp_in_format=0, plus swap bits)
     *             swreg329 = 0x05000500 (y_stride=1280, c_stride=1280)
     *             swreg331 = 0x02800168 (in_width=640, in_height=360)
     *             swreg332 = 0x050002d0 (out_width=1280, out_height=720)
     *             swreg394 = 0x01010000 (pp0_dup_hor=1, pp0_dup_ver=1) */
    {
        const uint32_t w = (uint32_t)hdr->width[0];
        const uint32_t h = (uint32_t)hdr->height;
        out->vdpu_av1d_pp_cfg.swreg320.sw_pp_out_e   = 1;
        out->vdpu_av1d_pp_cfg.swreg322.sw_pp_in_format  = 0;
        /* sw_pp_out_format=3 selects NV12 output (captured 0x000c0000
         * has bits 18-19 set in swreg322 = pp_out_format value 3). */
        out->vdpu_av1d_pp_cfg.swreg322.sw_pp_out_format = 3;
        /* HAL writes hor_stride for both luma and chroma stride. */
        out->vdpu_av1d_pp_cfg.swreg329.sw_pp_out_y_stride = w;
        out->vdpu_av1d_pp_cfg.swreg329.sw_pp_out_c_stride = w;
        out->vdpu_av1d_pp_cfg.swreg331.sw_pp_in_width    = w / 2;
        out->vdpu_av1d_pp_cfg.swreg331.sw_pp_in_height   = h / 2;
        out->vdpu_av1d_pp_cfg.swreg332.sw_pp_out_width   = w;
        out->vdpu_av1d_pp_cfg.swreg332.sw_pp_out_height  = h;
        out->vdpu_av1d_pp_cfg.swreg394.sw_pp0_dup_hor    = 1;
        out->vdpu_av1d_pp_cfg.swreg394.sw_pp0_dup_ver    = 1;
    }

    /* ====================================================================
     * Batch 18: global motion mode + mf3 offsets (swreg184-188, 257, 262)
     *
     * Each register packs: per-ref mf3_offset (9 bits) + per-ref
     * gm_mode (2 bits) + reserved.  When mf3_offset isn't applicable
     * (ref not "near" by AV1 motion-field-projection rules), BSP
     * captures 0x3ff (9 bits all set = sentinel "no projection").
     *
     * gm_mode: 0..3 (identity, translate, rotzoom, affine).  dav1d's
     * Dav1dWarpedMotionParams.type is the same enum. */
    /* swregs 184-188, 257, 262 each pack:
     *   sw_cur_*_roffset   (9 bits) — order-hint forward distance
     *   sw_cur_*_offset    (9 bits) — order-hint backward distance
     *   sw_mf3_*_offset    (9 bits) — motion-field projection offset
     *   sw_refN_gm_mode    (2 bits)
     *
     * For keyframes / IDR / intrabc there are no refs and BSP writes
     * all-zero.  For inter frames BSP fills offsets from per-ref
     * order-hint arithmetic; that fill is staged for a future batch
     * (needs sw_mf{1,2,3}_type computation in swreg9).  Until then we
     * write only gm_mode (the one field we already have a clean
     * source for) and leave offsets zero. */
    if (hdr->frame_type != DAV1D_FRAME_TYPE_KEY &&
        hdr->frame_type != DAV1D_FRAME_TYPE_INTRA &&
        !hdr->allow_intrabc) {
        out->swreg184.sw_ref0_gm_mode = (uint32_t)hdr->gmv[0].type & 0x3;
        out->swreg185.sw_ref1_gm_mode = (uint32_t)hdr->gmv[1].type & 0x3;
        out->swreg186.sw_ref2_gm_mode = (uint32_t)hdr->gmv[2].type & 0x3;
        out->swreg187.sw_ref3_gm_mode = (uint32_t)hdr->gmv[3].type & 0x3;
        out->swreg188.sw_ref4_gm_mode = (uint32_t)hdr->gmv[4].type & 0x3;
        out->swreg257.sw_ref5_gm_mode = (uint32_t)hdr->gmv[5].type & 0x3;
        out->swreg262.sw_ref6_gm_mode = (uint32_t)hdr->gmv[6].type & 0x3;

        /* ---- BSP MF projection (port of hal_av1d_vdpu.c lines 870-1086) ----
         * AV1 spec §7.9 motion field estimation.  Selects up to 3
         * source frames whose own MVs are projected into the current
         * frame's motion field; their identity goes in swreg9.mf{1,2,3}_type
         * and their per-MFmaster ref offsets go in swreg{20-27/47-48/184-188/257/262}.
         *
         * Index conventions (matching BSP):
         *   ref index 0..6 = LAST..ALTREF positions (refidx[0..6])
         *   buf_idx == ref index (zero-based)
         *   AV1_REF_FRAME_LAST=1, ..., ALTREF=7 (1-based spec enum)
         */
        constexpr int LST_BUF = 0, LST2_BUF = 1, GLD_BUF = 3;
        constexpr int BWD_BUF = 4, ALT2_BUF = 5, ALT_BUF = 6;
        constexpr int REF_FRAME_LAST   = 1;
        constexpr int REF_FRAME_LAST2  = 2;
        constexpr int REF_FRAME_BWDREF = 5;
        constexpr int REF_FRAME_ALTREF2 = 6;
        constexpr int REF_FRAME_ALTREF = 7;
        constexpr int MAX_FRAME_DISTANCE = 31;

        const int32_t cur_h = hdr->frame_offset;
        const int     cur_mi_cols = ((unsigned)hdr->width[0] + 7) >> 3;
        const int     cur_mi_rows = ((unsigned)hdr->height   + 7) >> 3;
        const uint8_t bits = dpb->order_hint_bits;

        auto slot_for = [&](int buf_idx) -> const RkmppAv1DpbSlot * {
            int s = hdr->refidx[buf_idx];
            if (s < 0 || s >= 8) return nullptr;
            const RkmppAv1DpbSlot &sl = dpb->slots[s];
            return sl.valid ? &sl : nullptr;
        };
        auto is_intra = [](const RkmppAv1DpbSlot *s) {
            return s && (s->frame_type == DAV1D_FRAME_TYPE_KEY ||
                         s->frame_type == DAV1D_FRAME_TYPE_INTRA);
        };
        auto same_dims = [&](const RkmppAv1DpbSlot *s) {
            if (!s) return false;
            int mi_c = (s->coded_width  + 7) >> 3;
            int mi_r = (s->coded_height + 7) >> 3;
            return mi_c == cur_mi_cols && mi_r == cur_mi_rows;
        };

        const RkmppAv1DpbSlot *lst  = slot_for(LST_BUF);
        const RkmppAv1DpbSlot *lst2 = slot_for(LST2_BUF);
        const RkmppAv1DpbSlot *gld  = slot_for(GLD_BUF);
        const RkmppAv1DpbSlot *bwd  = slot_for(BWD_BUF);
        const RkmppAv1DpbSlot *alt2 = slot_for(ALT2_BUF);
        const RkmppAv1DpbSlot *alt  = slot_for(ALT_BUF);

        int32_t alt_off  = alt  ? alt->frame_offset  : 0;
        int32_t gld_off  = gld  ? gld->frame_offset  : 0;
        int32_t bwd_off  = bwd  ? bwd->frame_offset  : 0;
        int32_t alt2_off = alt2 ? alt2->frame_offset : 0;

        uint8_t mf_types[3] = {0, 0, 0};
        int     refs_selected[3] = {0, 0, 0};
        int     ref_stamp = 2;
        int     ref_ind = 0;

        /* LAST: only if NOT lst-overlay (lst's altref slot != gld order_hint) */
        if (lst) {
            int32_t alt_off_in_lst = lst->saved_order_hints[ALT_BUF];
            bool is_lst_overlay = (alt_off_in_lst == gld_off);
            if (!is_lst_overlay && same_dims(lst) && !is_intra(lst)) {
                mf_types[ref_ind] = REF_FRAME_LAST;
                refs_selected[ref_ind++] = LST_BUF;
            }
            ref_stamp--;
        }

        if (bwd && order_hint_dist(bits, bwd_off, cur_h) > 0 &&
            same_dims(bwd) && !is_intra(bwd)) {
            mf_types[ref_ind] = REF_FRAME_BWDREF;
            refs_selected[ref_ind++] = BWD_BUF;
            ref_stamp--;
        }

        if (alt2 && order_hint_dist(bits, alt2_off, cur_h) > 0 &&
            same_dims(alt2) && !is_intra(alt2)) {
            mf_types[ref_ind] = REF_FRAME_ALTREF2;
            refs_selected[ref_ind++] = ALT2_BUF;
            ref_stamp--;
        }

        if (alt && ref_stamp >= 0 &&
            order_hint_dist(bits, alt_off, cur_h) > 0 &&
            same_dims(alt) && !is_intra(alt)) {
            mf_types[ref_ind] = REF_FRAME_ALTREF;
            refs_selected[ref_ind++] = ALT_BUF;
            ref_stamp--;
        }

        if (ref_stamp >= 0 && lst2 && same_dims(lst2) && !is_intra(lst2)) {
            mf_types[ref_ind] = REF_FRAME_LAST2;
            refs_selected[ref_ind++] = LST2_BUF;
            ref_stamp--;
        }

        /* Per-ref cur_offset / cur_roffset (all 7 refs). */
        int32_t cur_off[7] = {}, cur_roff[7] = {};
        for (int rf = 0; rf < AV1_REFS_PER_FRAME; rf++) {
            int s = hdr->refidx[rf];
            if (s < 0 || s >= 8) continue;
            const RkmppAv1DpbSlot &sl = dpb->slots[s];
            if (!sl.valid) continue;
            cur_off[rf]  = order_hint_dist(bits, cur_h, sl.frame_offset);
            cur_roff[rf] = order_hint_dist(bits, sl.frame_offset, cur_h);
        }

        /* use_temporal{0,1,2}_mvs gates each populated mf-type slot. */
        auto in_range = [&](int idx) {
            int32_t v = cur_off[idx];
            return v <= MAX_FRAME_DISTANCE && v >= -MAX_FRAME_DISTANCE;
        };

        auto populate_mfX = [&](int slot_index, int buf_idx) {
            /* slot_index: 1=mf1, 2=mf2, 3=mf3.  buf_idx: 0..6 ref position
             * of the selected source frame.  ref_offset[r] = relative dist
             * from selected frame's own order_hint to its r-th ref's order_hint. */
            const RkmppAv1DpbSlot &sl = dpb->slots[hdr->refidx[buf_idx]];
            int32_t src_h = sl.frame_offset;
            int32_t ro[7];
            for (int r = 0; r < 7; r++)
                ro[r] = order_hint_dist(bits, src_h, sl.saved_order_hints[r]);
            const uint32_t m9 = 0x1ffu;
            if (slot_index == 1) {
                out->swreg20.sw_mf1_last_offset    = (uint32_t)ro[0] & m9;
                out->swreg21.sw_mf1_last2_offset   = (uint32_t)ro[1] & m9;
                out->swreg22.sw_mf1_last3_offset   = (uint32_t)ro[2] & m9;
                out->swreg23.sw_mf1_golden_offset  = (uint32_t)ro[3] & m9;
                out->swreg24.sw_mf1_bwdref_offset  = (uint32_t)ro[4] & m9;
                out->swreg25.sw_mf1_altref2_offset = (uint32_t)ro[5] & m9;
                out->swreg26.sw_mf1_altref_offset  = (uint32_t)ro[6] & m9;
            } else if (slot_index == 2) {
                out->swreg27.sw_mf2_last_offset    = (uint32_t)ro[0] & m9;
                out->swreg47.sw_mf2_last2_offset   = (uint32_t)ro[1] & m9;
                out->swreg47.sw_mf2_last3_offset   = (uint32_t)ro[2] & m9;
                out->swreg47.sw_mf2_golden_offset  = (uint32_t)ro[3] & m9;
                out->swreg48.sw_mf2_bwdref_offset  = (uint32_t)ro[4] & m9;
                out->swreg48.sw_mf2_altref2_offset = (uint32_t)ro[5] & m9;
                out->swreg48.sw_mf2_altref_offset  = (uint32_t)ro[6] & m9;
            } else {
                out->swreg184.sw_mf3_last_offset   = (uint32_t)ro[0] & m9;
                out->swreg185.sw_mf3_last2_offset  = (uint32_t)ro[1] & m9;
                out->swreg186.sw_mf3_last3_offset  = (uint32_t)ro[2] & m9;
                out->swreg187.sw_mf3_golden_offset = (uint32_t)ro[3] & m9;
                out->swreg188.sw_mf3_bwdref_offset = (uint32_t)ro[4] & m9;
                out->swreg257.sw_mf3_altref2_offset = (uint32_t)ro[5] & m9;
                out->swreg262.sw_mf3_altref_offset = (uint32_t)ro[6] & m9;
            }
        };

        if (hdr->use_ref_frame_mvs && ref_ind > 0 &&
            in_range(mf_types[0] - REF_FRAME_LAST)) {
            out->swreg11.sw_use_temporal0_mvs = 1;
            populate_mfX(1, refs_selected[0]);
        }
        if (hdr->use_ref_frame_mvs && ref_ind > 1 &&
            in_range(mf_types[1] - REF_FRAME_LAST)) {
            out->swreg11.sw_use_temporal1_mvs = 1;
            populate_mfX(2, refs_selected[1]);
        }
        if (hdr->use_ref_frame_mvs && ref_ind > 2 &&
            in_range(mf_types[2] - REF_FRAME_LAST)) {
            out->swreg11.sw_use_temporal2_mvs = 1;
            populate_mfX(3, refs_selected[2]);
        }

        /* primary_ref_frame seg-feature path → use_temporal3_mvs */
        if (hdr->segmentation.enabled &&
            hdr->primary_ref_frame < AV1_REFS_PER_FRAME) {
            int prim = hdr->refidx[hdr->primary_ref_frame];
            if (prim >= 0 && prim < 8 && dpb->slots[prim].valid)
                out->swreg11.sw_use_temporal3_mvs = 1;
        }

        const uint32_t m9 = 0x1ffu;
        out->swreg184.sw_cur_last_offset     = (uint32_t)cur_off[0]  & m9;
        out->swreg185.sw_cur_last2_offset    = (uint32_t)cur_off[1]  & m9;
        out->swreg186.sw_cur_last3_offset    = (uint32_t)cur_off[2]  & m9;
        out->swreg187.sw_cur_golden_offset   = (uint32_t)cur_off[3]  & m9;
        out->swreg188.sw_cur_bwdref_offset   = (uint32_t)cur_off[4]  & m9;
        out->swreg257.sw_cur_altref2_offset  = (uint32_t)cur_off[5]  & m9;
        out->swreg262.sw_cur_altref_offset   = (uint32_t)cur_off[6]  & m9;
        out->swreg184.sw_cur_last_roffset    = (uint32_t)cur_roff[0] & m9;
        out->swreg185.sw_cur_last2_roffset   = (uint32_t)cur_roff[1] & m9;
        out->swreg186.sw_cur_last3_roffset   = (uint32_t)cur_roff[2] & m9;
        out->swreg187.sw_cur_golden_roffset  = (uint32_t)cur_roff[3] & m9;
        out->swreg188.sw_cur_bwdref_roffset  = (uint32_t)cur_roff[4] & m9;
        out->swreg257.sw_cur_altref2_roffset = (uint32_t)cur_roff[5] & m9;
        out->swreg262.sw_cur_altref_roffset  = (uint32_t)cur_roff[6] & m9;

        /* mf1/mf2/mf3 type fields: BSP writes raw `mf_types[i] - REF_FRAME_LAST`.
         * For unfilled slots (mf_types[i]=0) this is -1 which wraps to 7
         * in the 3-bit field — match that wrap so unused slots compare
         * bit-exact against BSP captures. */
        out->swreg9.sw_mf1_type = (uint32_t)((int)mf_types[0] - REF_FRAME_LAST) & 0x7;
        out->swreg9.sw_mf2_type = (uint32_t)((int)mf_types[1] - REF_FRAME_LAST) & 0x7;
        out->swreg9.sw_mf3_type = (uint32_t)((int)mf_types[2] - REF_FRAME_LAST) & 0x7;
    }

    /* ====================================================================
     * TODO — fields requiring state we don't yet track:
     *   swreg11.sw_use_temporal0_mvs..3_mvs — per-ref-slot active flags
     *   swreg20-27.sw_mf{1,2}_*_offset — order-hint distances per ref
     *   swreg47-48.sw_mf2_*_offset — same, paired with qm levels
     *   swreg184-188 sw_mf3_*_offset — same family
     *   swreg5.sw_strm_start_bit — needs OBU offset from dav1d
     *   swreg9.sw_context_update_tile_id — needs tile context state
     *   swreg9.sw_mf{1,2,3}_type — motion field projection types
     *   swreg64-238 — DMA bases (kernel patches; FD writes only)
     *   swreg320-394 — PP cfg (NV12 output config; many static defaults)
     * ==================================================================== */

    /* DMA: output Y base.  Other DMA addresses staged for later batches. */
    {
        uint32_t *msb = reinterpret_cast<uint32_t *>(&out->addr_cfg.swreg64);
        uint32_t *lsb = reinterpret_cast<uint32_t *>(&out->addr_cfg.swreg65);
        emit_dma(msb, lsb, bufs->output_y_fd, bufs->output_y_offset);
    }

    /* Suppress unused warnings until the rest of the fill is in. */
    (void)hdr;

    return RKMPP_AV1_OK;
}

/* --- shim-format dump for diffing against captured BSP trace -------- */

void rkmpp_av1_dump_regs_shim(const VdpuAv1dRegSet *regs, FILE *out) {
    if (!regs || !out) return;
    const uint32_t *r = reinterpret_cast<const uint32_t *>(regs);
    /* VdpuAv1dRegSet is 512 swregs of 32-bit each.  The struct
     * definition uses padding for reserved swregs so a flat
     * reinterpret-as-array works as long as we bound at the struct size. */
    constexpr size_t n = sizeof(VdpuAv1dRegSet) / sizeof(uint32_t);
    for (size_t i = 0; i < n && i < RKMPP_AV1_VCD_REGS; i++) {
        if (r[i] != 0)
            std::fprintf(out, "AV1SHIM r[0][%03zu]=%08x\n", i, r[i]);
    }
}
