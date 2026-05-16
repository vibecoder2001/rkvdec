/* mft/regbuilder_vp9.cpp — VP9 dense-bank regbuilder.
 *
 * B2 scope: common bank fill only (dec_mode, kick, stream length).
 * B3 fills codec_params (reg64/65/67-78/88-100/103-106).
 * B4 fills codec_addr + iova slots + prob-buffer helper.
 *
 * Reference: BSP hal_vp9d_vdpu34x_gen_regs in
 *   mpp/hal/rkdec/vp9d/hal_vp9d_vdpu34x.c (~line 410).
 * Field layout: mpp/hal/rkdec/inc/vdpu34x_vp9d.h.
 * BSP code is reference-only (licensing taint); all fill code is original.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "regbuilder_vp9.h"
#include "rkvdec2_vp9_regs.h"
#include "vp9_kf_probs.h"

#include <stdint.h>
#include <string.h>

namespace {

/* Stamp a plain (non-address) word into the dense bank. */
static int emit_plain(H26xDenseOutput *out, uint32_t off, uint32_t value)
{
    uint32_t idx = off / 4u;
    if (idx == RKMPP_DENSE_KICK_REG_IDX) {
        out->KickValue = value;
        return 0;
    }
    uint32_t *slot = H26xDenseSlotFor(out, off);
    if (!slot) return 1;
    *slot = value;
    return 0;
}

/* Record an iova-substitution slot.  Zero handle is a no-op. */
static int emit_iova(H26xDenseOutput *out, uint32_t off,
                     uint64_t handle, uint32_t iova_offset)
{
    if (handle == 0) return 0;
    if (out->IovaSlotCount >= RKMPP_MAX_DENSE_IOVA_SLOTS) return 1;
    uint32_t idx = off / 4u;
    if (!H26xDenseSlotFor(out, off)) return 1;
    if (!H26xDenseIsAddressReg(idx))  return 1;
    RKMPP_DENSE_IOVA_SLOT *s = &out->IovaSlots[out->IovaSlotCount++];
    s->RegIdx       = idx;
    s->IovaOffset   = iova_offset;
    s->BufferHandle = handle;
    return 0;
}

} /* anon namespace */

namespace vp9 {

/* Compute a VP9 §8.5.1 scaling factor: (ref_dim * 16384) / cur_dim.
 * Clamped to u16 range; returns 16384 (identity) when cur_dim is zero
 * to avoid division by zero. */
static uint32_t vp9_scale_factor(uint32_t ref_dim, uint32_t cur_dim)
{
    if (cur_dim == 0) return 16384u;
    uint32_t s = (ref_dim * 16384u) / cur_dim;
    return s > 0xFFFFu ? 0xFFFFu : s;
}

RegBuildStatus Vp9Regbuilder_Fill(const RegbuildInputs &in,
                                  H26xDenseOutput      *out)
{
    if (!in.pp || !in.dpb || !out) return RegBuildStatus::BadInput;

    const PicParams &pp  = *in.pp;
    const DpbCtx    &dpb = *in.dpb;
    int rc = 0;

    /* Determine whether this is an intra-type frame (used by both the
     * common error-mode register and the codec_params bank). */
    bool is_intra_pre = (pp.frame_type == 0) || pp.intra_only;

    /* Stride math (all per-ref strides use raw width/height, matching
     * BSP capture which differs from the upstream Linux vdpu381 patch's
     * round_up(height, 64) by omitting that round-up). */
    uint32_t bit_depth = pp.bit_depth ? pp.bit_depth : 8u;
    uint32_t aligned_pitch = ((pp.width * bit_depth + 511u) & ~511u) / 8u;
    uint32_t y_hor_virstride = aligned_pitch / 16u;
    uint32_t y_virstride     = (pp.height * aligned_pitch) / 16u;

    /* ---- Common bank (idx 8..32) ---------------------------------- *
     * Codec-agnostic registers shared with H.264/H.265.  Bit positions
     * verified against tests/data/vp9/bsp_capture/regs_kf/regs_000.bin
     * (RK3588 vdpu34x).  VP9 sets dec_mode = 2. */
    rc |= emit_plain(out, RKVDEC2_REG_DEC_MODE,    RKVDEC2_DEC_MODE_VP9);

    /* reg11 IMPORTANT_EN: BSP capture = 0x01000062
     *   bit 1  RKVDEC2_DEC_CLKGATE_E
     *   bit 5  RKVDEC2_DEC_TIMEOUT_E
     *   bit 6  RKVDEC2_BUF_EMPTY_EN
     *   bit 24 RKVDEC2_PIX_RANGE_DETECTION_E */
    rc |= emit_plain(out, RKVDEC2_REG_IMPORTANT_EN,
                     RKVDEC2_DEC_CLKGATE_E | RKVDEC2_DEC_TIMEOUT_E |
                     RKVDEC2_BUF_EMPTY_EN  | RKVDEC2_PIX_RANGE_DETECTION_E);

    /* reg12 SECONDARY_EN: BSP capture = 0x82.  Unlike H.265, VP9 does
     * NOT set bit 0 (dec_global_en). */
    rc |= emit_plain(out, RKVDEC2_REG_SECONDARY_EN,
                     RKVDEC2_COLMV_COMPRESS_EN | RKVDEC2_WAIT_RESET_EN);

    /* reg13 ERROR_MODE: BSP capture
     *   keyframe = 0x01000003 (TIMEOUT_MODE | req_timeout_rst_sel | CUR_PIC_IS_IDR)
     *   inter    = 0x00000003 (TIMEOUT_MODE | req_timeout_rst_sel)
     * Bit 1 (req_timeout_rst_sel) enables auto-reset on timeout — BSP
     * sets it on every kick and the codec apparently needs it for
     * inter to complete without wedging on Windows. */
    rc |= emit_plain(out, RKVDEC2_REG_ERROR_MODE,
                     RKVDEC2_TIMEOUT_MODE |
                     (1u << 1) |
                     (is_intra_pre ? RKVDEC2_CUR_PIC_IS_IDR : 0u));

    rc |= emit_plain(out, RKVDEC2_REG_STREAM_MODE, 0u);  /* rlc_mode = 0 */
    /* reg16 STR_LEN: round payload up to 16 bytes and add a 128-byte
     * tail-pad.  Matches BSP capture (0x2170 = align16(8429) + 128 on the
     * 720p keyframe test stream); the H.264/H.265 paths add only 64,
     * but VP9 BSP captures show 128. */
    {
        uint32_t str_len = ((in.bitstream_bytes + 15u) & ~15u) + 128u;
        rc |= emit_plain(out, RKVDEC2_REG_STR_LEN, str_len);
    }

    /* reg18/19/20: strides (common-bank y_hor / uv_hor / y virstride).
     * BSP uses raw height for y_virstride here, NOT 64-aligned. */
    rc |= emit_plain(out, RKVDEC2_REG_Y_HOR_VIRSTRIDE,  y_hor_virstride);
    rc |= emit_plain(out, RKVDEC2_REG_UV_HOR_VIRSTRIDE, y_hor_virstride);
    rc |= emit_plain(out, RKVDEC2_REG_Y_VIRSTRIDE,      y_virstride);

    /* reg26 BLOCK_GATING: BSP capture = 0x800FFFEF.
     *   RKVDEC2_BLOCK_GATING_RK3588 = 0x000FFFEF
     *   RKVDEC2_REG_CFG_GATING_EN   = 1u << 31 */
    rc |= emit_plain(out, RKVDEC2_REG_BLOCK_GATING,
                     RKVDEC2_BLOCK_GATING_RK3588 | RKVDEC2_REG_CFG_GATING_EN);

    /* reg32 TIMEOUT_THRESH: BSP capture = 0x00EFFFFF (~16M cycles ≈
     * 26 ms @ 600 MHz).  An earlier reading of 0x0003FFFF was off by
     * three nibbles — that's ~0.4 ms, just enough for a kf but far
     * short of an inter frame at 4K; the codec hit its internal
     * watchdog mid-decode and returned hw=0x23 (timeout) on every
     * inter kick. */
    rc |= emit_plain(out, RKVDEC2_REG_TIMEOUT_THRESH, 0x00EFFFFFu);

    /* reg28: prob-idx tracking.  Per BSP `Vdpu34xRegCommon.reg028`:
     *   bits 0..2  swreg_vp9_wr_prob_idx = frame_ctx_id + 1 (always)
     *   bits 4..6  swreg_vp9_rd_prob_idx = frame_ctx_id + 1 IF context
     *                                     already initialized, else 0
     *                                     (cold start or
     *                                     error_resilient_mode).
     * BSP capture: reg28 = 0x01 = wr=1, rd=0 (cold-start first kick). */
    {
        uint8_t  fcx  = pp.frame_context_idx & 0x3u;
        uint32_t widx = (uint32_t)fcx + 1u;
        /* BSP HAL hal_vp9d_vdpu34x.c:549-557 zeroes prob_ctx_valid[*] on
         * every keyframe / intra_only / error_resilient before deciding
         * reg028.rd_prob_idx — equivalent to forcing ctx_valid=false on
         * those frames here.  Without this, the second GOP's keyframe
         * (and every other one after) emits rd_prob_idx=fcx+1 with a
         * stale "context already initialised" flag, the codec
         * dereferences into prob_loop[fcx] (which it shouldn't on kf),
         * and the kick wedges with hw=0x23. */
        bool ctx_valid = in.prob_ctx_valid[fcx] &&
                         !pp.error_resilient_mode &&
                         !is_intra_pre;
        uint32_t ridx = ctx_valid ? widx : 0u;
        rc |= emit_plain(out, RKVDEC2_REG_FILM_IDX, widx | (ridx << 4));
    }

    /* DEC_E kick — written last by the kernel.  emit_plain on
     * RKVDEC2_REG_START_EN routes the value to KickValue (the dense
     * bank slot itself is left zero so the bulk write doesn't fight
     * the explicit final kick write). */
    rc |= emit_plain(out, RKVDEC2_REG_START_EN, RKVDEC2_DEC_E);

    /* ---- Codec_params bank (idx 64..107) -------------------------- */

    /* reg64: cprheader_offset = 0 (no compressed-header byte offset for VP9).
     * Bits [15:0] hold cprheader_offset; upper 16 bits are reserved.
     * BSP HAL line 671: reg64.cprheader_offset = 0. */
    rc |= emit_plain(out, RKVDEC2_VP9_REG_FRAME_FLAGS, 0u);

    /* reg65: cur_poc — synthesised POC for this kick.  BSP capture
     * starts at 1 for the first frame, so we offset by +1 vs the DPB's
     * next_poc (which starts at 0). */
    uint32_t cur_poc = (uint32_t)(dpb.next_poc + 1);
    rc |= emit_plain(out, RKVDEC2_VP9_REG_CUR_POC, cur_poc);

    /* Alias the pre-declared common-bank intra flag for the rest of
     * this function. */
    const bool is_intra = is_intra_pre;

    /* ---- reg67..reg74: per-segment parameter words (8 segments) ---- *
     * Each word carries the current frame's segmentation feature enables
     * and values for one VP9 segment.  BSP HAL fills these from the
     * previous frame's ls_info; we use pp.seg (current frame) since
     * RegbuildInputs carries current-frame state.
     * BSP struct Vdpu34xRegVp9dParam.reg67_74 (HAL lines 749-757).
     *
     * Bit layout per register (BSP reg67_74):
     *   [0]      segid_abs_delta        (only meaningful in reg67[0])
     *   [1]      segid_frame_qp_delta_en
     *   [10:2]   segid_frame_qp_delta   (9-bit signed, sign-extended from feature_data)
     *   [11]     segid_frame_loopfilter_value_en
     *   [18:12]  segid_frame_loopfilter_value (7-bit signed)
     *   [19]     segid_referinfo_en
     *   [21:20]  segid_referinfo
     *   [22]     segid_frame_skip_en
     *   [31:23]  reserved
     */
    if (pp.seg.enabled) {
        for (int seg = 0; seg < vp9::kMaxSegments; ++seg) {
            uint8_t mask = pp.seg.feature_mask[seg];
            /* ALT_Q (feature 0): QP delta */
            bool qp_en = (mask & 0x1u) != 0;
            int16_t qp_val = pp.seg.feature_data[seg][0]; /* signed */
            /* ALT_LF (feature 1): loop filter delta */
            bool lf_en = (mask & 0x2u) != 0;
            int16_t lf_val = pp.seg.feature_data[seg][1]; /* signed */
            /* REF_FRAME (feature 2): reference frame override */
            bool ref_en = (mask & 0x4u) != 0;
            int16_t ref_val = pp.seg.feature_data[seg][2]; /* 0..3 */
            /* SKIP (feature 3): force skip */
            bool skip_en = (mask & 0x8u) != 0;

            /* abs_delta only applies to seg 0's register (bit 0 of reg67) */
            uint32_t abs_delta_bit = (seg == 0) ? (pp.seg.abs_delta & 1u) : 0u;

            uint32_t word =
                abs_delta_bit                                        /* [0] */
                | ((qp_en ? 1u : 0u) << 1)                          /* [1] */
                | (((uint32_t)(qp_val & 0x1FFu)) << 2)              /* [10:2] 9-bit signed */
                | ((lf_en ? 1u : 0u) << 11)                         /* [11] */
                | (((uint32_t)(lf_val & 0x7Fu)) << 12)              /* [18:12] 7-bit signed */
                | ((ref_en ? 1u : 0u) << 19)                        /* [19] */
                | (((uint32_t)(ref_val & 0x3u)) << 20)              /* [21:20] */
                | ((skip_en ? 1u : 0u) << 22);                      /* [22] */

            rc |= emit_plain(out, RKVDEC2_VP9_REG_SEG0 + (uint32_t)(seg * 4), word);
        }
    }

    /* reg75: sticky state from the previous frame.  Bit layout per
     * upstream Linux vdpu381 patch:
     *   bits 0..13  mode_deltas_lastframe (2 × signed 7-bit packed)
     *   bit 16      segmentation_enable_lstframe
     *   bit 17      last_showframe
     *   bit 18      last_intra_only
     *   bit 19      last_widhheight_eqcur
     *   bits 20..22 color_space_lastkeyframe */
    {
        uint32_t reg75 = 0u;
        reg75 |= ((uint32_t)(in.last_mode_deltas[0] & 0x7F)) << 0;
        reg75 |= ((uint32_t)(in.last_mode_deltas[1] & 0x7F)) << 7;
        /* bit 15: vp9_segment_id_update (BSP HAL hal_vp9d_vdpu34x.c
         * ~line 534).  Set when the segmentation map must be rebuilt
         * — keyframe / intra-only, error-resilient mode, or an
         * explicit segmentation map update from the compressed header.
         * BSP also sets it on resolution change; we skip that branch
         * since the engine doesn't track previous dimensions yet. */
        const bool seg_id_update = is_intra
            || pp.error_resilient_mode
            || (pp.seg.enabled && pp.seg.update_map);
        if (seg_id_update) reg75 |= 1u << 15;
        if (in.last_segmentation_enabled)   reg75 |= 1u << 16;
        if (in.last_show_frame)              reg75 |= 1u << 17;
        /* reg75.last_intra_only reflects the CURRENT frame's intra-only
         * state (BSP HAL updates ls_info.last_intra_only = 1 on intra
         * frames BEFORE the reg75 write, so cold-start intra frames
         * already light up this bit even with prior in.last_intra_only=0). */
        if (in.last_intra_only || is_intra)  reg75 |= 1u << 18;
        if (in.last_widthheight_eqcur)       reg75 |= 1u << 19;
        reg75 |= ((uint32_t)(in.last_color_space & 0x7)) << 20;
        rc |= emit_plain(out, RKVDEC2_VP9_REG_STICKY_STATE, reg75);
    }

    /* reg76: tx_mode [2:0] and frame_reference_mode [4:3].
     * BSP HAL hal_vp9d_vdpu34x.c:760-761 writes the CURRENT frame's
     * values from pic_param->txmode / pic_param->refmode (despite the
     * upstream Linux patch naming the struct field `tx_mode_pre`).
     * Using prior-frame values worked accidentally on test streams
     * whose first-frame tx_mode happened to be 4 (= our default), but
     * broke on real-world streams whose keyframes use tx_mode=3
     * (ALLOW_32x32). */
    {
        uint32_t tx_ref = ((uint32_t)(pp.txmode         & 0x7u))
                        | (((uint32_t)(pp.reference_mode & 0x3u)) << 3);
        rc |= emit_plain(out, RKVDEC2_VP9_REG_TX_REF_MODE, tx_ref);
    }

    /* reg77: vp9_intercmd_num [23:0].  BSP capture shows 0 on our
     * streams; left explicit for the bulk-write path. */
    rc |= emit_plain(out, RKVDEC2_VP9_REG_INTERCMD_NUM, 0u);

    /* reg78: lasttile_size = stream payload minus first_partition_size.
     * pp.header_size is first_partition_size (the compressed header).
     * Field is [23:0]. */
    if (in.bitstream_bytes > pp.header_size) {
        uint32_t tile_sz = (in.bitstream_bytes - pp.header_size) & 0xFFFFFFu;
        rc |= emit_plain(out, RKVDEC2_VP9_REG_LAST_TILE_SIZE, tile_sz);
    }

    /* reg79..84 per-ref hor virstride (low 16); reg85..87 per-ref
     * y_virstride (low 28).  For each of last/golden/altref refs, use
     * the slot's stored dims.  Falls back to the current-frame strides
     * if the slot is invalid. */
    if (!is_intra) {
        struct StrideRegs { uint32_t y_hor, uv_hor, y_vir; };
        static const StrideRegs s_regs[3] = {
            { RKVDEC2_VP9_REG_LASTF_Y_HORSTRIDE,
              RKVDEC2_VP9_REG_LASTF_UV_HORSTRIDE,
              RKVDEC2_VP9_REG_LASTF_Y_VIRSTRIDE },
            { RKVDEC2_VP9_REG_GOLDF_Y_HORSTRIDE,
              RKVDEC2_VP9_REG_GOLDF_UV_HORSTRIDE,
              RKVDEC2_VP9_REG_GOLDF_Y_VIRSTRIDE },
            { RKVDEC2_VP9_REG_ALTRF_Y_HORSTRIDE,
              RKVDEC2_VP9_REG_ALTRF_UV_HORSTRIDE,
              RKVDEC2_VP9_REG_ALTRF_Y_VIRSTRIDE },
        };
        for (int r = 0; r < kRefsPerFrame; ++r) {
            uint8_t s = pp.frame_refs[r].index;
            uint32_t rw = pp.width, rh = pp.height, rbd = bit_depth;
            if (s < kNumRefFrames && dpb.slots[s].valid) {
                rw  = dpb.slots[s].width;
                rh  = dpb.slots[s].height;
                rbd = dpb.slots[s].bit_depth ? dpb.slots[s].bit_depth : 8u;
            }
            uint32_t ap = ((rw * rbd + 511u) & ~511u) / 8u;
            uint32_t hv = ap / 16u;
            uint32_t yv = (rh * ap) / 16u;
            rc |= emit_plain(out, s_regs[r].y_hor,  hv & 0xFFFFu);
            rc |= emit_plain(out, s_regs[r].uv_hor, hv & 0xFFFFu);
            rc |= emit_plain(out, s_regs[r].y_vir,  yv & 0x0FFFFFFFu);
        }
    }

    /* ---- Ref scaling + last-frame dims (inter frames only) ---------- *
     * Per BSP `Vdpu34xVp9dParam` and the upstream Linux vdpu381 patch
     * (https://github.com/dvab-sarma/.../aa00b89b):
     *   reg88/89  last  hor/ver scale  (16 bits each, separate regs)
     *   reg90/91  golden hor/ver scale
     *   reg92/93  altref hor/ver scale
     *   reg106/107 framewidth/height_last
     *   reg108/109 framewidth/height_golden
     *   reg110/111 framewidth/height_altref
     * reg104 is `reserved2` (no_use); reg105 carries count_update_en
     * + avs2_headlen — neither is a ref-scale register.  An earlier
     * version of this code wrote a packed (hor|ver) word to reg104..106;
     * that silently overwrote framewidth_last (reg106) with garbage,
     * which is why the BSP capture's reg106 always read 0x500 = 1280
     * while ours read scale_packed. */
    if (!is_intra) {
        static const uint32_t hor_regs[3] = {
            RKVDEC2_VP9_REG_LREF_HOR_SCALE,
            RKVDEC2_VP9_REG_GREF_HOR_SCALE,
            RKVDEC2_VP9_REG_AREF_HOR_SCALE,
        };
        static const uint32_t ver_regs[3] = {
            RKVDEC2_VP9_REG_LREF_VER_SCALE,
            RKVDEC2_VP9_REG_GREF_VER_SCALE,
            RKVDEC2_VP9_REG_AREF_VER_SCALE,
        };
        static const uint32_t dim_w_regs[3] = {
            RKVDEC2_VP9_REG_FRAMEWIDTH_LAST,
            RKVDEC2_VP9_REG_FRAMEWIDTH_GOLDEN,
            RKVDEC2_VP9_REG_FRAMEWIDTH_ALTREF,
        };
        static const uint32_t dim_h_regs[3] = {
            RKVDEC2_VP9_REG_FRAMEHEIGHT_LAST,
            RKVDEC2_VP9_REG_FRAMEHEIGHT_GOLDEN,
            RKVDEC2_VP9_REG_FRAMEHEIGHT_ALTREF,
        };

        for (int ref = 0; ref < vp9::kRefsPerFrame; ++ref) {
            uint8_t slot_idx = pp.frame_refs[ref].index;
            uint32_t ref_w = pp.width;
            uint32_t ref_h = pp.height;
            if (slot_idx < vp9::kNumRefFrames && dpb.slots[slot_idx].valid) {
                ref_w = dpb.slots[slot_idx].width;
                ref_h = dpb.slots[slot_idx].height;
            }
            rc |= emit_plain(out, hor_regs[ref],
                             vp9_scale_factor(ref_w, pp.width));
            rc |= emit_plain(out, ver_regs[ref],
                             vp9_scale_factor(ref_h, pp.height));
            rc |= emit_plain(out, dim_w_regs[ref], ref_w & 0xFFFFu);
            rc |= emit_plain(out, dim_h_regs[ref], ref_h & 0xFFFFu);
        }
    }

    /* ---- POC registers -------------------------------------------- */

    /* reg95/96/97: reference frame POCs (inter frames only).
     * frame_refs[0]=last, frame_refs[1]=golden, frame_refs[2]=altref.
     * Leave zero on keyframes. */
    if (!is_intra) {
        static const uint32_t poc_regs[3] = {
            RKVDEC2_VP9_REG_LAST_POC,
            RKVDEC2_VP9_REG_GOLDEN_POC,
            RKVDEC2_VP9_REG_ALTREF_POC,
        };
        for (int ref = 0; ref < vp9::kRefsPerFrame; ++ref) {
            uint8_t slot_idx = pp.frame_refs[ref].index;
            uint32_t poc_val = 0u;
            if (slot_idx < vp9::kNumRefFrames && dpb.slots[slot_idx].valid) {
                poc_val = (uint32_t)dpb.slots[slot_idx].poc;
            }
            rc |= emit_plain(out, poc_regs[ref], poc_val);
        }
    }

    /* reg94: ref_deltas_lastframe — pack 4 signed-7-bit deltas into
     * bits [27:0] (upstream vdpu381):
     *   delta[i] in bits [7i+6 : 7i], i ∈ {INTRA, LAST, GOLDEN, ALTREF}.
     * BSP HAL line 762-768: zeroed unconditionally then overwritten
     * only in the `if (!intraFlag)` branch — i.e. zero on keyframe. */
    {
        uint32_t reg94 = 0u;
        if (!is_intra_pre) {
            for (int i = 0; i < 4; ++i)
                reg94 |= ((uint32_t)(in.last_lf_ref_deltas[i] & 0x7F)) << (7 * i);
        }
        rc |= emit_plain(out, RKVDEC2_VP9_REG_REF_DELTAS_LAST, reg94 & 0x0FFFFFFFu);
    }

    /* reg98: col_ref_poc.
     * BSP HAL line 517-518: use col_ref_poc when non-zero, else cur_poc. */
    {
        uint32_t col_poc = (in.col_ref_poc != 0) ? (uint32_t)in.col_ref_poc : cur_poc;
        rc |= emit_plain(out, RKVDEC2_VP9_REG_COL_REF_POC, col_poc);
    }

    /* reg99: prob_ref_poc.  BSP HAL line 585-590: emits the engine's
     * tracked POC only when prob_ctx_valid[fcx]; on the cold-context
     * branch (which keyframes always hit because BSP just zeroed the
     * valid array), reg99 = 0. */
    {
        uint8_t ctx = pp.frame_context_idx & 0x3u;
        bool ctx_valid = in.prob_ctx_valid[ctx] &&
                         !pp.error_resilient_mode &&
                         !is_intra_pre;
        uint32_t v = ctx_valid
                         ? ((uint32_t)in.prob_ref_poc[ctx] & 0xFFFFu)
                         : 0u;
        rc |= emit_plain(out, RKVDEC2_VP9_REG_PROB_REF_POC, v);
    }

    /* reg100: segid_ref_poc.  BSP HAL line 540: forced to 0 when any
     * segmap rebuild trigger fires (intra, resolution change,
     * seg.update_map, error_resilient). */
    {
        bool seg_rebuild = is_intra_pre || pp.error_resilient_mode ||
                           (pp.seg.enabled && pp.seg.update_map);
        rc |= emit_plain(out, RKVDEC2_VP9_REG_SEGID_REF_POC,
                         seg_rebuild ? 0u : (uint32_t)in.segid_ref_poc);
    }

    /* ---- reg103: param_flags ---------------------------------------- *
     * Per BSP `Vdpu34xVp9dParam.reg103` (mpp/hal/rkdec/inc/vdpu34x_vp9d.h
     * lines 182-198), the flag bitfield starts at bit 20 with 20 bits of
     * leading reserve:
     *   [19:0]  reserved = 0
     *   [20]    prob_update_en
     *   [21]    refresh_en
     *   [22]    prob_save_en
     *   [23]    intra_only_flag
     *   [24]    txfmmode_rfsh_en
     *   [25]    ref_mode_rfsh_en
     *   [26]    single_ref_rfsh_en
     *   [27]    comp_ref_rfsh_en
     *   [28]    interp_filter_switch_en
     *   [29]    allow_high_precision_mv
     *   [30]    last_key_frame_flag
     *   [31]    inter_coef_rfsh_flag
     * Reference values from BSP capture (regs_kf/regs_000.bin, regs_inter):
     *   keyframe (intra): 0x0ED00000
     *   inter (no intra): 0x6F500000  */
    {
        uint32_t flags = 0u;
        flags |= (1u << 20);                                       /* prob_update_en */
        if (!pp.error_resilient_mode && !pp.frame_parallel_decoding_mode)
            flags |= (1u << 21);                                   /* refresh_en */
        if (pp.refresh_frame_context)
            flags |= (1u << 22);                                   /* prob_save_en */
        if (is_intra)
            flags |= (1u << 23);                                   /* intra_only_flag */
        if (!is_intra && pp.txmode == 4)
            flags |= (1u << 24);                                   /* txfmmode_rfsh_en */
        flags |= (1u << 25);                                       /* ref_mode_rfsh_en */
        flags |= (1u << 26);                                       /* single_ref_rfsh_en */
        flags |= (1u << 27);                                       /* comp_ref_rfsh_en */
        if (!is_intra && pp.interp_filter == 4)
            flags |= (1u << 28);                                   /* interp_filter_switch_en */
        if (pp.allow_high_precision_mv)
            flags |= (1u << 29);                                   /* allow_high_precision_mv */
        if (in.last_intra_only)
            flags |= (1u << 30);                                   /* last_key_frame_flag */
        /* bit 31: inter_coef_rfsh_flag = 0 */
        rc |= emit_plain(out, RKVDEC2_VP9_REG_PARAM_FLAGS, flags);
    }

    /* ---- Common-addr bank (idx 128..142) -------------------------------- *
     * Mirrors hal_vp9d_vdpu34x.c common-addr section (~lines 520..560). */

    /* Common-addr bank reg128..132 layout (per rkvdec2_h26x_regs.h and
     * BSP capture):
     *   reg128 RLC_BASE       — bitstream input
     *   reg129 RLCWRITE_BASE  — scratch (set to RLC_BASE per BSP/upstream)
     *   reg130 DECOUT_BASE    — output frame
     *   reg131 COLMV_CUR_BASE — colmv for current pic
     *   reg132 ERROR_REF_BASE — fallback ref */
    rc |= emit_iova(out, RKVDEC2_REG_RLC_BASE,      in.bitstream_handle,      in.bitstream_offset);
    rc |= emit_iova(out, RKVDEC2_REG_RLCWRITE_BASE, in.bitstream_handle,      in.bitstream_offset);
    rc |= emit_iova(out, RKVDEC2_REG_DECOUT_BASE,   in.decout_frame_handle,   0);
    rc |= emit_iova(out, RKVDEC2_REG_COLMV_CUR_BASE,in.decout_colmv_handle,   0);
    /* reg132 ERROR_REF_BASE: trans_tbl_vp9d excludes; BSP kernel never
     * writes it for VP9.  SUBMIT_DENSE writes everything we emit, so
     * leaving this in injects a stray pointer the codec then
     * prefetches → hw=0x23 timeout.  Skip. */

    /* reg133..142: RCB scratch regions (10 slots).
     * BSP HAL: vdpu34x_setup_rcb writes one buffer fd per region.
     * BSP capture confirms 10 regions backed by a single packed RCB
     * buffer (all 10 entries map to the same iova). */
    for (int i = 0; i < 10; ++i) {
        if (in.rcb_handles[i])
            rc |= emit_iova(out, RKVDEC2_REG_RCB_BASE_FIRST + (uint32_t)(i * 4),
                            in.rcb_handles[i], in.rcb_offsets[i]);
    }

    /* ---- Codec-addr bank (idx 160..197) --------------------------------- *
     * Layout verified against tests/data/vp9/bsp_capture/kf.dmesg (RK3588
     * mpi_dec_test trace) and BSP Vdpu34xVp9dAddr struct.  See
     * mft/rkvdec2_vp9_regs.h for the full slot table. */

    /* reg160: delta_prob_base — probe buffer offset 0.  HW reads
     * delta-coded prob updates from this region. */
    rc |= emit_iova(out, RKVDEC2_VP9_REG_DELTA_PROB_BASE, in.probe_handle, 0);

    /* reg162: last_prob_base — baseline prob HW starts from.
     * BSP HAL (~line 582-587): use prob_loop[fcx] iff prob_ctx_valid[fcx]
     * AND the frame is not intra (BSP zeroes prob_ctx_valid[*] on any
     * intra / error_resilient frame at line 549-557 before reaching
     * this decision).  On a 2nd-GOP keyframe with a hot prior context,
     * skipping the intra check routes reg162 to prob_loop[fcx] and the
     * kick wedges. */
    {
        uint8_t fcx = pp.frame_context_idx & 0x3u;
        bool ctx_valid = in.prob_ctx_valid[fcx] &&
                         !pp.error_resilient_mode &&
                         !is_intra_pre;
        uint64_t last_prob = ctx_valid ? in.prob_loop_handle
                                       : in.prob_default_handle;
        rc |= emit_iova(out, RKVDEC2_VP9_REG_LAST_PROB_BASE, last_prob, 0);
    }

    /* reg164/165/166: active inter-frame references (last/golden/altref).
     * BSP capture self-references these to the decout frame on
     * keyframes so the codec never dereferences a zero iova; do the
     * same. */
    {
        static const uint32_t ref_regs[3] = {
            RKVDEC2_VP9_REG_REF_LAST_BASE,
            RKVDEC2_VP9_REG_REF_GOLDEN_BASE,
            RKVDEC2_VP9_REG_REF_ALTREF_BASE,
        };
        for (int r = 0; r < kRefsPerFrame; ++r) {
            uint64_t h = in.decout_frame_handle;
            if (!is_intra) {
                uint8_t s = pp.frame_refs[r].index;
                if (s < kNumRefFrames && dpb.slots[s].valid)
                    h = dpb.slots[s].frame_handle;
            }
            rc |= emit_iova(out, ref_regs[r], h, 0);
        }
    }

    /* reg167: count_prob_base — HW writes prob counter accumulations
     * into the second region of the probe buffer (offset 0x2000 =
     * PROB_SIZE_ALIGN_TO_4K). */
    rc |= emit_iova(out, RKVDEC2_VP9_REG_COUNT_PROB_BASE,
                    in.probe_handle, 0x2000u);

    /* reg168/169: segment-ID maps (previous, current). */
    rc |= emit_iova(out, RKVDEC2_VP9_REG_SEGID_LAST_BASE, in.segid_last_handle, 0);
    rc |= emit_iova(out, RKVDEC2_VP9_REG_SEGID_CUR_BASE,  in.segid_cur_handle,  0);

    /* reg170: colmv of the "last" reference slot.  On keyframes BSP
     * self-references to the current colmv buffer so the codec never
     * dereferences iova 0. */
    {
        uint64_t h = in.decout_colmv_handle;
        if (!is_intra) {
            uint8_t last_slot = pp.frame_refs[0].index;
            if (last_slot < kNumRefFrames && dpb.slots[last_slot].valid)
                h = dpb.slots[last_slot].colmv_handle;
        }
        rc |= emit_iova(out, RKVDEC2_VP9_REG_REF_COLMV_BASE, h, 0);
    }

    /* reg161/r163/r171: HEVC-only regs; trans_tbl_vp9d excludes.  Don't
     * emit — codec prefetches the stray pointer and wedges with
     * hw=0x23. */

    /* reg172: update_prob_wr_base — where HW writes the post-decode
     * adapted prob context.  Points at prob_loop[fcx] so the next
     * inter frame in this context (via reg162) reads from it. */
    rc |= emit_iova(out, RKVDEC2_VP9_REG_UPDATE_PROB_WR_BASE,
                    in.prob_loop_handle, 0);

    /* ---- Statistic / AXI QoS bank (idx 256..277) -------------------- *
     * Mirrors BSP vdpu34x_setup_statistic.  Without these the codec's
     * AXI bus may stall waiting for memory and trip the internal cycle
     * timeout (INT_STATUS=0x23 with no obvious error bit).
     *
     *   reg256.axi_perf_work_e = 1   bit  0
     *   reg256.axi_perf_clr_e  = 1   bit  1
     *   reg256.axi_cnt_type    = 1   bit  3
     *   reg257.addr_align_type = 1   bits[1:0]
     *   reg270.bus2mc_buffer_qos_level = 0xFF  bits[7:0]
     *   reg270.axi_rd_hurry_level      = 3     bits[17:16]
     *   reg270.axi_wr_qos              = 1     bits[21:20]
     *   reg270.axi_wr_hurry_level      = 1     bits[25:24]
     *   reg270.axi_rd_qos              = 3     bits[29:28]
     *   → reg270 = 0x311300FF
     *   reg271_wr_wait_cycle_qos = 0  (already zero from memset). */
    rc |= emit_plain(out, 256u * 4u, 0x0000000Bu);
    rc |= emit_plain(out, 257u * 4u, 0x00000001u);
    rc |= emit_plain(out, 270u * 4u, 0x311300FFu);

    /* reg173..179: BSP `_no_use` — trans_tbl_vp9d excludes; kernel
     * writes 0.  Don't emit. */

    /* reg181..196: per-DPB-slot ref_colmv[16].  BSP HAL writes ONLY
     * the three active VP9 refs (r181-r183) and leaves r184-r196 at
     * 0 regardless of DPB validity (the loop bound is `i < 3`).  We
     * had a "if (dpb.slots[i].valid) emit" path which set r184-r188
     * on inter frames after the keyframe refreshed all 8 DPB slots;
     * the Windows codec prefetches those addresses and wedges with
     * hw=0x23.  Match BSP exactly: emit r181-r183 only. */
    for (int i = 0; i < 3; ++i) {
        uint32_t off = RKVDEC2_VP9_REG_REF_COLMV_SLOT_BASE + (uint32_t)(i * 4);
        uint64_t h = (i < kNumRefFrames && dpb.slots[i].valid)
                         ? dpb.slots[i].colmv_handle
                         : in.decout_colmv_handle;
        rc |= emit_iova(out, off, h, 0);
    }

    if (rc) return RegBuildStatus::UncoveredReg;
    return RegBuildStatus::Ok;
}

/* ---- Prob buffer fill ------------------------------------------------ *
 *
 * Bit-packs the VP9 codec's per-frame probability "delta" region.  The
 * layout is a sequence of 1-bit "flag" fields (which probs the
 * compressed header updated) followed by 8-bit "value" fields (the
 * actual delta — or, for keyframes, the default prob to load), with
 * 128-bit alignment between sections.  See:
 *   mpp/hal/rkdec/vp9d/hal_vp9d_com.c::hal_vp9d_prob_flag_delta
 * The BSP source is reference-only for layout; all code below is fresh.
 *
 * Buffer size is exactly PROB_SIZE = 4864 bytes.  The codec writes the
 * 16 KiB count-writeback region at offset 0x2000; this function only
 * touches the first 4864 bytes (CPU-write region).
 * --------------------------------------------------------------------- */

namespace {

/* LSB-first bit packer matching MPP's BitputCtx_t / mpp_put_bits.
 * Writes are packed into 64-bit words, then stored little-endian.
 * Caller pre-zeros the buffer so `put` only OR-s set bits. */
struct ProbBitPacker {
    uint64_t *data;
    size_t    cap_u64;
    size_t    word_idx;
    int       bit_in_word;   /* 0..63 */

    void init(uint64_t *buf, size_t words) {
        data = buf;
        cap_u64 = words;
        word_idx = 0;
        bit_in_word = 0;
    }
    void put(uint64_t value, int nbits) {
        if (nbits == 0 || word_idx >= cap_u64) return;
        uint64_t mask = (nbits == 64) ? ~0ULL : ((1ULL << nbits) - 1ULL);
        uint64_t v = value & mask;
        if (bit_in_word + nbits <= 64) {
            data[word_idx] |= v << bit_in_word;
            bit_in_word += nbits;
            if (bit_in_word == 64) { word_idx++; bit_in_word = 0; }
        } else {
            int low = 64 - bit_in_word;
            data[word_idx] |= (v & ((1ULL << low) - 1ULL)) << bit_in_word;
            word_idx++;
            if (word_idx < cap_u64) data[word_idx] |= v >> low;
            bit_in_word = nbits - low;
        }
    }
    void align(int nbits, int fill) {
        size_t total   = word_idx * 64 + (size_t)bit_in_word;
        size_t aligned = (total + (size_t)nbits - 1) & ~((size_t)nbits - 1);
        size_t pad     = aligned - total;
        while (pad > 0) {
            int chunk = pad > 32 ? 32 : (int)pad;
            put(fill ? ((1ULL << chunk) - 1ULL) : 0, chunk);
            pad -= chunk;
        }
    }
};

/* Partition order: our parser reads VP9 partition probs in bitstream
 * order, which is the SAME order the hardware expects (8x8 contexts
 * first → 64x64 contexts last).  BSP reads bitstream into a separate
 * 4x4x3 array with index inversion and then TRANSes it back; we skip
 * both steps.  See vp9d.c::vp9d_fill_picparams TRANS comment. */
static inline int partition_spec_row(int hw_row) {
    return hw_row;
}

}  /* anon */

void Vp9Regbuilder_FillProbs(const PicParams   &pp,
                              const ProbUpdates &pu,
                              uint8_t           *prob_buf)
{
    if (!prob_buf) return;
    constexpr size_t kProbSize = 4864;
    memset(prob_buf, 0, kProbSize);

    ProbBitPacker bp;
    bp.init(reinterpret_cast<uint64_t *>(prob_buf), kProbSize / 8);

    const bool is_intra = (pp.frame_type == 0) || pp.intra_only;

    /* ============================================================
     * Section 1: 1-bit update flags  ("sb info / mode info")
     * ============================================================ */

    /* 48 partition flags (16 contexts × 3 probs).  Zero on keyframe
     * (HW uses kKfPartitionProbs unconditionally); reorder from spec
     * to HW row order on inter. */
    if (is_intra) {
        for (int i = 0; i < 48; ++i) bp.put(0, 1);
    } else {
        for (int hw_row = 0; hw_row < 16; ++hw_row) {
            int sr = partition_spec_row(hw_row);
            for (int k = 0; k < 3; ++k)
                bp.put(pu.partition_flag[sr][k], 1);
        }
    }

    /* 3 segment pred_prob flags + 7 segment tree_prob flags — always 0
     * (BSP never sets these in the flag path). */
    for (int i = 0; i < 3 + 7; ++i) bp.put(0, 1);

    /* Per-context skip flags. */
    for (int i = 0; i < 3; ++i) bp.put(pu.skip_flag[i], 1);

    /* TX-size flags: tx32p[2][3], tx16p[2][2], tx8p[2]. */
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j)
            bp.put(pu.tx_size_32x32_flag[i][j], 1);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            bp.put(pu.tx_size_16x16_flag[i][j], 1);
    for (int i = 0; i < 2; ++i)
        bp.put(pu.tx_size_8x8_flag[i][0], 1);

    /* 4 intra-vs-inter flags: zero on keyframe, per-context on inter. */
    if (is_intra) {
        for (int i = 0; i < 4; ++i) bp.put(0, 1);
    } else {
        for (int i = 0; i < 4; ++i) bp.put(pu.is_inter_flag[i], 1);
    }

    /* 3-bit reserve closes the first "sb info" 80-bit section for
     * both intra and inter paths (BSP hal_vp9d_com.c lines 1498 and
     * 1644 — both call `mpp_put_bits(&bp, 0, 3)` here). */
    bp.put(0, 3);

    if (!is_intra) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 9; ++j) bp.put(pu.y_mode_flag[i][j], 1);

        for (int i = 0; i < 5; ++i) bp.put(pu.comp_mode_flag[i], 1);

        for (int i = 0; i < 5; ++i) bp.put(pu.comp_ref_flag[i], 1);

        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 2; ++j)
                bp.put(pu.single_ref_flag[i][j], 1);

        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 3; ++j)
                bp.put(pu.inter_mode_flag[i][j], 1);

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                bp.put(pu.interp_filter_flag[i][j], 1);

        bp.put(0, 11);  /* reserve */
    }

    /* ============================================================
     * Section 2: coef-update flags (intra then inter)
     *
     * Each (tx, plane, intra/inter) emits 4 chunks of 27 bits +
     * 5 zero-bit pad.  Flat-read pu.coef_changed[tx][plane][L][...]
     * as a 108-byte natural-order array.
     * ============================================================ */
    /* Our struct uses [tx][ref][plane] order (ref = intra/inter); BSP
     * iterates [tx][plane][ref].  Swap the inner two indices when
     * flat-reading the [band][ctx][node] block. */
    for (int L = 0; L < 2; ++L) {        /* L = intra/inter */
        for (int i = 0; i < 4; ++i) {    /* tx */
            for (int j = 0; j < 2; ++j) {/* plane */
                const uint8_t *p = &pu.coef_changed[i][L][j][0][0][0];
                for (int k = 0; k < 4; ++k) {
                    for (int m = 0; m < 27; ++m)
                        bp.put(p[27 * k + m] ? 1u : 0u, 1);
                    bp.put(0, 5);
                }
            }
        }
    }

    /* ============================================================
     * Section 3 (inter-only): UV-mode flags + MV-related flags.
     * Our parser doesn't update uv_mode (spec doesn't allow inter
     * uv-mode updates), so these are all zero — but we still need
     * to emit the same number of zero bits to keep alignment.
     * ============================================================ */
    if (!is_intra) {
        /* uv_mode flags: 10 × 9, but BSP inserts a 5-bit reserve
         * every 3 rows for alignment. */
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 1);
        bp.put(0, 5);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 1);
        bp.put(0, 5);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 1);
        bp.put(0, 5);
        for (int i = 0; i < 1; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 1);
        bp.put(0, 23);

        /* Per-context MV flag bits (matches BSP iteration order). */
        for (int i = 0; i < 3; ++i)
            bp.put(pu.mv_joints_flag[i], 1);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_sign_flag[c], 1);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 10; ++j)
                bp.put(pu.mv_classes_flag[c][j], 1);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_class0_flag[c], 1);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 10; ++j)
                bp.put(pu.mv_bits_flag[c][j], 1);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 3; ++k)
                    bp.put(pu.mv_class0_fp_flag[c][j][k], 1);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 3; ++j)
                bp.put(pu.mv_fp_flag[c][j], 1);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_class0_hp_flag[c], 1);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_hp_flag[c], 1);

        bp.put(0, 11);                  /* reserve */
        for (int i = 0; i < 8; ++i)
            bp.put(0, 16);              /* reserve block */
    }

    bp.align(128, 0);   /* BSP `mpp_put_align(&bp, 128, 0)` after the
                         * flag section, before the value section. */

    /* ============================================================
     * Section 4: 8-bit delta values, starting from the partition
     * row of the value region.
     * ============================================================ */

    /* Partition probs: keyframe uses static HW-order table; inter
     * uses pu.partition reordered spec→HW. */
    if (is_intra) {
        for (int row = 0; row < 16; ++row)
            for (int k = 0; k < 3; ++k)
                bp.put(kKfPartitionProbsHw[row][k], 8);
    } else {
        for (int hw_row = 0; hw_row < 16; ++hw_row) {
            int sr = partition_spec_row(hw_row);
            for (int k = 0; k < 3; ++k)
                bp.put(pu.partition[sr][k], 8);
        }
    }

    /* Segment pred_probs (3) + tree_probs (7).  Emit zeros when seg
     * isn't enabled — BSP's `s->prob.segpred` / `s->prob.seg` are zero
     * out of the gate and only get filled in when seg.update_map fires.
     * (Our parser uses 255 as the "inactive" default per spec §10.7; we
     * override here for HW-buffer-exact byte matching.) */
    if (pp.seg.enabled) {
        for (int i = 0; i < 3; ++i) bp.put(pp.seg.pred_probs[i], 8);
        for (int i = 0; i < 7; ++i) bp.put(pp.seg.tree_probs[i], 8);
    } else {
        for (int i = 0; i < 3 + 7; ++i) bp.put(0, 8);
    }

    /* skip deltas: 3. */
    for (int i = 0; i < 3; ++i) bp.put(pu.skip[i], 8);

    /* tx_size deltas: tx32p[2][3], tx16p[2][2], tx8p[2]. */
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 3; ++j) bp.put(pu.tx_size_32x32[i][j], 8);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) bp.put(pu.tx_size_16x16[i][j], 8);
    for (int i = 0; i < 2; ++i)     bp.put(pu.tx_size_8x8[i][0], 8);

    /* intra-vs-inter deltas: 4. */
    for (int i = 0; i < 4; ++i) bp.put(pu.is_inter[i], 8);

    if (is_intra) {
        bp.align(128, 0);

        /* Intra coef_delta: 4 (tx) × 2 (plane) × natural [6][6][3].
         * 5-byte zero pad after each 27-byte chunk; 128-bit align at
         * end of every (tx, plane) pair. */
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 2; ++j) {
                int byte_count = 0;
                for (int k = 0; k < 6; ++k)
                    for (int m = 0; m < 6; ++m)
                        for (int n = 0; n < 3; ++n) {
                            bp.put(pu.coef_values[i][0][j][k][m][n], 8);
                            if (++byte_count == 27) {
                                bp.align(128, 0);
                                byte_count = 0;
                            }
                        }
                bp.align(128, 0);
            }
        }

        /* Keyframe intra-mode probs: 10 × (10 × 9 y_mode + 23 uv_mode +
         * 128-align).  vp9_kf_uv_mode_prob is read flat at i*23. */
        const uint8_t *uv_flat = &kKfUvModeProbs[0][0];
        for (int i = 0; i < 10; ++i) {
            int byte_count = 0;
            for (int j = 0; j < 10; ++j)
                for (int k = 0; k < 9; ++k) {
                    bp.put(kKfYModeProbs[i][j][k], 8);
                    if (++byte_count == 27) {
                        bp.align(128, 0);
                        byte_count = 0;
                    }
                }
            if (i < 4) {
                int n_real = (i < 3) ? 23 : 21;
                int m;
                for (m = 0; m < n_real; ++m)
                    bp.put(uv_flat[i * 23 + m], 8);
                for (; m < 23; ++m) bp.put(0, 8);
            } else {
                for (int m = 0; m < 23; ++m) bp.put(0, 8);
            }
            bp.align(128, 0);
        }

        /* Inter coef_delta: ref=1 (inter). */
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 2; ++j) {
                int byte_count = 0;
                for (int k = 0; k < 6; ++k)
                    for (int m = 0; m < 6; ++m)
                        for (int n = 0; n < 3; ++n) {
                            bp.put(pu.coef_values[i][1][j][k][m][n], 8);
                            if (++byte_count == 27) {
                                bp.align(128, 0);
                                byte_count = 0;
                            }
                        }
                bp.align(128, 0);
            }
        }
    } else {
        bp.put(0, 24);   /* reserve */

        /* y_mode delta: 4 × 9. */
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 9; ++j) bp.put(pu.y_mode[i][j], 8);

        /* reference_mode delta: 5 (comp_mode). */
        for (int i = 0; i < 5; ++i) bp.put(pu.comp_mode[i], 8);

        /* comp_ref delta: 5. */
        for (int i = 0; i < 5; ++i) bp.put(pu.comp_ref[i], 8);

        /* single_ref delta: 5 × 2. */
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 2; ++j) bp.put(pu.single_ref[i][j], 8);

        /* mv_mode delta: 7 × 3. */
        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 3; ++j) bp.put(pu.inter_mode[i][j], 8);

        /* interp_filter delta: 4 × 2. */
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j) bp.put(pu.interp_filter[i][j], 8);

        for (int i = 0; i < 11; ++i) bp.put(0, 8);  /* reserve */

        /* Intra coef_delta then inter coef_delta — same walk shape
         * as keyframe, but with the inline 40-bit pad after each 27
         * entries instead of an align() call. */
        for (int L = 0; L < 2; ++L) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 2; ++j) {
                    int byte_count = 0;
                    for (int k = 0; k < 6; ++k)
                        for (int m = 0; m < 6; ++m)
                            for (int n = 0; n < 3; ++n) {
                                bp.put(pu.coef_values[i][L][j][k][m][n], 8);
                                if (++byte_count == 27) {
                                    bp.put(0, 40);
                                    byte_count = 0;
                                }
                            }
                }
            }
        }

        /* uv_mode deltas: blocks of 27 bytes + 40-bit pad, then 23×0xFF
         * trailer.  Parser doesn't update uv_mode for inter frames so
         * values are all zero in pu (unused — BSP keeps the bytes for
         * alignment regardless). */
        const uint8_t zeros_uv[9] = {0};
        (void)zeros_uv;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 8);
        bp.put(0, 40);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 8);
        bp.put(0, 40);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 8);
        bp.put(0, 40);
        for (int i = 0; i < 1; ++i)
            for (int j = 0; j < 9; ++j) bp.put(0, 8);
        for (int i = 0; i < 23; ++i) bp.put(0xff, 8);

        /* MV deltas. */
        for (int i = 0; i < 3; ++i) bp.put(pu.mv_joints[i], 8);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_sign[c], 8);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 10; ++j) bp.put(pu.mv_classes[c][j], 8);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_class0[c], 8);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 10; ++j) bp.put(pu.mv_bits[c][j], 8);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 3; ++k)
                    bp.put(pu.mv_class0_fp[c][j][k], 8);
        for (int c = 0; c < 2; ++c)
            for (int j = 0; j < 3; ++j) bp.put(pu.mv_fp[c][j], 8);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_class0_hp[c], 8);
        for (int c = 0; c < 2; ++c) bp.put(pu.mv_hp[c], 8);

        for (int i = 0; i < 27; ++i) bp.put(0, 8);  /* reserve */
    }

    bp.align(128, 0);
}

} /* namespace vp9 */
