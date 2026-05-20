/* mft/regbuilder_h265.cpp — parsed HEVC bitstream → register-write list.
 *
 * Phase 3b Task 4 (HEVC).  Mirrors hal_h265d_vdpu34x_gen_regs
 * (rockchip-linux/mpp, mpp/hal/rkdec/h265d/hal_h265d_vdpu34x.c:836)
 * line by line.  Each non-trivial fill cites the BSP source line so a
 * reviewer can audit against vdpu34x_h265d.h / vdpu34x_com.h.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "regbuilder_h265.h"
#include "rkvdec2_h265_regs.h"
#include "rkmpp_resolution.h"

#include <string.h>

namespace {

/* ---- Bit-packing helpers (used to build the register banks the BSP
 * struct fills via bitfield assignments).  The vdpu34x bit-packing is
 * little-endian, LSB-first, no spilling across word boundaries. */
static inline uint32_t set_bits(uint32_t reg, uint32_t value,
                                uint32_t shift, uint32_t mask)
{
    return (reg & ~(mask << shift)) | ((value & mask) << shift);
}

/* Stamp one plain value into the dense bank.  Idx 10 (kick) is routed
 * to out->KickValue.  Returns 1 on uncovered offset. */
static int emit_plain(H26xDenseOutput *out, uint32_t off, uint32_t value) {
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

/* Record an iova-substitution slot.  Zero handle is a no-op (slot
 * remains 0), matching the sparse path's gating-on-handle pattern. */
static int emit_iova(H26xDenseOutput *out, uint32_t off,
                     uint64_t handle, uint32_t iova_offset) {
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

extern "C"
H265RegBuildStatus H265BuildDenseRegs(const H265ParseResult *parsed,
                                      const H265BufferRefs  *bufs,
                                      uint32_t               current_pic_index,
                                      H26xDenseOutput       *out)
{
    if (!parsed || !bufs || !out) return H265_REGBUILD_MISSING_INPUT;
    if (!parsed->has_slice)       return H265_REGBUILD_MISSING_INPUT;
    if (parsed->active_sps_id < 0 || parsed->active_pps_id < 0)
        return H265_REGBUILD_MISSING_INPUT;
    if (!bufs->bitstream || !bufs->output_frame || !bufs->colmv_cur)
        return H265_REGBUILD_MISSING_INPUT;
    /* HEVC always needs the CABAC init blob — the engine has no CAVLC
     * fallback (unlike H.264 which can decode CAVLC streams with the
     * cabac table absent). */
    if (!bufs->cabac_init_table)
        return H265_REGBUILD_UNSUPPORTED;

    /* Bank MUST start zero so the bulk MMIO write only sets bits the
     * regbuilder actually means to set. */
    memset(out, 0, sizeof(*out));
    (void)current_pic_index;  /* see reg028 note below */

    const H265Sps *sps = &parsed->sps[parsed->active_sps_id];
    const H265Pps *pps = &parsed->pps[parsed->active_pps_id];
    const H265SliceHeader *slice = &parsed->slice;
    if (!sps->valid || !pps->valid)
        return H265_REGBUILD_MISSING_INPUT;

    /* Bit-depth + chroma-format gate.  Mirrors the H.264 regbuilder:
     * accept 8-bit (minus8=0) and 10-bit (minus8=2) 4:2:0 only, with
     * luma == chroma bit-depth.  4:2:2/4:4:4 and 12-bit are silicon-
     * supported on vdpu34x in theory but our engine output path isn't
     * wired up for them — reject loudly. */
    if (sps->chroma_format_idc != 1)
        return H265_REGBUILD_UNSUPPORTED;
    if (sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8)
        return H265_REGBUILD_UNSUPPORTED;
    if (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2)
        return H265_REGBUILD_UNSUPPORTED;

    /* Frame-geometry derivation matches gen_regs:836..914 — luma stride
     * is hor_stride from the frame allocator (16-aligned for raster NV12,
     * width*10/8 16-aligned for NV15 Main10), NOT CtbSizeY-aligned.
     * CtbSizeY only enters here through the RCB-info sizing (handled by
     * h265_packed_tables).  The packed SPS unit carries the bit-depth
     * (h265_packed_tables.cpp:160-161); these stride registers must
     * match or the SAO write fabric corrupts adjacent rows. */
    uint32_t width_px      = sps->pic_width_in_luma_samples;
    uint32_t height_px     = sps->pic_height_in_luma_samples;
    /* Trust-boundary gate — kernel doesn't re-validate dimensions. */
    if (!RkmppValidateResolution(width_px, height_px))
        return H265_REGBUILD_UNSUPPORTED;
    const uint32_t bpp_num = (sps->bit_depth_luma_minus8 == 2) ? 10u : 8u;
    uint32_t coded_w_px    = (width_px + 15u) & ~15u;
    uint32_t luma_stride   = ((coded_w_px * bpp_num + 7u) / 8u + 15u) & ~15u;
    uint32_t chroma_stride = luma_stride;          /* 4:2:0 same horiz */
    uint32_t y_height      = (height_px + 15u) & ~15u;
    uint32_t y_size        = luma_stride * y_height;

#define EMIT_P(off, val) do { if (emit_plain(out, (off), (val))) \
                              return H265_REGBUILD_TOO_MANY_REGS; } while (0)
#define EMIT_I(off, h, ioff) do { if (emit_iova(out, (off), (h), (ioff))) \
                                  return H265_REGBUILD_TOO_MANY_REGS; } while (0)

    /* ---- Common bank: dec_mode + clk-gate + error config ----------- *
     * BSP: hal_h265d_vdpu34x_gen_regs:836..1010.  The struct is memset
     * to 0 before fill so any field we don't write reads back as zero —
     * matches the kernel-side BSP capture for HEVC IDR submissions. */

    /* reg009.dec_mode == 0 for HEVC (gen_regs leaves the memset-zero
     * untouched; cf. rkvdec2_h265_regs.h commentary).  We still emit
     * it explicitly so the kernel's iova-substitution scan sees a
     * deterministic value at offset 36. */
    EMIT_P(RKVDEC2_REG_DEC_MODE, RKVDEC2_DEC_MODE_HEVC);

    /* reg011 — gen_regs:1018,1042: dec_clkgate_e=1, dec_timeout_e=1,
     * buf_empty_en=1, plus pix_range_detection_e=1 from
     * vdpu34x_setup_statistic (gen_regs:1144). */
    EMIT_P(RKVDEC2_REG_IMPORTANT_EN,
           RKVDEC2_DEC_CLKGATE_E | RKVDEC2_DEC_TIMEOUT_E |
           RKVDEC2_BUF_EMPTY_EN  | RKVDEC2_PIX_RANGE_DETECTION_E);

    /* reg012 — BSP HEVC capture (mpp.shim.h265.log AU 0) shows 0x00000083
     * = bit 0 + bit 1 + bit 7.  Differences vs. our earlier H.264-derived
     * guess:
     *   - bit 0  : rk3588 HEVC HAL sets this unconditionally; the
     *              vdpu34x_com.h SWREG12 docs label it dec_global_en.
     *              Both the H.264 and HEVC BSP captures show it set,
     *              but the H.264 regbuilder still gets away with it
     *              cleared because reg012 is rewritten elsewhere on the
     *              kernel path; HEVC needs it from user-mode.
     *   - bit 6  : NOT set in BSP HEVC capture (wr_ddr_align_en is left
     *              cleared even with tiles disabled — the HEVC HAL
     *              skips that bit, our "always-on for non-tile" guess
     *              was wrong).
     *   - bit 8  : NOT set in BSP HEVC capture (scanlist_addr_valid_en
     *              cleared because the HEVC HAL gates it on
     *              scaling_list_enabled_flag, unlike H.264 which sets
     *              it unconditionally).
     * Match BSP: bit 0 + COLMV_COMPRESS_EN + WAIT_RESET_EN. */
    {
        uint32_t reg012 = (1u << 0) |               /* dec_global_en       */
                          RKVDEC2_COLMV_COMPRESS_EN |
                          RKVDEC2_WAIT_RESET_EN;
        if (sps->scaling_list_enabled_flag)
            reg012 |= RKVDEC2_SCANLIST_ADDR_VALID_EN;
        EMIT_P(RKVDEC2_REG_SECONDARY_EN, reg012);
    }

    /* reg013 — gen_regs:944..946,1138..1141: timeout_mode=1,
     * h26x_streamd_error_mode=1, colmv_error_mode=1, h26x_error_mode=1,
     * cur_pic_is_idr derived from NAL unit type.
     *
     * REQ_TIMEOUT_RST_SEL (bit 1) intentionally NOT set — neither
     * rockchip-mpp nor upstream/BSP Linux programs it, and on the
     * Windows path it correlates with the codec self-gating its leaf
     * clocks at TIMEOUT_STA.  See regbuilder_h264.cpp reg13 comment. */
    {
        uint32_t reg013 = RKVDEC2_TIMEOUT_MODE |
                          RKVDEC2_H26X_STREAMD_ERROR_MODE |
                          RKVDEC2_COLMV_ERROR_MODE |
                          RKVDEC2_H26X_ERROR_MODE;
        if (parsed->is_idr)
            reg013 |= RKVDEC2_CUR_PIC_IS_IDR;
        EMIT_P(RKVDEC2_REG_ERROR_MODE, reg013);
    }

    EMIT_P(RKVDEC2_REG_FBC_PARAMS,  0);                        /* no FBC for raster */
    EMIT_P(RKVDEC2_REG_STREAM_MODE, 0);                        /* full-frame, rlc_mode=0 */

    /* reg016 — gen_regs:996..1000: round bitstream_size up to 16, add
     * 64 byte tail-pad.  Caller's bitstream buffer must have at least
     * 64 zero-bytes after the payload (BSP memsets the tail; we rely on
     * caller pre-zeroing). */
    {
        uint32_t str_len = ((bufs->bitstream_size + 15u) & ~15u) + 64u;
        EMIT_P(RKVDEC2_REG_STR_LEN, str_len);
    }

    /* reg017 — gen_regs:917: slice_count.  BSP HEVC capture shows the
     * exact slice count (1 for our single-slice test streams), NOT the
     * H.264-style 0x3FFF sentinel.  The H.264 path uses 0x3FFF as
     * "decode until end of bitstream" because its slice loop is hostile
     * to exact counts; HEVC's HAL writes the exact count and the HW
     * uses it as a hard upper bound on slice headers parsed.
     *
     * Until a multi-slice HEVC stream lands in the test corpus we hold
     * this at 1 (matching capture).  When per-slice iteration lands,
     * the caller will surface an explicit count. */
    EMIT_P(RKVDEC2_REG_SLICE_NUM, 1u);

    /* reg018/019/020 — gen_regs:929..933 (raster path).  Strides in
     * 16-byte units; y_virstride is the y-plane byte size /16. */
    EMIT_P(RKVDEC2_REG_Y_HOR_VIRSTRIDE,  luma_stride   / 16u);
    EMIT_P(RKVDEC2_REG_UV_HOR_VIRSTRIDE, chroma_stride / 16u);
    EMIT_P(RKVDEC2_REG_Y_VIRSTRIDE,      y_size        / 16u);

    /* reg021 — gen_regs:948..950: error_deb_en=1 always; error_intra_mode
     * gates on intra (IDR / I-slice) only.  BSP capture: IDR writes 0x6
     * (intra+deb), non-IDR P/B writes 0x4 (deb only).  inter_error_prc
     * stays 0.  Earlier blanket 0x6 was wrong for inter slices. */
    {
        uint32_t reg021 = RKVDEC2_ERROR_DEB_EN;
        /* H265_SLICE_TYPE_I = 2 in the parser's slice.slice_type encoding
         * (P=1, B=0, I=2, mirroring the bitstream slice_type values). */
        if (parsed->is_idr || slice->slice_type == 2)
            reg021 |= RKVDEC2_ERROR_INTRA_MODE;
        EMIT_P(RKVDEC2_REG_ERROR_CTRL, reg021);
    }

    /* reg024/025 — gen_regs:1029..1037: CABAC error-tolerance window for
     * the bitstream-conformance check.  BSP HEVC capture (RK3588) shows
     * 0xFFFFDFFF / 0x3FFBF9FF — the same per-codec table values shipped
     * in vdpu38x_com.c (see rkvdec2_h265_regs.h field commentary).
     * Setting these to zero would trigger an error-abort partway through
     * the frame because every CABAC value would be flagged "out of
     * range".  The earlier "0 for our SoC" comment was wrong — both the
     * H.264 and HEVC HAL programme non-zero values for RK3588. */
    EMIT_P(RKVDEC2_REG_CABAC_ERR_LOW,  0xFFFFDFFFu);
    EMIT_P(RKVDEC2_REG_CABAC_ERR_HIGH, 0x3FFBF9FFu);

    /* reg026 — gen_regs:1031,1042: RK3588 uses 0xfffef block-gate mask
     * (one bit lower than non-RK3588 0xfffff), plus reg_cfg_gating_en
     * in bit 31. */
    EMIT_P(RKVDEC2_REG_BLOCK_GATING,
           RKVDEC2_BLOCK_GATING_RK3588 | RKVDEC2_REG_CFG_GATING_EN);

    /* reg028 — gen_regs leaves zero for HEVC.  No fast-mode slot
     * threading in our build; the H.264 path documents the BSP's
     * fast-mode "pick slot 11" quirk, which the HEVC kernel-side
     * register handler does NOT replicate per the BSP capture. */
    /* (intentionally not emitted — memset-zero default) */

    /* reg032 — resolution-scaled timeout matching upstream
     * rkvdec-vdpu381-regs.h:
     *   1080p  0x00EFFFFF  (~16M cycles ≈ 26 ms @ 600 MHz)
     *   4K     0x02CFFFFF  (~47M cycles ≈ 78 ms)
     *   8K     0x04FFFFFF  (~83M cycles)
     * The 1080p value previously read 0x000EFFFF (one F short) —
     * only ~983k cycles ≈ 1.6 ms.  Codec hit its own TIMEOUT_STA on
     * normal-sized H.264 frames under concurrent-decode AXI
     * contention despite clean MMU.  Caught by Codex review of the
     * 2026-05-16 concurrent-decode timeout trace; see regbuilder_h264.cpp
     * matching fix and parser_dump_harness memory. */
    {
        const uint32_t pixels = width_px * height_px;
        uint32_t timeout;
        if (pixels <= 1920u * 1088u)        timeout = 0x00EFFFFFu;
        else if (pixels <= 3840u * 2176u)   timeout = 0x02CFFFFFu;
        else                                timeout = 0x04FFFFFFu;
        EMIT_P(RKVDEC2_REG_TIMEOUT_THRESH, timeout);
    }

    /* ---- HEVC codec params bank (idx 64..112) --------------------- */

    /* reg64 — gen_regs:921..923 (HW_RPS mode):
     *   h26x_frame_orslice  = 0  (frame mode in HW_RPS)
     *   h26x_rps_mode       = 0  (DPB-style; HW re-derives RPS)
     *   h26x_stream_mode    = 0  (full frame)
     *   h26x_stream_lastpacket = 0 (BSP leaves it zero in HW_RPS — the
     *     codec consumes the entire reg016 length as one packet)
     *   h264_firstslice_flag = 0 (BSP doesn't set it for HEVC; the
     *     "firstslice" semantics live in the slice-header re-parse)
     * Net result: reg64 is 0 for HEVC.  The brief's pre-Task-1 plan
     * mentioned setting bits 0/3/4 — the BSP source disagrees, and the
     * capture log shows 0x00000000 at reg64 byte 0..3 for every HEVC
     * IDR.  Match the BSP. */
    EMIT_P(RKVDEC2_REG_H265_FLAGS, 0u);

    /* reg65 — gen_regs:954: cur_top_poc = CurrPicOrderCntVal (low 32
     * bits of the 33-bit signed POC).  The high bit goes in reg204. */
    EMIT_P(RKVDEC2_REG_H265_CUR_TOP_POC, (uint32_t)parsed->poc);

    /* reg66 — gen_regs leaves cur_bot_poc untouched (memset-zero).  HEVC
     * is frame-coded in our scope; the codec consults reg65 only and the
     * BSP capture confirms reg66 == 0 for every HEVC AU regardless of
     * POC.  Earlier mirror-of-top write was speculative. */
    EMIT_P(RKVDEC2_REG_H265_CUR_BOT_POC, 0u);

    /* reg67..82 — gen_regs:1063: per-ref low-32-bit POCs from
     * dxva.PicOrderCntValList.  Slots not in use must be written as 0;
     * the DPB layer hands us H265_DPB_REF_POC_SENTINEL (0x33333333) for
     * empty slots so its own searches produce predictable misses, but
     * the codec reads reg67..82 directly and the BSP capture for an IDR
     * (no refs) shows all 16 slots == 0.  Gate on refs[i] non-zero. */
    for (int i = 0; i < 16; i++) {
        uint32_t val = bufs->refs[i] ? (uint32_t)bufs->ref_poc[i] : 0u;
        EMIT_P(RKVDEC2_REG_H265_REF_POC_BASE + i * 4, val);
    }

    /* reg99 — gen_regs:1099 SET_REF_VALID(i,1) per used slot.  The
     * 15-bit field has 4-bit gaps (see RKVDEC2_H265_REF_VALID_BIT). */
    {
        uint32_t reg99 = 0;
        for (int i = 0; i < 15; i++) {
            if (bufs->refs[i])
                reg99 |= RKVDEC2_H265_REF_VALID_BIT(i);
        }
        EMIT_P(RKVDEC2_REG_H265_REF_VALID, reg99);
    }

    /* reg103 — gen_regs:961: ref_pic_layer_same_with_cur = 0xffff
     * (all 16 slots flagged as same layer; we don't support multi-layer
     * HEVC in this scope so this is a constant). */
    EMIT_P(RKVDEC2_REG_H265_REF_LAYER_BITS, 0x0000FFFFu);

    /* reg104, reg112 — left at 0 (no scalability, no field coding). */
    /* (intentionally not emitted) */

    /* ---- Common addresses (idx 128..142) -------------------------- */

    /* reg128/129 — gen_regs:991..992: bitstream input + scratch (same
     * iova).  IovaOffset is bitstream_offset for the slice payload. */
    EMIT_I(RKVDEC2_REG_RLC_BASE,      bufs->bitstream, bufs->bitstream_offset);
    EMIT_I(RKVDEC2_REG_RLCWRITE_BASE, bufs->bitstream, bufs->bitstream_offset);

    /* reg130 — gen_regs:946,952: decout = current pic frame buffer. */
    EMIT_I(RKVDEC2_REG_DECOUT_BASE, bufs->output_frame, 0);

    /* reg131 — gen_regs:953: colmv for current pic. */
    EMIT_I(RKVDEC2_REG_COLMV_CUR_BASE, bufs->colmv_cur, 0);

    /* reg132 — gen_regs:1051,1078: error_ref_base.  BSP defaults this
     * to the current pic, then walks the ref list and re-points it at
     * the closest-POC valid ref.  We follow the H.264 regbuilder's
     * empirical finding that pointing at output_frame regresses decode
     * quality and use a dedicated error_ref scratch buffer. */
    if (bufs->error_ref)
        EMIT_I(RKVDEC2_REG_ERROR_REF_BASE, bufs->error_ref, 0);
    else
        EMIT_I(RKVDEC2_REG_ERROR_REF_BASE, bufs->output_frame, 0);

    /* reg133..142 — RCB scratch.  Codec-agnostic; vdpu34x_setup_rcb
     * writes the same buffer fd to all 10 slots and the kernel applies
     * per-region offsets via mpp_dev_set_reg_offset.  We expose this
     * via the rcb[]+rcb_offset[] arrays (caller chooses split scheme). */
    for (int i = 0; i < 10; i++) {
        if (bufs->rcb[i])
            EMIT_I(RKVDEC2_REG_RCB_BASE_FIRST + i * 4,
                   bufs->rcb[i], bufs->rcb_offset[i]);
    }

    /* ---- HEVC addresses (idx 160..197) ---------------------------- */

    /* reg161 — gen_regs:984: pps_base.  IovaOffset addresses the
     * SPS+PPS slot inside the consolidated info buffer (Task 3 builds
     * the layout; harness passes the offset via spspps_offset which we
     * fold into IovaOffset = 0 for now since our consolidated buffer
     * starts the SPS+PPS unit at offset 0). */
    if (bufs->pps_table)
        EMIT_I(RKVDEC2_REG_H265_PPS_BASE, bufs->pps_table, 0);

    /* reg163 — gen_regs:985: rps_base.  Same buffer as pps_base in the
     * consolidated layout, but a different region — the harness sets
     * the per-resource iova offset. */
    if (bufs->rps_table)
        EMIT_I(RKVDEC2_REG_H265_RPS_BASE, bufs->rps_table, 0);

    /* reg164..179 — gen_regs:1077: per-ref frame iovas.  When a slot is
     * empty, BSP fills with the running `valid_ref` (the most recent
     * valid ref base, defaulting to the current pic).  We mirror the
     * H.264 regbuilder convention: empty slot -> error_ref. */
    {
        uint64_t fallback = bufs->error_ref ? bufs->error_ref : bufs->output_frame;
        for (int i = 0; i < 16; i++) {
            uint64_t ref = bufs->refs[i] ? bufs->refs[i] : fallback;
            EMIT_I(RKVDEC2_REG_H265_REF_BASE_FIRST + i * 4, ref, 0);
        }
    }

    /* reg180 — scanlist iova.  When the stream has no scaling list the
     * caller passes a buffer pre-filled with flat-16 defaults.  Even if
     * scaling_list_enabled_flag==0 we still write a valid iova because
     * reg012.scanlist_addr_valid_en=1 and the codec faults on iova-0. */
    if (bufs->scanlist)
        EMIT_I(RKVDEC2_REG_H265_SCANLIST_ADDR, bufs->scanlist, 0);
    else
        EMIT_P(RKVDEC2_REG_H265_SCANLIST_ADDR, 0u);

    /* reg181..196 — gen_regs:1115: per-ref colmv iovas.  Empty slot
     * defaults to the error_ref's colmv (we use colmv_cur as the
     * fallback since we don't track a separate error-colmv buffer). */
    {
        uint64_t fallback = bufs->colmv_cur;
        for (int i = 0; i < 16; i++) {
            uint64_t cmv = bufs->ref_colmv[i] ? bufs->ref_colmv[i] : fallback;
            EMIT_I(RKVDEC2_REG_H265_COLMV_BASE_FIRST + i * 4, cmv, 0);
        }
    }

    /* reg197 — gen_regs:983: cabac init table.  Required (we returned
     * UNSUPPORTED above if cabac_init_table is zero). */
    EMIT_I(RKVDEC2_REG_H265_CABACTBL_BASE, bufs->cabac_init_table, 0);

    /* ---- POC high bits (idx 200..204) ----------------------------- *
     * gen_regs:1118 SET_POC_HIGNBIT_INFO.  HEVC reg200..203 each hold
     * 8 refs × 4 bits.  Empty ref slots (those where error_index ==
     * cur_pic) must carry the sentinel value 3 in their nibble per
     * gen_regs:1109; slot 15 is reserved as the cur-pic error slot
     * itself and gets 0.  Slots 16..31 are unused on RK3588 (no
     * multilayer HEVC) and the BSP capture shows reg202/reg203 == 0.
     *
     * BSP capture for a single-IDR HEVC submission:
     *   reg200 = 0x33333333  (slots 0..7 all empty, all sentinel)
     *   reg201 = 0x03333333  (slots 8..14 empty/sentinel, slot 15 = 0)
     *   reg202 = 0x00000000
     *   reg203 = 0x00000000
     * Match by treating refs[i]==0 as empty + special-casing slot 15. */
    {
        uint32_t reg200 = 0, reg201 = 0;
        /* Empty-slot sentinel `3` is BSP-written only for IDR pictures
         * (where every slot is empty so the codec must distinguish
         * "no ref" from "ref with POC high bit 0").  For non-IDR the
         * BSP writes 0 in empty slots because reg99.hevc_ref_valid
         * disambiguates — capture confirms reg200..203 == 0 across
         * every non-IDR AU even when only refs[0..1] are filled. */
        const bool idr_empty_fill = parsed->is_idr;
        for (int i = 0; i < 8; i++) {
            uint32_t nib;
            if (bufs->refs[i])
                nib = bufs->ref_poc_high[i] & 0xFu;
            else
                nib = idr_empty_fill ? 0x3u : 0x0u;
            reg200 = set_bits(reg200, nib, i * 4, 0xF);
        }
        for (int i = 0; i < 8; i++) {
            int idx = 8 + i;
            uint32_t nib;
            if (idx == 15) {
                /* reserved error-recovery cur-pic slot — always 0. */
                nib = 0;
            } else if (bufs->refs[idx]) {
                nib = bufs->ref_poc_high[idx] & 0xFu;
            } else {
                nib = idr_empty_fill ? 0x3u : 0x0u;
            }
            reg201 = set_bits(reg201, nib, i * 4, 0xF);
        }
        EMIT_P(RKVDEC2_REG_H265_POC_HIGHBIT_FIRST + 0 * 4, reg200);
        EMIT_P(RKVDEC2_REG_H265_POC_HIGHBIT_FIRST + 1 * 4, reg201);
        EMIT_P(RKVDEC2_REG_H265_POC_HIGHBIT_FIRST + 2 * 4, 0u);   /* refs 16..23 */
        EMIT_P(RKVDEC2_REG_H265_POC_HIGHBIT_FIRST + 3 * 4, 0u);   /* refs 24..31 */
    }

    /* reg204 — current pic high bit.  The codec's POC field is 36-bit
     * (32-bit low half from reg174 + 4-bit high nibble here), so the
     * nibble holds POC[35:32].  Our parser stores POC as int32_t, so
     * we only have one sign bit to encode into a 4-bit nibble — for a
     * POC in range [-2^31, 2^31) the high nibble equals the sign bit
     * replicated across all 4 positions (sign-extension of int32 to
     * 36-bit), which simplifies to: 0 if POC>=0, 0xF if POC<0.
     *
     * Long-running stream risk (Review parser #18): a single decode
     * session that accumulates more than ~2^31 POC values overflows
     * our int32_t poc.  At 60 fps that's >414 days continuous; not a
     * practical concern, but if it ever matters the fix is to widen
     * the parser's poc field to int64_t and emit the actual top-4
     * bits here.  The V4L2 ABI's top/bottom_field_order_cnt fields
     * cap H.264 at int32_t already, so HEVC is the only codec where
     * the widening is purely internal. */
    {
        uint32_t cur_high = (parsed->poc < 0) ? 0xFu : 0u;
        EMIT_P(RKVDEC2_REG_H265_CUR_POC_HIGHBIT, cur_high);
    }

    /* ---- AXI perf-and-QoS statistic bank (idx 256..277) ----------- *
     * vdpu34x_setup_statistic (vdpu34x_com.c:177).  Codec-agnostic —
     * identical writes for H.264 and HEVC.  Without these the AXI
     * write-back fabric stalls during SAO write-back.
     *
     * Note: the H.264 regbuilder uses the same QOS_CTRL constant
     * (0x311300FF assembled via the per-field shifts).  Match. */
    EMIT_P(RKVDEC2_REG_PERF_LATENCY0,
           RKVDEC2_AXI_PERF_WORK_E | RKVDEC2_AXI_PERF_CLR_E |
           RKVDEC2_AXI_CNT_TYPE);
    EMIT_P(RKVDEC2_REG_PERF_LATENCY1, 1u);                     /* addr_align_type=1 */
    EMIT_P(RKVDEC2_REG_QOS_CTRL,
           (255u << RKVDEC2_BUS2MC_BUFFER_QOS_SHIFT) |
           (3u   << RKVDEC2_AXI_RD_HURRY_LEVEL_SHIFT) |
           (1u   << RKVDEC2_AXI_WR_QOS_SHIFT)         |
           (1u   << RKVDEC2_AXI_WR_HURRY_LEVEL_SHIFT) |
           (3u   << RKVDEC2_AXI_RD_QOS_SHIFT));
    EMIT_P(RKVDEC2_REG_WR_WAIT_CYCLE_QOS, 0u);

    /* ---- Kick (must be last, after all addresses are settled) ---- *
     * gen_regs:1017: dec_e=1.  Mirror the H.264 regbuilder's "kick is
     * part of the produced bank" convention; the engine's IOCTL chain
     * (cmd=0x300 POLL_HW_FINISH) flushes this last. */
    EMIT_P(RKVDEC2_REG_START_EN, RKVDEC2_DEC_E);

    (void)slice;  /* slice header fields don't drive any reg directly in
                   * HW_RPS mode — the codec re-parses the slice header
                   * from the bitstream to build the per-AU RPS.  Kept
                   * as a parameter for future per-slice extensions. */
    return H265_REGBUILD_OK;
}
