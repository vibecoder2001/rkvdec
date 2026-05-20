/* mft/regbuilder_h264.cpp — V4L2 H.264 controls → register-write list.
 *
 * Phase 3b Task 8.  Coverage limited to first-IDR decode; missing pieces
 * (Rockchip-packed PPS/RPS/CABAC tables) are flagged with TODOs and
 * gated behind buffer-handle != 0 checks so callers can wire them in
 * progressively.
 */
#include "regbuilder_h264.h"
#include "rkvdec2_h264_regs.h"
#include "rkmpp_resolution.h"

#include <string.h>

namespace {

/* Convenience: stamp one plain (non-substituted) register value into the
 * dense bank.  Idx 10 (DEC_E kick) is routed to out->KickValue — the
 * kernel writes the bank in bulk first, then idx 10 last as the kick.
 * Returns 1 if the offset doesn't land in a covered bank range. */
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

/* Record an iova-substitution slot.  The bank entry at `off/4` stays at
 * 0 (zero-init); the kernel resolves handle→iova and stamps the
 * resolved value before the bulk MMIO write.  A zero handle is treated
 * as "no write" (slot remains 0), matching the prior sparse-list
 * semantics where callers gated EMIT_I on handle != 0. */
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

/* Compute frame width / height in pixels from SPS. */
static void frame_dims(const v4l2_ctrl_h264_sps *sps,
                       uint32_t *out_w, uint32_t *out_h) {
    *out_w = ((uint32_t)sps->pic_width_in_mbs_minus1 + 1) * 16u;
    uint32_t map_units = (uint32_t)sps->pic_height_in_map_units_minus1 + 1;
    uint32_t h = map_units * 16u;
    if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) h *= 2;
    *out_h = h;
}

} /* anon namespace */

extern "C"
H264RegBuildStatus H264BuildDenseRegs(const H264ParseResult *parsed,
                                      const H264BufferRefs  *bufs,
                                      uint32_t               current_pic_index,
                                      H26xDenseOutput       *out)
{
    if (!parsed || !bufs || !out) return H264_REGBUILD_MISSING_INPUT;
    if (!parsed->has_sps || !parsed->has_pps || !parsed->has_slice)
        return H264_REGBUILD_MISSING_INPUT;
    if (!bufs->bitstream || !bufs->output_frame)
        return H264_REGBUILD_MISSING_INPUT;

    /* Caller is expected to zero-init `out`, but be defensive — the bank
     * MUST start zero so the bulk MMIO write only sets bits the
     * regbuilder actually means to set. */
    memset(out, 0, sizeof(*out));
    const auto &sps   = parsed->sps;
    const auto &pps   = parsed->pps;
    (void)parsed->slice;
    const auto &dec   = parsed->decode;

    /* CABAC streams need the static cabac_init table.  We can decode
     * CAVLC streams without it for the first test. */
    if ((pps.flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
        bufs->cabac_init_table == 0) {
        return H264_REGBUILD_UNSUPPORTED;
    }

    /* Bit-depth + chroma-format gate.  Mirrors upstream Linux
     * rkvdec_h264_validate_sps (drivers/media/platform/rockchip/rkvdec/
     * rkvdec-h264-common.c): we accept 8-bit (minus8=0) and 10-bit
     * (minus8=2) 4:2:0 (chroma_format_idc=1) only.  Luma and chroma
     * bit-depth must match.  4:2:2 (NV16/NV20) is silicon-supported but
     * the engine output path isn't wired up for it yet — reject loudly
     * rather than emit wrong-stride output. */
    if (sps.chroma_format_idc != 1)
        return H264_REGBUILD_UNSUPPORTED;
    if (sps.bit_depth_luma_minus8 != sps.bit_depth_chroma_minus8)
        return H264_REGBUILD_UNSUPPORTED;
    if (sps.bit_depth_luma_minus8 != 0 && sps.bit_depth_luma_minus8 != 2)
        return H264_REGBUILD_UNSUPPORTED;

    uint32_t width_px, height_px;
    frame_dims(&sps, &width_px, &height_px);
    /* Trust-boundary gate: reject any resolution the silicon can't
     * actually handle.  The parser caps SPS mb-dim values, but a
     * defensive gate at every regbuilder entry is the cross-cutting
     * fix from the 2026-05-19 review (single arbiter of "what can
     * the kernel be programmed with"). */
    if (!RkmppValidateResolution(width_px, height_px))
        return H264_REGBUILD_UNSUPPORTED;
    uint32_t mb_w     = (width_px  + 15) / 16;
    uint32_t mb_h     = (height_px + 15) / 16;
    /* NV12 (8-bit) or NV15 (10-bit packed: 4 samples in 5 bytes) horiz stride.
     * Matches upstream rkvdec-h264.c bytesperline derivation: width * bpp / 8,
     * 16-aligned.  Chroma stride is identical for 4:2:0.  The codec selects
     * the output bit-depth via the packed PPS unit (bit_depth_luma_minus8);
     * these stride registers must match or the SAO write fabric corrupts
     * adjacent rows. */
    const uint32_t bpp_num = (sps.bit_depth_luma_minus8 == 2) ? 10u : 8u;
    uint32_t luma_stride   = ((mb_w * 16u * bpp_num + 7u) / 8u + 15u) & ~15u;
    uint32_t chroma_stride = luma_stride;
    uint32_t y_size        = luma_stride * (mb_h * 16);

#define EMIT_P(off, val) do { if (emit_plain(out, (off), (val))) \
                              return H264_REGBUILD_TOO_MANY_REGS; } while (0)
#define EMIT_I(off, h, ioff) do { if (emit_iova(out, (off), (h), (ioff))) \
                                  return H264_REGBUILD_TOO_MANY_REGS; } while (0)

    /* ---- Common bank: hard-coded init + geometry --------------- */
    EMIT_P(RKVDEC2_REG_DEC_MODE,  1u);                      /* H.264 */
    /* pix_range_detection_e (bit 16) is required: it gates the SAO
     * write-back path through the AXI hurry/QoS arbitration.  Without
     * it the codec produces a few MB rows of output then the AXI
     * write fabric stalls and the watchdog fires at variable points.
     * Set in BSP via `vdpu34x_setup_statistic`. */
    EMIT_P(RKVDEC2_REG_IMPORTANT_EN,
        RKVDEC2_DEC_CLKGATE_E | RKVDEC2_DEC_TIMEOUT_E |
        RKVDEC2_BUF_EMPTY_EN | RKVDEC2_PIX_RANGE_DETECTION_E);
    /* reg012: scanlist_addr_valid_en is set unconditionally — the HW
     * fetches a scaling list every decode regardless of whether the
     * stream actually carries a per-PPS scaling matrix.  When the
     * stream has none, the caller must still supply a valid backing
     * buffer with the H.264 default flat=16 lists; supplying iova 0
     * faults the IOMMU at the codec's first internal offset (~0xae0). */
    uint32_t secondary = RKVDEC2_WAIT_RESET_EN | RKVDEC2_SCANLIST_ADDR_VALID_EN;
    if (sps.flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)
        secondary |= RKVDEC2_COLMV_COMPRESS_EN;
    EMIT_P(RKVDEC2_REG_SECONDARY_EN, secondary);

    /* reg013 — error-mode config (SWREG13_EN_MODE_SET).
     *
     * REQ_TIMEOUT_RST_SEL (bit 1) is intentionally NOT set: neither
     * rockchip-mpp userspace nor upstream/BSP Linux rkvdec2 programs
     * this bit, and CRU dumps captured at codec TIMEOUT_STA show the
     * leaf clocks (CLKGATE_CON40 bits 7,8,9) gated when the bit is on
     * — consistent with the codec driving a hardware gate-request out
     * to the CRU as part of its internal timeout-reset.  Setting it
     * was an empirical Windows-path bandaid that turned out to mask
     * (and possibly trigger) the underlying wedge rather than fix it. */
    uint32_t error_mode = RKVDEC2_TIMEOUT_MODE |
                          RKVDEC2_H26X_STREAMD_ERROR_MODE |
                          RKVDEC2_COLMV_ERROR_MODE |
                          RKVDEC2_H26X_ERROR_MODE;
    if (dec.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)
        error_mode |= RKVDEC2_CUR_PIC_IS_IDR;
    EMIT_P(RKVDEC2_REG_ERROR_MODE, error_mode);

    EMIT_P(RKVDEC2_REG_FBC_PARAMS,  0);                     /* no FBC for raster output */
    EMIT_P(RKVDEC2_REG_STREAM_MODE, 0);                     /* rlc_mode = 0, full bitstream */
    /* reg16 str_len — BSP consistently rounds up to 16-byte boundary:
     * confirmed by comparing kernel debug output (BSP: 8928 vs raw: 8920,
     * 3712 vs 3700, 4048 vs 4044).  Without alignment the hardware stalls. */
    EMIT_P(RKVDEC2_REG_STR_LEN, (bufs->bitstream_size + 15u) & ~15u);
    EMIT_P(RKVDEC2_REG_SLICE_NUM,   0x3FFF);
    EMIT_P(RKVDEC2_REG_Y_HOR_VIRSTRIDE,  luma_stride / 16);
    EMIT_P(RKVDEC2_REG_UV_HOR_VIRSTRIDE, chroma_stride / 16);
    EMIT_P(RKVDEC2_REG_Y_VIRSTRIDE,      y_size / 16);
    /* reg21 (error_ctrl): BSP HAL `set_registers` walks the active ref
     * list and clears error_intra_mode the moment it finds a valid
     * ref with a closer frame_num than the current minimum (and the
     * stream isn't using weighted prediction).  Net result observed
     * in capture:
     *   IDR              : 0x6 (intra_mode + deb_en) — no refs, stays 1
     *   first-P-after-IDR: 0x6 — closest ref distance ties, no clear
     *   subsequent P / B : 0x4 — closer ref found, intra cleared
     * Approximate this with a heuristic: keep intra_mode set if there
     * are <= 1 active short-term refs (the IDR-or-first-P case),
     * clear it otherwise.  The DPB selection's dpb_entries[] count
     * gives us the active-ref count without re-walking the DPB. */
    {
        uint32_t reg21 = RKVDEC2_ERROR_DEB_EN;
        /* BSP clears error_intra_mode iff some valid ref has
         *   0 < FrameNumList[i] < pp->frame_num
         * i.e. there's at least one non-IDR ref older than current.
         * IDR-only DPB and first-P-after-IDR (only the IDR is a ref,
         * fn=0) keep error_intra_mode set.  Mirror that rule directly
         * so reg21 is bit-exact across the bframe.h264 stream. */
        bool clear_intra = false;
        uint16_t cur_fn = parsed->decode.frame_num;
        for (int i = 0; i < 16; i++) {
            const auto &d = dec.dpb[i];
            if (!(d.flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)) continue;
            if (d.frame_num > 0 && d.frame_num < cur_fn) {
                clear_intra = true;
                break;
            }
        }
        if (!clear_intra) reg21 |= RKVDEC2_ERROR_INTRA_MODE;
        EMIT_P(RKVDEC2_REG_ERROR_CTRL, reg21);
    }
    /* CABAC error detection mask.  BSP source (init_common_regs) sets these
     * to 0 for RK3588 and 0xffffffff/0x3ff3ffff for older SoCs; however the
     * BSP shim capture on this board shows 0xffffffff/0x3ff3ffff — the source
     * does not match the firmware running here.  Match the captured values. */
    EMIT_P(RKVDEC2_REG_CABAC_ERR_LOW,  0xFFFFFFFFu);
    EMIT_P(RKVDEC2_REG_CABAC_ERR_HIGH, 0x3FF3FFFFu);
    EMIT_P(RKVDEC2_REG_BLOCK_GATING,
        RKVDEC2_BLOCK_GATING_RK3588 | RKVDEC2_REG_CFG_GATING_EN);
    /* reg028: BSP only sets sw_poc_arb_flag=0 (already default).  The
     * slot index field (sw_film_idx, bits 16..25) is for the BSP's
     * fast_mode multi-buffer pipeline and is harmless at 0 in our
     * single-slot setup.  poc_only_highbit_flag (bit 10) goes with the
     * reg200..204 high-bit POC bank; we keep all-zero high bits, so
     * there's no need to enable that mode. */
    /* reg028: BSP only sets sw_poc_arb_flag=0 (already default).  Earlier
     * experiment to write `frame_num << 16` here (matching what BSP kernel
     * rewrites the field to) had zero effect on the B-frame divergence and
     * is reverted.  Match the user-mode wire value. */
    (void)current_pic_index;
    EMIT_P(RKVDEC2_REG_FILM_IDX, 0u);
    /* reg032 (timeout_thresh) — resolution-scaled to match upstream
     * rkvdec-vdpu381-regs.h:
     *   1080p  0x00EFFFFF  (~16M cycles ≈ 26 ms @ 600 MHz)
     *   4K     0x02CFFFFF  (~47M cycles ≈ 78 ms)
     *   8K     0x04FFFFFF  (~83M cycles)
     * The 1080p value previously read 0x000EFFFF (one F short) — that's
     * only ~983k cycles ≈ 1.6 ms, well below a real 1080p decode time
     * under any concurrent-decode AXI contention.  Codec hit its own
     * TIMEOUT_STA on ordinary frames despite a clean MMU and complete
     * register programming.  Caught by Codex review of the concurrent
     * H.264+VP9 timeout trace.  The original `0x3FFFF` value (before
     * resolution scaling) was a misread of the BSP shim capture. */
    {
        const uint32_t pixels = width_px * height_px;
        uint32_t timeout;
        if (pixels <= 1920u * 1088u)        timeout = 0x00EFFFFFu;
        else if (pixels <= 3840u * 2176u)   timeout = 0x02CFFFFFu;
        else                                timeout = 0x04FFFFFFu;
        EMIT_P(RKVDEC2_REG_TIMEOUT_THRESH, timeout);
    }

    /* ---- AXI perf-and-QoS statistic bank ----------------------- *
     * BSP `vdpu34x_setup_statistic` (vdpu34x_com.c) — required.
     * Without these writes the AXI write-back fabric leaves hurry/
     * QoS at reset defaults (0); back-pressure builds during SAO
     * write-back and the codec goes idle a few MB rows in. */
    EMIT_P(RKVDEC2_REG_PERF_LATENCY0,
        RKVDEC2_AXI_PERF_WORK_E | RKVDEC2_AXI_PERF_CLR_E |
        RKVDEC2_AXI_CNT_TYPE);                          /* 0x0B */
    EMIT_P(RKVDEC2_REG_PERF_LATENCY1, 1u);              /* addr_align_type=1 */
    EMIT_P(RKVDEC2_REG_QOS_CTRL,
        (255u << RKVDEC2_BUS2MC_BUFFER_QOS_SHIFT) |
        (3u   << RKVDEC2_AXI_RD_HURRY_LEVEL_SHIFT) |
        (1u   << RKVDEC2_AXI_WR_QOS_SHIFT)         |
        (1u   << RKVDEC2_AXI_WR_HURRY_LEVEL_SHIFT) |
        (3u   << RKVDEC2_AXI_RD_QOS_SHIFT));            /* 0x311300FF */
    EMIT_P(RKVDEC2_REG_WR_WAIT_CYCLE_QOS, 0u);

    /* ---- H.264 codec params bank ------------------------------- */
    /* idx 64 (H264_FLAGS): BSP writes 0x00000000 for all frames including IDR.
     * Empirically confirmed: BSP decode succeeds with this register zero.
     * Our earlier STREAM_LASTPACKET+FIRSTSLICE_FLAG bits caused TIMEOUT_STA. */
    EMIT_P(RKVDEC2_REG_H264_FLAGS, 0);
    EMIT_P(RKVDEC2_REG_CUR_TOP_POC,   (uint32_t)dec.top_field_order_cnt);
    EMIT_P(RKVDEC2_REG_CUR_BOT_POC,   (uint32_t)dec.bottom_field_order_cnt);

    /* Per-ref POC pairs (32 words at idx 67..98).  IDR has no refs but
     * the kernel still expects the bank zeroed. */
    for (int i = 0; i < 16; i++) {
        const auto &d = dec.dpb[i];
        EMIT_P(RKVDEC2_REG_REF_POC_BASE + (i * 2 + 0) * 4,
               (uint32_t)d.top_field_order_cnt);
        EMIT_P(RKVDEC2_REG_REF_POC_BASE + (i * 2 + 1) * 4,
               (uint32_t)d.bottom_field_order_cnt);
    }
    /* reg99..102 — per-slot ref-info nibble (8 bits per slot, 4 slots
     * per word, vdpu34x_h264d.h SET_REF_INFO).  Bit layout per slot:
     *   bit 0 : ref{i}_field           (1 = field, 0 = frame)
     *   bit 1 : ref{i}_topfield_used   (1 = top field is a ref)
     *   bit 2 : ref{i}_botfield_used   (1 = bot field is a ref)
     *   bit 3 : ref{i}_colmv_use_flag  (1 = use colmv from this ref)
     *   bits 4..7 : reserved / 0
     * BSP shim capture for an active short-term frame ref shows 0x0e
     * = bits 1+2+3 (top_used + bot_used + colmv_use), with field=0.
     * Empty slots are 0. */
    for (int word = 0; word < 4; word++) {
        uint32_t v = 0;
        for (int sub = 0; sub < 4; sub++) {
            int idx = word * 4 + sub;
            const v4l2_h264_dpb_entry &e = dec.dpb[idx];
            if (!(e.flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)) continue;
            uint32_t nib = 0;
            /* Frame coded: top_used + bot_used; colmv_use_flag for any
             * short-term ref that the codec might consult for col-MV
             * temporal direct prediction.  Field bit stays 0. */
            if (e.fields & V4L2_H264_TOP_FIELD_REF) nib |= (1u << 1);
            if (e.fields & V4L2_H264_BOTTOM_FIELD_REF) nib |= (1u << 2);
            if ((e.fields & V4L2_H264_FRAME_REF) == V4L2_H264_FRAME_REF) {
                nib |= (1u << 1) | (1u << 2);
            }
            if (e.flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE)
                nib |= (1u << 3);
            v |= (nib & 0xFFu) << (sub * 8);
        }
        EMIT_P(RKVDEC2_REG_REF_FLAGS_BASE + word * 4, v);
    }
    EMIT_P(RKVDEC2_REG_ERROR_REF_FLAGS, 0);

    /* ---- DMA addresses (iova-substituted by kernel) ----------- */
    EMIT_I(RKVDEC2_REG_RLC_BASE,        bufs->bitstream,    bufs->bitstream_offset);
    EMIT_I(RKVDEC2_REG_RLCWRITE_BASE,   bufs->bitstream,    bufs->bitstream_offset);
    EMIT_I(RKVDEC2_REG_DECOUT_BASE,     bufs->output_frame, 0);
    if (bufs->colmv_cur)
        EMIT_I(RKVDEC2_REG_COLMV_CUR_BASE, bufs->colmv_cur,  0);
    /* ERROR_REF_BASE: BSP kernel writes this == output_frame iova, but
     * empirically setting it that way regressed our codec from 50% (with
     * separate error_ref) back to 25% — codec must do something with it
     * that we don't replicate.  Keep separate error_ref buffer. */
    if (bufs->error_ref)
        EMIT_I(RKVDEC2_REG_ERROR_REF_BASE, bufs->error_ref, 0);

    /* RCB scratch buffers (133..142). */
    for (int i = 0; i < 10; i++) {
        if (bufs->rcb[i])
            EMIT_I(RKVDEC2_REG_RCB_BASE_FIRST + i * 4,
                   bufs->rcb[i], bufs->rcb_offset[i]);
    }

    /* Rockchip-packed tables.  TODO(Task 8b): port hal_h264d_com.c
     * `prepare_spspps_packet` to produce pps_table content; until then
     * the harness must supply pre-baked buffers.  Skip writes when
     * the corresponding handle is zero so a partial bring-up still
     * exercises the rest of the path. */
    if (bufs->pps_table)
        EMIT_I(RKVDEC2_REG_PPS_BASE,    bufs->pps_table, 0);
    if (bufs->rps_table)
        EMIT_I(RKVDEC2_REG_RPS_BASE,    bufs->rps_table, 0);
    /* SW180 is in trans_tbl_h264d — on Linux mpp_service IOVA 0 is unmapped
     * so we must point to a real buffer (zeroed = flat scaling list).
     * On Windows the driver maps a guard page at IOVA 0 so 0 worked there. */
    if (bufs->scaling_list)
        EMIT_I(RKVDEC2_REG_SCANLIST_ADDR, bufs->scaling_list, 0);
    else if (bufs->output_frame)
        EMIT_I(RKVDEC2_REG_SCANLIST_ADDR, bufs->output_frame, 0);
    if (bufs->cabac_init_table)
        EMIT_I(RKVDEC2_REG_CABACTBL_BASE, bufs->cabac_init_table, 0);

    /* Reference-frame iovas (164..179) and per-ref colmv (181..196).
     * BSP kernel writes output_frame here for empty refs but pointing
     * ours at output_frame regressed us — kept on error_ref buffer. */
    for (int i = 0; i < 16; i++) {
        uint64_t ref     = bufs->refs[i]      ? bufs->refs[i]      : bufs->error_ref;
        uint64_t ref_cmv = bufs->ref_colmv[i] ? bufs->ref_colmv[i] : bufs->colmv_cur;
        if (ref)     EMIT_I(RKVDEC2_REG_REF_BASE_FIRST   + i * 4, ref,     0);
        if (ref_cmv) EMIT_I(RKVDEC2_REG_COLMV_BASE_FIRST + i * 4, ref_cmv, 0);
    }

    /* ---- POC high bits (RK3588 only) -------------------------- *
     * BSP shim capture shows 0x33333333 in reg200..203 ONLY for IDR
     * pictures (with empty DPB — every slot gets the "no-valid-ref"
     * sentinel nibble 3 to keep the codec from chasing low iovas).
     * For non-IDR slices BSP writes 0 because reg99..102 disambiguates
     * empty vs. valid via the topfield_used/botfield_used bits.  The
     * earlier "always-0x33" was derived from an IDR-only capture. */
    {
        uint32_t fill = (dec.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)
                          ? 0x33333333u : 0u;
        EMIT_P(RKVDEC2_REG_POC_HIGHBIT_FIRST + 0 * 4, fill);
        EMIT_P(RKVDEC2_REG_POC_HIGHBIT_FIRST + 1 * 4, fill);
        EMIT_P(RKVDEC2_REG_POC_HIGHBIT_FIRST + 2 * 4, fill);
        EMIT_P(RKVDEC2_REG_POC_HIGHBIT_FIRST + 3 * 4, fill);
        EMIT_P(RKVDEC2_REG_POC_HIGHBIT_FIRST + 4 * 4, 0u);
    }

    /* ---- IRQ enable ------------------------------------------- *
     * Don't write reg224 / 0x380 here: the BSP user-mode register
     * builder never writes the IRQ-status bank; the hardware comes
     * up with interrupts enabled and the kernel ISR clears the
     * status (writing 0 disables, writing arbitrary bits has
     * undocumented effects on rkvdec2).  We poll the status from
     * the kernel poller thread and ack via write-1-to-clear there. */

    /* ---- Kick (must be last, after all addresses are settled) - */
    EMIT_P(RKVDEC2_REG_START_EN, RKVDEC2_DEC_E);

    return H264_REGBUILD_OK;
}
