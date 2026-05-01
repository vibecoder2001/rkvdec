/* mft/regbuilder_h264.cpp — V4L2 H.264 controls → register-write list.
 *
 * Phase 3b Task 8.  Coverage limited to first-IDR decode; missing pieces
 * (Rockchip-packed PPS/RPS/CABAC tables) are flagged with TODOs and
 * gated behind buffer-handle != 0 checks so callers can wire them in
 * progressively.
 */
#include "regbuilder_h264.h"
#include "rkvdec2_h264_regs.h"

#include <string.h>

namespace {

/* Convenience: append one plain (non-substituted) register write. */
static int emit_plain(H264RegWriteList *list, uint32_t off, uint32_t value) {
    if (list->count >= RKMPP_MAX_REG_WRITES) return 1;
    RKMPP_REG_WRITE *w = &list->entries[list->count++];
    w->Offset       = off;
    w->Value        = value;
    w->BufferHandle = 0;
    w->IovaOffset   = 0;
    w->Reserved     = 0;
    return 0;
}

/* Append an iova-substitution register write (kernel rewrites Value at
 * submit time using the buffer handle's resolved iova + IovaOffset). */
static int emit_iova(H264RegWriteList *list, uint32_t off,
                     uint64_t handle, uint32_t iova_offset) {
    if (list->count >= RKMPP_MAX_REG_WRITES) return 1;
    RKMPP_REG_WRITE *w = &list->entries[list->count++];
    w->Offset       = off;
    w->Value        = 0;
    w->BufferHandle = handle;
    w->IovaOffset   = iova_offset;
    w->Reserved     = 0;
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
H264RegBuildStatus H264BuildRegisterList(const H264ParseResult *parsed,
                                         const H264BufferRefs  *bufs,
                                         uint32_t               current_pic_index,
                                         H264RegWriteList      *out)
{
    if (!parsed || !bufs || !out) return H264_REGBUILD_MISSING_INPUT;
    if (!parsed->has_sps || !parsed->has_pps || !parsed->has_slice)
        return H264_REGBUILD_MISSING_INPUT;
    if (!bufs->bitstream || !bufs->output_frame)
        return H264_REGBUILD_MISSING_INPUT;

    out->count = 0;
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

    uint32_t width_px, height_px;
    frame_dims(&sps, &width_px, &height_px);
    uint32_t mb_w     = (width_px  + 15) / 16;
    uint32_t mb_h     = (height_px + 15) / 16;
    uint32_t luma_stride   = mb_w * 16;        /* NV12, 16-aligned */
    uint32_t chroma_stride = mb_w * 16;        /* 4:2:0, same horiz stride as luma */
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

    /* reg013 — BSP shim capture for the bframe.h264 stream:
     *   IDR     : 0x01048201  (bit 24 = cur_pic_is_idr SET)
     *   non-IDR : 0x00048201  (cur_pic_is_idr CLEAR)
     * The previously-cargo-culted "bit 1 SET on IDR" came from a kernel-
     * side dmesg trace where the kernel rewrote the value before MMIO;
     * the user-mode wire value never sets bit 1. */
    uint32_t error_mode = RKVDEC2_TIMEOUT_MODE |
                          RKVDEC2_H26X_STREAMD_ERROR_MODE |
                          RKVDEC2_COLMV_ERROR_MODE |
                          RKVDEC2_H26X_ERROR_MODE;
    if (dec.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)
        error_mode |= RKVDEC2_CUR_PIC_IS_IDR;
    EMIT_P(RKVDEC2_REG_ERROR_MODE, error_mode);

    EMIT_P(RKVDEC2_REG_FBC_PARAMS,  0);                     /* no FBC for raster output */
    EMIT_P(RKVDEC2_REG_STREAM_MODE, 0);                     /* rlc_mode = 0, full bitstream */
    /* reg16 str_len — BSP H.264 HAL writes p_hal->strm_len directly
     * (hal_h264d_vdpu34x.c:321) without rounding.  Match: caller's
     * raw slice byte count. */
    EMIT_P(RKVDEC2_REG_STR_LEN, bufs->bitstream_size);
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
    /* CABAC error-tolerance range for the bitstream-conformance check.
     * Captured from a live BSP `mpi_dec_test` IOCTL trace on the same
     * RK3588 board running mainline rockchip-linux/mpp:
     *   reg024 = 0xFFFFFFFF, reg025 = 0x3FF3FFFF
     * Setting these to zero means every CABAC value is "out of range",
     * the codec triggers error abort partway through the frame, the
     * AXI bus goes idle, and the watchdog catches it as a timeout. */
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
    /* reg028 — BSP shim capture (cmd=0x200 idx 20 in the bframe.h264
     * log) shows 0x00000000 from user-mode for every AU.  An earlier
     * kernel-side dmesg trace caught a 0x000b0000 write but that's the
     * BSP kernel rewriting it before MMIO with sw_film_idx=11; from
     * user-mode the IOCTL sends 0.  Match the user-mode wire value. */
    (void)current_pic_index;
    EMIT_P(RKVDEC2_REG_FILM_IDX, 0u);
    /* reg032 (timeout_thresh) — BSP shim capture writes 0x0003ffff for
     * H.264, matching the HEVC HAL.  Earlier "kernel writes 0xefffff"
     * note was about a kernel-side override not visible at the IOCTL
     * boundary; the user-mode wire value is 0x3ffff. */
    EMIT_P(RKVDEC2_REG_TIMEOUT_THRESH, 0x0003ffffu);

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
    /* idx 64 (H264_FLAGS): BSP kernel-side log shows 0 (matches the raw
     * cmd=0x200 byte).  Our previous "0 causes dec_error" empirical
     * result was with the bug-stack of pre-kick reset every kick +
     * scanlist=valid_iova + poc_hi=0.  Re-test 0 with all fixes. */
    EMIT_P(RKVDEC2_REG_H264_FLAGS, 0u);
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
    /* scanlist_addr: BSP emits 0 explicitly.  We previously thought 0
     * caused an iova-0xae0 fault, but that was actually a different
     * bug (write-order/explicit-zeros in the driver's bank-write loop,
     * since fixed).  Match BSP — verified bit-exact in winreplay. */
    EMIT_P(RKVDEC2_REG_SCANLIST_ADDR, 0u);
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
