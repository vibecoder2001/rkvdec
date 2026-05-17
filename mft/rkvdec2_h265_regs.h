/* mft/rkvdec2_h265_regs.h — RK3588 rkvdec2 (vdpu34x) HEVC register catalog.
 *
 * The hardware exposes a packed 360-entry uint32_t register array; byte
 * offset = index * 4.  The userspace BSP (rockchip-linux/mpp,
 * mpp/hal/rkdec/inc/vdpu34x_h265d.h + vdpu34x_com.h) defines bitfield
 * structs covering each bank; this header collects the indices + named
 * bit-positions we drive from the regbuilder for HEVC decode.
 *
 * The common control / common-addr / IRQ-status / statistic banks are
 * codec-agnostic on vdpu34x and are duplicated from rkvdec2_h264_regs.h
 * verbatim (same indices, same bit positions).  Only the codec-param,
 * codec-addr, and POC-high-bits banks differ.
 *
 * Cross-reference: BSP capture
 *   Z:\drivers-arm\bsp_capture\hevc_multi_capture\mpp.shim.h265.log
 * shows six SWREG windows per HEVC decode:
 *
 *   offset  size  reg-range   bank
 *      32   100   8..32       common control       (Vdpu34xRegCommon)
 *     256   196   64..112     h265d codec params   (Vdpu34xRegH265d)
 *     512    60   128..142    common addresses     (Vdpu34xRegCommonAddr)
 *     640   152   160..197    h265d addresses      (Vdpu34xRegH265dAddr)
 *     800    20   200..204    POC high bits        (Vdpu34xH2645HighPoc)
 *    1024    88   256..277    AXI / statistic ext  (Vdpu34xRegStatistic)
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>

/* ---- Banks (index ranges) ---------------------------------------- */
/* 8..32   Common control     bytes 0x020..0x080  (Vdpu34xRegCommon)     */
/* 64..112 H.265 codec params bytes 0x100..0x1C0  (Vdpu34xRegH265d)      */
/* 128..142 Common addresses  bytes 0x200..0x238  (Vdpu34xRegCommonAddr) */
/* 160..197 H.265 addresses   bytes 0x280..0x314  (Vdpu34xRegH265dAddr)  */
/* 200..204 POC high bits     bytes 0x320..0x330  (32 refs, RK3588)      */
/* 224..237 IRQ / status RO   bytes 0x380..0x3B4                         */
/* 256..277 AXI perf / QoS    bytes 0x400..0x454  (Vdpu34xRegStatistic)  */

/* ---- Common bank (idx 8..32) ------------------------------------- *
 * These are identical to H.264 — the vdpu34x common control bank is
 * codec-agnostic.  Names mirror rkvdec2_h264_regs.h verbatim. */
#define RKVDEC2_REG_DEC_MODE          (9   * 4)   /* dec_mode[9:0]: 0 = HEVC */
#define RKVDEC2_REG_START_EN          (10  * 4)   /* dec_e[0] = 1 → kick */
#define RKVDEC2_REG_IMPORTANT_EN      (11  * 4)   /* clkgate / irq / softrst */
#define RKVDEC2_REG_SECONDARY_EN      (12  * 4)   /* colmv / fbc / scanlist */
#define RKVDEC2_REG_ERROR_MODE        (13  * 4)   /* timeout / cur_pic_is_idr */
#define RKVDEC2_REG_FBC_PARAMS        (14  * 4)
#define RKVDEC2_REG_STREAM_MODE       (15  * 4)   /* rlc_mode = 0 = compressed */
#define RKVDEC2_REG_STR_LEN           (16  * 4)   /* bitstream length, bytes */
#define RKVDEC2_REG_SLICE_NUM         (17  * 4)   /* slice_num[24:0] = 0x3FFF */
#define RKVDEC2_REG_Y_HOR_VIRSTRIDE   (18  * 4)   /* luma_stride / 16 */
#define RKVDEC2_REG_UV_HOR_VIRSTRIDE  (19  * 4)   /* chroma_stride / 16 */
#define RKVDEC2_REG_Y_VIRSTRIDE       (20  * 4)   /* y_size_bytes / 16 */
#define RKVDEC2_REG_ERROR_CTRL        (21  * 4)
#define RKVDEC2_REG_CABAC_ERR_LOW     (24  * 4)   /* RK3588: 0xFFFFDFFF for HEVC
                                                   * (per vdpu38x_com.c codec table) */
#define RKVDEC2_REG_CABAC_ERR_HIGH    (25  * 4)   /* RK3588: 0xFFFBF9FF for HEVC */
#define RKVDEC2_REG_BLOCK_GATING      (26  * 4)   /* RK3588: 0xFFFEF | (1<<31) */
#define RKVDEC2_REG_FILM_IDX          (28  * 4)   /* sw_film_idx[19:10] + flags */
#define RKVDEC2_REG_TIMEOUT_THRESH    (32  * 4)   /* 0x3FFFF */

/* ---- HEVC dec_mode value ---------------------------------------- *
 * The H.264 hal explicitly writes common.reg009.dec_mode = 1.  The
 * H.265 hal (hal_h265d_vdpu34x.c) does NOT touch reg009 — the regset
 * is memset-zero before fill, so HEVC implicitly runs with dec_mode=0.
 * Confirmed against the BSP capture: window @offset=32 byte 4..7
 * (reg9) is 0x00000000 for every HEVC submission.  vdpu38x_com.c
 * makes the mapping explicit: HEVC=0, H264=1, VP9=2, AVS2=3, AV1=4. */
#define RKVDEC2_DEC_MODE_HEVC               0u

/* Bit positions (identical to H.264) */
#define RKVDEC2_DEC_E                       (1u <<  0)  /* in REG_START_EN */

#define RKVDEC2_DEC_CLKGATE_E               (1u <<  1)  /* in REG_IMPORTANT_EN */
#define RKVDEC2_DEC_TIMEOUT_E               (1u <<  5)
#define RKVDEC2_BUF_EMPTY_EN                (1u <<  6)
#define RKVDEC2_PIX_RANGE_DETECTION_E       (1u << 24)  /* verified via BSP ioctl trace */

#define RKVDEC2_COLMV_COMPRESS_EN           (1u <<  1)  /* in REG_SECONDARY_EN */
#define RKVDEC2_FBC_E                       (1u <<  2)
#define RKVDEC2_WAIT_RESET_EN               (1u <<  7)
#define RKVDEC2_SCANLIST_ADDR_VALID_EN      (1u <<  8)

/* reg013 (SWREG13_EN_MODE_SET, vdpu34x_com.h) field shifts:
 *   bit  0  timeout_mode
 *   bit  1  req_timeout_rst_sel
 *   bit  3  dec_commonirq_mode
 *   bit  6  stmerror_waitdecfifo_empty
 *   bit  9  h26x_streamd_error_mode
 *   bit 12  allow_not_wr_unref_bframe
 *   bit 13  fbc_output_wr_disable
 *   bit 15  colmv_error_mode
 *   bit 18  h26x_error_mode
 *   bit 21  ycacherd_prior
 *   bit 24  cur_pic_is_idr
 *   bit 26  right_auto_rst_disable
 * BSP h265d sets timeout_mode + h26x_error_mode + h26x_streamd_error_mode
 * + colmv_error_mode for HEVC (hal_h265d_vdpu34x.c:944-946). */
#define RKVDEC2_TIMEOUT_MODE                (1u <<  0)  /* in REG_ERROR_MODE */
#define RKVDEC2_REQ_TIMEOUT_RST_SEL         (1u <<  1)
#define RKVDEC2_H26X_STREAMD_ERROR_MODE     (1u <<  9)
#define RKVDEC2_COLMV_ERROR_MODE            (1u << 15)
#define RKVDEC2_H26X_ERROR_MODE             (1u << 18)
#define RKVDEC2_CUR_PIC_IS_IDR              (1u << 24)

/* HEVC also drives reg021 — BSP sets inter_error_prc_mode=0,
 * error_intra_mode=1 (hal_h265d_vdpu34x.c:948-949). */
#define RKVDEC2_INTER_ERROR_PRC_MODE        (1u <<  0)  /* in REG_ERROR_CTRL */
#define RKVDEC2_ERROR_INTRA_MODE             (1u <<  1)
#define RKVDEC2_ERROR_DEB_EN                (1u <<  2)

#define RKVDEC2_REG_CFG_GATING_EN           (1u << 31)  /* in REG_BLOCK_GATING */
#define RKVDEC2_BLOCK_GATING_RK3588         0x000FFFEFu

/* reg028 layout (SWREG28_MULTIPLY_CORE_CTRL, vdpu34x_com.h):
 *   bits  0..2  swreg_vp9_wr_prob_idx
 *   bits  4..6  swreg_vp9_rd_prob_idx
 *   bit   8     swreg_ref_req_advance_flag
 *   bit   9     sw_colmv_req_advance_flag
 *   bit  10     sw_poc_only_highbit_flag
 *   bit  11     sw_poc_arb_flag
 *   bits 16..25 sw_film_idx (per-pic slot, fast mode only)
 * For HEVC single-slot decode we leave reg028 at 0. */
#define RKVDEC2_FILM_IDX_SHIFT              16
#define RKVDEC2_POC_ONLY_HIGHBIT_FLAG       (1u << 10)

/* ---- HEVC codec params (idx 64..112) ----------------------------- *
 * Mirrors Vdpu34xRegH265d_t (vdpu34x_h265d.h).  Note that reg64's
 * bit-4 field is named `h264_firstslice_flag` in the BSP struct but
 * is reused for HEVC first-slice signalling — keep the legacy name
 * so cross-references to the BSP source line up. */
#define RKVDEC2_REG_H265_FLAGS        (64  * 4)   /* stream-mode, first-slice */
#define RKVDEC2_REG_H265_CUR_TOP_POC  (65  * 4)   /* current pic POC (low 32) */
#define RKVDEC2_REG_H265_CUR_BOT_POC  (66  * 4)   /* HEVC: unused (mirrors top) */
/* Per-ref low-32-bit POCs: 16 entries, one word each (regs 67..82). */
#define RKVDEC2_REG_H265_REF_POC_BASE (67  * 4)
#define RKVDEC2_REG_H265_REF_POC_LAST (82  * 4)
/* regs 83..98: BSP labels these `ref_poc_no_use[16]` — present in the
 * struct for byte alignment with H.264, never written by hal_h265d.
 * TODO: confirm we can leave them at 0 (capture says yes). */

/* reg99: 15-of-32-bits ref_valid mask, but the layout has 4-bit gaps.
 * Bits used (each 1 wide): 0,1,2,3, 8,9,10,11, 16,17,18,19, 24,25,26.
 * Helper macro builds the packed value from a flat 0..14 ref index. */
#define RKVDEC2_REG_H265_REF_VALID    (99  * 4)
#define RKVDEC2_H265_REF_VALID_BIT(i) \
    (1u << (((i) & 0x3) + (((i) >> 2) << 3)))   /* group of 4, stride 8 */

/* regs 100..102: BSP `reg100_102_no_use[3]` — leave at 0. */

/* reg103: `ref_pic_layer_same_with_cur` bitmask (bits 0..15). */
#define RKVDEC2_REG_H265_REF_LAYER_BITS (103 * 4)

/* reg104: multi-layer / scalability flags.  Set to 0 for plain HEVC
 * Main/Main10; populate for SHVC/MV-HEVC (currently unsupported). */
#define RKVDEC2_REG_H265_LAYER_FLAGS    (104 * 4)
#define RKVDEC2_H265_POC_LSB_NOT_PRESENT_FLAG     (1u <<  0)
#define RKVDEC2_H265_NUM_DIRECT_REF_LAYERS_SHIFT   1u   /* 6 bits */
#define RKVDEC2_H265_NUM_REFLAYER_PICS_SHIFT       8u   /* 6 bits */
#define RKVDEC2_H265_DEFAULT_REF_LAYERS_ACTIVE_FLAG (1u << 14)
#define RKVDEC2_H265_MAX_ONE_ACTIVE_REF_LAYER_FLAG  (1u << 15)
#define RKVDEC2_H265_POC_RESET_INFO_PRESENT_FLAG    (1u << 16)
#define RKVDEC2_H265_VPS_POC_LSB_ALIGNED_FLAG       (1u << 17)
#define RKVDEC2_H265_MVC_POC15_VALID_FLAG           (1u << 18)

/* regs 105..111: BSP `no_use_regs[7]` — leave at 0. */

/* reg112: AVS2/HEVC error-ref-field flags. */
#define RKVDEC2_REG_H265_ERROR_REF_FLAGS (112 * 4)
#define RKVDEC2_H265_REF_ERROR_TOPFIELD_USED  (1u <<  2)
#define RKVDEC2_H265_REF_ERROR_BOTFIELD_USED  (1u <<  3)

/* reg64 bit positions — same names as H.264 (the field is shared in
 * the BSP struct for both codecs). */
#define RKVDEC2_H26X_FRAME_OR_SLICE         (1u <<  0)
#define RKVDEC2_H26X_RPS_MODE_DPB           (0u <<  1)  /* mode=0: DPB-style */
#define RKVDEC2_H26X_RPS_MODE_RPS           (1u <<  1)  /* mode=1: HEVC explicit RPS */
#define RKVDEC2_H26X_STREAM_MODE_FULL_FRAME (0u <<  2)
#define RKVDEC2_H26X_STREAM_LASTPACKET      (1u <<  3)
#define RKVDEC2_H264_FIRSTSLICE_FLAG        (1u <<  4)  /* HEVC reuses this bit */

/* ---- Common addresses (idx 128..142) ----------------------------- *
 * Identical to H.264 — codec-agnostic. */
#define RKVDEC2_REG_RLC_BASE          (128 * 4)   /* bitstream input iova */
#define RKVDEC2_REG_RLCWRITE_BASE     (129 * 4)   /* scratch, often = RLC_BASE */
#define RKVDEC2_REG_DECOUT_BASE       (130 * 4)   /* output frame iova */
#define RKVDEC2_REG_COLMV_CUR_BASE    (131 * 4)   /* colmv for current pic */
#define RKVDEC2_REG_ERROR_REF_BASE    (132 * 4)   /* fallback ref frame */
/* RCB scratch (133..142): intra, transd_row, transd_col, streamd_row,
 * inter_row, inter_col, dblk, sao, fbc, filter_col */
#define RKVDEC2_REG_RCB_BASE_FIRST    (133 * 4)
#define RKVDEC2_REG_RCB_BASE_LAST     (142 * 4)

/* ---- HEVC addresses (idx 160..197) ------------------------------- *
 * Mirrors Vdpu34xRegH265dAddr_t.  Index numbers match H.264 (PPS/RPS
 * at 161/163, refs 164..179, scanlist 180, colmv 181..196, cabac 197).
 * reg160 (`vp9_delta_prob_base`) and reg162 are unused for HEVC. */
#define RKVDEC2_REG_H265_PPS_BASE         (161 * 4)   /* Rockchip-packed HEVC PPS table */
#define RKVDEC2_REG_H265_RPS_BASE         (163 * 4)   /* Rockchip-packed HEVC RPS table */
#define RKVDEC2_REG_H265_REF_BASE_FIRST   (164 * 4)   /* ref frame[0..15] iovas */
#define RKVDEC2_REG_H265_REF_BASE_LAST    (179 * 4)
#define RKVDEC2_REG_H265_SCANLIST_ADDR    (180 * 4)   /* scaling-list iova */
#define RKVDEC2_REG_H265_COLMV_BASE_FIRST (181 * 4)   /* per-ref colmv iovas */
#define RKVDEC2_REG_H265_COLMV_BASE_LAST  (196 * 4)
#define RKVDEC2_REG_H265_CABACTBL_BASE    (197 * 4)   /* HEVC CABAC init table */

/* ---- POC high bits (idx 200..204, RK3588 only) ------------------- *
 * Mirrors Vdpu34xH2645HighPoc_t.  Critical difference vs. H.264:
 * HEVC packs **32** ref-pic POC high-bits across regs 200..203
 * (8 refs per reg, 4 bits each); reg204 holds the current-pic
 * POC high-bit (4 bits).  H.264 only has 16 ref slots, so the H.264
 * driver only writes regs 200..201 + 204.  For HEVC we must populate
 * all 5 words.
 *
 *   reg200: ref[0..7].poc_highbit  (4 bits each)
 *   reg201: ref[8..15].poc_highbit
 *   reg202: ref[16..23].poc_highbit
 *   reg203: ref[24..31].poc_highbit
 *   reg204: cur_pic.poc_highbit (bits 0..3)
 *
 * The "high bit" is the bit-32 carry of a 33-bit signed POC; this
 * lets the codec compare POC values that wrap a 32-bit integer. */
#define RKVDEC2_REG_H265_POC_HIGHBIT_FIRST (200 * 4)
#define RKVDEC2_REG_H265_POC_HIGHBIT_LAST  (203 * 4)
#define RKVDEC2_REG_H265_CUR_POC_HIGHBIT   (204 * 4)
#define RKVDEC2_H265_POC_HIGHBIT_REF_COUNT 32u   /* vs. 16 for H.264 */

/* Pack the i-th ref's 4-bit highbit into the right (reg, shift) pair. */
#define RKVDEC2_H265_POC_HIGHBIT_REG(i)   ((200 + ((i) >> 3)) * 4)
#define RKVDEC2_H265_POC_HIGHBIT_SHIFT(i) (((i) & 0x7) * 4)
#define RKVDEC2_H265_POC_HIGHBIT_MASK     0xFu

/* ---- Statistic / AXI perf-and-QoS bank (idx 256..277) ------------ *
 * Same as H.264 — vdpu34x_setup_statistic is codec-agnostic.  The BSP
 * capture for HEVC writes 88 bytes (regs 256..277) at offset 1024. */
#define RKVDEC2_REG_PERF_LATENCY0     (256 * 4)
#define RKVDEC2_REG_PERF_LATENCY1     (257 * 4)
#define RKVDEC2_REG_QOS_CTRL          (270 * 4)
#define RKVDEC2_REG_WR_WAIT_CYCLE_QOS (271 * 4)

/* SWREG256 fields */
#define RKVDEC2_AXI_PERF_WORK_E             (1u <<  0)
#define RKVDEC2_AXI_PERF_CLR_E              (1u <<  1)
#define RKVDEC2_AXI_CNT_TYPE                (1u <<  3)

/* SWREG257 fields */
#define RKVDEC2_ADDR_ALIGN_TYPE_SHIFT       0u

/* SWREG270 layout */
#define RKVDEC2_BUS2MC_BUFFER_QOS_SHIFT     0u   /* 8 bits */
#define RKVDEC2_AXI_RD_HURRY_LEVEL_SHIFT   16u   /* 2 bits */
#define RKVDEC2_AXI_WR_QOS_SHIFT           20u   /* 2 bits */
#define RKVDEC2_AXI_WR_HURRY_LEVEL_SHIFT   24u   /* 2 bits */
#define RKVDEC2_AXI_RD_QOS_SHIFT           28u   /* 2 bits */

/* ---- IRQ status (idx 224, read-only) ----------------------------- *
 * Identical to H.264. */
#define RKVDEC2_REG_INT_EN            (224 * 4)
#define RKVDEC2_INT_DEC_IRQ                 (1u << 0)
#define RKVDEC2_INT_DEC_RDY_STA             (1u << 2)
#define RKVDEC2_INT_DEC_BUS_STA             (1u << 3)
#define RKVDEC2_INT_DEC_ERROR_STA           (1u << 4)
#define RKVDEC2_INT_DEC_TIMEOUT_STA         (1u << 5)
#define RKVDEC2_INT_BUF_EMPTY_STA           (1u << 6)
#define RKVDEC2_INT_COLMV_REF_ERROR_STA     (1u << 7)

/* ---- Iova-substitution allowlist --------------------------------- *
 * Mirrors RkvdecH264IsIovaReg.  The kernel side (mpp_rkvdec2.c
 * trans_tbl_h265d) treats the values at these register indices as
 * buffer handles to be replaced by the resolved iova at submit time.
 *
 * Indices: 128..142 (15 + colmv_cur + error_ref + RCB), 161, 163,
 *          164..179 (16 ref bases), 180, 181..196 (16 colmv refs),
 *          197 (cabac), and via reg012.scanlist_addr_valid_en gate.
 * This is the same shape as H.264 — vdpu34x's address layout is
 * identical between H.264 and HEVC. */
static inline int RkvdecH265IsIovaReg(uint32_t off)
{
    /* Common-addr bank (idx 128..142). */
    if (off >= (128 * 4) && off <= (142 * 4)) return 1;
    if (off == RKVDEC2_REG_H265_PPS_BASE)         return 1;
    if (off == RKVDEC2_REG_H265_RPS_BASE)         return 1;
    if (off >= (164 * 4) && off <= (179 * 4))     return 1;  /* ref_base[16] */
    if (off == RKVDEC2_REG_H265_SCANLIST_ADDR)    return 1;
    if (off >= (181 * 4) && off <= (196 * 4))     return 1;  /* colmv ref[16] */
    if (off == RKVDEC2_REG_H265_CABACTBL_BASE)    return 1;
    return 0;
}
