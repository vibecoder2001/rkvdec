/* mft/rkvdec2_h264_regs.h — RK3588 rkvdec2 (vdpu34x) register catalog.
 *
 * The hardware exposes a packed 360-entry uint32_t register array; byte
 * offset = index * 4.  The userspace BSP (rockchip-linux/mpp,
 * mpp/hal/rkdec/inc/vdpu34x_*.h) defines bitfield structs covering each
 * bank; this header collects the indices + named bit-positions we
 * actually drive from the regbuilder.
 *
 * Only the registers used for *first* H.264 IDR decode are catalogued
 * here; expand as we extend coverage.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>

/* ---- Banks (index ranges) ---------------------------------------- */
/* 8..32   Common control     bytes 0x020..0x080  (Vdpu34xRegCommon)     */
/* 64..112 H.264 codec params bytes 0x100..0x1C0  (Vdpu34xRegH264dParam) */
/* 128..142 Common addresses  bytes 0x200..0x238  (Vdpu34xRegCommonAddr) */
/* 160..199 H.264 addresses   bytes 0x280..0x31C  (Vdpu34xRegH264dAddr)  */
/* 200..204 POC high bits     bytes 0x320..0x330  (RK3588 only)          */
/* 224..237 IRQ / status RO   bytes 0x380..0x3B4                         */

/* ---- Common bank (idx 8..32) ------------------------------------- */
#define RKVDEC2_REG_DEC_MODE          (9   * 4)   /* dec_mode[9:0] = 1 for H.264 */
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
#define RKVDEC2_REG_CABAC_ERR_LOW     (24  * 4)   /* CABAC error detect mask; RK3588 BSP = 0 */
#define RKVDEC2_REG_CABAC_ERR_HIGH    (25  * 4)   /* CABAC error detect mask; RK3588 BSP = 0 */
#define RKVDEC2_REG_BLOCK_GATING      (26  * 4)   /* RK3588: 0xFFFEF | (1<<31) */
#define RKVDEC2_REG_FILM_IDX          (28  * 4)   /* sw_film_idx[19:10] + flags */
#define RKVDEC2_REG_TIMEOUT_THRESH    (32  * 4)   /* 0x3FFFF */

/* Bit positions */
#define RKVDEC2_DEC_E                       (1u <<  0)  /* in REG_START_EN */

#define RKVDEC2_DEC_CLKGATE_E               (1u <<  1)  /* in REG_IMPORTANT_EN */
#define RKVDEC2_DEC_TIMEOUT_E               (1u <<  5)
#define RKVDEC2_BUF_EMPTY_EN                (1u <<  6)
#define RKVDEC2_PIX_RANGE_DETECTION_E       (1u << 24)  /* verified via BSP ioctl trace */

#define RKVDEC2_COLMV_COMPRESS_EN           (1u <<  1)  /* in REG_SECONDARY_EN */
#define RKVDEC2_FBC_E                       (1u <<  2)
#define RKVDEC2_WAIT_RESET_EN               (1u <<  7)
#define RKVDEC2_SCANLIST_ADDR_VALID_EN      (1u <<  8)

/* reg013 (SWREG13_EN_MODE_SET, vdpu34x_com.h:110) field shifts:
 *   bit  0  timeout_mode
 *   bit  1  req_timeout_rst_sel
 *   bit  3  dec_commonirq_mode
 *   bit  6  stmerror_waitdecfifo_empty   <- NOT h26x_streamd_error
 *   bit  9  h26x_streamd_error_mode      <- the real one
 *   bit 12  allow_not_wr_unref_bframe
 *   bit 13  fbc_output_wr_disable
 *   bit 15  colmv_error_mode
 *   bit 18  h26x_error_mode
 *   bit 21  ycacherd_prior
 *   bit 24  cur_pic_is_idr
 *   bit 26  right_auto_rst_disable
 * Earlier copy used bit 6 for h26x_streamd_error_mode — wrong field. */
#define RKVDEC2_TIMEOUT_MODE                (1u <<  0)  /* in REG_ERROR_MODE */
#define RKVDEC2_H26X_STREAMD_ERROR_MODE     (1u <<  9)
#define RKVDEC2_COLMV_ERROR_MODE            (1u << 15)
#define RKVDEC2_H26X_ERROR_MODE             (1u << 18)
#define RKVDEC2_CUR_PIC_IS_IDR              (1u << 24)

#define RKVDEC2_INTER_ERROR_PRC_MODE        (1u <<  0)  /* in REG_ERROR_CTRL */
#define RKVDEC2_ERROR_INTRA_MODE            (1u <<  1)
#define RKVDEC2_ERROR_DEB_EN                (1u <<  2)

#define RKVDEC2_REG_CFG_GATING_EN           (1u << 31)  /* in REG_BLOCK_GATING */
#define RKVDEC2_BLOCK_GATING_RK3588         0x000FFFEFu

/* reg028 layout (from vdpu34x_com.h:240 SWREG28_MULTIPLY_CORE_CTRL):
 *   bits  0..2  swreg_vp9_wr_prob_idx
 *   bit   3     reserved
 *   bits  4..6  swreg_vp9_rd_prob_idx
 *   bit   7     reserved
 *   bit   8     swreg_ref_req_advance_flag
 *   bit   9     sw_colmv_req_advance_flag
 *   bit  10     sw_poc_only_highbit_flag
 *   bit  11     sw_poc_arb_flag
 *   bits 12..15 reserved
 *   bits 16..25 sw_film_idx (per-pic slot, only meaningful in fast mode)
 *
 * The first earlier-agent recipe quoted bit 14 / shift 10 — both wrong;
 * those land on reserved bits.  The BSP set_registers leaves all of
 * reg028 zero except sw_poc_arb_flag=0 (already default) — so for our
 * single-slot decoder we can write 0. */
#define RKVDEC2_FILM_IDX_SHIFT              16
#define RKVDEC2_POC_ONLY_HIGHBIT_FLAG       (1u << 10)

/* ---- H.264 codec params (idx 64..112) ---------------------------- */
#define RKVDEC2_REG_H264_FLAGS        (64  * 4)   /* stream-mode, first-slice */
#define RKVDEC2_REG_CUR_TOP_POC       (65  * 4)
#define RKVDEC2_REG_CUR_BOT_POC       (66  * 4)
/* refs: 16 entries, each is { top_poc, bot_poc } pair → 32 words */
#define RKVDEC2_REG_REF_POC_BASE      (67  * 4)
/* per-ref field/used flags packed 4-per-word, 4 words */
#define RKVDEC2_REG_REF_FLAGS_BASE    (99  * 4)
#define RKVDEC2_REG_ERROR_REF_FLAGS   (112 * 4)

#define RKVDEC2_H26X_FRAME_OR_SLICE         (1u <<  0)
#define RKVDEC2_H26X_RPS_MODE_DPB           (0u <<  1)  /* mode=0: DPB-style */
#define RKVDEC2_H26X_STREAM_MODE_FULL_FRAME (0u <<  2)
#define RKVDEC2_H26X_STREAM_LASTPACKET      (1u <<  3)
#define RKVDEC2_H264_FIRSTSLICE_FLAG        (1u <<  4)

/* ---- Common addresses (idx 128..142) ----------------------------- */
#define RKVDEC2_REG_RLC_BASE          (128 * 4)   /* bitstream input iova */
#define RKVDEC2_REG_RLCWRITE_BASE     (129 * 4)   /* scratch, often = RLC_BASE */
#define RKVDEC2_REG_DECOUT_BASE       (130 * 4)   /* output frame iova */
#define RKVDEC2_REG_COLMV_CUR_BASE    (131 * 4)   /* colmv for current pic */
#define RKVDEC2_REG_ERROR_REF_BASE    (132 * 4)   /* fallback ref frame */
/* RCB scratch (133..142): intra, transd_row, transd_col, streamd_row,
 * inter_row, inter_col, dblk, sao, fbc, filter_col */
#define RKVDEC2_REG_RCB_BASE_FIRST    (133 * 4)
#define RKVDEC2_REG_RCB_BASE_LAST     (142 * 4)

/* ---- H.264 addresses (idx 160..199) ------------------------------ */
#define RKVDEC2_REG_PPS_BASE          (161 * 4)   /* Rockchip-packed PPS table */
#define RKVDEC2_REG_RPS_BASE          (163 * 4)   /* Rockchip-packed RPS table */
#define RKVDEC2_REG_REF_BASE_FIRST    (164 * 4)   /* ref frame[0..15] iovas */
#define RKVDEC2_REG_REF_BASE_LAST     (179 * 4)
#define RKVDEC2_REG_SCANLIST_ADDR     (180 * 4)   /* scaling-list iova */
#define RKVDEC2_REG_COLMV_BASE_FIRST  (181 * 4)   /* per-ref colmv iovas */
#define RKVDEC2_REG_COLMV_BASE_LAST   (196 * 4)
#define RKVDEC2_REG_CABACTBL_BASE     (197 * 4)   /* 3680B static CABAC init */

/* ---- POC high bits (idx 200..204, RK3588 only) ------------------- */
#define RKVDEC2_REG_POC_HIGHBIT_FIRST (200 * 4)
#define RKVDEC2_REG_CUR_POC_HIGHBIT   (204 * 4)

/* ---- Statistic / AXI perf-and-QoS bank (idx 256..271) ------------ *
 * BSP `vdpu34x_setup_statistic` (vdpu34x_com.c) is called per decode
 * by every codec that uses vdpu34x (H.264 / HEVC / AV1).  Without
 * these writes the SAO write-back fabric stalls a few MB rows in:
 * the codec's AXI write hurry/QoS bits sit at reset defaults (0),
 * back-pressure builds, and the codec stops issuing AXI traffic. */
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

/* ---- IRQ status (idx 224, read-only) ----------------------------- */
#define RKVDEC2_REG_INT_EN            (224 * 4)
#define RKVDEC2_INT_DEC_IRQ                 (1u << 0)
#define RKVDEC2_INT_DEC_RDY_STA             (1u << 2)
#define RKVDEC2_INT_DEC_BUS_STA             (1u << 3)
#define RKVDEC2_INT_DEC_ERROR_STA           (1u << 4)
#define RKVDEC2_INT_DEC_TIMEOUT_STA         (1u << 5)
#define RKVDEC2_INT_BUF_EMPTY_STA           (1u << 6)
#define RKVDEC2_INT_COLMV_REF_ERROR_STA     (1u << 7)

/* ---- Iova-substitution allowlist --------------------------------- *
 * The kernel side (mpp_rkvdec2.c trans_tbl_h264d) treats the values
 * at these register indices as buffer handles to be replaced by the
 * resolved iova at submit time.  Our SUBMIT_JOB iova-substitution
 * (Phase 3b Task 4) covers the same set: any RKMPP_REG_WRITE entry
 * that targets an offset in this list and carries a BufferHandle != 0
 * gets rewritten before the kick.
 *
 * Indices: 128..142 (15 + colmv_cur + error_ref + RCB), 161, 163,
 *          164..179 (16 ref bases), 180, 181..196 (16 colmv refs),
 *          197 (cabac), and via reg012.scanlist_addr_valid_en gate. */
static inline int RkvdecH264IsIovaReg(uint32_t off)
{
    /* Common-addr bank (idx 128..142). */
    if (off >= (128 * 4) && off <= (142 * 4)) return 1;
    if (off == RKVDEC2_REG_PPS_BASE)          return 1;
    if (off == RKVDEC2_REG_RPS_BASE)          return 1;
    if (off >= (164 * 4) && off <= (179 * 4)) return 1;  /* ref_base[16] */
    if (off == RKVDEC2_REG_SCANLIST_ADDR)     return 1;
    if (off >= (181 * 4) && off <= (196 * 4)) return 1;  /* colmv ref[16] */
    if (off == RKVDEC2_REG_CABACTBL_BASE)     return 1;
    return 0;
}
