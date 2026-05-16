/* mft/rkvdec2_vp9_regs.h — RK3588 rkvdec2 (vdpu34x) VP9 register catalog.
 *
 * The hardware family is the same as H.264 / H.265 (rkvdec2 / vdpu34x);
 * VP9 differs only in the codec-specific banks.  Common bank, common-addr
 * bank, IRQ/status, and statistic banks are all identical and are aliased
 * from rkvdec2_h265_regs.h so the regbuilder doesn't redefine them.
 *
 * Cross-reference: BSP `mpp/hal/rkdec/vp9d/hal_vp9d_vdpu34x.c`
 *   vp9_hw_regs->common.reg009.dec_mode = 2  (line 610)
 *   reg64..reg107  vp9d codec_params bank   (Vdpu34xVp9dParam)
 *   reg160..reg174 vp9d codec_addr bank     (Vdpu34xVp9dAddr)
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>

#include "rkvdec2_h265_regs.h"   /* common bank + common-addr + stat banks */

/* dec_mode value for VP9 — per vdpu38x_com.c codec table
 * (HEVC=0, H264=1, VP9=2, AVS2=3, AV1=4). */
#define RKVDEC2_DEC_MODE_VP9                2u

/* ---- VP9 codec_params bank (idx 64..107, byte 0x100..0x1B0) -------- *
 * Slot offsets in CodecParams[] are (idx - RKMPP_DENSE_CPARAM_FIRST).
 * Names follow BSP Vdpu34xVp9dParam field-by-field.
 * BSP header: mpp/hal/rkdec/inc/vdpu34x_vp9d.h */
#define RKVDEC2_VP9_REG_FRAME_FLAGS         (64  * 4)   /* reg64: cprheader_offset [15:0] (BSP line 671) */
#define RKVDEC2_VP9_REG_CUR_POC             (65  * 4)   /* reg65: cur_poc [31:0] */

/* reg67..reg74: per-segment param words (8 entries, one per VP9 segment).
 * Each 32-bit word packs: segid_abs_delta[0], qp_delta_en, qp_delta[8:0],
 * lf_value_en, lf_value[6:0], referinfo_en, referinfo[1:0], skip_en.
 * Only reg67 carries segid_abs_delta; reg68..74 reserve that bit.
 * BSP struct: Vdpu34xRegVp9dParam.reg67_74[8] */
#define RKVDEC2_VP9_REG_SEG0                (67  * 4)   /* first segment slot */

/* reg75: sticky last-frame state word.
 * Fields: mode_deltas_lastframe[13:0], vp9_segment_id_clear[14],
 *   vp9_segment_id_update[15], segmentation_enable_lstframe[16],
 *   last_show_frame[17], last_intra_only[18], last_widthheight_eqcur[19],
 *   color_space_lastkeyframe[22:20].
 * BSP struct: Vdpu34xRegVp9dParam.reg75 */
#define RKVDEC2_VP9_REG_STICKY_STATE        (75  * 4)

/* reg76: current frame tx_mode and reference_mode.
 * Fields: tx_mode[2:0], frame_reference_mode[4:3].
 * BSP struct: Vdpu34xRegVp9dParam.reg76 (BSP HAL line 760) */
#define RKVDEC2_VP9_REG_TX_REF_MODE         (76  * 4)

/* reg77: vp9_intercmd_num[23:0] — number of inter commands.
 * Per upstream Linux vdpu381 struct; BSP capture shows 0 on our streams. */
#define RKVDEC2_VP9_REG_INTERCMD_NUM        (77  * 4)

/* reg78: last-tile byte size (stream_len - first_partition_size).
 * BSP struct: Vdpu34xRegVp9dParam.reg78, field lasttile_size[23:0]
 * BSP HAL line 779 */
#define RKVDEC2_VP9_REG_LAST_TILE_SIZE      (78  * 4)

/* reg79..84: per-ref hor virstride (low 16 bits, value = aligned_pitch/16).
 * reg85..87: per-ref y_virstride    (low 28 bits, value = y_len/16).
 * Per upstream vdpu381 + BSP capture (regs_inter/regs_001.bin):
 *   reg79..84 = 0x50, reg85..87 = 0xe100 on the 1280x720 8-bit test. */
#define RKVDEC2_VP9_REG_LASTF_Y_HORSTRIDE   (79  * 4)
#define RKVDEC2_VP9_REG_LASTF_UV_HORSTRIDE  (80  * 4)
#define RKVDEC2_VP9_REG_GOLDF_Y_HORSTRIDE   (81  * 4)
#define RKVDEC2_VP9_REG_GOLDF_UV_HORSTRIDE  (82  * 4)
#define RKVDEC2_VP9_REG_ALTRF_Y_HORSTRIDE   (83  * 4)
#define RKVDEC2_VP9_REG_ALTRF_UV_HORSTRIDE  (84  * 4)
#define RKVDEC2_VP9_REG_LASTF_Y_VIRSTRIDE   (85  * 4)
#define RKVDEC2_VP9_REG_GOLDF_Y_VIRSTRIDE   (86  * 4)
#define RKVDEC2_VP9_REG_ALTRF_Y_VIRSTRIDE   (87  * 4)

/* reg88..reg93: per-ref scaling factors (separate hor/ver registers).
 * BSP struct: Vdpu34xRegVp9dParam.reg88..reg93 (BSP HAL lines 783-788).
 * Each register holds a single u16 scale value in bits [15:0].
 * Scale formula: (ref_dim * 16384) / cur_dim  (VP9 spec §8.5.1). */
#define RKVDEC2_VP9_REG_LREF_HOR_SCALE     (88  * 4)
#define RKVDEC2_VP9_REG_LREF_VER_SCALE     (89  * 4)
#define RKVDEC2_VP9_REG_GREF_HOR_SCALE     (90  * 4)
#define RKVDEC2_VP9_REG_GREF_VER_SCALE     (91  * 4)
#define RKVDEC2_VP9_REG_AREF_HOR_SCALE     (92  * 4)
#define RKVDEC2_VP9_REG_AREF_VER_SCALE     (93  * 4)

/* reg94: last-frame ref_deltas packed word.
 * Packs 4 × 7-bit signed ref_deltas into bits [27:0]:
 *   delta[i] in bits [7i+6 : 7i], i = 0..3 (INTRA, LAST, GOLDEN, ALTREF).
 * BSP struct: Vdpu34xRegVp9dParam.reg94 (BSP HAL lines 765-766) */
#define RKVDEC2_VP9_REG_REF_DELTAS_LAST    (94  * 4)

#define RKVDEC2_VP9_REG_LAST_POC            (95  * 4)   /* reg95: last_poc [31:0] */
#define RKVDEC2_VP9_REG_GOLDEN_POC          (96  * 4)   /* reg96: golden_poc [31:0] */
#define RKVDEC2_VP9_REG_ALTREF_POC          (97  * 4)   /* reg97: altref_poc [31:0] */
#define RKVDEC2_VP9_REG_COL_REF_POC         (98  * 4)   /* reg98: col_ref_poc [31:0] */
#define RKVDEC2_VP9_REG_PROB_REF_POC        (99  * 4)   /* reg99: prob_ref_poc [15:0] */
#define RKVDEC2_VP9_REG_SEGID_REF_POC       (100 * 4)   /* reg100: segid_ref_poc [31:0] */
#define RKVDEC2_VP9_REG_PARAM_FLAGS         (103 * 4)   /* reg103: prob_update / refresh / save / hp_mv ... */

/* reg104:  BSP `reg104_no_use` — do NOT write.
 * reg105:  count_update_en[4] + avs2_headlen[3:0] — do NOT write
 *          unless deliberately driving the entropy-update path.
 * reg106..111: per-ref last-frame dims (framewidth/height of each active
 *          reference, low 16 bits).  BSP capture confirms reg106/107
 *          carry 1280/720 for our test stream. */
#define RKVDEC2_VP9_REG_FRAMEWIDTH_LAST     (106 * 4)
#define RKVDEC2_VP9_REG_FRAMEHEIGHT_LAST    (107 * 4)
#define RKVDEC2_VP9_REG_FRAMEWIDTH_GOLDEN   (108 * 4)
#define RKVDEC2_VP9_REG_FRAMEHEIGHT_GOLDEN  (109 * 4)
#define RKVDEC2_VP9_REG_FRAMEWIDTH_ALTREF   (110 * 4)
#define RKVDEC2_VP9_REG_FRAMEHEIGHT_ALTREF  (111 * 4)

/* ---- VP9 codec_addr bank (idx 160..197) ----------------------------- *
 * Authoritative layout from BSP `Vdpu34xVp9dAddr` (mpp/hal/rkdec/inc/
 * vdpu34x_vp9d.h) cross-checked against kernel MMIO trace in
 * tests/data/vp9/bsp_capture/. */
#define RKVDEC2_VP9_REG_DELTA_PROB_BASE     (160 * 4)   /* reg160 = delta_prob_base */
#define RKVDEC2_VP9_REG_PPS_BASE            (161 * 4)   /* reg161 = pps_base (unused for VP9 today) */
#define RKVDEC2_VP9_REG_LAST_PROB_BASE      (162 * 4)   /* reg162 = last_prob_base */
#define RKVDEC2_VP9_REG_RPS_BASE            (163 * 4)   /* reg163 = rps_base (unused for VP9 today) */
#define RKVDEC2_VP9_REG_REF_LAST_BASE       (164 * 4)   /* reg164 = ref frame map "last" (inter only) */
#define RKVDEC2_VP9_REG_REF_GOLDEN_BASE     (165 * 4)   /* reg165 = ref frame map "golden" (inter only) */
#define RKVDEC2_VP9_REG_REF_ALTREF_BASE     (166 * 4)   /* reg166 = ref frame map "altref" (inter only) */
#define RKVDEC2_VP9_REG_COUNT_PROB_BASE     (167 * 4)   /* reg167 = count_prob_base */
#define RKVDEC2_VP9_REG_SEGID_LAST_BASE     (168 * 4)   /* reg168 = segid_last_base */
#define RKVDEC2_VP9_REG_SEGID_CUR_BASE      (169 * 4)   /* reg169 = segid_cur_base */
#define RKVDEC2_VP9_REG_REF_COLMV_BASE      (170 * 4)   /* reg170 = colmv of "last" ref (inter only) */
#define RKVDEC2_VP9_REG_INTERCMD_BASE       (171 * 4)   /* reg171 = intercmd_base (NOT a ref slot) */
#define RKVDEC2_VP9_REG_UPDATE_PROB_WR_BASE (172 * 4)   /* reg172 = HW prob writeback target */
/* reg173..179: BSP `reg173_179_no_use[7]` — unused */
#define RKVDEC2_VP9_REG_SCANLIST_BASE       (180 * 4)   /* reg180 = scanlist_base (unused for VP9) */
/* reg181..196: per-DPB-slot ref_colmv (16 entries, written across all 8
 * VP9 ref slots; the array stride is 2 regs per slot — see BSP comment).
 * Filled by the regbuilder for inter frames when ref slot is valid. */
#define RKVDEC2_VP9_REG_REF_COLMV_SLOT_BASE (181 * 4)
#define RKVDEC2_VP9_REG_CABACTBL_BASE       (197 * 4)   /* reg197 = cabactbl_base (unused for VP9) */
