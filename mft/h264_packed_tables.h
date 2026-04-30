/* mft/h264_packed_tables.h — Rockchip-format packed buffers feeding the
 * rkvdec2 H.264 register banks.
 *
 * The hardware reads three side tables in addition to the bitstream:
 *
 *   - SPS+PPS unit  (reg161 pps_base, 48 bytes per slot)
 *   - frame RPS     (reg163 rps_base, 384 bytes)
 *   - CABAC init    (reg197 cabactbl_base, 3712 bytes static)
 *   - scaling list  (reg180 scanlist_addr, 224 bytes optional)
 *
 * Plus 10 RCB scratch regions (reg133..142) sized from frame dimensions.
 *
 * All ports of the BSP userspace mpp library
 * (rockchip-linux/mpp / mpp/hal/rkdec/h264d/...).  Field semantics and
 * bit layouts match `prepare_spspps` / `prepare_framerps` /
 * `prepare_scanlist` / `vdpu34x_get_rcb_buf_size` byte-for-byte; see
 * h264_packed_tables.cpp for line-level cross references.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#include "../external/v4l2-h264-controls/v4l2-h264-controls.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Buffer sizing constants ------------------------------------- */

#define RKH264_SPSPPS_UNIT_SIZE    48u    /* one (sps,pps) packed entry */
#define RKH264_RPS_SIZE           384u    /* one packed RPS table       */
#define RKH264_SCALING_LIST_SIZE  224u    /* 6*16 + 2*64 raw bytes      */
#define RKH264_CABAC_INIT_SIZE  (928u * 4u)  /* 3712 bytes               */

/* The hardware prefetcher reads slightly past the table end, so each
 * buffer is allocated with this much trailing pad and zero-filled. */
#define RKH264_TABLE_TAIL_PAD    128u

#define RKH264_RCB_COUNT          10u    /* per-frame RCB scratch regions */

/* ---- CABAC init blob --------------------------------------------- */

/* Returns the static 928 u32 (3712 bytes) CABAC init table.  Copy
 * verbatim into the cabac_init buffer at offset 0; pad trailing
 * RKH264_TABLE_TAIL_PAD bytes with zero. */
const uint32_t *H264GetCabacInitTable(size_t *out_word_count);

/* ---- SPS+PPS packed unit ----------------------------------------- */

/* Pack one V4L2 SPS + PPS pair into the 48-byte unit format the
 * hardware reads from `pps_base`.  `dpb` is the 16-entry array from
 * v4l2_ctrl_h264_decode_params.dpb (the trailing 32-bit DPB word is
 * always re-emitted, even when the SPS+PPS prefix is reused).
 *
 * `field_pic` should be 1 if the picture is field-coded, else 0
 * (controls MbaffFrameFlag derivation). */
void H264PackSpsPpsUnit(uint8_t out[RKH264_SPSPPS_UNIT_SIZE],
                        const struct v4l2_ctrl_h264_sps           *sps,
                        const struct v4l2_ctrl_h264_pps           *pps,
                        const struct v4l2_h264_dpb_entry          *dpb,
                        int field_pic);

/* ---- Frame RPS --------------------------------------------------- */

/* Pack the per-frame RPS (frame_num_wrap[16] + RefPicList[3][32]) into
 * the 384-byte buffer the hardware reads from `rps_base`.
 *
 * `frame_num` is the current picture's frame_num (from
 * decode_params.frame_num).
 * `dpb`        — 16 v4l2_h264_dpb_entry; non-valid entries cleared.
 * `ref_lists`  — 3 lists of 32 references each (L0, L1, L2 for MVC;
 *                non-MVC uses L0 only, L1/L2 must be all-invalid).  An
 *                "invalid" reference is encoded by setting `index=0`
 *                AND clearing the V4L2_H264_*_REF bits in `fields`.
 *
 * For an IDR stream pass `frame_num=0`, all `dpb` entries with
 * `flags==0`, all `ref_lists` entries with `fields==0` — the resulting
 * buffer is all zeros, which the HW interprets as "no refs". */
void H264PackFrameRps(uint8_t out[RKH264_RPS_SIZE],
                      uint32_t frame_num,
                      uint32_t log2_max_frame_num_minus4,
                      const struct v4l2_h264_dpb_entry  *dpb,
                      const struct v4l2_h264_reference   ref_lists[3][32]);

/* ---- Scaling list ----------------------------------------------- */

/* Pack the V4L2 scaling matrix into the 224-byte buffer.  If `enable`
 * is 0, fills all bytes with zero (the HW treats zero entries as flat
 * 16). */
void H264PackScalingList(uint8_t out[RKH264_SCALING_LIST_SIZE],
                         const struct v4l2_ctrl_h264_scaling_matrix *sm,
                         int enable);

/* ---- RCB scratch sizing ------------------------------------------ */

typedef struct H264RcbInfo {
    uint32_t reg_idx;  /* register the base goes in (133..142) */
    uint32_t offset;   /* byte offset within the consolidated RCB buffer */
    uint32_t size;     /* aligned size in bytes */
} H264RcbInfo;

/* Fill `info[10]` with per-RCB region offsets/sizes for a frame of the
 * given pixel dimensions.  Returns the total buffer size needed (sum
 * of all aligned sub-sizes). */
uint32_t H264GetRcbBufferSizes(H264RcbInfo info[RKH264_RCB_COUNT],
                               uint32_t width_px, uint32_t height_px);

#ifdef __cplusplus
}
#endif
