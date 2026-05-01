/* mft/h265_packed_tables.h — Rockchip-format packed buffers feeding the
 * rkvdec2 H.265 (vdpu34x) register banks.
 *
 * The hardware reads three side tables in addition to the bitstream and
 * the (codec-agnostic) CABAC init table:
 *
 *   - SPS+PPS unit  (reg161 pps_base, 80 bytes per slot replicated 64x)
 *   - SPS RPS table (reg163 rps_base, 3200 bytes — SPS-side LT + STRPS)
 *   - scaling list  (reg180 scanlist_addr, 1360 bytes per (sps_id,pps_id)
 *                    combo; only consumed when scaling_list_enabled_flag)
 *
 * Unlike the H.264 path, the HEVC RPS table is *not* per-AU: it carries
 * only SPS-side state.  In `HW_RPS` mode (the only mode the rk3588
 * hal_h265d_vdpu34x.c uses), reg64.bit1 = 0 (DPB-style), and the
 * hardware re-derives the per-AU RPS itself by re-parsing
 * st_ref_pic_set( … ) out of the slice header.  So the regbuilder only
 * needs to refresh this buffer when the active SPS changes.
 *
 * Cross-reference:
 *   prepare_pps        : hal_h265d_vdpu34x.c:443 (hal_h265d_output_pps_packet)
 *   prepare_rps        : hal_h265d_com.c:300    (hal_h265d_slice_hw_rps)
 *   prepare_scaling    : hal_h265d_com.c:84     (hal_record_scaling_list)
 *                        + hal_h265d_com.c:687  (hal_h265d_output_scalinglist_packet)
 *   buffer geometry    : hal_h265d_vdpu34x.c:88..99 (CABAC/SPSPPS/RPS/SCALIST)
 *
 * Capture cross-check: Z:\drivers-arm\bsp_capture\hevc_capture\mpp.shim.h265.log
 * shows fd=12 alloc len=122880 == ALIGN(7168,4K) + ALIGN(3200,4K) +
 * ALIGN(110160,4K) — the consolidated info buffer for one decode slot.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#include "parser_glue_h265.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Buffer sizing constants ------------------------------------- *
 * Each constant is the *useful* payload size; the BSP rounds these up
 * to 4 KiB for DMA-buffer allocation but writes are bounded by the
 * payload sizes here.  See hal_h265d_vdpu34x.c PPS_SIZE / RPS_ALIGEND_SIZE
 * / SCALING_LIST_SIZE macros for the matching aligned-allocation sizes. */

/* One SPS+PPS slot replicated 64 times (the codec uses pps_id mod 64
 * to index into this table).  Each slot is 112 bytes on vdpu34x v345
 * (RK3568/RK3588) — `for (i=0; i<64; i++) memcpy(pps_ptr + i*112,
 * pps_buf, 112);` in hal_h265d_vdpu34x.c:432 (v345 packer).  The
 * older non-v345 path uses 80 bytes / slot but it's not the one
 * RK3588 hardware reads.  See PPS_SIZE = 112*64 at line 42 of the
 * same file. */
#define RKH265_SPSPPS_SLOT_SIZE     112u                      /* one entry  */
#define RKH265_SPSPPS_NUM_SLOTS     64u                       /* hw lookup  */
#define RKH265_SPSPPS_UNIT_SIZE     (RKH265_SPSPPS_SLOT_SIZE * RKH265_SPSPPS_NUM_SLOTS)

/* SPS-side RPS table (HW_RPS mode): 32 LT entries + 64 STRPS slots,
 * BSP allocates 400 u64 = 3200 bytes (hal_h265d_vdpu34x.c:90 RPS_ALIGEND_SIZE
 * is 4 KiB rounded). */
#define RKH265_RPS_SIZE             (400u * 8u)               /* 3200 bytes */

/* scalingFactor_t (hal_h265d_ctx.h:97): 1248+96+12+4 = 1360 bytes per
 * (sps_id, pps_id) combo.  The BSP allocates 81 slots (= sps_max +
 * pps_max + default) of 1360 bytes for a total of 110160 bytes; we
 * pack just one slot at offset 0 since the regbuilder always points
 * reg180 at offset 0 of this buffer. */
#define RKH265_SCALING_LIST_SIZE    1360u

/* HEVC CABAC init table (codec-agnostic from rkvdec2 PoV): 27456 bytes,
 * exposed here only so the harness allocator can size the consolidated
 * info buffer.  The actual blob lives in hal_h265d_com.c cabac_table[]
 * and is mirrored verbatim in h265_cabac_init.inc; H265GetCabacInitTable
 * returns a pointer to it for the harness/MFT to memcpy into the cabac
 * dma buffer. */
#define RKH265_CABAC_INIT_SIZE      27456u

/* HEVC also wants ~128B prefetch slack at end of buffer like H.264. */
#define RKH265_TABLE_TAIL_PAD       128u

/* Returns the 27456-byte HEVC CABAC init blob (static, owned by the
 * library).  `out_byte_count` is set to RKH265_CABAC_INIT_SIZE on
 * success (always).  Mirror of H264GetCabacInitTable. */
const uint8_t *H265GetCabacInitTable(size_t *out_byte_count);

/* ---- SPS+PPS packed unit ----------------------------------------- *
 *
 * Packs one (VPS, SPS, PPS) tuple into the 80-byte unit format the
 * hardware reads from `pps_base`, then replicates that 80-byte unit 64
 * times into `out` (total RKH265_SPSPPS_UNIT_SIZE bytes).
 *
 * `scanlist_offset_words` is the (sps_id*1360 / 4) offset embedded in
 * the BSP scanlist-addr field (lower 22 bits hold a buffer fd, upper 10
 * hold the offset; we always pass 0 for offset since we use a single-
 * slot scanlist buffer — task 4 fills the fd via the iova-substitution
 * mechanism).
 *
 * Returns the number of bytes written (RKH265_SPSPPS_UNIT_SIZE) on
 * success, or -1 if `out_size` is insufficient.
 *
 * Mirrors hal_h265d_output_pps_packet (hal_h265d_vdpu34x.c:443..655). */
int H265PackPPS(const H265Vps *vps,
                const H265Sps *sps,
                const H265Pps *pps,
                uint8_t *out, size_t out_size);

/* ---- SPS RPS table ----------------------------------------------- *
 *
 * Pack the SPS-side LT-ref + STRPS data into the 3200-byte buffer the
 * hardware reads from `rps_base`.  Inputs come from the *active* SPS
 * in the parse result; per-AU DPB info is *not* used in HW_RPS mode
 * (kept as a parameter only for the regbuilder's call-site symmetry
 * with the H.264 packer — the codec re-derives the per-pic RPS on its
 * own from the slice header's st_ref_pic_set() bits).
 *
 * Returns RKH265_RPS_SIZE on success, -1 on out-of-buffer.
 *
 * Mirrors hal_h265d_slice_hw_rps (hal_h265d_com.c:300..360). */
int H265PackRPS(const H265ParseResult *parsed,
                uint8_t *out, size_t out_size);

/* ---- Scaling list ------------------------------------------------ *
 *
 * Pack the active scaling list (selection rule per BSP: PPS list if
 * pps_scaling_list_data_present_flag, else SPS list if
 * sps_scaling_list_data_present_flag, else default flat-16 lists)
 * into the 1360-byte scalingFactor_t blob the codec consumes.
 *
 * The parser stores the lists in RASTER order (parser_glue_h265.cpp
 * applies the inverse diagonal scan during parse), so this function
 * only handles the BSP's per-matrix 4x4/8x8 transposition + the
 * scalingfactor1 4x4-rotated copy + DC-coeff section.
 *
 * Returns RKH265_SCALING_LIST_SIZE on success, -1 on out-of-buffer. */
int H265PackScalingList(const H265Sps *sps,
                        const H265Pps *pps,
                        uint8_t *out, size_t out_size);

#ifdef __cplusplus
}
#endif
