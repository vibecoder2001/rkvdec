/* mft/regbuilder_av1.h — AV1 register-array builder.
 *
 * Consumes parsed AV1 sequence + frame headers (typically from dav1d's
 * Dav1dSequenceHeader / Dav1dFrameHeader) plus DMA buffer FDs, and
 * fills a VdpuAv1dRegSet sized 512-u32 array suitable for submission
 * to the rkvdec_av1 hardware via /dev/mpp_service or our Windows
 * rkmpp.sys driver.
 *
 * Output layout matches the kernel-side flat regs[] array used by
 * BSP's mpp_av1dec.c — index N = swreg N, byte offset = N * 4.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>  /* FILE * for the dump helper */

#include "regbuilder_av1_reg.h"  /* VdpuAv1dRegSet bitfield struct (Apache-2.0) */

#ifdef __cplusplus
extern "C" {
#endif

/* Number of u32 entries in the VCD register window we emit.  Matches
 * the BSP's reg_class[VCD] kernel allocation (512 slots = 2 KiB). */
#define RKMPP_AV1_VCD_REGS  512

/* DMA-buffer handles — opaque ints (FDs on Linux, device handles on
 * Windows) that the kernel-side IOMMU framework will translate to
 * IOVAs at submission time.  We write them into the *_lsb register
 * fields; *_msb stays 0 — kernel patches in the high byte. */
typedef struct RkmppAv1Buffers {
    /* Output frame buffer (NV12). */
    int      output_y_fd;
    int      output_uv_fd;     /* if separate plane allocation; else equal to output_y_fd */
    uint32_t output_y_offset;
    uint32_t output_uv_offset;

    /* Up to 7 reference frames (AV1 has 8 slots indexed 0..6 by RefFrame) */
    int      ref_y_fd[7];
    int      ref_uv_fd[7];
    uint32_t ref_y_offset[7];
    uint32_t ref_uv_offset[7];

    /* Input compressed bitstream buffer */
    int      bitstream_fd;
    uint32_t bitstream_offset;
    uint32_t bitstream_length;

    /* Tile-info buffer (cdef strengths, segmentation params, etc.) */
    int      tile_info_fd;
    uint32_t tile_info_offset;

    /* Reference film-grain LUT (when seq has film_grain_present) */
    int      film_grain_fd;
    uint32_t film_grain_offset;

    /* Reference buffer for missing/error frames */
    int      error_ref_fd;
} RkmppAv1Buffers;

/* Forward declarations — caller passes these in.  We don't include
 * dav1d/headers.h here so that consumers without dav1d can still use
 * this header; the cpp file does the actual dav1d API include. */
struct Dav1dSequenceHeader;
struct Dav1dFrameHeader;

/* AV1 DPB tracking — 8 reference slots that frames can read from
 * (selected via Dav1dFrameHeader::refidx[]) and update on completion
 * (via refresh_frame_flags bitmask).  The regbuilder consumes this
 * state to fill per-ref register fields (dimensions, scales, sign
 * bias, offset deltas) and the caller updates it after each decode.
 *
 * Design note: the slots store decoded-frame info we need REG-side
 * (offsets, dims, valid bit) — NOT the actual NV12 buffers.  The
 * caller owns those and passes per-slot DMA fds in RkmppAv1Buffers
 * when relevant. */
typedef struct RkmppAv1DpbSlot {
    uint8_t  valid;
    uint8_t  frame_type;     /* Dav1dFrameType; KEY/INTRA invalidate refs */
    int16_t  frame_offset;   /* AV1 order hint (signed delta arithmetic) */
    uint16_t coded_width;
    uint16_t coded_height;
    uint16_t upscaled_width;
    /* Buffer identity (decode-order serial) — incremented per decoded
     * frame.  Slots refreshed by the same frame share the same buffer_id
     * so the ref_frames count can de-duplicate aliased slots, matching
     * BSP HAL's "unique buffer index" counting. */
    uint32_t buffer_id;
    /* Order hints of the 7 refs (LAST..ALTREF) that the frame in this
     * slot used at the time it was decoded.  Per AV1 spec §7.20,
     * SavedOrderHints[ref] = OrderHints[ref] at decode time.
     * Used by the BSP MF projection algorithm when this slot is later
     * selected as one of the up-to-3 motion-field source frames. */
    int16_t  saved_order_hints[7];
} RkmppAv1DpbSlot;

typedef struct RkmppAv1Dpb {
    RkmppAv1DpbSlot slots[8];
    /* OrderHintBits from sequence header, captured at first init.  Used
     * for signed arithmetic on frame_offset diffs.  0 disables hints
     * (forced low-delay / error-resilient mode). */
    uint8_t  order_hint_bits;
    /* Decode-order counter incremented on every successful post_decode
     * call (regardless of refresh_frame_flags); used to tag refreshed
     * slots so callers can distinguish "two slots holding the same
     * buffer" from "two slots holding different buffers that happen to
     * share an order_hint". */
    uint32_t next_buffer_id;
} RkmppAv1Dpb;

/* Reset all 8 slots to invalid (called at stream start or seq change). */
void rkmpp_av1_dpb_init(RkmppAv1Dpb *dpb);

/* Update the DPB after a successful decode: any slot whose bit is set
 * in hdr->refresh_frame_flags is overwritten with the new frame's
 * info.  Caller invokes this after a successful build_regs+kick. */
void rkmpp_av1_dpb_post_decode(
    RkmppAv1Dpb                      *dpb,
    const struct Dav1dSequenceHeader *seq,
    const struct Dav1dFrameHeader    *hdr);

typedef enum {
    RKMPP_AV1_OK = 0,
    RKMPP_AV1_ERR_BAD_INPUT,
    RKMPP_AV1_ERR_UNSUPPORTED,
} RkmppAv1Status;

/* Fill a VdpuAv1dRegSet with the register state for one AV1 frame
 * decode kick.  Caller must zero `out` before calling.
 *
 *   seq, hdr   — parsed AV1 syntax (dav1d structures)
 *   bufs       — DMA buffer handles for output, refs, bitstream, etc.
 *   out        — destination register set (must be zeroed by caller)
 *
 * Returns RKMPP_AV1_OK if the frame can be programmed; an error code
 * otherwise (e.g., 10-bit input, scalable layers).
 */
RkmppAv1Status rkmpp_av1_build_regs(
    const struct Dav1dSequenceHeader *seq,
    const struct Dav1dFrameHeader    *hdr,
    const RkmppAv1Dpb                *dpb,
    const RkmppAv1Buffers            *bufs,
    VdpuAv1dRegSet                   *out);

/* Convenience: dump the produced register array in the same format
 * as the BSP shim trace: "AV1SHIM r[0][NNN]=VVVVVVVV" lines for each
 * non-zero register.  Useful for diffing against the captured trace. */
void rkmpp_av1_dump_regs_shim(const VdpuAv1dRegSet *regs, FILE *out);

#ifdef __cplusplus
}  /* extern "C" */
#endif
