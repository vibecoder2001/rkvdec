/* tests/harness/rkmpp_decode/decode_engine_av1.h
 *
 * AV1 hardware-decode engine — sibling of decode_engine.h for H.264 / HEVC.
 *
 * Architecture:
 *   - dav1d as parser:  feed each IVF/OBU temporal unit via dav1d_send_data.
 *     dav1d returns a Dav1dPicture per displayed frame (in display order)
 *     plus a Dav1dSequenceHeader / Dav1dFrameHeader pair.
 *   - regbuilder_av1 builds the VCD register array from those headers and
 *     the engine's per-stream RkmppAv1Dpb tracking state.
 *   - rkmpp.sys (RKCP3560 device personality) handles the actual kick.
 *
 * Operating modes (selected at Init time):
 *   - Av1EngineMode::Hardware — open RKCP3560 device, allocate buffers,
 *     submit kicks, return HW-decoded output.  Requires the rkmpp.sys
 *     AV1 personality to be enabled (see profile.c).
 *   - Av1EngineMode::Software — skip IOCTL path; return dav1d's software
 *     output as the "decoded" frame.  Used for dev-machine validation
 *     of the regbuilder + DPB + dav1d framing without hardware.
 *
 * Output is always NV12 (Y plane + interleaved CbCr plane), packed
 * width*height*3/2 bytes per frame, no padding.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <windows.h>
#include <cstdint>
#include <vector>

#include "../../../mft/regbuilder_av1.h"

extern "C" {
#include <dav1d/dav1d.h>
}

enum class Av1EngineMode {
    Hardware,   /* full HW kick path; needs rkmpp.sys with AV1 personality */
    Software,   /* dav1d-only; dev-machine validation, no IOCTLs           */
};

struct Av1DecodedFrame {
    int64_t   pts_hns         = 0;
    int64_t   dur_hns         = 0;
    uint32_t  width           = 0;
    uint32_t  height          = 0;
    /* Packed NV12 — width*height bytes Y followed by width*height/2
     * bytes interleaved CbCr.  Owned by the engine until ReleaseFrame
     * is called. */
    std::vector<uint8_t> yuv;
    /* Slot index in the engine's DPB pool; held until ReleaseFrame. */
    int       slot_idx        = -1;
    /* True if dav1d marked this picture as carrying film-grain synthesis;
     * the hardware path applies grain in-place (reg9.av1_fgs_en).         */
    bool      has_film_grain  = false;
};

struct Av1DecodeEngine {
    Av1EngineMode mode = Av1EngineMode::Software;

    /* dav1d context (always present). */
    Dav1dContext *dav1d = nullptr;
    /* Frame-header DPB tracking driven by regbuilder_av1.  Lives across
     * AUs; reset in Av1DecodeEngine_Flush. */
    RkmppAv1Dpb dpb{};

    /* Hardware-mode resources.  Unused in Software mode. */
    HANDLE   device = INVALID_HANDLE_VALUE;
    struct HwBuf {
        uint64_t handle  = 0;
        uint64_t iova    = 0;
        void    *user_va = nullptr;
        uint32_t size    = 0;
    };
    HwBuf bitstream;             /* OBU input */
    HwBuf tile_info;             /* packed uncompressed-header blob (~640 B) */
    HwBuf film_grain;            /* film-grain LUT scratch */
    HwBuf error_ref;             /* fallback reference */
    /* Reference + output frame pool.  AV1 needs 8 reference slots; we keep
     * a small surplus so the in-flight output frame doesn't collide with a
     * still-referenced slot. */
    static constexpr int kPoolSize = 12;
    HwBuf pool_output[kPoolSize];
    /* Maps DPB slot index → pool_output index (or -1 if unmapped). */
    int   dpb_to_pool[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };

    uint32_t frame_width  = 0;
    uint32_t frame_height = 0;

    /* Pending decoded frames in display order, ready for PollFrame. */
    std::vector<Av1DecodedFrame> ready_q;
    /* Synthetic timestamp counter for callers that pass pts_hns=-1. */
    uint64_t  submit_count = 0;
};

/* Open dav1d (and the AV1 device, if mode == Hardware), prepare buffers
 * for `width x height` output frames.  Returns 0 on success. */
int Av1DecodeEngine_Init(Av1DecodeEngine *e, Av1EngineMode mode,
                         uint32_t width, uint32_t height);

/* Free everything. */
void Av1DecodeEngine_Shutdown(Av1DecodeEngine *e);

/* Submit one IVF temporal-unit worth of OBU bytes.  dav1d may emit any
 * number of pictures (0..N); each emitted picture results in a hardware
 * kick (Hardware mode) or a dav1d-software output (Software mode) and
 * appends one Av1DecodedFrame to ready_q.  pts_hns is forwarded to every
 * picture produced from this call; pass -1 to use a synthetic counter. */
int Av1DecodeEngine_Submit(Av1DecodeEngine *e,
                           const uint8_t *obu, size_t len,
                           int64_t pts_hns);

/* Pop the next display-order frame.  Returns 1 if *out was filled, 0 if
 * the queue is empty (caller should Submit more), -1 on internal error. */
int Av1DecodeEngine_PollFrame(Av1DecodeEngine *e, Av1DecodedFrame *out);

/* Release the engine's hold on a previously-polled frame's pool slot.
 * Idempotent; sets f->slot_idx to -1. */
void Av1DecodeEngine_ReleaseFrame(Av1DecodeEngine *e, Av1DecodedFrame *f);

/* Drain dav1d's internal pipeline — call at end-of-stream so any pictures
 * dav1d is still holding for reordering get flushed into ready_q. */
void Av1DecodeEngine_Drain(Av1DecodeEngine *e);

/* Reset DPB + reorder state (e.g. after seek).  Buffers stay allocated. */
int Av1DecodeEngine_Flush(Av1DecodeEngine *e);

/* Total frames currently sitting in ready_q. */
size_t Av1DecodeEngine_QueueDepth(const Av1DecodeEngine *e);
