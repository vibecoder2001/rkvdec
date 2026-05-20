/* mft/engine/decode_engine_av1.h
 *
 * AV1 hardware-decode engine — sibling of decode_engine.h for H.264 / HEVC.
 *
 * Architecture:
 *   - av1_parser (clean-room) as parser: parse each OBU in the TU directly,
 *     producing Dav1dSequenceHeader / Dav1dFrameHeader structs (dav1d struct
 *     definitions only, no dav1d library functions called).
 *   - regbuilder_av1 builds the VCD register array from those headers and
 *     the engine's per-stream RkmppAv1Dpb tracking state.
 *   - rkmpp.sys (RKCP3560 device personality) handles the actual kick.
 *
 * Operating modes (selected at Init time):
 *   - Av1EngineMode::Hardware — open RKCP3560 device, allocate buffers,
 *     submit kicks, return HW-decoded output.  Requires the rkmpp.sys
 *     AV1 personality to be enabled (see profile.c).
 *   - Av1EngineMode::Software — no-op; Software mode requires dav1d which
 *     is no longer linked.  The engine returns an empty queue when this
 *     mode is selected (hardware path only now).
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

#include "regbuilder_av1.h"
#include "av1_parser.h"

enum class Av1EngineMode {
    Hardware,   /* full HW kick path; needs rkmpp.sys with AV1 personality */
    Software,   /* legacy mode; no-op now that dav1d is removed             */
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
    /* True if film-grain synthesis is active (num_y_points > 0 in frame hdr;
     * the hardware path applies grain in-place (reg9.av1_fgs_en).         */
    bool      has_film_grain  = false;
};

/* One coded-frame OBU within a TU.  Public so Av1DecodeEngine can
 * hold a std::vector of these as per-instance state (was a static
 * thread_local global — race hazard once MF cross-thread-dispatches).
 * Definition kept compact; field semantics documented at the use
 * site in decode_engine_av1.cpp. */
struct AV1ObuRecord {
    uint32_t slice_start;
    uint32_t slice_size;
    uint32_t frame_tag_off;
    uint8_t  obu_type;
    bool     show_existing;
};

struct Av1DecodeEngine {
    Av1EngineMode mode = Av1EngineMode::Software;

    /* Clean-room parser state.  seq_hdr_valid is false until the first
     * OBU_SEQUENCE_HEADER has been parsed from the bitstream. */
    Dav1dSequenceHeader cached_seq_hdr{};
    bool                seq_hdr_valid = false;
    /* Per-DPB-slot saved frame states for primary_ref_frame inheritance
     * and show_existing_frame metadata (width/height/order_hint). */
    Av1SavedFrameState  saved_states[8]{};
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
    HwBuf tile_info;              /* per-tile dim/offset table (16 B per tile, max 128 tiles) */
    HwBuf global_model;           /* warped-motion params (224 B = 7 refs × 32 B) */
    HwBuf prob_tbl;               /* CDF probability tables in (sizeof AV1CDFs ≈ 12 KB) */
    HwBuf prob_tbl_out;           /* CDF probability tables out (codec writes here) */
    HwBuf film_grain;             /* film-grain LUT scratch */
    HwBuf filter_mem;             /* shared CDEF/LR/SR/DB col scratch (~100 KB at 720p) */
    HwBuf tile_buf;               /* mc_sync per-tile-col scratch */
    HwBuf tile_out_internal;      /* codec's per-kick internal Y/UV/colmv
                                   * write target (same buffer every kick;
                                   * proven bit-exact for kick 0). */
    HwBuf error_ref;              /* fallback reference */
    /* Reference + output frame pool.  AV1 needs 8 reference slots; we keep
     * a small surplus so the in-flight output frame doesn't collide with a
     * still-referenced slot. */
    static constexpr int kPoolSize = 12;
    /* Per-pool-slot snapshot of the codec's internal Y/UV/colmv layout
     * after a successful kick. Future kicks that reference this slot
     * read from here. Copied from tile_out_internal post-kick. */
    HwBuf pool_internal[kPoolSize];
    /* Per-pool-slot user-visible NV12 (PP module's output). */
    HwBuf pool_output[kPoolSize];
    /* Maps DPB slot index → pool index (or -1 if unmapped). Updated
     * post-kick from refresh_frame_flags. Read at kick time via
     * frame_hdr->refidx[r] to find each ref's pool buffer. */
    int   dpb_to_pool[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
    /* Per-DPB-slot snapshot of prob_tbl_out captured after each kick.
     * AV1 inter-frame decoding seeds CDFs from refidx[primary_ref_frame]'s
     * post-decode state; mirrors what BSP's MPP parser does in software. */
    std::vector<uint8_t> saved_cdf[8];

    uint32_t frame_width  = 0;
    uint32_t frame_height = 0;

    /* Per-Submit OBU walk results, consumed by DrainPictures.  Previously
     * `static thread_local` globals — fine for a single engine instance
     * on one thread, but a footgun: MF can call ProcessInput on a
     * different thread from ProcessOutput for sync MFTs.  Moved to
     * per-instance state.  Review MFT #14. */
    const uint8_t           *tu_ptr     = nullptr;
    size_t                   tu_len     = 0;
    std::vector<AV1ObuRecord> tu_obus;
    size_t                   tu_obu_idx = 0;

    /* Filter column buffer sub-offsets within filter_mem (bytes).
     * Computed per-kick from frame dims + bit depth; match
     * hal_av1d_vdpu.c:filtermem_alloc + mpp_dev_set_reg_offset(). */
    uint32_t filt_db_ctrl_off  = 0;
    uint32_t filt_cdef_col_off = 0;
    uint32_t filt_sr_col_off   = 0;
    uint32_t filt_lr_col_off   = 0;

    /* Reorder buffer mirrors decode_engine.cpp's H.265 pattern: each
     * decoded picture lands in reorder_q first; once the queue exceeds
     * `max_reorder_pics`, the lowest-PTS entry is bumped into ready_q
     * so PollFrame returns frames in display order.
     *
     * H.265 sources `max_reorder_pics` from the SPS's
     * sps_max_num_reorder_pics.  AV1 has no direct equivalent in the
     * sequence header, but `enable_order_hint` is a reliable proxy:
     * SVT-AV1 emits enable_order_hint=0 for pred-struct=1 (low-delay,
     * decode==display order) and =1 for pred-struct=2 (hierarchical,
     * needs reorder).  We resolve once when the first seq header is
     * parsed:
     *     enable_order_hint=0  →  max_reorder_pics = 0  (zero latency)
     *     enable_order_hint=1  →  max_reorder_pics = 8  (covers
     *                              hierarchical-levels up to 4) */
    size_t max_reorder_pics = 0;
    std::vector<Av1DecodedFrame> reorder_q;
    /* Pending decoded frames in display order, ready for PollFrame. */
    std::vector<Av1DecodedFrame> ready_q;
    /* Synthetic timestamp counter for callers that pass pts_hns=-1. */
    uint64_t  submit_count = 0;
};

/* Open the AV1 device (if mode == Hardware), prepare buffers for
 * `width x height` output frames.  Returns 0 on success. */
int Av1DecodeEngine_Init(Av1DecodeEngine *e, Av1EngineMode mode,
                         uint32_t width, uint32_t height);

/* Free everything. */
void Av1DecodeEngine_Shutdown(Av1DecodeEngine *e);

/* Submit one IVF temporal-unit worth of OBU bytes.  Each picture-producing
 * OBU (OBU_FRAME / OBU_FRAME_HEADER) is parsed and kicked; 0..N frames may
 * be appended to ready_q.  pts_hns is forwarded to every picture produced
 * from this call; pass -1 to use a synthetic counter. */
int Av1DecodeEngine_Submit(Av1DecodeEngine *e,
                           const uint8_t *obu, size_t len,
                           int64_t pts_hns);

/* Pop the next display-order frame.  Returns 1 if *out was filled, 0 if
 * the queue is empty (caller should Submit more), -1 on internal error. */
int Av1DecodeEngine_PollFrame(Av1DecodeEngine *e, Av1DecodedFrame *out);

/* Release the engine's hold on a previously-polled frame's pool slot.
 * Idempotent; sets f->slot_idx to -1. */
void Av1DecodeEngine_ReleaseFrame(Av1DecodeEngine *e, Av1DecodedFrame *f);

/* Flush parser state at end-of-stream.  The clean-room parser has no
 * internal picture reorder buffer, so this is a no-op for the output
 * queue; it clears parser DPB state matching Flush semantics. */
void Av1DecodeEngine_Drain(Av1DecodeEngine *e);

/* Reset DPB + reorder state (e.g. after seek).  Buffers stay allocated. */
int Av1DecodeEngine_Flush(Av1DecodeEngine *e);

/* Total frames currently sitting in ready_q. */
size_t Av1DecodeEngine_QueueDepth(const Av1DecodeEngine *e);
