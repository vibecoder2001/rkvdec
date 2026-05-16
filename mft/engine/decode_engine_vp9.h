/* mft/engine/decode_engine_vp9.h — VP9 decode pipeline.
 *
 * Mirrors the AV1 engine shape (own struct + Init/Shutdown/DecodeOne)
 * but routes submit through the existing DecodeEngineBackend vtable,
 * so the same engine builds for Windows (rkmpp.sys) and Linux (mpp
 * service).
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once

#include <cstdint>
#include <vector>

#include "../vp9_parser.h"
#include "../vp9_dpb.h"
#include "../regbuilder_vp9.h"
#include "decode_engine_backend.h"

struct Vp9DecodedFrame {
    int64_t              pts_hns   = 0;
    uint32_t             width     = 0;
    uint32_t             height    = 0;
    int                  slot_idx  = -1;
    bool                 show      = true;
    uint8_t              bit_depth = 8;        /* 8 for NV12, 10 for P010 */
    /* Packed NV12 (bit_depth=8) or P010 (bit_depth=10).
     * NV12: width*height*3/2 bytes.
     * P010: width*height*3/2 * 2 bytes (each sample is uint16, 10 valid
     * bits in upper 10).  Owned by the engine until
     * Vp9DecodeEngine_ReleaseFrame is called. */
    std::vector<uint8_t> yuv;
};

struct Vp9DecodeEngine {
    void                *device = nullptr;
    DecodeEngineBackend  backend_storage{};
    DecodeEngineBackend *backend = nullptr;

    /* Persistent parser + DPB state. */
    vp9::ParserState parser_state{};
    vp9::DpbCtx      dpb{};

    /* Per-session buffers.
     *
     * BSP's "probe" buffer (reg160 delta_prob, reg167 count_prob) is a
     * combined 24K region: 8K for the per-kick prob-delta workspace +
     * 16K for the count-update writeback area.  reg160 points at
     * offset 0, reg167 at offset 8192 (PROB_SIZE_ALIGN_TO_4K).
     *
     * prob_default (PROB_SIZE = 4864B) holds the spec-default CDFs
     * loaded into reg162 on keyframes.
     *
     * prob_loop[4] (PROB_SIZE each) are the per-frame-context loop
     * buffers — reg162 reads from prob_loop[fcx] on inter frames
     * (carrying last frame's adapted CDFs), reg172 is the writeback
     * target so HW saves updated CDFs for next frame.
     */
    DecodeEngineBuf bitstream{};
    DecodeEngineBuf probe{};         /* reg160 (offset 0) + reg167 (offset 8192) */
    DecodeEngineBuf prob_default{};  /* reg162 keyframe (4864B) */
    DecodeEngineBuf prob_loop[4]{};  /* reg162 inter / reg172 writeback (per fcx) */
    DecodeEngineBuf segid[2]{};      /* ping-pong */
    DecodeEngineBuf rcb[10]{};
    uint32_t        rcb_offsets[10]{};  /* per-reg-133..142 sub-region offset
                                         * within the shared RCB allocation */
    DecodeEngineBuf error_ref{};

    /* DPB pool: enough slots so the in-flight output frame doesn't
     * collide with a still-referenced VP9 ref slot (max 8).  Sized at
     * 16 = 8 (DPB max) + 4 (mft sample-pump queue depth) + 4 (margin)
     * so a fast consumer (mpv keeping several frames in flight) can't
     * starve the decoder. */
    static constexpr int kPoolSize = 16;
    DecodeEngineBuf pool_output[kPoolSize]{};
    DecodeEngineBuf pool_colmv[kPoolSize]{};
    bool            pool_in_use[kPoolSize]{};

    /* Sticky engine state threaded through regbuilder for reg94/99/etc. */
    uint8_t  last_tx_mode    = 4;     /* tx_mode_pre (reg76); BSP capture sets
                                       * it to 4 (TX_MODE_SELECT) on cold start
                                       * — written by hal_vp9d_setup_mode_info
                                       * before the first kick. */
    uint8_t  last_ref_mode   = 0;     /* frame_reference_mode_pre (reg76). */
    uint8_t  last_intra_only = 0;     /* prior frame was keyframe / intra-only.
                                       * 0 on cold start matches BSP regs_000
                                       * (true first kick, reg103 bit 30
                                       * cleared); we update to 1 after every
                                       * keyframe/intra-only kick. */
    uint8_t  last_show_frame = 0;
    uint8_t  last_segmentation_enabled = 0;
    uint8_t  last_widthheight_eqcur    = 0;
    uint8_t  last_color_space          = 0;
    int16_t  last_mode_deltas[2]      = {0, 0};
    int8_t   last_lf_ref_deltas[4]    = {0, 0, 0, 0};
    int32_t  col_ref_poc              = 0;
    int32_t  segid_ref_poc            = 0;
    int32_t  prob_ref_poc[4]          = {0, 0, 0, 0};
    uint8_t  segid_phase              = 0;
    uint8_t  prob_ctx_valid[4]        = {0, 0, 0, 0};

    uint32_t frame_width  = 0;
    uint32_t frame_height = 0;

    /* Cascade firebreak — set when SubmitDense (or any pre-kick fallible
     * step) fails.  While set, DecodeOne silently drops every non-
     * keyframe/non-intra-only frame (returning 0 with show=false / empty
     * yuv) so the codec isn't fed inter frames whose refs / colmv /
     * probability state were poisoned by the failed kick.  Cleared on
     * the first successful kick of a key/intra-only frame.  Parallel to
     * DecodeEngine.wait_for_idr for H.264/H.265 (see
     * [[decode_fail_cascade_firebreaks]]).  Without this the engine's
     * prob_ctx_valid clear only invalidates one CDF context — the next
     * inter frame may still fail for a different reason (corrupt ref
     * colmv, bad bool decoder state) and cascade indefinitely. */
    bool     wait_for_keyframe = false;

    /* Diagnostic: when non-null, every DecodeOne kick fwrites the
     * pre-submit H26xDenseOutput (Bank as raw u32 + iova slot table)
     * into a file named "<dump_prefix>_NNN.bin".  Set by the harness
     * via the --dump-bank flag. */
    const char *dump_prefix = nullptr;
    int         dump_idx    = 0;

    /* Diagnostic override for pp.header_size — our parser's f(16)
     * read at the end of the uncompressed header is off by 6 bits
     * vs BSP (open bug).  Setting this non-zero short-circuits the
     * parser-read value before the regbuilder fills reg78
     * lasttile_size.  Set by the harness via --header-size-override. */
    uint16_t header_size_override = 0;

    /* Diagnostic: when set, load this file's contents into e->probe
     * before the first kick.  BSP's `hal_vp9d_prob_flag_delta` walks
     * the parsed compressed header and writes specific byte offsets
     * we don't implement yet — using a BSP-captured probe blob lets
     * us bring up the HW path without the parser-side fill.
     * Captured via the BSP-side userspace patch in
     * tests/data/vp9/bsp_capture/probe_kf_000.bin. */
    const char *probe_blob_path = nullptr;
};

/* Open the rkmpp/mpp device and allocate per-session buffers sized for
 * `width x height` 8-bit VP9 (Profile 0).  Returns 0 on success. */
int  Vp9DecodeEngine_Init(Vp9DecodeEngine *e,
                          uint32_t width, uint32_t height);

/* Same as Init but routes everything through a caller-supplied backend
 * (linux_mpp_decode_vp9 uses this with LinuxBackend; mft_decode uses
 * WindowsBackend). */
int  Vp9DecodeEngine_InitWithBackend(Vp9DecodeEngine *e,
                                     DecodeEngineBackend *be,
                                     uint32_t width, uint32_t height);

void Vp9DecodeEngine_Shutdown(Vp9DecodeEngine *e);

/* Decode one VP9 frame (caller has already split superframes).
 * On success returns 0 and fills `*out` with the decoded NV12 + slot
 * index (caller releases via Vp9DecodeEngine_ReleaseFrame).
 *
 * `frame` points at the contiguous VP9 frame bytes; `len` is the frame
 * length.  `pts_hns` is forwarded to the produced Vp9DecodedFrame.
 *
 * Return codes:
 *    0  success, *out populated
 *   -1  parser error (malformed bitstream)
 *   -2  DPB pool exhausted
 *   -3  hardware submit/wait failure
 *   -4  bad output (hardware status reported error)
 *
 * On show_existing_frame the engine emits the cached slot without
 * touching hardware. */
int  Vp9DecodeEngine_DecodeOne(Vp9DecodeEngine *e,
                               const uint8_t *frame, size_t len,
                               int64_t pts_hns,
                               Vp9DecodedFrame *out);

/* Release a frame's slot back to the pool.  Required after every
 * DecodeOne that returned 0. */
void Vp9DecodeEngine_ReleaseFrame(Vp9DecodeEngine *e, Vp9DecodedFrame *f);
