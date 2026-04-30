/* tests/harness/rkmpp_decode/decode_engine.h
 *
 * Phase 3b first-decode harness.  Glue between rkmpp.sys IOCTLs and
 * the parser_glue/dpb/regbuilder/packed_tables user-mode pieces.
 *
 * Single-frame-at-a-time API: feed one access unit, get one decoded
 * NV12 frame back.  No threading, no streaming MFT shell, no error
 * recovery — the goal is to drive a real decode end-to-end against
 * the rkvdec2 hardware and see what comes out.
 */
#pragma once
#include <windows.h>
#include <cstdint>
#include <vector>

#include "../../../mft/parser_glue.h"
#include "../../../mft/regbuilder_h264.h"
#include "../../../mft/h264_packed_tables.h"
#include "../../../mft/dpb.h"

struct DecodeEngine {
    /* Device handle from CreateFile on the rkmpp interface. */
    HANDLE device = INVALID_HANDLE_VALUE;

    /* Buffer cookies + iovas + user VAs returned by IOCTL_RKMPP_ALLOC_BUFFER. */
    struct Buf {
        uint64_t handle = 0;
        uint64_t iova   = 0;
        void    *user_va = nullptr;
        uint32_t size   = 0;
    };
    Buf bitstream;            /* Annex-B input */
    Buf cabac_init;           /* 3712B static blob */
    Buf pps_table;             /* RKH264_SPSPPS_UNIT_SIZE */
    Buf rps_table;             /* RKH264_RPS_SIZE */
    Buf scaling_list;          /* RKH264_SCALING_LIST_SIZE */
    Buf rcb;                   /* consolidated RCB scratch */
    Buf error_ref;             /* fallback ref frame */
    static const int kPoolSize = 4;
    Buf pool_output[kPoolSize]; /* DPB output frames */
    Buf pool_colmv[kPoolSize];  /* per-slot colmv */

    /* DPB context. */
    DpbCtx dpb{};

    /* Per-RCB sub-region offsets (into rcb buffer). */
    H264RcbInfo rcb_info[RKH264_RCB_COUNT]{};

    uint32_t frame_width  = 0;
    uint32_t frame_height = 0;

    /* Parse scratch (RBSP unescape). */
    std::vector<uint8_t> scratch;
};

/* Open the rkmpp device and the per-stream resources for a frame of
 * `width x height` (px).  Pre-fills CABAC + RCB. */
int DecodeEngine_Init(DecodeEngine *e, uint32_t width, uint32_t height);

/* Free everything. */
void DecodeEngine_Shutdown(DecodeEngine *e);

/* Decode one Annex-B-framed access unit (typically one NAL set: SPS +
 * PPS + IDR slice for the first decode test).  Returns:
 *   0 on success — output frame is ready in *out_handle (bytes copied
 *     into out_yuv if the caller passed one)
 *   non-zero on error (with stderr message) */
int DecodeEngine_DecodeOne(DecodeEngine *e,
                           const uint8_t *au, size_t au_len,
                           std::vector<uint8_t> *out_yuv);
