/* tests/harness/rkmpp_decode/decode_engine.h
 *
 * Phase 3b first-decode harness.  Glue between rkmpp.sys IOCTLs and
 * the parser_glue/dpb/regbuilder/packed_tables user-mode pieces.
 *
 * Single-frame-at-a-time API: feed one access unit, get one decoded
 * NV12 frame back.  No threading, no streaming MFT shell, no error
 * recovery — the goal is to drive a real decode end-to-end against
 * the rkvdec2 hardware and see what comes out.
 *
 * Codec selection: DecodeEngine_Init takes a Codec enum and configures
 * per-codec buffer sizes + parser/regbuilder/DPB choice.  The driver-
 * facing IOCTL chain is identical (codec mode is encoded by the
 * register-list values themselves).
 */
#pragma once
#include <windows.h>
#include <cstdint>
#include <vector>

#include "../../../mft/parser_glue.h"
#include "../../../mft/parser_glue_h265.h"
#include "../../../mft/regbuilder_h264.h"
#include "../../../mft/regbuilder_h265.h"
#include "../../../mft/h264_packed_tables.h"
#include "../../../mft/h265_packed_tables.h"
#include "../../../mft/dpb.h"

enum class Codec {
    H264 = 0,
    H265 = 1,
};

struct DecodeEngine {
    Codec codec = Codec::H264;

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
    Buf cabac_init;           /* H.264: 3712B; H.265: 27456B */
    Buf pps_table;             /* H.264 SPS+PPS unit / HEVC 80*64 packed unit */
    Buf rps_table;             /* H.264 384B / HEVC 3200B */
    Buf scaling_list;          /* H.264 224B / HEVC 1360B */
    Buf rcb;                   /* consolidated RCB scratch (codec-agnostic geometry) */
    Buf error_ref;             /* fallback ref frame */
    /* DPB pool: must be >= max_dec_pic_buffering of any stream we accept.
     * H.264 / HEVC profiles cap max_dec_pic_buffering at 16; B-pyramid
     * streams typically need 5..8 live slots.  Pool of 4 was the Phase 3
     * "P-only first decode" minimum and caused H265Dpb_Select to return
     * DPB_FULL on the 6th AU of bframe.h265 (B-pyramid keeps 4+ refs live
     * across the I/P boundary, plus the in-flight pic, plus the output
     * reorder queue: 4 < 5 needed).  16 = spec maximum, no headroom math
     * needed; modest memory cost (~48 MiB at 1080p NV12). */
    static const int kPoolSize = 16;
    Buf pool_output[kPoolSize]; /* DPB output frames */
    Buf pool_colmv[kPoolSize];  /* per-slot colmv */

    /* DPB context — codec-specific, only one is used per Init. */
    DpbCtx     dpb_h264{};
    H265DpbCtx dpb_h265{};

    /* Per-RCB sub-region offsets (into rcb buffer).  Identical layout
     * for both codecs (vdpu34x is codec-agnostic at the RCB level). */
    H264RcbInfo rcb_info[RKH264_RCB_COUNT]{};

    uint32_t frame_width  = 0;
    uint32_t frame_height = 0;

    /* Parse scratch (RBSP unescape). */
    std::vector<uint8_t> scratch;

    /* Persistent parser state across AUs. */
    H264ParseResult parsed_h264{};
    H265ParseResult parsed_h265{};

    /* ---- Output reordering (display-order emit) -------------------- *
     * The codec hardware decodes in bitstream (decode) order; downstream
     * sinks expect display-order frames.  We implement a simplified spec-
     * compliant bump for HEVC C.5.2 / H.264 C.4:
     *
     *   - On every Submit: append (poc, yuv, pts, dur) to reorder_q.
     *   - While reorder_q.size() > max_num_reorder_pics, move the lowest
     *     POC entry to ready_q.
     *   - On Drain: move all reorder_q entries to ready_q in POC order.
     *   - PollFrame: pop ready_q front.
     *
     * Assumptions valid for our test corpus (B-pyramid streams, no
     * field coding, single-IDR open-GOP boundary): every decoded pic
     * has pic_output_flag=1 (no PPS output_flag_present rewrites);
     * no long-term refs; no missing pictures.  When we ship streams
     * that violate these we'll widen the algorithm.
     *
     * max_num_reorder_pics is set on the first slice from SPS:
     *   HEVC: sps_max_num_reorder_pics[sps_max_sub_layers_minus1]
     *   H.264: max_num_ref_frames (VUI's max_num_reorder_frames isn't
     *          parsed yet — fallback that's correct for B-pyramid
     *          streams since reorder ≤ ref_frames).
     */
    struct ReorderEntry {
        int32_t                poc;
        int64_t                pts_hns;
        int64_t                dur_hns;
        std::vector<uint8_t>   yuv;
    };
    std::vector<ReorderEntry>  reorder_q;
    std::vector<ReorderEntry>  ready_q;
    uint32_t                   max_num_reorder_pics = 0;
    /* Synthetic timeline counter used when the caller passes pts=-1
     * (Annex-B harness mode); HNS units. */
    uint64_t                   submit_count = 0;
};

/* Open the rkmpp device and the per-stream resources for a frame of
 * `width x height` (px) of the requested codec.  Pre-fills the
 * codec-appropriate CABAC table + RCB sizing. */
int DecodeEngine_Init(DecodeEngine *e, Codec codec,
                      uint32_t width, uint32_t height);

/* Free everything. */
void DecodeEngine_Shutdown(DecodeEngine *e);

/* Decode one Annex-B-framed access unit (typically one NAL set: VPS/SPS/
 * PPS + IDR slice for the first decode test).  Returns:
 *   0 on success — output frame is ready in *out_handle (bytes copied
 *     into out_yuv if the caller passed one)
 *   non-zero on error (with stderr message) */
int DecodeEngine_DecodeOne(DecodeEngine *e,
                           const uint8_t *au, size_t au_len,
                           std::vector<uint8_t> *out_yuv);

/* AVCC/HVCC variant: input `au` is a length-prefixed NAL buffer (as
 * delivered by MP4 / fragmented MP4 / WebM containers).  The buffer is
 * converted in-place into a scratch buffer and forwarded to
 * DecodeEngine_DecodeOne.  `len_size` is typically AVCC_LEN_4 (the only
 * width seen in real MP4 streams).  The MFT shell is the expected
 * caller; existing Annex-B test apps continue to use _DecodeOne. */
#include "../../../mft/avcc_to_annexb.h"
int DecodeEngine_DecodeOneFramed(DecodeEngine *e,
                                 NalFraming framing,
                                 AvccLenSize len_size,
                                 const uint8_t *au, size_t au_len,
                                 std::vector<uint8_t> *out_yuv);

/* Run the parser over `au` only — populate persistent SPS/PPS/(VPS)
 * state in the engine's parsed_h264/parsed_h265 structures, but do NOT
 * submit a job.  Suitable for feeding container-level extradata blobs
 * (avcC/hvcC parsed into Annex-B NAL bytes) before the first slice.
 * Returns 0 on success or if the parser reports NEED_MORE (i.e., parsed
 * SPS/PPS but found no slice — which is exactly the extradata case).
 * Non-zero on hard parse error.
 *
 * `framing`/`len_size` mean the same as DecodeOneFramed; AVCC inputs
 * are converted to Annex-B in a scratch buffer first. */
int DecodeEngine_FeedExtradata(DecodeEngine *e,
                               NalFraming framing,
                               AvccLenSize len_size,
                               const uint8_t *au, size_t au_len);

/* Reset the DPB and per-AU parser state.  Persistent SPS/PPS state is
 * preserved (a flush in MFT terms means "drop pending output, expect a
 * new IDR" — the SPS/PPS are still valid until the next IDR re-sends
 * them anyway).  Buffers are not reallocated. */
int DecodeEngine_Flush(DecodeEngine *e);

/* ---- Submit / Poll API (display-order output) -------------------- *
 *
 * Use this triple to emit frames in display order:
 *
 *   while (more input) {
 *     DecodeEngine_Submit(e, au, len, pts);
 *     while (DecodeEngine_PollFrame(e, &frame))
 *         consume(frame);
 *   }
 *   DecodeEngine_Drain(e);
 *   while (DecodeEngine_PollFrame(e, &frame))
 *     consume(frame);
 *
 * `Submit` accepts the same Annex-B-framed input as `DecodeEngine_DecodeOne`.
 * `pts_hns` is forwarded into the emitted DecodedFrame; pass -1 to use a
 * synthetic monotonic counter.
 *
 * `PollFrame` returns 1 if a frame was popped (and *out filled), 0 if the
 * reorder window isn't ready yet (caller should Submit more), -1 on a
 * post-decode internal error.
 *
 * `Drain` flushes the reorder window — every still-pending frame becomes
 * available via PollFrame in POC order. */
struct DecodedFrame {
    int32_t              poc;
    int64_t              pts_hns;
    int64_t              dur_hns;
    std::vector<uint8_t> yuv;
};
int DecodeEngine_Submit(DecodeEngine *e,
                        const uint8_t *au, size_t au_len,
                        int64_t pts_hns);
int DecodeEngine_SubmitFramed(DecodeEngine *e,
                              NalFraming framing, AvccLenSize len_size,
                              const uint8_t *au, size_t au_len,
                              int64_t pts_hns);
int DecodeEngine_PollFrame(DecodeEngine *e, DecodedFrame *out);
void DecodeEngine_Drain(DecodeEngine *e);
