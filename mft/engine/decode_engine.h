/* mft/engine/decode_engine.h
 *
 * Decode-engine library: glue between rkmpp.sys IOCTLs and the
 * parser_glue / dpb / regbuilder / packed_tables user-mode pieces.
 * Linked into both the production MFT DLL (mft/dll/) and the
 * rkmpp_decode CLI test harness (tests/harness/rkmpp_decode/).
 *
 * Single-frame-at-a-time API: feed one access unit, get one decoded
 * NV12 frame back, plus a reorder window for B-frame display order.
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

#include "parser_glue.h"
#include "parser_glue_h265.h"
#include "regbuilder_h264.h"
#include "regbuilder_h265.h"
#include "h264_packed_tables.h"
#include "h265_packed_tables.h"
#include "dpb.h"

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

    /* Slot of the most-recent successful DecodeOne_* — captured so
     * Submit can stash it in the queued ReorderEntry and PollFrame can
     * surface it to the consumer. */
    int last_decoded_slot = -1;

    /* Zero-copy readout opt-in.  When false, DecodeOne_* skips the
     * per-frame kernel→vector memcpy.  Consumers must read directly
     * from pool_output[slot_idx].user_va via DecodedFrame.slot_idx and
     * call DecodeEngine_ReleaseFrame when done.  Safety guaranteed by
     * the DPB's lifecycle-hold flags (see dpb.h Dpb_AddHold).
     * At 4K NV12 this avoids ~12 ms / frame of uncached read work. */
    bool populate_yuv = true;

    /* Per-class skip flag for non-reference frames.  When false, the
     * engine still decodes a non-ref AU (so the codec's internal state
     * advances) but skips the kernel→vector memcpy — leaving frame.yuv
     * empty.  The MFT consumer must skip emitting such frames.  Used
     * in concert with IMFQualityAdvise drop-mode handling: when EVR
     * signals we're behind, the MFT flips this off and skips the
     * engine memcpy + emit work for non-refs to keep up with audio. */
    bool populate_yuv_nonrefs = true;

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

    /* H.265 packed-table cache.  H265PackPPS / H265PackRPS /
     * H265PackScalingList output is fully determined by (active_vps_id,
     * active_sps_id, active_pps_id) and the parameter set contents.  When
     * the same IDs are active and no VPS/SPS/PPS was redefined this AU,
     * the buffers in pps_table/rps_table/scaling_list still hold the
     * exact bytes the codec needs — skip the rebuild.  Hits on every AU
     * after the first for any normal stream (single SPS+PPS).
     * Sentinel -1 means "no cached state, must build". */
    int last_h265_sps_id = -1;
    int last_h265_pps_id = -1;

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
        /* Slot the codec wrote this picture into — held via the DPB's
         * lifecycle-hold flags (REORDER → READY → CONSUMER) for the lifetime of this entry, so the
         * codec can't reuse the slot while it sits in reorder_q/ready_q.
         * -1 if no hold is taken. */
        int                    slot_idx = -1;
        /* True if this picture is a reference frame (nal_ref_flag for
         * HEVC, nal_ref_idc != 0 for H.264).  Consumed by IMFQualityAdvise
         * drop-mode logic in the MFT layer to skip non-ref outputs without
         * breaking inter-prediction for the rest of the GOP. */
        bool                   is_ref = false;
        /* Stream-epoch tag.  Caller-supplied at Submit time and forwarded
         * to PollFrame's DecodedFrame.epoch.  Lets the consumer drop
         * results that survived a flush/seek by comparing against its
         * current epoch — important on Media Foundation playback paths
         * where MediaSession can race new-timeline submits against
         * still-decoding old-timeline samples around loop / seek points. */
        uint32_t               epoch = 0;
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
    /* `yuv` is populated only when DecodeEngine.populate_yuv is true.
     * In zero-copy mode the buffer is empty and the consumer reads via
     * src_ptr instead. */
    std::vector<uint8_t> yuv;
    /* Slot index of the codec's pool_output for this frame — the same
     * slot is held via the DPB external-hold mechanism while this
     * DecodedFrame is alive.  Consumer must call DecodeEngine_ReleaseFrame
     * when done so the codec can reuse the slot for a future decode. */
    int                  slot_idx = -1;
    /* Reference-picture flag.  Consumed by EVR drop-mode handling to
     * skip non-ref frames when falling behind the audio clock. */
    bool                 is_ref = false;
    /* Stream-epoch tag — see ReorderEntry::epoch.  Set by Submit;
     * compared against the consumer's current epoch in ProcessOutput.
     * Mismatch ⇒ the frame survived a flush and should be discarded. */
    uint32_t             epoch = 0;
    /* Zero-copy readout fields.  Layout is codec-padded NV12: Y plane
     * occupies (src_width × src_height_pad) bytes at offset 0, UV plane
     * starts at offset (src_width × src_height_pad).  Valid until
     * DecodeEngine_ReleaseFrame is called. */
    void                *src_ptr        = nullptr;
    uint32_t             src_width      = 0;
    uint32_t             src_height_pad = 0;
};
int DecodeEngine_Submit(DecodeEngine *e,
                        const uint8_t *au, size_t au_len,
                        int64_t pts_hns,
                        uint32_t epoch = 0);
int DecodeEngine_SubmitFramed(DecodeEngine *e,
                              NalFraming framing, AvccLenSize len_size,
                              const uint8_t *au, size_t au_len,
                              int64_t pts_hns,
                              uint32_t epoch = 0);
int DecodeEngine_PollFrame(DecodeEngine *e, DecodedFrame *out);
void DecodeEngine_Drain(DecodeEngine *e);

/* Release the DPB external-hold for a previously-polled DecodedFrame.
 * Must be called after the consumer has copied / consumed the frame's
 * data.  Idempotent; sets f->slot_idx to -1.  Safe to skip when the
 * DecodedFrame has already been moved-from. */
void DecodeEngine_ReleaseFrame(DecodeEngine *e, DecodedFrame *f);

/* Total queued frames currently holding pool slots — reorder_q + ready_q.
 * MFT layer uses this to backpressure ProcessInput so the slot pool
 * doesn't get exhausted when the engine pumps faster than the consumer
 * (audio-locked EVR) drains.  Read only; does not lock. */
size_t DecodeEngine_QueueDepth(const DecodeEngine *e);

/* Post-decode reorder-queue management — factored out of Submit so it
 * can be exercised host-side without going through the kernel/IOCTL
 * decode path.
 *
 *   - If `is_idr_h264_boundary` is true, drain reorder_q to ready_q
 *     (lowest-POC first) BEFORE pushing the entry.  H.264 POC resets
 *     to 0 at every IDR; without this spill, pre-IDR tail frames stay
 *     stuck in reorder_q because all post-IDR entries have lower POCs
 *     and bump_lowest never picks them.  See h264_idr_reorder_fix.md.
 *   - Push `entry` into reorder_q.
 *   - Bump lowest-POC entries to ready_q while reorder_q size exceeds
 *     `max_num_reorder_pics`.
 *
 * Caller is responsible for filling entry.poc / pts_hns / dur_hns /
 * yuv / slot_idx / is_ref / epoch before calling.  Submit does this
 * automatically; tests do it manually. */
void DecodeEngine_OnDecodeComplete(DecodeEngine *e,
                                   DecodeEngine::ReorderEntry entry,
                                   uint32_t max_num_reorder_pics,
                                   bool is_idr_h264_boundary);
