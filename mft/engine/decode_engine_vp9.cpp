/* mft/engine/decode_engine_vp9.cpp — VP9 decode pipeline.
 *
 * Synchronous decode_one path: parse → regbuilder → SubmitDense →
 * wait → copy NV12 out.  Sticky engine state (last_show / prob_ref_poc
 * / segid_phase / DPB / etc.) lives on Vp9DecodeEngine and is threaded
 * into regbuilder_vp9 via RegbuildInputs each kick.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "decode_engine_vp9.h"
#include "../h264_packed_tables.h"  /* H264GetRcbBufferSizes (codec-agnostic) */
#include "../vp9_default_probs.h"   /* kDefaultProbs[] from BSP capture */
#include "repack_yuv.h"             /* RepackCodecOutputToNV12orP010 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#ifdef _WIN32
#  include <windows.h>
#endif

namespace {

constexpr uint32_t kProbSize        = 4864;       /* BSP PROB_SIZE */
constexpr uint32_t kProbSizeAlign4K = 0x2000;     /* BSP PROB_SIZE_ALIGN_TO_4K */
constexpr uint32_t kCountSizeAlign4K= 0x4000;     /* BSP COUNT_SIZE_ALIGN_TO_4K */
constexpr uint32_t kProbeSize       = kProbSizeAlign4K + kCountSizeAlign4K;
constexpr uint32_t kSegidSize       = 0x10000;    /* MAX_SEGMAP_SIZE_ALIGN_TO_4K */
constexpr uint32_t kBitstreamSize   = 4u * 1024u * 1024u; /* 4 MiB scratch */

static inline uint32_t align_up(uint32_t v, uint32_t a) {
    return (v + a - 1u) & ~(a - 1u);
}

static int free_pool_slot(Vp9DecodeEngine *e) {
    /* A pool slot is "available" only if neither the user nor the DPB
     * still references its buffers.  Releasing a frame back to the
     * user (Vp9DecodeEngine_ReleaseFrame) clears pool_in_use[], but
     * the DPB may still hold the same buffer as a reference for
     * subsequent inter frames; reusing it as the next output would
     * have the codec write into its own reference. */
    for (int i = 0; i < Vp9DecodeEngine::kPoolSize; ++i) {
        if (e->pool_in_use[i]) continue;
        uint64_t h = e->pool_output[i].handle;
        bool in_dpb = false;
        for (int r = 0; r < vp9::kNumRefFrames; ++r) {
            if (e->dpb.slots[r].valid && e->dpb.slots[r].frame_handle == h) {
                in_dpb = true; break;
            }
        }
        if (!in_dpb) return i;
    }
    return -1;
}

static int alloc_or_fail(Vp9DecodeEngine *e, uint32_t size,
                          DecodeEngineBufUsage usage, DecodeEngineBuf *buf,
                          const char *what)
{
    if (e->backend->AllocBuf(e->backend->ctx, size, usage, buf) != 0) {
        std::fprintf(stderr, "vp9_engine: AllocBuf(%s, %u) failed\n", what, size);
        return 1;
    }
    return 0;
}

} /* anon namespace */

int Vp9DecodeEngine_Init(Vp9DecodeEngine *e, uint32_t width, uint32_t height)
{
#ifdef _WIN32
    WindowsBackend_Init(&e->backend_storage, &e->device);
    return Vp9DecodeEngine_InitWithBackend(e, &e->backend_storage, width, height);
#else
    (void)e; (void)width; (void)height;
    std::fprintf(stderr, "Vp9DecodeEngine_Init: no default backend on Linux — "
                          "use Vp9DecodeEngine_InitWithBackend\n");
    return 1;
#endif
}

int Vp9DecodeEngine_InitWithBackend(Vp9DecodeEngine *e,
                                    DecodeEngineBackend *be,
                                    uint32_t width, uint32_t height)
{
    e->backend      = be;
    e->frame_width  = width;
    e->frame_height = height;

    if (be->Open(be->ctx, DE_CODEC_VP9) != 0) {
        std::fprintf(stderr, "vp9_engine: backend Open failed\n");
        return 1;
    }

    /* ---- Buffer sizing ---------------------------------------------- */
    /* Size for 10-bit (NV15-packed) so the same pool serves Profile 0/2
     * without reallocation if a stream changes bit-depth mid-decode.
     * Stride formula matches regbuilder_vp9.cpp:84 — 512-bit (64-byte)
     * aligned, in bytes.  8-bit costs ~1.25x more memory than strictly
     * needed; trivial at frame-pool sizes. */
    uint32_t hpad         = align_up(height, 64);
    uint32_t aligned_pitch_10 = ((width * 10u + 511u) & ~511u) / 8u;
    uint32_t frame_bytes  = aligned_pitch_10 * hpad * 3u / 2u;
    if (frame_bytes < 64u * 1024u) frame_bytes = 64u * 1024u;

    /* colmv: same shape as H.264/H.265 (codec-agnostic on vdpu34x). */
    uint32_t seg_cnt_w   = align_up(width,  64) / 64u;
    uint32_t seg_cnt_h   = align_up(height, 16) / 16u;
    uint32_t seg_head    = align_up(seg_cnt_w, 16u) * seg_cnt_h;
    uint32_t seg_payload = seg_cnt_w * seg_cnt_h * 64u * 16u;
    uint32_t colmv_bytes = align_up(seg_head + seg_payload, 128u);
    if (colmv_bytes < 4096u) colmv_bytes = 4096u;

    H264RcbInfo rcb_info[RKH264_RCB_COUNT]{};
    uint32_t rcb_total = H264GetRcbBufferSizes(rcb_info, width, height);
    if (rcb_total == 0) {
        std::fprintf(stderr, "vp9_engine: rcb sizer returned 0\n");
        goto fail;
    }
    for (int i = 0; i < 10 && i < RKH264_RCB_COUNT; ++i) {
        e->rcb_offsets[i] = rcb_info[i].offset;
    }

    /* ---- Single-shot allocations ------------------------------------ */
    if (alloc_or_fail(e, kBitstreamSize, DE_BUF_BITSTREAM_INPUT, &e->bitstream, "bitstream")) goto fail;
    if (alloc_or_fail(e, kProbeSize,     DE_BUF_SCRATCH,         &e->probe,        "probe")) goto fail;
    if (alloc_or_fail(e, kProbSize,      DE_BUF_SCRATCH,         &e->prob_default, "prob_default")) goto fail;
    for (int i = 0; i < 4; ++i) {
        char nm[32]; std::snprintf(nm, sizeof(nm), "prob_loop[%d]", i);
        if (alloc_or_fail(e, kProbSize, DE_BUF_SCRATCH, &e->prob_loop[i], nm)) goto fail;
    }
    if (alloc_or_fail(e, kSegidSize, DE_BUF_SCRATCH, &e->segid[0], "segid[0]")) goto fail;
    if (alloc_or_fail(e, kSegidSize, DE_BUF_SCRATCH, &e->segid[1], "segid[1]")) goto fail;
    if (alloc_or_fail(e, rcb_total,  DE_BUF_SCRATCH, &e->rcb[0],   "rcb")) goto fail;
    /* Re-expose the single RCB buffer at all 10 region handles — the
     * regbuilder writes one per region, but the BSP packs all 10 into
     * one allocation (capture confirmed). */
    for (int i = 1; i < 10; ++i) e->rcb[i] = e->rcb[0];
    if (alloc_or_fail(e, frame_bytes, DE_BUF_REFERENCE_FRAME, &e->error_ref, "error_ref")) goto fail;
    for (int i = 0; i < Vp9DecodeEngine::kPoolSize; ++i) {
        char nm[32]; std::snprintf(nm, sizeof(nm), "frame[%d]", i);
        if (alloc_or_fail(e, frame_bytes, DE_BUF_OUTPUT_FRAME, &e->pool_output[i], nm)) goto fail;
        std::snprintf(nm, sizeof(nm), "colmv[%d]", i);
        if (alloc_or_fail(e, colmv_bytes, DE_BUF_SCRATCH, &e->pool_colmv[i], nm)) goto fail;
        e->pool_in_use[i] = false;
    }

    /* segid[0]/segid[1] and probe DELIBERATELY NOT pre-zeroed — both
     * are HW-writeback targets (codec writes segment IDs into
     * segid_cur per kick; codec writes prob counts into probe at
     * offset 0x2000+).  A CPU memset here populates the cache with
     * dirty write-back lines; under cache pressure (especially with
     * a concurrent peer codec on RVD1 or a CPU-bound peer process)
     * those lines evict to DRAM at unpredictable times and clobber
     * the codec's writeback.  The next frame then reads garbage and
     * the kick wedges with hw=0x23.  The allocator returns zero-filled
     * pages anyway, and the codec doesn't require any particular
     * pre-seeded state for these buffers — leaving them untouched at
     * the CPU side means no dirty lines exist to evict.
     *
     * The same reasoning applies to prob_loop[fcx] — see below. */

    /* prob_default — load the BSP-captured initial probability table.
     * The codec reads this on keyframes (via reg162) to bootstrap its
     * CDF state; an all-zero buffer causes entropy decode to wedge. */
    if (e->prob_default.user_va) {
        std::memcpy(e->prob_default.user_va, vp9::kDefaultProbs,
                    vp9::kDefaultProbsSize <= e->prob_default.size
                        ? vp9::kDefaultProbsSize : e->prob_default.size);
    }
    /* prob_loop[fcx] buffers — DELIBERATELY NOT pre-seeded.  An earlier
     * memcpy of defaults here was defensive ("fresh context shouldn't
     * read stale data") but it's redundant: when prob_ctx_valid[fcx]
     * is 0, the regbuilder routes reg162 to prob_default_handle, so
     * the codec never reads prob_loop[fcx] in the cold case.  The
     * memcpy also produced dirty CPU cache lines that later evicted
     * to DRAM and clobbered the codec's writeback target — exactly
     * the hazard that wedged inter frames on Windows ARM with
     * hw=0x23.  Leaving the buffer uninitialised at the CPU side
     * means no dirty lines exist; the codec's first write (kf or
     * cold-context) is the first writer to the underlying DRAM. */

    return 0;

fail:
    Vp9DecodeEngine_Shutdown(e);
    return 1;
}

void Vp9DecodeEngine_Shutdown(Vp9DecodeEngine *e)
{
    if (!e || !e->backend) return;
    auto *be = e->backend;
    auto free_buf = [&](DecodeEngineBuf *b) {
        if (b->handle) be->FreeBuf(be->ctx, b);
    };
    /* RCB buffers all alias rcb[0]; only free once. */
    if (e->rcb[0].handle) {
        free_buf(&e->rcb[0]);
        for (int i = 1; i < 10; ++i) e->rcb[i] = DecodeEngineBuf{};
    }
    free_buf(&e->bitstream);
    free_buf(&e->probe);
    free_buf(&e->prob_default);
    for (int i = 0; i < 4; ++i) free_buf(&e->prob_loop[i]);
    free_buf(&e->segid[0]);
    free_buf(&e->segid[1]);
    free_buf(&e->error_ref);
    for (int i = 0; i < Vp9DecodeEngine::kPoolSize; ++i) {
        free_buf(&e->pool_output[i]);
        free_buf(&e->pool_colmv[i]);
    }
    be->Close(be->ctx);
    e->backend = nullptr;
}

int Vp9DecodeEngine_DecodeOne(Vp9DecodeEngine *e,
                              const uint8_t *frame, size_t len,
                              int64_t pts_hns,
                              Vp9DecodedFrame *out)
{
    if (!e || !out || !frame || !len) return -1;

    vp9::PicParams pp{};
    vp9::ProbUpdates pu{};
    auto pr = vp9::Vp9Parser_Parse(frame, len, e->parser_state, pp, pu);
    if (pr != vp9::ParseResult::Ok) {
        std::fprintf(stderr, "vp9_engine: parser failed\n");
        return -1;
    }

    /* Cascade firebreak gate.  See wait_for_keyframe docstring.  A key
     * frame (frame_type==0) or an intra_only frame is a self-contained
     * refresh point — safe to kick after a prior cascade.  Inter frames
     * and show_existing_frame are dropped silently (out->show=false,
     * empty yuv) so the consumer's emit loop just skips them. */
    bool is_refresh = (pp.frame_type == 0) || pp.intra_only;
    if (e->wait_for_keyframe && !is_refresh) {
        out->pts_hns = pts_hns;
        out->show = false;
        out->slot_idx = -1;
        out->yuv.clear();
        return 0;
    }

    /* show_existing_frame: emit cached slot, no kick. */
    if (pp.show_existing_frame) {
        uint8_t idx = pp.show_existing_frame_idx;
        if (idx >= vp9::kNumRefFrames || !e->dpb.slots[idx].valid) {
            std::fprintf(stderr, "vp9_engine: show_existing_frame on invalid slot %u\n", idx);
            return -1;
        }
        /* Find which pool slot owns that frame_handle. */
        int slot = -1;
        for (int i = 0; i < Vp9DecodeEngine::kPoolSize; ++i)
            if (e->pool_output[i].handle == e->dpb.slots[idx].frame_handle) {
                slot = i; break;
            }
        if (slot < 0) {
            std::fprintf(stderr, "vp9_engine: show_existing_frame slot not in pool\n");
            return -1;
        }
        out->pts_hns   = pts_hns;
        out->width     = e->dpb.slots[idx].width;
        out->height    = e->dpb.slots[idx].height;
        out->slot_idx  = slot;
        out->show      = true;
        out->bit_depth = e->dpb.slots[idx].bit_depth
                           ? e->dpb.slots[idx].bit_depth : 8u;
        {
            /* UV plane sits at src_stride * raw_height — VP9 regbuilder
             * programs y_virstride with the RAW height (regbuilder_vp9
             * .cpp:130), so HW writes UV immediately after the last
             * filled Y row with no vertical padding gap. */
            uint32_t src_stride = ((out->width * out->bit_depth + 511u) & ~511u) / 8u;
            RepackCodecOutputToNV12orP010(
                (const uint8_t*)e->pool_output[slot].user_va,
                src_stride, out->height,
                out->width, out->height, out->bit_depth,
                &out->yuv);
        }
        e->pool_in_use[slot] = true;
        return 0;
    }

    int slot = free_pool_slot(e);
    if (slot < 0) {
        std::fprintf(stderr, "vp9_engine: DPB pool exhausted\n");
        return -2;
    }

    /* Stage bitstream into our pinned input buffer.  BSP HAL zeros
     * the post-payload region up to align16(len)+128 (= reg16 STR_LEN);
     * the codec DMAs the full str_len and a non-zero tail can wedge
     * the bool decoder. */
    if (len > e->bitstream.size) {
        std::fprintf(stderr, "vp9_engine: frame %zu > bitstream buf %u\n", len, e->bitstream.size);
        return -1;
    }
    std::memcpy(e->bitstream.user_va, frame, len);
    size_t padded = ((len + 15u) & ~15u) + 128u;
    if (padded > e->bitstream.size) padded = e->bitstream.size;
    if (padded > len) {
        std::memset((uint8_t*)e->bitstream.user_va + len, 0, padded - len);
    }

    if (e->header_size_override) {
        pp.header_size = e->header_size_override;
    }

    /* Fill the prob buffer for this frame_context_idx. */
    uint8_t fcx = pp.frame_context_idx & 0x3u;
    /* Fill the probe buffer (reg160 = delta region).  Until our
     * parser-side prob-delta fill matches BSP byte-for-byte, allow
     * the harness to drop in a BSP-captured probe blob for the
     * specific stream under test. */
    if (e->probe.user_va) {
        bool used_blob = false;
        if (e->probe_blob_path) {
            FILE *fp = std::fopen(e->probe_blob_path, "rb");
            if (fp) {
                size_t n = std::fread(e->probe.user_va, 1,
                                       e->probe.size, fp);
                std::fclose(fp);
                used_blob = (n > 0);
                std::fprintf(stderr,
                    "vp9_engine: probe blob %s loaded %zu / %u bytes\n",
                    e->probe_blob_path, n, e->probe.size);
            } else {
                std::fprintf(stderr,
                    "vp9_engine: probe blob %s OPEN FAILED (errno=%d)\n",
                    e->probe_blob_path, (int)errno);
            }
            std::fflush(stderr);
        }
        if (!used_blob) {
            vp9::Vp9Regbuilder_FillProbs(pp, pu, (uint8_t*)e->probe.user_va);
        }
    }

    /* Build dense bank. */
    vp9::RegbuildInputs in{};
    in.pp                    = &pp;
    in.dpb                   = &e->dpb;
    in.bitstream_handle      = e->bitstream.handle;
    in.bitstream_offset      = 0;
    in.bitstream_bytes       = (uint32_t)len;
    in.decout_frame_handle   = e->pool_output[slot].handle;
    in.decout_colmv_handle   = e->pool_colmv[slot].handle;
    in.probe_handle          = e->probe.handle;
    in.prob_loop_handle      = e->prob_loop[fcx].handle;
    in.prob_default_handle   = e->prob_default.handle;
    in.segid_last_handle     = e->segid[e->segid_phase ^ 1u].handle;
    in.segid_cur_handle      = e->segid[e->segid_phase].handle;
    for (int i = 0; i < 10; ++i) {
        in.rcb_handles[i] = e->rcb[i].handle;
        in.rcb_offsets[i] = e->rcb_offsets[i];
    }
    in.error_ref_handle      = e->error_ref.handle;
    in.last_intra_only       = e->last_intra_only;
    in.last_tx_mode          = e->last_tx_mode;
    in.last_ref_mode         = e->last_ref_mode;
    in.col_ref_poc           = e->col_ref_poc;
    in.segid_ref_poc         = e->segid_ref_poc;
    for (int i = 0; i < 4; ++i) {
        in.prob_ref_poc[i]   = e->prob_ref_poc[i];
        in.prob_ctx_valid[i] = e->prob_ctx_valid[i];
    }
    in.last_show_frame             = e->last_show_frame;
    in.last_segmentation_enabled   = e->last_segmentation_enabled;
    in.last_widthheight_eqcur      = e->last_widthheight_eqcur;
    in.last_color_space            = e->last_color_space;
    in.last_mode_deltas[0]         = e->last_mode_deltas[0];
    in.last_mode_deltas[1]         = e->last_mode_deltas[1];
    for (int i = 0; i < 4; ++i) in.last_lf_ref_deltas[i] = e->last_lf_ref_deltas[i];

    H26xDenseOutput dense{};
    if (vp9::Vp9Regbuilder_Fill(in, &dense) != vp9::RegBuildStatus::Ok) {
        std::fprintf(stderr, "vp9_engine: regbuilder failed\n");
        e->wait_for_keyframe = true;
        return -3;
    }

    /* Optional pre-submit dump for BSP diff. */
    if (e->dump_prefix) {
        char path[256];
        std::snprintf(path, sizeof(path), "%s_%03d.bin",
                      e->dump_prefix, e->dump_idx++);
        FILE *fp = std::fopen(path, "wb");
        if (fp) {
            std::fwrite(&dense.Bank.Common,      sizeof(dense.Bank.Common),      1, fp);
            std::fwrite(&dense.Bank.CodecParams, sizeof(dense.Bank.CodecParams), 1, fp);
            std::fwrite(&dense.Bank.CommonAddr,  sizeof(dense.Bank.CommonAddr),  1, fp);
            std::fwrite(&dense.Bank.CodecAddr,   sizeof(dense.Bank.CodecAddr),   1, fp);
            std::fwrite(&dense.Bank.HighPoc,     sizeof(dense.Bank.HighPoc),     1, fp);
            std::fwrite(&dense.Bank.Stat,        sizeof(dense.Bank.Stat),        1, fp);
            std::fwrite(&dense.KickValue,        sizeof(dense.KickValue),        1, fp);
            std::fwrite(&dense.IovaSlotCount,    sizeof(dense.IovaSlotCount),    1, fp);
            std::fwrite(dense.IovaSlots,
                        sizeof(RKMPP_DENSE_IOVA_SLOT), dense.IovaSlotCount, fp);
            std::fclose(fp);
        }
    }

    /* Optional buffer-content dump for BSP diff.  Writes per-frame
     * probe/prob_default/prob_loop/segid/bitstream as separate files
     * under $RKMPP_VP9_BUF_DUMP_DIR. */
    if (const char *dd = std::getenv("RKMPP_VP9_BUF_DUMP_DIR")) {
        static int s_dump_idx = 0;
        auto dump = [&](const char *name, const void *p, size_t n) {
            char path[256];
            std::snprintf(path, sizeof(path), "%s/%03d_%s.bin",
                          dd, s_dump_idx, name);
            FILE *fp = std::fopen(path, "wb");
            if (fp) { std::fwrite(p, 1, n, fp); std::fclose(fp); }
        };
        if (e->probe.user_va)        dump("probe",        e->probe.user_va,        e->probe.size);
        if (e->prob_default.user_va) dump("prob_default", e->prob_default.user_va, e->prob_default.size);
        if (e->prob_loop[fcx].user_va)
            dump("prob_loop", e->prob_loop[fcx].user_va, e->prob_loop[fcx].size);
        if (e->segid[e->segid_phase].user_va)
            dump("segid_cur",  e->segid[e->segid_phase].user_va,  e->segid[e->segid_phase].size);
        if (e->segid[e->segid_phase ^ 1u].user_va)
            dump("segid_last", e->segid[e->segid_phase ^ 1u].user_va, e->segid[e->segid_phase ^ 1u].size);
        if (e->bitstream.user_va)
            dump("bitstream",  e->bitstream.user_va, len);
        std::fprintf(stderr, "vp9_engine: dumped buffers idx=%d\n", s_dump_idx);
        s_dump_idx++;
    }

    /* Submit + wait. */
    uint32_t hw_status = 0;
    int sub = e->backend->SubmitDense(e->backend->ctx, &dense, 1000, &hw_status);
    if (sub != 0) {
        std::fprintf(stderr, "vp9_engine: SubmitDense returned %d (hw=0x%x)\n", sub, hw_status);
        /* Failed kick may have partially written prob_loop[fcx] (HW
         * writeback target via reg172) before timing out — leaving
         * the buffer with corrupt mid-decode state.  If we leave
         * prob_ctx_valid[fcx]=1, the next inter frame using the same
         * context reads garbage CDFs through reg162 and wedges too,
         * cascading the failure indefinitely.  Invalidate the
         * context so the next frame routes reg162 to prob_default. */
        if (pp.refresh_frame_context) {
            uint8_t fcx_fail = pp.frame_context_idx & 0x3u;
            e->prob_ctx_valid[fcx_fail] = 0u;
        }
        /* Arm the cascade firebreak so subsequent inter frames are
         * dropped until the next keyframe / intra_only refresh point. */
        e->wait_for_keyframe = true;
        return -3;
    }

    /* Successful kick — if this was a refresh point, disarm the
     * firebreak so subsequent inter frames are submitted normally. */
    if (is_refresh) {
        e->wait_for_keyframe = false;
    }

    /* Repack codec output to display layout.
     *   Profile 0 (8-bit):  NV12 width*height*3/2 bytes.
     *   Profile 2 (10-bit): NV15 source → P010 upper-10 destination,
     *                       width*height*3/2 * 2 bytes.
     * Source stride matches regbuilder_vp9.cpp:84 (512-bit aligned). */
    out->pts_hns   = pts_hns;
    out->width     = pp.width;
    out->height    = pp.height;
    out->slot_idx  = slot;
    out->show      = pp.show_frame != 0;
    out->bit_depth = pp.bit_depth ? pp.bit_depth : 8u;
    {
        /* UV plane sits at src_stride * raw_height — see show_existing
         * branch above for full reasoning. */
        uint32_t src_stride = ((pp.width * out->bit_depth + 511u) & ~511u) / 8u;
        RepackCodecOutputToNV12orP010(
            (const uint8_t*)e->pool_output[slot].user_va,
            src_stride, pp.height,
            pp.width, pp.height, out->bit_depth,
            &out->yuv);
    }
    e->pool_in_use[slot] = true;

    /* Update DPB + sticky state. */
    vp9::Vp9Dpb_Update(e->dpb, pp,
                       e->pool_output[slot].handle,
                       e->pool_colmv[slot].handle,
                       pp.width, pp.height, pp.bit_depth ? pp.bit_depth : 8u);
    vp9::Vp9Parser_ApplyDpbUpdate(e->parser_state, pp);

    int32_t new_poc = (int32_t)(e->dpb.next_poc); /* DPB already incremented */
    bool is_intra = (pp.frame_type == 0) || pp.intra_only;
    e->last_intra_only            = is_intra ? 1u : 0u;
    e->last_tx_mode               = pp.txmode;
    e->last_ref_mode              = pp.reference_mode;
    e->last_show_frame            = pp.show_frame;
    e->last_segmentation_enabled  = pp.seg.enabled;
    e->last_widthheight_eqcur     = 1u;     /* updated to match next frame in caller */
    e->last_color_space           = pp.color_space & 0x7u;
    e->last_mode_deltas[0]        = pp.lf.mode_deltas[0];
    e->last_mode_deltas[1]        = pp.lf.mode_deltas[1];
    for (int i = 0; i < 4; ++i)
        e->last_lf_ref_deltas[i] = pp.lf.ref_deltas[i];
    if (pp.show_frame && !pp.show_existing_frame)
        e->col_ref_poc = new_poc;
    /* segid_ref_poc updates on the same conditions that set reg75
     * bit 15 (vp9_segment_id_update): seg.update_map, intra frames,
     * or error_resilient.  segid_phase MUST flip on the same predicate
     * (BSP hal_vp9d_vdpu34x.c:534-543) — when the map isn't rebuilt,
     * HW reads it from segid_last, so the buffer holding the persistent
     * map must stay parked at segid_last across consecutive inter
     * frames.  Unconditional flipping made segid_last point at the
     * 2-frames-old buffer once a stream had two inter frames in a row
     * with seg.enabled && !seg.update_map, corrupting the segment map. */
    const bool seg_id_update_now = is_intra
        || pp.error_resilient_mode
        || (pp.seg.enabled && pp.seg.update_map);
    if (seg_id_update_now)
        e->segid_ref_poc = new_poc;
    if (pp.refresh_frame_context) {
        e->prob_ref_poc[fcx]   = new_poc;
        e->prob_ctx_valid[fcx] = 1u;
    }
    if (seg_id_update_now)
        e->segid_phase ^= 1u;

    return 0;
}

void Vp9DecodeEngine_ReleaseFrame(Vp9DecodeEngine *e, Vp9DecodedFrame *f)
{
    if (!e || !f || f->slot_idx < 0) return;
    if (f->slot_idx < Vp9DecodeEngine::kPoolSize)
        e->pool_in_use[f->slot_idx] = false;
    f->slot_idx = -1;
    f->yuv.clear();
}
