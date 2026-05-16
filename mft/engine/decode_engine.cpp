/* mft/engine/decode_engine.cpp */
#include "decode_engine.h"
#include "repack_yuv.h"
#include "../au_iter.h"

#ifdef _WIN32
#include "../../shared/rkmpp_ioctl.h"
#endif

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <utility>

#ifdef _WIN32
#  define DECODE_ENV_GET(name, buf) \
        (GetEnvironmentVariableA(name, buf, sizeof(buf)) > 0)
#else
#  include <time.h>
#  include <stdio.h>
#  define DECODE_ENV_GET(name, buf) ({ \
        const char *_v = ::getenv(name); \
        if (_v) { strncpy(buf, _v, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0; } \
        _v != nullptr; \
    })
typedef unsigned long DWORD;   /* used by Fail's ec parameter; harmless on Linux */
#endif

static int Fail(const char *m, unsigned long ec = 0) {
    std::fprintf(stderr, "decode_engine: %s (%lu)\n", m, ec);
    return 1;
}

/* Diagnostic: pre-fill the output frame buffer with a marker byte before
 * each H.264 kick, gated on RKMPP_OUTPUT_FILL=<byte>.  Targets the open
 * hypothesis from h264_cavlc_idr_divergence.md that BSP allocates fd 11
 * with a 0xD6 fill pattern (probably mpp's debug-fill init) while our
 * `RkMppBufAlloc` zero-fills, and the codec leaves *some* MB region
 * untouched during I-slice decode → the init pattern leaks into output.
 *
 * Set RKMPP_OUTPUT_FILL=0xD6 to mimic BSP, RKMPP_OUTPUT_FILL=0xAA / 0x55
 * for distinct sentinels, unset to keep zero-fill (default).  Compares
 * against the BSP YUV with `fc /b ours.yuv bsp.yuv` to determine whether
 * the divergence comes from leak-through.  Returns -1 when unset. */
static int OutputFillByte() {
    static int cached = -2;
    if (cached == -2) {
        char buf[16] = {};
        if (!DECODE_ENV_GET("RKMPP_OUTPUT_FILL", buf)) {
            cached = -1;
            std::fprintf(stderr,
                "RKMPP_OUTPUT_FILL: unset (output buffer left as kernel-zeroed)\n");
            return -1;
        }
        unsigned long v = strtoul(buf, nullptr,
                                  (buf[0]=='0' && (buf[1]=='x' || buf[1]=='X')) ? 16 : 10);
        cached = (int)(v & 0xff);
        std::fprintf(stderr,
            "RKMPP_OUTPUT_FILL=%s -> pre-filling each H.264 output buffer with 0x%02x\n",
            buf, cached);
    }
    return cached;
}

/* Per-frame debug spam (DMA buffer dumps to win_*.bin, register-list
 * dumps, slice-offset prints) was useful during bring-up but each frame
 * triggers ~5 fopen/fwrite/fclose cycles + 100+ printf lines.  On a
 * network-redirected stdout or Z:\ working dir that adds ~50 ms / frame.
 * Gate it behind RKMPP_DECODE_DEBUG=1 (read once on first use). */
static bool DecodeDebugEnabled() {
    static int cached = -1;
    if (cached < 0) {
        char buf[8] = {};
        cached = (DECODE_ENV_GET("RKMPP_DECODE_DEBUG", buf) && buf[0] != '0') ? 1 : 0;
    }
    return cached != 0;
}

/* Per-stage timing: gated on RKMPP_TIMING=1.  Prints one CSV line per
 * decode with microseconds spent in each engine stage so the overall
 * frame budget can be apportioned (parser / regbuilder / kernel kick /
 * codec wait / kernel→vector copy).  Header line printed on first call. */
static bool TimingEnabled() {
    static int cached = -1;
    if (cached < 0) {
        char buf[8] = {};
        cached = (DECODE_ENV_GET("RKMPP_TIMING", buf) && buf[0] != '0') ? 1 : 0;
    }
    return cached != 0;
}

#ifdef _WIN32
static int64_t QpcNow() {
    LARGE_INTEGER c; QueryPerformanceCounter(&c); return c.QuadPart;
}
static int64_t QpcFreq() {
    static int64_t f = 0;
    if (!f) { LARGE_INTEGER q; QueryPerformanceFrequency(&q); f = q.QuadPart; }
    return f;
}
static int64_t QpcUs(int64_t a, int64_t b) {
    return (b - a) * 1'000'000LL / QpcFreq();
}
#else
static int64_t QpcNow() {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1'000'000'000LL + ts.tv_nsec;
}
static int64_t QpcUs(int64_t a, int64_t b) { return (b - a) / 1000LL; }
#endif

struct StageTimes {
    int64_t parse_us  = 0;
    int64_t pack_us   = 0;  /* DPB select + pack PPS/RPS/scaling */
    int64_t regbuild_us = 0;
    int64_t submit_us = 0;
    int64_t wait_us   = 0;
    int64_t copy_us   = 0;  /* kernel → out_yuv */
};

static void EmitTimingCsv(const StageTimes &t, const char *codec, int32_t poc) {
    static bool header_printed = false;
    if (!header_printed) {
        std::fprintf(stderr,
            "TIMING,codec,poc,parse_us,pack_us,regbuild_us,submit_us,wait_us,copy_us,total_us\n");
        header_printed = true;
    }
    int64_t total = t.parse_us + t.pack_us + t.regbuild_us +
                    t.submit_us + t.wait_us + t.copy_us;
    std::fprintf(stderr,
        "TIMING,%s,%d,%lld,%lld,%lld,%lld,%lld,%lld,%lld\n",
        codec, poc,
        t.parse_us, t.pack_us, t.regbuild_us,
        t.submit_us, t.wait_us, t.copy_us, total);
    std::fflush(stderr);
}

/* OpenDevice / AllocBuf / FreeBuf + the WindowsBackend vtable live in
 * mft/engine/backend_windows.cpp.  WindowsBackend_Init is the only
 * symbol DecodeEngine_Init needs from there. */

int DecodeEngine_Init(DecodeEngine *e, Codec codec,
                      uint32_t width, uint32_t height)
{
#ifdef _WIN32
    /* Default backend: per-engine inline Windows backend whose ctx
     * points at e->device (so H.265 / AV1 paths still see the handle). */
    WindowsBackend_Init(&e->backend_storage, &e->device);
    return DecodeEngine_InitWithBackend(e, &e->backend_storage,
                                        codec, width, height);
#else
    (void)e; (void)codec; (void)width; (void)height;
    return Fail("DecodeEngine_Init has no default backend on Linux; "
                "call DecodeEngine_InitWithBackend(LinuxBackend_New(...))");
#endif
}

int DecodeEngine_InitWithBackend(DecodeEngine *e,
                                 DecodeEngineBackend *backend,
                                 Codec codec,
                                 uint32_t width, uint32_t height)
{
    e->codec        = codec;
    e->frame_width  = width;
    e->frame_height = height;
    e->backend      = backend;

    if (e->backend->Open(e->backend->ctx,
                         codec == Codec::H265 ? DE_CODEC_H265 : DE_CODEC_H264) != 0)
        return 1;

    /* RCB sizing — codec-agnostic geometry on vdpu34x.  Reuse the H.264
     * sizer for both: the 10 sub-regions, alignment, and per-region
     * register indices match on the HEVC path. */
    uint32_t rcb_total = H264GetRcbBufferSizes(e->rcb_info, width, height);

    /* NV12 with codec-internal MB (16-row) height padding.  The vdpu34x
     * codec writes the full coded raster — height is padded up to a 16-row
     * multiple regardless of the displayed height — so the output buffer
     * must be sized to padded height, not displayed height.  Confirmed by
     * MMIO trace at 1080p: reg[020] = 0x1fe00 = width*1088/16 (padded),
     * not width*1080/16 (displayed).  Under-sized allocation here at
     * 1080p produced the dark-green top bar in 1080p HEVC playback (UV
     * plane was read 8 luma rows before codec's actual UV start). */
    auto align16 = [](uint32_t v) { return (v + 15u) & ~15u; };
    uint32_t height_pad  = align16(height);
    /* Size for 10-bit (NV15 packed: 4 samples in 5 bytes => stride =
     * width*10/8).  Same backing buffer serves 8-bit streams unchanged
     * (just costs 1.25x the bytes — ~24 MiB at 4K).  Per-AU regbuilder
     * picks the actual stride from the SPS; we don't need to reallocate
     * on a profile / bit-depth change mid-stream. */
    uint32_t stride_8  = align16(width);
    uint32_t stride_10 = align16((width * 10u + 7u) / 8u);
    uint32_t frame_bytes_8  = stride_8  * height_pad * 3u / 2u;
    uint32_t frame_bytes_10 = stride_10 * height_pad * 3u / 2u;
    uint32_t frame_bytes = frame_bytes_10;

    /* Colmv buffer geometry — same compressed path as H.264 (vdpu34x is
     * codec-agnostic for colmv on rk3588). */
    auto align_up = [](uint32_t v, uint32_t a) { return (v + a - 1) & ~(a - 1); };
    uint32_t seg_cnt_w   = align_up(width, 64) / 64;
    uint32_t seg_cnt_h   = align_up(height, 16) / 16;
    uint32_t seg_head    = align_up(seg_cnt_w, 16) * seg_cnt_h;
    uint32_t seg_payload = seg_cnt_w * seg_cnt_h * 64u * 16u;
    uint32_t colmv_bytes = align_up(seg_head + seg_payload, 128);
    if (colmv_bytes < 4096) colmv_bytes = 4096;
    uint32_t frame_alloc = frame_bytes;

    /* Codec-specific table sizes. */
    uint32_t cabac_size   = (codec == Codec::H265)
                                ? RKH265_CABAC_INIT_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_CABAC_INIT_SIZE + RKH264_TABLE_TAIL_PAD;
    uint32_t pps_size     = (codec == Codec::H265)
                                ? RKH265_SPSPPS_UNIT_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_SPSPPS_UNIT_SIZE + RKH264_TABLE_TAIL_PAD;
    uint32_t rps_size     = (codec == Codec::H265)
                                ? RKH265_RPS_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_RPS_SIZE + RKH264_TABLE_TAIL_PAD;
    uint32_t scaling_size = (codec == Codec::H265)
                                ? RKH265_SCALING_LIST_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_SCALING_LIST_SIZE + RKH264_TABLE_TAIL_PAD;

    std::printf("alloc sizes: codec=%s frame=%u rcb=%u colmv=%u cabac=%u pps=%u rps=%u scaling=%u\n",
                codec == Codec::H265 ? "H265" : "H264",
                frame_bytes, rcb_total, colmv_bytes,
                cabac_size, pps_size, rps_size, scaling_size);

#define ALLOC(b, sz, usage) do { \
    if (e->backend->AllocBuf(e->backend->ctx, (sz), (usage), &(b))) return 1; \
} while (0)
    ALLOC(e->bitstream,    1u << 20,                       DE_BUF_BITSTREAM_INPUT);
    ALLOC(e->cabac_init,   cabac_size,                     DE_BUF_SCRATCH);
    ALLOC(e->pps_table,    pps_size,                       DE_BUF_SCRATCH);
    ALLOC(e->rps_table,    rps_size,                       DE_BUF_SCRATCH);
    ALLOC(e->scaling_list, scaling_size,                   DE_BUF_SCRATCH);
    ALLOC(e->rcb,          rcb_total > 4096 ? rcb_total : 4096,
                                                           DE_BUF_SCRATCH);
    ALLOC(e->error_ref,    frame_alloc,                    DE_BUF_REFERENCE_FRAME);
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        ALLOC(e->pool_output[i], frame_alloc, DE_BUF_OUTPUT_FRAME);
        ALLOC(e->pool_colmv[i],  colmv_bytes, DE_BUF_SCRATCH);
    }
#undef ALLOC

    /* Pre-fill CABAC table. */
    if (codec == Codec::H265) {
        size_t nbytes = 0;
        const uint8_t *tab = H265GetCabacInitTable(&nbytes);
        std::memcpy(e->cabac_init.user_va, tab, nbytes);
    } else {
        size_t nwords = 0;
        const uint32_t *tab = H264GetCabacInitTable(&nwords);
        std::memcpy(e->cabac_init.user_va, tab, nwords * sizeof(uint32_t));
    }

    /* Init DPB pool. */
    DpbPoolEntry pool[DecodeEngine::kPoolSize];
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        pool[i].output_frame = e->pool_output[i].handle;
        pool[i].colmv        = e->pool_colmv[i].handle;
    }
    if (codec == Codec::H265) {
        if (H265Dpb_Init(&e->dpb_h265, pool, DecodeEngine::kPoolSize) != DPB_OK)
            return Fail("H265Dpb_Init");
        H265ParseResultInit(&e->parsed_h265);
    } else {
        if (Dpb_Init(&e->dpb_h264, pool, DecodeEngine::kPoolSize) != DPB_OK)
            return Fail("Dpb_Init");
    }

    e->scratch.resize(2u << 20);
    return 0;
}

void DecodeEngine_Shutdown(DecodeEngine *e)
{
    if (!e->backend) return;
    auto *be = e->backend;
    be->FreeBuf(be->ctx, &e->bitstream);
    be->FreeBuf(be->ctx, &e->cabac_init);
    be->FreeBuf(be->ctx, &e->pps_table);
    be->FreeBuf(be->ctx, &e->rps_table);
    be->FreeBuf(be->ctx, &e->scaling_list);
    be->FreeBuf(be->ctx, &e->rcb);
    be->FreeBuf(be->ctx, &e->error_ref);
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        be->FreeBuf(be->ctx, &e->pool_output[i]);
        be->FreeBuf(be->ctx, &e->pool_colmv[i]);
    }
    be->Close(be->ctx);
    e->backend = nullptr;
}

/* Annex-B slice NAL locators live in mft/au_iter.{h,cpp} — H264FindSliceNal
 * for type 1/5 slice; H265FindSliceNal for the first VCL NAL (type < 32).
 * Both return the pre-startcode offset so the staged slice prefix matches
 * what BSP hardware reads. */

/* Decode the H.264 path. */
static int DecodeOne_H264(DecodeEngine *e,
                          const uint8_t *au, size_t au_len,
                          std::vector<uint8_t> *out_yuv)
{
    StageTimes timing{};
    int64_t t0 = TimingEnabled() ? QpcNow() : 0;

    /* 1. Locate the slice NAL. */
    size_t slice_off = 0, slice_size = 0;
    if (H264FindSliceNal(au, au_len, &slice_off, &slice_size) != 0)
        return Fail("no H.264 slice NAL found");
    size_t sc_len   = (au[slice_off + 2] == 1) ? 3u : 4u;
    size_t skip     = sc_len - 3u;
    size_t copy_off = slice_off + skip;
    size_t copy_len = slice_size - skip;

    if (copy_len > e->bitstream.size)
        return Fail("slice larger than bitstream buf");
    if (DecodeDebugEnabled())
        std::printf("slice NAL at offset 0x%zx (sc=%zu), staged %zu bytes\n",
                    slice_off, sc_len, copy_len);
    std::memcpy(e->bitstream.user_va, au + copy_off, copy_len);

    H264ParseResult &parsed = e->parsed_h264;
    H264ParseStatus s = H264ParseAccessUnit(au, au_len,
                                            e->scratch.data(), e->scratch.size(),
                                            &parsed);
    if (s != H264_PARSE_OK) {
        std::fprintf(stderr, "parser status=%d (need_more=1 invalid=2 unsupported=3)\n",
                     (int)s);
        return Fail("parser failed");
    }

    uint32_t w_px = ((uint32_t)parsed.sps.pic_width_in_mbs_minus1 + 1) * 16;
    uint32_t h_px = ((uint32_t)parsed.sps.pic_height_in_map_units_minus1 + 1) * 16;
    /* The SPS-derived dims are the coded raster, 16-aligned by H.264
     * construction.  The harness was initialised with the *display* size
     * (typically from container metadata), which can be smaller when the
     * stream uses frame_cropping (e.g. 1080p coded as 1920x1088 with the
     * bottom 8 rows cropped).  Engine geometry already sizes the output
     * frame buffer for the coded raster via align16(frame_height), so
     * accept any display height that's within one MB row of the coded
     * height. */
    auto align_up16 = [](uint32_t v) { return (v + 15u) & ~15u; };
    if (w_px != align_up16(e->frame_width) || h_px != align_up16(e->frame_height)) {
        std::fprintf(stderr, "stream %ux%u, harness inited for %ux%u\n",
                     w_px, h_px, e->frame_width, e->frame_height);
        return Fail("dim mismatch");
    }

    /* MMCO trace — print first ~80 AUs that carry MMCO ops or LT-flagged
     * IDR.  Gated by RKMPP_DECODE_DEBUG=1 OR the first hits regardless,
     * so we can see what the stream is doing without re-running. */
    {
        static int mmco_au = 0;
        static int mmco_hits = 0;
        mmco_au++;
        bool fire = parsed.adaptive_ref_pic_marking_mode_flag ||
                    parsed.idr_long_term_reference_flag ||
                    parsed.ref_pic_list_modification_flag_l0 ||
                    parsed.ref_pic_list_modification_flag_l1;
        if (fire && mmco_hits < 80) {
            std::printf("[MMCO au=%d fn=%u idr=%d nri=%u adaptive=%u idr_lt=%u n_ops=%u",
                        mmco_au,
                        parsed.decode.frame_num,
                        (parsed.decode.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) ? 1 : 0,
                        parsed.decode.nal_ref_idc,
                        parsed.adaptive_ref_pic_marking_mode_flag,
                        parsed.idr_long_term_reference_flag,
                        parsed.n_mmco);
            for (uint32_t i = 0; i < parsed.n_mmco; i++) {
                const H264Mmco &m = parsed.mmco[i];
                std::printf(" op%u(d=%u ltp=%u lti=%u maxlt+1=%u)",
                            m.op,
                            m.difference_of_pic_nums_minus1,
                            m.long_term_pic_num,
                            m.long_term_frame_idx,
                            m.max_long_term_frame_idx_plus1);
            }
            if (parsed.ref_pic_list_modification_flag_l0) {
                std::printf(" RPLM_L0[");
                for (uint32_t i = 0; i < parsed.n_rplm_l0; i++)
                    std::printf("%c%u=%u",
                                i ? ',' : ' ',
                                parsed.rplm_l0[i].op,
                                parsed.rplm_l0[i].value);
                std::printf("]");
            }
            if (parsed.ref_pic_list_modification_flag_l1) {
                std::printf(" RPLM_L1[");
                for (uint32_t i = 0; i < parsed.n_rplm_l1; i++)
                    std::printf("%c%u=%u",
                                i ? ',' : ' ',
                                parsed.rplm_l1[i].op,
                                parsed.rplm_l1[i].value);
                std::printf("]");
            }
            std::printf("]\n");
            std::fflush(stdout);
            mmco_hits++;
        }
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.parse_us = QpcUs(t0, t); t0 = t; }

    DpbSelection sel{};
    if (Dpb_Select(&e->dpb_h264, &parsed, &sel) != DPB_OK)
        return Fail("Dpb_Select failed");

    /* The regbuilder reads `parsed.decode.dpb[]` for reg99..102 (per-slot
     * ref-info nibble) and reg67..98 (per-ref top/bottom POC pairs).  Our
     * parser leaves those zeroed (they're a V4L2-control field on Linux,
     * derived by the user-mode DPB on Windows).  Without this copy every
     * non-IDR frame ships reg99..102 = 0 — codec sees "no valid refs",
     * skips inter prediction, and bands of zeros get written into the
     * output frame.  P-only streams limped along because the codec's
     * implicit fallback uses REF_BASE[0]; B-frames fail outright. */
    memcpy(parsed.decode.dpb, sel.dpb_entries, sizeof(sel.dpb_entries));

    H264PackSpsPpsUnit(static_cast<uint8_t *>(e->pps_table.user_va),
                       &parsed.sps, &parsed.pps,
                       sel.dpb_entries, /*field_pic=*/0);
    H264PackFrameRps(static_cast<uint8_t *>(e->rps_table.user_va),
                     parsed.decode.frame_num,
                     parsed.sps.log2_max_frame_num_minus4,
                     sel.dpb_entries, sel.ref_lists);
    H264PackScalingList(static_cast<uint8_t *>(e->scaling_list.user_va),
                        &parsed.scaling_matrix,
                        parsed.has_scaling_matrix);

    if (DecodeDebugEnabled()) {
        FILE *f;
        if ((f = fopen("win_pps.bin", "wb"))) {
            fwrite(e->pps_table.user_va, 1, RKH264_SPSPPS_UNIT_SIZE, f);
            fclose(f);
        }
        if ((f = fopen("win_rps.bin", "wb"))) {
            fwrite(e->rps_table.user_va, 1, RKH264_RPS_SIZE, f);
            fclose(f);
        }
        if ((f = fopen("win_cabac.bin", "wb"))) {
            fwrite(e->cabac_init.user_va, 1, RKH264_CABAC_INIT_SIZE, f);
            fclose(f);
        }
        if ((f = fopen("win_bitstream.bin", "wb"))) {
            fwrite(e->bitstream.user_va, 1, copy_len, f);
            fclose(f);
        }
        std::printf("dumped win_{pps,rps,cabac,bitstream}.bin\n");
    }

    /* Per-AU table dump for cross-checking against BSP shim DMA captures.
     * Gated on `RKMPP_DUMP_TABLES=1` so normal debug runs aren't polluted.
     * Writes win_{pps,rps}_au{NNN}.bin for each AU; cmp against the same
     * AU's bytes from the rk-Linux bsp_capture mpp_dump_h264 dec dump
     * files captured via capture_h264_dma.sh.  See memory:
     * h264_bframe_colmv_investigation.md. */
    {
        static int dump_au_idx = -1;
        dump_au_idx++;
        const char *flag = ::getenv("RKMPP_DUMP_TABLES");
        if (flag && flag[0] == '1') {
            char path[64];
            FILE *f;
            std::snprintf(path, sizeof(path), "win_rps_au%03d.bin", dump_au_idx);
            if ((f = fopen(path, "wb"))) {
                fwrite(e->rps_table.user_va, 1, RKH264_RPS_SIZE, f);
                fclose(f);
            }
            std::snprintf(path, sizeof(path), "win_pps_au%03d.bin", dump_au_idx);
            if ((f = fopen(path, "wb"))) {
                fwrite(e->pps_table.user_va, 1, RKH264_SPSPPS_UNIT_SIZE, f);
                fclose(f);
            }
        }
    }

    /* Zero this frame's colmv slot before the kick.  The codec writes
     * inter-MB colmv during decode but skips intra-MB locations — those
     * retain whatever was in the buffer from a prior reuse of this slot.
     * When this frame later becomes an L1 ref for a B-frame's temporal-
     * direct prediction, intra-MB colmv reads return stale MVs from the
     * prior decode → divergent / non-deterministic B output.
     *
     * Linux BSP avoids this because dma-heap zeros pages on alloc AND
     * its colmv slots are sized 1:1 with the DPB so each frame gets a
     * fresh buffer; we reuse a fixed pool across frames within a session.
     * Per-kick zero of COLMV_CUR replicates the BSP guarantee.
     *
     * Windows maps user buffers cached, so rkvdec/job.c treats
     * COLMV_CUR as both a pre-kick clean target (for this zero) and a
     * post-kick invalidate target (for diagnostic dumps and later CPU
     * reads). ~150 KB per frame on 1280×720, negligible. */
    if (e->pool_colmv[sel.current_slot].user_va) {
        std::memset(e->pool_colmv[sel.current_slot].user_va, 0,
                    e->pool_colmv[sel.current_slot].size);
    }

    /* Output pre-fill experiment — see OutputFillByte() docstring.  Only
     * fires when RKMPP_OUTPUT_FILL is set; default path leaves the
     * kernel-zeroed buffer alone so this is opt-in for diagnostics. */
    {
        int fill = OutputFillByte();
        if (fill >= 0 && e->pool_output[sel.current_slot].user_va) {
            std::memset(e->pool_output[sel.current_slot].user_va,
                        fill,
                        e->pool_output[sel.current_slot].size);
        }
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.pack_us = QpcUs(t0, t); t0 = t; }

    H264BufferRefs refs{};
    refs.bitstream        = e->bitstream.handle;
    refs.bitstream_offset = 0;
    refs.bitstream_size   = (uint32_t)copy_len;
    refs.output_frame     = sel.current_output;
    refs.colmv_cur        = sel.current_colmv;
    /* error_ref: separate scratch buffer.  Earlier experiment to point this
     * at sel.refs[0] (the most-recent past ref, matching BSP) had zero
     * effect on the B-frame divergence and may corrupt IDR output (no past
     * ref → fallback_pool aliases current output, so error_ref would alias
     * the current decode buffer). */
    refs.error_ref        = e->error_ref.handle;
    refs.pps_table        = e->pps_table.handle;
    refs.rps_table        = e->rps_table.handle;
    refs.cabac_init_table = e->cabac_init.handle;
    refs.scaling_list     = e->scaling_list.handle;
    for (int i = 0; i < RKH264_RCB_COUNT; i++) {
        refs.rcb[i]        = e->rcb.handle;
        refs.rcb_offset[i] = e->rcb_info[i].offset;
    }
    for (int i = 0; i < 16; i++) {
        refs.refs[i]      = sel.refs[i];
        refs.ref_colmv[i] = sel.ref_colmv[i];
    }

    H26xDenseOutput dense{};
    H264RegBuildStatus rs = H264BuildDenseRegs(&parsed, &refs,
                                               sel.current_slot, &dense);
    if (rs != H264_REGBUILD_OK) {
        std::fprintf(stderr, "regbuilder status=%d\n", (int)rs);
        Dpb_OnDecodeFailed(&e->dpb_h264);
        return Fail("regbuilder failed");
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.regbuild_us = QpcUs(t0, t); t0 = t; }

    /* Backend handles SUBMIT + (debug PEEK) + WAIT synchronously.
     * Returns the codec HardwareStatus reg in *hw_status; non-zero rc
     * either means an IOCTL failed outright (negative / hard error) or
     * that wout.Status was non-zero while wait succeeded (kept as the
     * positive Status value).  The original logic only failed when BOTH
     * Status != 0 AND the codec didn't set RDY in hw_status. */
    uint32_t hw_status = 0;
    int submit_rc = e->backend->SubmitDense(e->backend->ctx, &dense,
                                            1000, &hw_status);
    if (TimingEnabled()) {
        int64_t t = QpcNow();
        timing.submit_us = 0;     /* backend collapses submit+wait */
        timing.wait_us   = QpcUs(t0, t);
        t0 = t;
    }
    if (DecodeDebugEnabled())
        std::printf("decode: status=%d hwstatus=0x%08x iova_slots=%u\n",
                    submit_rc, hw_status, dense.IovaSlotCount);
    const uint32_t kRdySta = 1u << 2;
    bool have_output = (hw_status & kRdySta) != 0;
    if (submit_rc != 0 && !have_output) {
        std::fprintf(stderr, "decode reported non-success status (no output)\n");
        Dpb_OnDecodeFailed(&e->dpb_h264);
        return 5;
    }
    if (submit_rc != 0 && DecodeDebugEnabled()) {
        std::fprintf(stderr, "decode partial: hwstatus=0x%08x has DEC_RDY but other flags too — dumping anyway\n",
                     hw_status);
    }

    if (DecodeDebugEnabled()) {
        uint32_t bytes = e->frame_width * e->frame_height * 3u / 2u;
        auto count_nonzero = [](const void *p, size_t n) -> size_t {
            const uint8_t *b = (const uint8_t*)p; size_t k = 0;
            for (size_t i = 0; i < n; i++) if (b[i] != 0) k++;
            return k;
        };
        size_t nz_dec = count_nonzero(e->pool_output[sel.current_slot].user_va, bytes);
        size_t nz_err = count_nonzero(e->error_ref.user_va, bytes);
        size_t nz_cmv = count_nonzero(e->pool_colmv[sel.current_slot].user_va,
                                       e->pool_colmv[sel.current_slot].size);
        std::printf("nonzero bytes: decout=%zu error_ref=%zu colmv_cur=%zu\n",
                    nz_dec, nz_err, nz_cmv);
    }

    /* Per-AU colmv dump for cross-checking against BSP shim DMA captures.
     * Gated on `RKMPP_DUMP_COLMV=1` and `RKMPP_DUMP_COLMV_MAX_AU` (default
     * 30) — each dump is ~480 KB, cap prevents tens-of-GB on long streams.
     * Pair with BSP capture in
     * Z:\drivers-arm\bsp_capture\h264_dancing_capture\mpp_dump_h264\
     * dec{N+1}_fdM_len491520.bin (BSP captures pre-next-kick state, so AU
     * N's colmv output appears in dec{N+1}_fdM where M is BSP's fd for
     * slot sel.current_slot — see colmv_slot_map.txt). */
    {
        static int colmv_dump_idx = -1;
        colmv_dump_idx++;
        const char *flag = ::getenv("RKMPP_DUMP_COLMV");
        if (flag && flag[0] == '1') {
            int max_au = 30;
            const char *maxs = ::getenv("RKMPP_DUMP_COLMV_MAX_AU");
            if (maxs) {
                int parsed_max = atoi(maxs);
                if (parsed_max > 0) max_au = parsed_max;
            }
            if (colmv_dump_idx < max_au) {
                char path[96];
                std::snprintf(path, sizeof(path),
                              "our_colmv_au%03d_slot%u.bin",
                              colmv_dump_idx, sel.current_slot);
                FILE *f = fopen(path, "wb");
                if (f) {
                    fwrite(e->pool_colmv[sel.current_slot].user_va, 1,
                           e->pool_colmv[sel.current_slot].size, f);
                    fclose(f);
                }
            }
        }
    }

    e->last_decoded_slot = (int)sel.current_slot;

    /* Skip the memcpy when (a) global zero-copy mode (populate_yuv=false),
     * or (b) this is a non-ref frame and the consumer asked for non-ref
     * skip (populate_yuv_nonrefs=false).  Safe now that the kernel
     * driver does explicit codec leaf-clock gate/ungate between every
     * kick — the 12 ms uncached read previously masquerading as an
     * inter-kick settle is no longer load-bearing. */
    bool is_ref_h264 = e->dpb_h264.slots[sel.current_slot].is_ref ? true : false;
    if (out_yuv && e->populate_yuv &&
        (is_ref_h264 || e->populate_yuv_nonrefs)) {
        const uint32_t bit_depth = (parsed.sps.bit_depth_luma_minus8 == 2) ? 10u : 8u;
        const uint32_t height_pad = (e->frame_height + 15u) & ~15u;
        const uint32_t coded_w    = (e->frame_width  + 15u) & ~15u;
        const uint32_t src_stride = CodecOutputStride(coded_w, bit_depth);
        const uint8_t *src = (const uint8_t *)e->pool_output[sel.current_slot].user_va;
        RepackCodecOutputToNV12orP010(src, src_stride, height_pad,
                                      e->frame_width, e->frame_height,
                                      bit_depth, out_yuv);
        /* Localize "garbage YUV" issues: hash the first 4 KiB of the
         * post-memcpy buffer.  Distinct hashes here but recurring hashes
         * in the final file = reorder window bug; recurring hashes here
         * = cache/coherency issue or codec not actually writing. */
        if (DecodeDebugEnabled()) {
            static int au_idx = 0;
            const uint8_t *p = out_yuv->data();
            uint32_t h = 0x811c9dc5;
            size_t nhash = out_yuv->size() < 4096u ? out_yuv->size() : 4096u;
            for (size_t i = 0; i < nhash; i++) {
                h ^= p[i]; h *= 16777619;
            }
            std::printf("post-memcpy au=%d slot=%u poc=%d hash=%08x\n",
                        au_idx++, sel.current_slot,
                        parsed.decode.top_field_order_cnt, h);
            std::fflush(stdout);
        }
    }

    if (TimingEnabled()) {
        timing.copy_us = QpcUs(t0, QpcNow());
        EmitTimingCsv(timing, "h264", parsed.decode.top_field_order_cnt);
    }

    Dpb_OnDecodeComplete(&e->dpb_h264);
    return 0;
}

#ifdef _WIN32
/* Decode the HEVC path.
 *
 * NOTE: not yet routed through DecodeEngineBackend (decode-engine-
 * backend-split.md scope).  Calls DeviceIoControl(e->device, ...)
 * directly + uses MSVC-only fopen_s / Windows env-var APIs in its
 * debug paths.  Wrapped #ifdef _WIN32 so the engine compiles on Linux,
 * where the H.264-only LinuxBackend is the only one we exercise. */
static int DecodeOne_H265(DecodeEngine *e,
                          const uint8_t *au, size_t au_len,
                          std::vector<uint8_t> *out_yuv)
{
    StageTimes timing{};
    int64_t t0 = TimingEnabled() ? QpcNow() : 0;

    /* 1. Parse the AU first — VPS/SPS/PPS state needs to land before we
     * dimension anything off it.  H265ParseAccessUnit also resolves
     * slice_data + slice_data_size for the *RBSP* slice; we still want
     * the original Annex-B-framed slice NAL bytes in the bitstream
     * buffer (rkvdec2 reads RLC-stream with the NAL header intact, like
     * the H.264 path) so we run a separate slice-NAL locator on the AU. */
    H265ParseResult &parsed = e->parsed_h265;
    H265ParseStatus s = H265ParseAccessUnit(au, au_len,
                                            e->scratch.data(), e->scratch.size(),
                                            &parsed);
    if (s != H265_PARSE_OK) {
        std::fprintf(stderr, "h265 parser status=%d\n", (int)s);
        return Fail("parser failed");
    }
    if (!parsed.has_slice || parsed.active_sps_id < 0 ||
        parsed.active_pps_id < 0) {
        return Fail("no slice / active SPS+PPS after parse");
    }
    const H265Sps *sps = &parsed.sps[parsed.active_sps_id];
    const H265Pps *pps = &parsed.pps[parsed.active_pps_id];
    const H265Vps *vps = (parsed.active_vps_id >= 0)
                            ? &parsed.vps[parsed.active_vps_id]
                            : nullptr;

    uint32_t w_px = sps->pic_width_in_luma_samples;
    uint32_t h_px = sps->pic_height_in_luma_samples;
    if (w_px != e->frame_width || h_px != e->frame_height) {
        std::fprintf(stderr, "stream %ux%u, harness inited for %ux%u\n",
                     w_px, h_px, e->frame_width, e->frame_height);
        return Fail("dim mismatch");
    }

    /* 2. Locate first VCL NAL — strip leading VPS/SPS/PPS/SEI so STR_LEN
     * (reg016) is *just* the slice byte count, mirroring what the BSP
     * writes.  Earlier H.264 mistake (passing full AU length) sent the
     * codec into runaway parsing past slice end. */
    size_t slice_off = 0, slice_size = 0;
    if (H265FindSliceNal(au, au_len, &slice_off, &slice_size) != 0)
        return Fail("no HEVC slice NAL found");
    size_t sc_len   = (au[slice_off + 2] == 1) ? 3u : 4u;
    size_t skip     = sc_len - 3u;
    size_t copy_off = slice_off + skip;
    size_t copy_len = slice_size - skip;
    if (copy_len > e->bitstream.size)
        return Fail("slice larger than bitstream buf");
    if (DecodeDebugEnabled())
        std::printf("h265 slice NAL at offset 0x%zx (sc=%zu), staged %zu bytes\n",
                    slice_off, sc_len, copy_len);
    std::memcpy(e->bitstream.user_va, au + copy_off, copy_len);

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.parse_us = QpcUs(t0, t); t0 = t; }

    /* 3. DPB selection. */
    H265DpbSelection sel{};
    if (H265Dpb_Select(&e->dpb_h265, &parsed, &sel) != DPB_OK)
        return Fail("H265Dpb_Select failed");

    /* 4. Pack tables.  Skip when the active SPS+PPS match the previous
     * AU and no VPS/SPS/PPS was redefined this AU — the persistent
     * pps_table/rps_table/scaling_list buffers still hold the right
     * bytes (uncached MmNonCached, no flush concern).  Hits on every
     * AU after the first for typical single-SPS+PPS streams. */
    const bool params_changed =
        parsed.got_vps_in_au || parsed.got_sps_in_au ||
        parsed.got_pps_in_au ||
        parsed.active_sps_id != e->last_h265_sps_id ||
        parsed.active_pps_id != e->last_h265_pps_id;
    if (params_changed) {
        if (H265PackPPS(vps, sps, pps,
                        static_cast<uint8_t *>(e->pps_table.user_va),
                        e->pps_table.size) < 0) {
            H265Dpb_OnDecodeFailed(&e->dpb_h265);
            return Fail("H265PackPPS");
        }
        if (H265PackRPS(&parsed,
                        static_cast<uint8_t *>(e->rps_table.user_va),
                        e->rps_table.size) < 0) {
            H265Dpb_OnDecodeFailed(&e->dpb_h265);
            return Fail("H265PackRPS");
        }
        if (H265PackScalingList(sps, pps,
                                static_cast<uint8_t *>(e->scaling_list.user_va),
                                e->scaling_list.size) < 0) {
            H265Dpb_OnDecodeFailed(&e->dpb_h265);
            return Fail("H265PackScalingList");
        }
        e->last_h265_sps_id = parsed.active_sps_id;
        e->last_h265_pps_id = parsed.active_pps_id;
    }

    if (DecodeDebugEnabled()) {
        FILE *f;
        if (fopen_s(&f, "win_h265_pps.bin", "wb") == 0) {
            fwrite(e->pps_table.user_va, 1, RKH265_SPSPPS_UNIT_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_rps.bin", "wb") == 0) {
            fwrite(e->rps_table.user_va, 1, RKH265_RPS_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_scaling.bin", "wb") == 0) {
            fwrite(e->scaling_list.user_va, 1, RKH265_SCALING_LIST_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_cabac.bin", "wb") == 0) {
            fwrite(e->cabac_init.user_va, 1, RKH265_CABAC_INIT_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_bitstream.bin", "wb") == 0) {
            fwrite(e->bitstream.user_va, 1, copy_len, f);
            fclose(f);
        }
        std::printf("dumped win_h265_{pps,rps,scaling,cabac,bitstream}.bin\n");
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.pack_us = QpcUs(t0, t); t0 = t; }

    /* 5. Compose buffer-refs for the regbuilder. */
    H265BufferRefs refs{};
    refs.bitstream        = e->bitstream.handle;
    refs.bitstream_offset = 0;
    refs.bitstream_size   = (uint32_t)copy_len;
    refs.output_frame     = sel.current_output_iova;
    refs.colmv_cur        = sel.current_colmv_iova;
    refs.error_ref        = e->error_ref.handle;
    refs.pps_table        = e->pps_table.handle;
    refs.rps_table        = e->rps_table.handle;
    refs.scanlist         = e->scaling_list.handle;
    refs.cabac_init_table = e->cabac_init.handle;
    for (int i = 0; i < 10; i++) {
        refs.rcb[i]        = e->rcb.handle;
        refs.rcb_offset[i] = e->rcb_info[i].offset;
    }
    for (int i = 0; i < 16; i++) {
        refs.refs[i]      = sel.refs[i];
        refs.ref_colmv[i] = sel.ref_colmv[i];
        refs.ref_poc[i]   = sel.ref_pocs[i];
        refs.ref_poc_high[i]     = (uint8_t)((sel.ref_pocs[i] >> 28) & 0xF);
        refs.ref_is_long_term[i] = 0;
    }

    /* 6. Build dense bank. */
    H26xDenseOutput dense{};
    H265RegBuildStatus rs = H265BuildDenseRegs(&parsed, &refs,
                                               sel.current_slot, &dense);
    if (rs != H265_REGBUILD_OK) {
        std::fprintf(stderr, "h265 regbuilder status=%d\n", (int)rs);
        H265Dpb_OnDecodeFailed(&e->dpb_h265);
        return Fail("h265 regbuilder failed");
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.regbuild_us = QpcUs(t0, t); t0 = t; }

    /* 7. Submit via the dense-bank IOCTL. */
    RKMPP_SUBMIT_DENSE_JOB_IN  sin{};
    RKMPP_SUBMIT_DENSE_JOB_OUT sout{};
    sin.StructSize    = sizeof(sin);
    sin.IovaSlotCount = dense.IovaSlotCount;
    sin.BufRefCount   = 0;
    sin.TimeoutMs     = 1000;
    sin.KickValue     = dense.KickValue;
    sin.Bank          = dense.Bank;
    if (dense.IovaSlotCount > 0) {
        std::memcpy(sin.IovaSlots, dense.IovaSlots,
                    dense.IovaSlotCount * sizeof(RKMPP_DENSE_IOVA_SLOT));
    }

    DWORD got = 0;
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_SUBMIT_DENSE_JOB, &sin, sizeof(sin),
                         &sout, sizeof(sout), &got, nullptr)) {
        H265Dpb_OnDecodeFailed(&e->dpb_h265);
        return Fail("SUBMIT_DENSE_JOB", GetLastError());
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.submit_us = QpcUs(t0, t); t0 = t; }

    if (DecodeDebugEnabled()) {
        RKMPP_PEEK_JOB_IN        pin{ sout.JobId };
        RKMPP_PEEK_DENSE_JOB_OUT pout{};
        if (DeviceIoControl(e->device, IOCTL_RKMPP_PEEK_DENSE_JOB, &pin, sizeof(pin),
                            &pout, sizeof(pout), &got, nullptr)) {
            std::printf("--- post-subst dense bank (kick=0x%08x) ---\n",
                        pout.KickValue);
            auto dump = [](const char *label, uint32_t first,
                            const uint32_t *src, uint32_t n) {
                for (uint32_t i = 0; i < n; i++) {
                    if (src[i] == 0) continue;
                    std::printf("  [%s idx %3u] = 0x%08x\n",
                                label, first + i, src[i]);
                }
            };
            dump("com ", RKMPP_DENSE_COMMON_FIRST,  pout.Bank.Common,
                 RKMPP_DENSE_COMMON_WORDS);
            dump("cpar", RKMPP_DENSE_CPARAM_FIRST,  pout.Bank.CodecParams,
                 RKMPP_DENSE_CPARAM_WORDS);
            dump("cadr", RKMPP_DENSE_CADDR_FIRST,   pout.Bank.CommonAddr,
                 RKMPP_DENSE_CADDR_WORDS);
            dump("codr", RKMPP_DENSE_CODADDR_FIRST, pout.Bank.CodecAddr,
                 RKMPP_DENSE_CODADDR_WORDS);
            dump("hpoc", RKMPP_DENSE_HIPOC_FIRST,   pout.Bank.HighPoc,
                 RKMPP_DENSE_HIPOC_WORDS);
            dump("stat", RKMPP_DENSE_STAT_FIRST,    pout.Bank.Stat,
                 RKMPP_DENSE_STAT_WORDS);
        }
    }

    /* 8. Wait. */
    RKMPP_WAIT_JOB_IN  win{ sout.JobId, 1000, 0 };
    RKMPP_WAIT_JOB_OUT wout{};
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                         &wout, sizeof(wout), &got, nullptr)) {
        H265Dpb_OnDecodeFailed(&e->dpb_h265);
        return Fail("WAIT_JOB", GetLastError());
    }

    if (TimingEnabled()) { int64_t t = QpcNow(); timing.wait_us = QpcUs(t0, t); t0 = t; }
    if (DecodeDebugEnabled())
        std::printf("h265 decode: jobid=%llu status=0x%08x hwstatus=0x%08x iova_slots=%u\n",
                    (unsigned long long)sout.JobId, wout.Status,
                    wout.HardwareStatus, dense.IovaSlotCount);
    const uint32_t kRdySta = 1u << 2;
    bool have_output = (wout.HardwareStatus & kRdySta) != 0;
    if (wout.Status != 0 && !have_output) {
        std::fprintf(stderr, "h265 decode reported non-success status (no output)\n");
        H265Dpb_OnDecodeFailed(&e->dpb_h265);
        return 5;
    }
    if (wout.Status != 0 && DecodeDebugEnabled()) {
        std::fprintf(stderr, "h265 decode partial: hwstatus=0x%08x — dumping anyway\n",
                     wout.HardwareStatus);
    }

    if (DecodeDebugEnabled()) {
        uint32_t bytes = e->frame_width * e->frame_height * 3u / 2u;
        auto count_nonzero = [](const void *p, size_t n) -> size_t {
            const uint8_t *b = (const uint8_t*)p; size_t k = 0;
            for (size_t i = 0; i < n; i++) if (b[i] != 0) k++;
            return k;
        };
        size_t nz_dec = count_nonzero(e->pool_output[sel.current_slot].user_va, bytes);
        size_t nz_err = count_nonzero(e->error_ref.user_va, bytes);
        size_t nz_cmv = count_nonzero(e->pool_colmv[sel.current_slot].user_va,
                                       e->pool_colmv[sel.current_slot].size);
        std::printf("nonzero bytes: decout=%zu error_ref=%zu colmv_cur=%zu\n",
                    nz_dec, nz_err, nz_cmv);
    }

    /* Per-AU colmv dump for cross-checking against BSP shim DMA captures.
     * Gated on `RKMPP_DUMP_COLMV=1` and `RKMPP_DUMP_COLMV_MAX_AU` (default
     * 30) — each dump is ~480 KB, cap prevents tens-of-GB on long streams.
     * Pair with BSP capture in
     * Z:\drivers-arm\bsp_capture\h264_dancing_capture\mpp_dump_h264\
     * dec{N+1}_fdM_len491520.bin (BSP captures pre-next-kick state, so AU
     * N's colmv output appears in dec{N+1}_fdM where M is BSP's fd for
     * slot sel.current_slot — see colmv_slot_map.txt). */
    {
        static int colmv_dump_idx = -1;
        colmv_dump_idx++;
        const char *flag = ::getenv("RKMPP_DUMP_COLMV");
        if (flag && flag[0] == '1') {
            int max_au = 30;
            const char *maxs = ::getenv("RKMPP_DUMP_COLMV_MAX_AU");
            if (maxs) {
                int parsed_max = atoi(maxs);
                if (parsed_max > 0) max_au = parsed_max;
            }
            if (colmv_dump_idx < max_au) {
                char path[96];
                std::snprintf(path, sizeof(path),
                              "our_colmv_au%03d_slot%u.bin",
                              colmv_dump_idx, sel.current_slot);
                FILE *f = fopen(path, "wb");
                if (f) {
                    fwrite(e->pool_colmv[sel.current_slot].user_va, 1,
                           e->pool_colmv[sel.current_slot].size, f);
                    fclose(f);
                }
            }
        }
    }

    e->last_decoded_slot = (int)sel.current_slot;

    /* See H.264 path: skip the memcpy on non-ref when the consumer
     * asks for the skip path.  Now safe with kernel-side per-kick
     * leaf-clock gate/ungate. */
    bool is_ref_h265 = e->dpb_h265.slots[sel.current_slot].is_ref ? true : false;
    if (out_yuv && e->populate_yuv &&
        (is_ref_h265 || e->populate_yuv_nonrefs)) {
        /* Bit-depth-aware repack: HEVC Main10 streams have the codec
         * emit NV15-packed luma+chroma at width*10/8 stride.  Main
         * (8-bit) keeps NV12 plain.  See repack_yuv.h for the
         * NV15 → P010 unpack details. */
        const uint32_t bit_depth  = (sps->bit_depth_luma_minus8 == 2) ? 10u : 8u;
        const uint32_t height_pad = (e->frame_height + 15u) & ~15u;
        const uint32_t coded_w    = (e->frame_width  + 15u) & ~15u;
        const uint32_t src_stride = CodecOutputStride(coded_w, bit_depth);
        const uint8_t *src = (const uint8_t *)e->pool_output[sel.current_slot].user_va;
        RepackCodecOutputToNV12orP010(src, src_stride, height_pad,
                                      e->frame_width, e->frame_height,
                                      bit_depth, out_yuv);
    }

    if (TimingEnabled()) {
        timing.copy_us = QpcUs(t0, QpcNow());
        EmitTimingCsv(timing, "h265", parsed.poc);
    }

    H265Dpb_OnDecodeComplete(&e->dpb_h265);
    return 0;
}
#endif  /* _WIN32 (DecodeOne_H265) */

int DecodeEngine_DecodeOne(DecodeEngine *e,
                           const uint8_t *au, size_t au_len,
                           std::vector<uint8_t> *out_yuv)
{
    if (au_len > e->bitstream.size) return Fail("AU larger than bitstream buf");
#ifdef _WIN32
    if (e->codec == Codec::H265)
        return DecodeOne_H265(e, au, au_len, out_yuv);
#else
    if (e->codec == Codec::H265)
        return Fail("H.265 path not built on Linux (decode-engine-backend-split.md)");
#endif
    return DecodeOne_H264(e, au, au_len, out_yuv);
}

int DecodeEngine_DecodeOneFramed(DecodeEngine *e,
                                 NalFraming framing,
                                 AvccLenSize len_size,
                                 const uint8_t *au, size_t au_len,
                                 std::vector<uint8_t> *out_yuv)
{
    if (framing == FRAMING_ANNEXB)
        return DecodeEngine_DecodeOne(e, au, au_len, out_yuv);

    /* AVCC4: convert into a scratch buffer.  Annex-B 4-byte start code
     * matches AVCC4 length-field width, so output size == input size. */
    std::vector<uint8_t> tmp(au_len + 64);
    int n = AvccToAnnexB(au, au_len, len_size, tmp.data(), tmp.size());
    if (n < 0) return Fail("AvccToAnnexB: malformed input");
    return DecodeEngine_DecodeOne(e, tmp.data(), (size_t)n, out_yuv);
}

int DecodeEngine_FeedExtradata(DecodeEngine *e,
                               NalFraming framing,
                               AvccLenSize len_size,
                               const uint8_t *au, size_t au_len)
{
    if (au_len == 0) return 0;

    const uint8_t *p = au;
    size_t         n = au_len;
    std::vector<uint8_t> tmp;
    if (framing == FRAMING_AVCC4) {
        tmp.resize(au_len + 64);
        int got = AvccToAnnexB(au, au_len, len_size, tmp.data(), tmp.size());
        if (got < 0) return Fail("AvccToAnnexB(extradata): malformed");
        p = tmp.data();
        n = (size_t)got;
    }

    if (e->scratch.size() < n * 2) e->scratch.resize(n * 2 + 64);
    if (e->codec == Codec::H265) {
        H265ParseStatus s = H265ParseAccessUnit(p, n,
                                                e->scratch.data(),
                                                e->scratch.size(),
                                                &e->parsed_h265);
        /* NEED_MORE = no slice found, but VPS/SPS/PPS landed → fine. */
        if (s != H265_PARSE_OK && s != H265_PARSE_NEED_MORE) {
            std::fprintf(stderr,
                         "FeedExtradata: H265 parser status=%d\n", (int)s);
            return 1;
        }
    } else {
        H264ParseStatus s = H264ParseAccessUnit(p, n,
                                                e->scratch.data(),
                                                e->scratch.size(),
                                                &e->parsed_h264);
        if (s != H264_PARSE_OK && s != H264_PARSE_NEED_MORE) {
            std::fprintf(stderr,
                         "FeedExtradata: H264 parser status=%d\n", (int)s);
            return 1;
        }
    }
    return 0;
}

/* ---- Submit / Poll / Drain (display-order reorder) ---------------- *
 *
 * Implementation pattern: Submit runs the existing per-AU decode flow,
 * captures POC + YUV from the result, and appends to reorder_q.  After
 * each Submit, while reorder_q exceeds max_num_reorder_pics, the
 * lowest-POC entry is moved to ready_q.  Drain spills everything left
 * in reorder_q into ready_q in POC ascending order.  PollFrame pops the
 * front of ready_q.
 *
 * We compute max_num_reorder_pics from the active SPS on the first
 * Submit that produces a frame; subsequent SPS updates (e.g. a mid-
 * stream resolution change) refresh it.  Fallback to 4 (a generous
 * lower bound that fits every B-pyramid we ship) when SPS is absent. */

static uint32_t resolve_max_reorder(const DecodeEngine *e) {
    if (e->codec == Codec::H265) {
        if (e->parsed_h265.active_sps_id >= 0) {
            const H265Sps &sps = e->parsed_h265.sps[e->parsed_h265.active_sps_id];
            int idx = sps.sps_max_sub_layers_minus1;
            if (idx < 0) idx = 0;
            if (idx > H265_MAX_SUB_LAYERS - 1) idx = H265_MAX_SUB_LAYERS - 1;
            uint32_t v = sps.sps_max_num_reorder_pics[idx];
            return v;
        }
    } else {
        if (e->parsed_h264.has_sps) {
            return e->parsed_h264.sps.max_num_ref_frames;
        }
    }
    return 4;
}

static int32_t current_poc(const DecodeEngine *e) {
    if (e->codec == Codec::H265) return e->parsed_h265.poc;
    return (int32_t)e->parsed_h264.decode.top_field_order_cnt;
}

static void bump_lowest(DecodeEngine *e) {
    /* Find lowest-POC entry in reorder_q and move it to ready_q. */
    if (e->reorder_q.empty()) return;
    size_t best = 0;
    for (size_t i = 1; i < e->reorder_q.size(); i++) {
        if (e->reorder_q[i].poc < e->reorder_q[best].poc) best = i;
    }
    auto &entry = e->reorder_q[best];
    /* Move the slot's hold from REORDER → READY so the diagnostic state
     * matches which queue the entry is in.  Skipped when slot_idx<0
     * (engine_reorder_test bypasses the hold path). */
    if (entry.slot_idx >= 0 && entry.slot_idx < DecodeEngine::kPoolSize) {
        if (e->codec == Codec::H265) {
            H265Dpb_TransferHold(&e->dpb_h265, (uint32_t)entry.slot_idx,
                                 DPB_HOLD_REORDER, DPB_HOLD_READY,
                                 entry.epoch);
        } else {
            Dpb_TransferHold(&e->dpb_h264, (uint32_t)entry.slot_idx,
                             DPB_HOLD_REORDER, DPB_HOLD_READY,
                             entry.epoch);
        }
    }
    e->ready_q.push_back(std::move(e->reorder_q[best]));
    e->reorder_q.erase(e->reorder_q.begin() + best);
}

/* C++ linkage */
/* Public test/host entry point — see decode_engine.h for rationale.
 *
 * Layered defense for H.264's POC-resets-at-IDR pitfall:
 *   1. PRIMARY: resolve_max_reorder() prefers the VUI's
 *      bitstream_restriction.max_num_reorder_frames, which is the
 *      actual display-reorder bound (typically 0-2).  When the bound
 *      is tight enough, lingering pre-IDR entries can't survive long
 *      enough to be passed by post-IDR entries' lower POCs.
 *   2. SAFETY BELT: when this function is called with
 *      `is_idr_h264_boundary=true`, drain the entire reorder_q to
 *      ready_q before pushing the IDR.  Catches streams where VUI
 *      bitstream_restriction is absent and the conservative
 *      max_num_ref_frames fallback over-approximates the bound. */
void DecodeEngine_OnDecodeComplete(DecodeEngine *e,
                                   DecodeEngine::ReorderEntry entry,
                                   uint32_t max_num_reorder_pics,
                                   bool is_idr_h264_boundary)
{
    if (is_idr_h264_boundary) {
        while (!e->reorder_q.empty()) bump_lowest(e);
    }
    e->reorder_q.push_back(std::move(entry));
    e->max_num_reorder_pics = max_num_reorder_pics;
    /* Spec C.5.2 / C.4 bump: while DPB output queue has more pending
     * pics than max_num_reorder_pics, emit the lowest-POC. */
    while (e->reorder_q.size() > e->max_num_reorder_pics) {
        bump_lowest(e);
    }
}

/* Scan an Annex-B AU for an IDR (H.264 nal_unit_type==5) or IRAP
 * (H.265 NAL types 16..23: BLA_W_LP..RSV_IRAP_VCL23) slice.  Used by
 * the post-flush wait-for-IDR gate: until the first IRAP arrives, the
 * DPB is empty and any P/B slice would feed bad ref-list entries to
 * the codec.  Returns true if an IRAP slice is present anywhere in the
 * AU. */
static bool au_has_irap(const uint8_t *au, size_t len, Codec codec)
{
    if (!au || len < 4) return false;
    /* Walk start codes (00 00 00 01 or 00 00 01).  We only need to look
     * at the NAL header byte(s) immediately after each prefix. */
    size_t i = 0;
    while (i + 3 < len) {
        bool is_sc = false;
        size_t sc_len = 0;
        if (au[i] == 0 && au[i + 1] == 0 && au[i + 2] == 1) {
            is_sc = true;
            sc_len = 3;
        } else if (i + 4 < len && au[i] == 0 && au[i + 1] == 0 &&
                   au[i + 2] == 0 && au[i + 3] == 1) {
            is_sc = true;
            sc_len = 4;
        }
        if (!is_sc) { i++; continue; }
        size_t hdr = i + sc_len;
        if (hdr >= len) break;
        if (codec == Codec::H265) {
            uint8_t nut = (uint8_t)((au[hdr] >> 1) & 0x3f);
            if (nut >= 16 && nut <= 23) return true;
        } else {
            uint8_t nut = (uint8_t)(au[hdr] & 0x1f);
            if (nut == 5) return true;
        }
        i = hdr + 1;
    }
    return false;
}

int DecodeEngine_Submit(DecodeEngine *e,
                        const uint8_t *au, size_t au_len,
                        int64_t pts_hns,
                        uint32_t epoch)
{
    /* Post-flush IRAP gate: drop AUs until the first IDR/IRAP arrives.
     * See DecodeEngine.wait_for_idr docstring. */
    if (e->wait_for_idr) {
        if (!au_has_irap(au, au_len, e->codec)) {
            return 0;
        }
        e->wait_for_idr = false;
    }

    DecodeEngine::ReorderEntry entry;
    entry.epoch = epoch;
    int rc = DecodeEngine_DecodeOne(e, au, au_len, &entry.yuv);
    if (rc != 0) {
        /* Decode failed — codec wedge, parser hard-error, or DPB rejection.
         * Re-arm the IRAP gate so subsequent AUs are dropped until the
         * next IDR/IRAP arrives.  Without this, MFT keeps feeding P/B
         * slices into a half-baked DPB while the kernel cycles through
         * NeedsCoreReset on each kick, and every one of those kicks
         * re-trips the same wedge — visible to the user as "playback
         * stops a second or two after seek".  Skipping forward to the
         * next IRAP lets the codec recover into a known-clean state
         * (the IDR's DPB flush re-bases everything). */
        e->wait_for_idr = true;
        return rc;
    }

    /* Take an external hold on the slot we just decoded into so that any
     * subsequent decode can't overwrite it while this entry sits in the
     * reorder queue.  Released in DecodeEngine_ReleaseFrame (called by
     * the consumer after the DecodedFrame has been copied/consumed). */
    if (e->last_decoded_slot >= 0 &&
        e->last_decoded_slot < DecodeEngine::kPoolSize) {
        entry.slot_idx = e->last_decoded_slot;
        if (e->codec == Codec::H265) {
            H265Dpb_AddHold(&e->dpb_h265, (uint32_t)entry.slot_idx,
                            DPB_HOLD_REORDER, entry.epoch);
            entry.is_ref = e->dpb_h265.slots[entry.slot_idx].is_ref ? true : false;
        } else {
            Dpb_AddHold(&e->dpb_h264, (uint32_t)entry.slot_idx,
                        DPB_HOLD_REORDER, entry.epoch);
            entry.is_ref = e->dpb_h264.slots[entry.slot_idx].is_ref ? true : false;
        }
    }

    entry.poc = current_poc(e);
    if (pts_hns < 0) {
        /* Synthetic monotonic timeline if caller doesn't supply pts. */
        entry.pts_hns = (int64_t)(e->submit_count *
                                  (uint64_t)10'000'000ULL / 30ULL);
    } else {
        entry.pts_hns = pts_hns;
    }
    entry.dur_hns = (int64_t)(10'000'000ULL / 30ULL);
    e->submit_count++;

    /* H.264 IDR detection: POC resets at every IDR; reorder_q's POC-only
     * sort would otherwise leave pre-IDR tail frames stuck.  See
     * DecodeEngine_OnDecodeComplete docstring + h264_idr_reorder_fix.md. */
    bool is_idr = false;
    if (e->codec != Codec::H265) {
        is_idr = (e->parsed_h264.decode.flags &
                  V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;
    }
    DecodeEngine_OnDecodeComplete(e, std::move(entry),
                                  resolve_max_reorder(e), is_idr);
    return 0;
}

/* C++ linkage */
int DecodeEngine_SubmitFramed(DecodeEngine *e,
                              NalFraming framing, AvccLenSize len_size,
                              const uint8_t *au, size_t au_len,
                              int64_t pts_hns,
                              uint32_t epoch)
{
    if (framing == FRAMING_ANNEXB)
        return DecodeEngine_Submit(e, au, au_len, pts_hns, epoch);
    std::vector<uint8_t> tmp(au_len + 64);
    int n = AvccToAnnexB(au, au_len, len_size, tmp.data(), tmp.size());
    if (n < 0) return Fail("AvccToAnnexB(submit): malformed");
    return DecodeEngine_Submit(e, tmp.data(), (size_t)n, pts_hns, epoch);
}

/* C++ linkage */
int DecodeEngine_PollFrame(DecodeEngine *e, DecodedFrame *out)
{
    if (e->ready_q.empty()) return 0;
    DecodeEngine::ReorderEntry entry = std::move(e->ready_q.front());
    e->ready_q.erase(e->ready_q.begin());
    out->poc      = entry.poc;
    out->pts_hns  = entry.pts_hns;
    out->dur_hns  = entry.dur_hns;
    out->yuv      = std::move(entry.yuv);
    out->epoch    = entry.epoch;
    /* Move the slot's hold from READY → CONSUMER as the entry leaves
     * ready_q for a live DecodedFrame.  Consumer must call
     * DecodeEngine_ReleaseFrame to clear the CONSUMER hold; failing to
     * do so would leak holds and eventually exhaust the slot pool. */
    out->slot_idx = entry.slot_idx;
    if (entry.slot_idx >= 0 && entry.slot_idx < DecodeEngine::kPoolSize) {
        if (e->codec == Codec::H265) {
            H265Dpb_TransferHold(&e->dpb_h265, (uint32_t)entry.slot_idx,
                                 DPB_HOLD_READY, DPB_HOLD_CONSUMER,
                                 entry.epoch);
        } else {
            Dpb_TransferHold(&e->dpb_h264, (uint32_t)entry.slot_idx,
                             DPB_HOLD_READY, DPB_HOLD_CONSUMER,
                             entry.epoch);
        }
    }
    entry.slot_idx = -1;
    out->is_ref   = entry.is_ref;
    /* Zero-copy fields — valid in either populate_yuv mode but only
     * load-bearing when populate_yuv=false (yuv is empty in that case). */
    if (out->slot_idx >= 0 && out->slot_idx < DecodeEngine::kPoolSize) {
        out->src_ptr        = e->pool_output[out->slot_idx].user_va;
        out->src_width      = e->frame_width;
        out->src_height_pad = (e->frame_height + 15u) & ~15u;
    }
    return 1;
}

size_t DecodeEngine_QueueDepth(const DecodeEngine *e)
{
    if (!e) return 0;
    return e->reorder_q.size() + e->ready_q.size();
}

size_t DecodeEngine_InputQueueCapacity(const DecodeEngine *e)
{
    if (!e) return 4;
    size_t cap = (size_t)e->max_num_reorder_pics + 1u;
    if (cap < 4u) cap = 4u;
    const size_t max_safe = (size_t)DecodeEngine::kPoolSize - 1u;
    if (cap > max_safe) cap = max_safe;
    return cap;
}

void DecodeEngine_ReleaseFrame(DecodeEngine *e, DecodedFrame *f)
{
    if (!e || !f) return;
    if (f->slot_idx < 0 || f->slot_idx >= DecodeEngine::kPoolSize) return;
    if (e->codec == Codec::H265)
        H265Dpb_ReleaseHold(&e->dpb_h265, (uint32_t)f->slot_idx,
                            DPB_HOLD_CONSUMER);
    else
        Dpb_ReleaseHold(&e->dpb_h264, (uint32_t)f->slot_idx,
                        DPB_HOLD_CONSUMER);
    f->slot_idx = -1;
}

/* C++ linkage */
void DecodeEngine_Drain(DecodeEngine *e)
{
    /* Move every remaining reorder_q entry to ready_q in POC ascending
     * order.  Stable for ties (shouldn't happen with our streams). */
    while (!e->reorder_q.empty()) bump_lowest(e);
}

int DecodeEngine_Flush(DecodeEngine *e)
{
    /* Drop any pending reorder window — flush in MFT terms means "the
     * caller is dropping output and starting fresh from the next IDR".
     * Release the per-entry holds before clearing the queues so the
     * DPB's lifecycle flags match. */
    auto release_reorder = [&](int slot_idx) {
        if (slot_idx < 0 || slot_idx >= DecodeEngine::kPoolSize) return;
        if (e->codec == Codec::H265)
            H265Dpb_ReleaseHold(&e->dpb_h265, (uint32_t)slot_idx,
                                DPB_HOLD_REORDER);
        else
            Dpb_ReleaseHold(&e->dpb_h264, (uint32_t)slot_idx,
                            DPB_HOLD_REORDER);
    };
    auto release_ready = [&](int slot_idx) {
        if (slot_idx < 0 || slot_idx >= DecodeEngine::kPoolSize) return;
        if (e->codec == Codec::H265)
            H265Dpb_ReleaseHold(&e->dpb_h265, (uint32_t)slot_idx,
                                DPB_HOLD_READY);
        else
            Dpb_ReleaseHold(&e->dpb_h264, (uint32_t)slot_idx,
                            DPB_HOLD_READY);
    };
    for (auto &entry : e->reorder_q) release_reorder(entry.slot_idx);
    for (auto &entry : e->ready_q)   release_ready  (entry.slot_idx);
    e->reorder_q.clear();
    e->ready_q.clear();
    e->submit_count = 0;
    e->max_num_reorder_pics = 0;
    e->wait_for_idr = true;

    /* Bump the DPB epoch and clear reference state on slots not held by
     * a live DecodedFrame.  Consumer-held slots keep their pre-flush
     * epoch so the consumer's eventual ReleaseFrame still asserts
     * cleanly.  Pool buffers untouched. */
    if (e->codec == Codec::H265) {
        H265Dpb_Flush(&e->dpb_h265);
    } else {
        Dpb_Flush(&e->dpb_h264);
    }
    return 0;
}
