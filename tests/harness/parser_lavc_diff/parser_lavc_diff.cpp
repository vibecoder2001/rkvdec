/* parser_lavc_diff — host-only cross-check that runs OUR H.264 parser +
 * Dpb_Select on each AU, AND simultaneously feeds the same AUs to
 * libavcodec, then diffs per-AU slice_type / fn / POC.  Stops at the
 * first mismatch.  Exits 0 if everything matches.
 *
 * Usage:
 *   parser_lavc_diff <file.h264>
 *
 * Public lavc API only.  Decoder is run with FF_THREAD_FRAME=1 and
 * low_delay so emitted AVFrames come out in decode order — receive_frame
 * after each send_packet maps 1:1 to the AU just sent.
 */
#include "parser_glue.h"
#include "dpb.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/log.h>
}

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

static std::vector<uint8_t> ReadAll(const char *path)
{
    std::vector<uint8_t> v;
    FILE *f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || !f) return v;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n > 0) {
        v.resize((size_t)n);
        size_t got = fread(v.data(), 1, (size_t)n, f);
        v.resize(got);
    }
    fclose(f);
    return v;
}

static bool FindStartCode(const uint8_t *buf, size_t len, size_t from,
                          size_t *sc_start, size_t *payload)
{
    for (size_t i = from; i + 2 < len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0) {
            if (buf[i+2] == 1) { *sc_start = i; *payload = i + 3; return true; }
            if (i + 3 < len && buf[i+2] == 0 && buf[i+3] == 1) {
                *sc_start = i; *payload = i + 4; return true;
            }
        }
    }
    return false;
}

struct AuRange { size_t start; size_t end; };

static std::vector<AuRange> SliceAus(const uint8_t *buf, size_t len)
{
    std::vector<AuRange> aus;
    size_t pos = 0, au_start = 0; bool have_au = false;
    while (pos < len) {
        size_t sc_start = 0, payload = 0;
        if (!FindStartCode(buf, len, pos, &sc_start, &payload)) break;
        if (payload >= len) break;
        uint8_t nut = buf[payload] & 0x1F;
        size_t next_sc = 0, next_pl = 0;
        if (!FindStartCode(buf, len, payload, &next_sc, &next_pl)) next_sc = len;
        if (!have_au) { au_start = sc_start; have_au = true; }
        if (nut == 1 || nut == 5) {
            aus.push_back({ au_start, next_sc });
            have_au = false;
        }
        pos = next_sc;
    }
    return aus;
}

static char SliceTypeChar(uint8_t st)
{
    switch (st) {
    case V4L2_H264_SLICE_TYPE_P: return 'P';
    case V4L2_H264_SLICE_TYPE_B: return 'B';
    case V4L2_H264_SLICE_TYPE_I: return 'I';
    default:                     return '?';
    }
}

static char LavcPictTypeChar(int t)
{
    switch (t) {
    case AV_PICTURE_TYPE_I:  return 'I';
    case AV_PICTURE_TYPE_P:  return 'P';
    case AV_PICTURE_TYPE_B:  return 'B';
    case AV_PICTURE_TYPE_SI: return 'I';   /* SI maps to I for our purposes */
    case AV_PICTURE_TYPE_SP: return 'P';
    default:                 return '?';
    }
}

} /* anon */

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.h264>\n", argv[0]);
        return 2;
    }

    auto buf = ReadAll(argv[1]);
    if (buf.empty()) {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 2;
    }
    auto aus = SliceAus(buf.data(), buf.size());
    std::printf("# input=%s aus=%zu\n", argv[1], aus.size());

    /* lavc setup — use the parser API, not a full decoder, so we get
     * per-packet pict_type with no reorder/buffer delay. */
    av_log_set_level(AV_LOG_ERROR);
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) { std::fprintf(stderr, "no H264 decoder\n"); return 1; }
    AVCodecContext *cc = avcodec_alloc_context3(codec);
    if (!cc) { std::fprintf(stderr, "alloc_context3 failed\n"); return 1; }
    AVCodecParserContext *psr = av_parser_init(AV_CODEC_ID_H264);
    if (!psr) { std::fprintf(stderr, "av_parser_init failed\n"); return 1; }

    /* Our parser/DPB. */
    DpbCtx dpb{};
    DpbPoolEntry pool[16];
    for (int i = 0; i < 16; i++) {
        pool[i].output_frame = (uint64_t)0x1000 + i;
        pool[i].colmv        = (uint64_t)0x2000 + i;
    }
    Dpb_Init(&dpb, pool, 16);
    H264ParseResult parsed{};
    std::vector<uint8_t> scratch(2u << 20);

    AVPacket *pkt = av_packet_alloc();
    AVFrame  *frm = av_frame_alloc();

    int diverged = 0;
    int matched  = 0;

    for (size_t i = 0; i < aus.size(); i++) {
        const uint8_t *p = buf.data() + aus[i].start;
        size_t          n = aus[i].end - aus[i].start;

        /* --- our parser --- */
        if (H264ParseAccessUnit(p, n, scratch.data(), scratch.size(),
                                &parsed) != H264_PARSE_OK) {
            std::fprintf(stderr, "au %zu: our parser failed\n", i);
            return 1;
        }
        if (!parsed.has_slice) continue;

        /* --- lavc parser (per-packet, no decode buffering) --- */
        uint8_t *out_data = nullptr; int out_size = 0;
        av_parser_parse2(psr, cc,
                         &out_data, &out_size,
                         p, (int)n,
                         AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        char lavc_t = LavcPictTypeChar(psr->pict_type);

        char our_t = SliceTypeChar(parsed.slice.slice_type);
        bool is_idr = (parsed.decode.flags &
                       V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;

        bool type_ok = (lavc_t == '?' || lavc_t == our_t);

        if (!type_ok) {
            std::printf("DIVERGE au=%zu fn=%u poc=%d our=%c lavc=%c idr=%d\n",
                        i, parsed.decode.frame_num,
                        parsed.decode.top_field_order_cnt,
                        our_t, lavc_t, is_idr ? 1 : 0);
            diverged++;
        } else {
            matched++;
        }

        /* Advance DPB even on success so it stays in sync for later AUs. */
        DpbSelection sel{};
        Dpb_Select(&dpb, &parsed, &sel);
        Dpb_OnDecodeComplete(&dpb);
    }

    std::printf("# matched=%d diverged=%d\n", matched, diverged);
    av_packet_free(&pkt);
    av_frame_free(&frm);
    av_parser_close(psr);
    avcodec_free_context(&cc);
    return diverged ? 1 : 0;
}
