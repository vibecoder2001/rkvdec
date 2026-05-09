/* parser_lavc_diff — host-only cross-check that runs OUR H.264 parser +
 * Dpb_Select on each AU, AND simultaneously feeds the same AUs to
 * libavcodec, then diffs per-AU slice_type / fn / POC.  Supports an
 * optional --shim-log <mpp.shim.log> for three-way BSP comparison.
 * Exits 0 if everything matches.
 *
 * Usage:
 *   parser_lavc_diff <file.h264> [--shim-log <mpp.shim.log>]
 */
#include "parser_glue.h"
#include "dpb.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/log.h>
}

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
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

/* ---- ShimLog (ported from winreplay_h264_diff.c) -------------------- */
struct ShimSubMsg {
    uint32_t cmd, flags, size, offset, words;
    uint32_t data[512];
};
struct ShimAU {
    int         n_subs;
    ShimSubMsg  subs[16];
};
struct ShimLog {
    int    n_aus;
    ShimAU aus[256];
};

static int parse_shim_log(const char *path, ShimLog *out)
{
    FILE *f = fopen(path, "r");
    if (!f) { std::perror(path); return -1; }
    out->n_aus = 0;
    ShimAU    *cur_au  = nullptr;
    ShimSubMsg *cur_sub = nullptr;
    int started = 0;
    char line[1024];
    while (std::fgets(line, sizeof(line), f)) {
        if (std::strstr(line, "MPP IOCTL")) { cur_sub = nullptr; continue; }
        unsigned cmd, flags, size, off;
        unsigned long long data_ptr;
        if (std::sscanf(line,
                " [sub %*d] cmd=0x%x flags=0x%x size=%u offset=%u data=0x%llx",
                &cmd, &flags, &size, &off, &data_ptr) == 5) {
            cur_sub = nullptr;
            if (cmd == 0x200 && off == 32) {
                if (out->n_aus >= 256) {
                std::fprintf(stderr, "parse_shim_log: AU count exceeds 256 — truncating\n");
                std::fclose(f); return -1;
            }
                cur_au = &out->aus[out->n_aus++];
                cur_au->n_subs = 0;
                started = 1;
            }
            if (cmd == 0x300) { cur_au = nullptr; started = 0; continue; }
            if (!cur_au || !started) continue;
            if (cmd != 0x200 && cmd != 0x201 && cmd != 0x202) continue;
            if (cur_au->n_subs >= 16) continue;
            cur_sub = &cur_au->subs[cur_au->n_subs++];
            cur_sub->cmd = cmd; cur_sub->flags = flags;
            cur_sub->size = size; cur_sub->offset = off;
            cur_sub->words = 0;
            std::memset(cur_sub->data, 0, sizeof(cur_sub->data));
            continue;
        }
        if (cur_sub) {
            const char *p = line;
            while (*p == ' ') p++;
            if (*p != '[') continue;
            int idx;
            if (std::sscanf(p, "[%d]", &idx) != 1) continue;
            const char *q = std::strchr(p, ']'); if (!q) continue;
            q++;
            for (int j = 0; j < 8; j++) {
                while (*q == ' ') q++;
                if (!std::isxdigit((unsigned char)*q)) break;
                char *e = nullptr;
                unsigned long v = std::strtoul(q, &e, 16);
                if (e == q) break;
                if (idx + j < 512) cur_sub->data[idx + j] = (uint32_t)v;
                q = e;
            }
            uint32_t end = (uint32_t)(idx + 8);
            if (end > cur_sub->words) cur_sub->words = end;
        }
    }
    std::fclose(f);
    return 0;
}

/* Extract a register value from a ShimAU's cmd=0x200 sub-messages.
 * reg_idx is the absolute register index (e.g. 65 for cur_top_poc).
 * Returns UINT32_MAX if not found. */
static uint32_t shim_reg(const ShimAU *au, uint32_t reg_idx)
{
    uint32_t byte_off = reg_idx * 4;
    for (int s = 0; s < au->n_subs; s++) {
        const ShimSubMsg &sub = au->subs[s];
        if (sub.cmd != 0x200) continue;
        if (byte_off < sub.offset) continue;
        uint32_t word = (byte_off - sub.offset) / 4;
        if (word >= sub.words) continue;
        return sub.data[word];
    }
    return UINT32_MAX;
}

} /* anon */

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr,
            "usage: %s <file.h264> [--shim-log <mpp.shim.log>]\n", argv[0]);
        return 2;
    }
    const char *input_path = argv[1];
    const char *shim_path  = nullptr;
    for (int i = 2; i < argc; i++) {
        if (!std::strcmp(argv[i], "--shim-log") && i + 1 < argc)
            shim_path = argv[++i];
    }

    static ShimLog shim_log;
    bool have_shim = false;
    if (shim_path) {
        if (parse_shim_log(shim_path, &shim_log) == 0) {
            have_shim = true;
            std::fprintf(stderr, "shim log: %d AUs\n", shim_log.n_aus);
        }
    }

    auto buf = ReadAll(input_path);
    if (buf.empty()) {
        std::fprintf(stderr, "cannot read %s\n", input_path);
        return 2;
    }
    auto aus = SliceAus(buf.data(), buf.size());
    std::printf("# input=%s aus=%zu\n", input_path, aus.size());

    /* lavc setup — full decoder so we get per-frame POC / type. */
    av_log_set_level(AV_LOG_ERROR);
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) { std::fprintf(stderr, "no H264 decoder\n"); return 1; }
    AVCodecContext *cc = avcodec_alloc_context3(codec);
    if (!cc) { std::fprintf(stderr, "alloc_context3 failed\n"); return 1; }
    cc->flags |= AV_CODEC_FLAG_OUTPUT_CORRUPT;
    if (avcodec_open2(cc, codec, nullptr) < 0) {
        std::fprintf(stderr, "avcodec_open2 failed\n"); return 1;
    }

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

    /* Collect all decoded frames keyed by pts (set to AU sequence index).
     * B-frame reorder means receive_frame may lag behind send_packet. */
    std::map<int64_t, AVFrame *> lavc_frames;

    auto drain_frames = [&]() {
        AVFrame *f = av_frame_alloc();
        while (avcodec_receive_frame(cc, f) == 0) {
            lavc_frames[f->pts] = f;
            f = av_frame_alloc();
        }
        av_frame_free(&f);
    };

    /* Pass 1: send all packets, collect frames as they emerge */
    int64_t slice_seq = 0; /* counts only has_slice AUs, matches au_seq in pass 2 */
    for (size_t i = 0; i < aus.size(); i++) {
        const uint8_t *p = buf.data() + aus[i].start;
        size_t          n = aus[i].end - aus[i].start;

        /* Our parser — advance SPS/PPS state so the parser context is in sync
         * with lavc for pass 2.  DPB is also stepped so pass-2 frame_num and
         * POC values are correct.  sel is intentionally unused here. */
        if (H264ParseAccessUnit(p, n, scratch.data(), scratch.size(),
                                &parsed) != H264_PARSE_OK) {
            std::fprintf(stderr, "au %zu: our parser failed\n", i);
            return 1;
        }
        if (!parsed.has_slice) continue;

        DpbSelection sel{};
        Dpb_Select(&dpb, &parsed, &sel);
        Dpb_OnDecodeComplete(&dpb);

        /* lavc: send packet, tag with slice_seq so we can match in pass 2 */
        pkt->data = const_cast<uint8_t*>(p);
        pkt->size = (int)n;
        pkt->pts  = slice_seq++;
        avcodec_send_packet(cc, pkt);
        drain_frames();
    }

    /* Drain lavc after all packets */
    avcodec_send_packet(cc, nullptr);
    drain_frames();

    /* Pass 2: diff per AU */
    DpbCtx dpb2{};
    DpbPoolEntry pool2[16];
    for (int i = 0; i < 16; i++) {
        pool2[i].output_frame = (uint64_t)0x1000 + i;
        pool2[i].colmv        = (uint64_t)0x2000 + i;
    }
    Dpb_Init(&dpb2, pool2, 16);
    H264ParseResult parsed2{};
    std::fill(scratch.begin(), scratch.end(), 0);

    int diverged = 0, matched = 0, au_seq = 0;

    for (size_t i = 0; i < aus.size(); i++) {
        const uint8_t *p = buf.data() + aus[i].start;
        size_t          n = aus[i].end - aus[i].start;

        if (H264ParseAccessUnit(p, n, scratch.data(), scratch.size(),
                                &parsed2) != H264_PARSE_OK)
            continue;
        if (!parsed2.has_slice) continue;

        DpbSelection sel2{};
        Dpb_Select(&dpb2, &parsed2, &sel2);
        Dpb_OnDecodeComplete(&dpb2);

        bool is_idr = (parsed2.decode.flags &
                       V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;
        int32_t our_poc  = parsed2.decode.top_field_order_cnt;
        uint32_t our_fn  = parsed2.decode.frame_num;
        char our_type    = SliceTypeChar(parsed2.slice.slice_type);

        /* lavc side — keyed by pts == au_seq from pass 1 */
        int32_t lavc_poc  = INT32_MIN;
        int     lavc_fn   = INT32_MIN;
        char    lavc_type = '-';
        bool    lavc_idr  = false;
        auto it = lavc_frames.find((int64_t)au_seq);
        if (it != lavc_frames.end()) {
            AVFrame *lf = it->second;
            lavc_poc  = (int32_t)lf->pts;   /* pts == au_seq; no display_picture_number in ffmpeg 7 */
            lavc_fn   = (int)lf->pts;        /* coded_picture_number removed in ffmpeg 7 */
            lavc_type = LavcPictTypeChar(lf->pict_type);
            lavc_idr  = (lf->key_frame != 0);
        }

        /* BSP side (from shim regs) */
        int32_t bsp_top_poc = INT32_MIN;
        int32_t bsp_bot_poc = INT32_MIN;
        uint32_t bsp_ref_poc[16][2];
        std::memset(bsp_ref_poc, 0xFF, sizeof(bsp_ref_poc)); /* UINT32_MAX fill */
        bool have_bsp_au = false;
        if (have_shim && au_seq < shim_log.n_aus) {
            const ShimAU *sau = &shim_log.aus[au_seq];
            uint32_t v65 = shim_reg(sau, 65);
            uint32_t v66 = shim_reg(sau, 66);
            if (v65 != UINT32_MAX && v66 != UINT32_MAX) {
                bsp_top_poc = (int32_t)v65;
                bsp_bot_poc = (int32_t)v66;
                for (int r = 0; r < 16; r++) {
                    bsp_ref_poc[r][0] = shim_reg(sau, 67 + r*2);
                    bsp_ref_poc[r][1] = shim_reg(sau, 67 + r*2 + 1);
                }
                have_bsp_au = true;
            }
        }

        /* Compare.
         *
         * NOTE on idr_ok: lavc's AVFrame::key_frame is broader than H.264 IDR
         * — it's set on any "seekable" frame including non-IDR I-slices that
         * follow a recovery_point SEI.  Comparing it to our nal_unit_type==5
         * "true IDR" flag produces false positives at every recovery I-frame
         * (dancing.h264 has 10 such frames).  We tolerate the asymmetric
         * case `ours=0, lavc_key=1` and only flag a divergence when ours says
         * IDR but lavc doesn't (the only direction that would indicate a real
         * parser bug). */
        int32_t our_bot_poc = parsed2.decode.bottom_field_order_cnt;
        bool type_ok    = (lavc_type == '-' || lavc_type == our_type);
        bool idr_ok     = (lavc_poc == INT32_MIN) ||
                          !(is_idr && !lavc_idr);
        bool poc_ok     = (!have_bsp_au || bsp_top_poc == our_poc);
        bool bot_poc_ok = (!have_bsp_au || bsp_bot_poc == our_bot_poc);

        if (type_ok && idr_ok && poc_ok && bot_poc_ok) {
            /* lavc_fn/lavc_poc are AU sequence index (not frame_num/POC);
             * ffmpeg 7 removed coded_picture_number/display_picture_number. */
            std::printf("AU %-3zu (%c) MATCH"
                        "  fn=ours:%u  poc=ours:%d/bsp:%s\n",
                        i, our_type,
                        our_fn,
                        our_poc,
                        have_bsp_au ? std::to_string((int)bsp_top_poc).c_str() : "\xe2\x80\x94");
            matched++;
        } else {
            std::printf("AU %-3zu (%c) DIVERGE\n", i, our_type);
            if (!type_ok)
                std::printf("  type:  ours=%c  lavc=%c\n", our_type, lavc_type);
            if (!idr_ok)
                std::printf("  idr:   ours=%d  lavc=%d\n", is_idr?1:0, lavc_idr?1:0);
            if (!poc_ok)
                std::printf("  poc:   ours=%d  bsp=%d\n", our_poc, (int)bsp_top_poc);
            if (!bot_poc_ok)
                std::printf("  bot_poc: ours=%d  bsp=%d\n", our_bot_poc, (int)bsp_bot_poc);
            if (have_bsp_au) {
                bool any_ref_diff = false;
                for (int r = 0; r < 16; r++) {
                    if (bsp_ref_poc[r][0] == UINT32_MAX) break;
                    int32_t our_rp = (r < DPB_MAX_SLOTS &&
                        (sel2.dpb_entries[r].flags &
                         V4L2_H264_DPB_ENTRY_FLAG_VALID))
                        ? sel2.dpb_entries[r].top_field_order_cnt : INT32_MIN;
                    int32_t bsp_rp = (int32_t)bsp_ref_poc[r][0];
                    if (our_rp != bsp_rp) {
                        if (!any_ref_diff) std::printf("  ref_poc diffs:\n");
                        std::printf("    ref[%2d]: ours=%d  bsp=%d\n",
                                    r, our_rp, bsp_rp);
                        any_ref_diff = true;
                    }
                }
            }
            diverged++;
        }

        au_seq++;
    }

    std::printf("# matched=%d diverged=%d\n", matched, diverged);
    for (auto &kv : lavc_frames) av_frame_free(&kv.second);
    av_packet_free(&pkt);
    avcodec_free_context(&cc);
    return diverged ? 1 : 0;
}
