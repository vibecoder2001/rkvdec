/* parser_dump — host-only harness that exercises our H.264 parser +
 * Dpb_Select on each AU of an Annex-B file and prints one structured
 * line per AU.  Pair with `lavc_dump` (or `ffprobe -show_frames`) to
 * cross-check slice_type / POC ordering / RefPicList against ground
 * truth without needing the ARM target hardware.
 *
 * Usage:
 *   parser_dump <input.h264>
 *
 * Output (one line per AU, space-separated key=value):
 *   au=N  type=<I|P|B|SI|SP> nri=<0..3> idr=<0|1> fn=<frame_num>
 *   poc=<top_field_order_cnt>
 *   dpb=[slot/fn/poc/lt,slot/fn/poc/lt,...]
 *   l0=[idx,idx,...]  l1=[idx,...]  l2=[idx,...]
 *
 * Exits non-zero on parser/DPB errors.  Pure user-mode — no driver.
 */
#include "parser_glue.h"
#include "parser_glue_h265.h"
#include "dpb.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

/* Find the next "AU boundary" — a NAL whose nal_unit_type is in {1,5,9}
 * (slice / IDR / AUD).  We treat each slice NAL as a full AU for this
 * dump (sufficient for single-slice-per-pic streams).  Returns offset
 * range [au_start, au_end) of the AU's bytes including SPS/PPS NALs
 * that immediately precede the slice. */
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

/* Walk NALs and emit one AU at a time.  An AU is the run of NALs
 * starting at the first non-AUD/SEI NAL after the previous slice NAL,
 * up to and including the next slice NAL (1 or 5). */
struct AuRange { size_t start; size_t end; };

static std::vector<AuRange> SliceAus(const uint8_t *buf, size_t len, bool h265)
{
    std::vector<AuRange> aus;
    size_t pos = 0, au_start = 0; bool have_au = false;
    while (pos < len) {
        size_t sc_start = 0, payload = 0;
        if (!FindStartCode(buf, len, pos, &sc_start, &payload)) break;
        if (payload >= len) break;
        uint8_t nut;
        bool is_slice;
        if (h265) {
            /* HEVC NAL header: 2 bytes; nal_unit_type = bits 1..6 of byte 0.
             * VCL slice NAL types are 0..31 (TRAIL_N=0 .. RSV_VCL31=31). */
            nut = (buf[payload] >> 1) & 0x3F;
            is_slice = (nut <= 31);
        } else {
            /* H.264: 1-byte header, nal_unit_type = low 5 bits.
             * VCL slice NAL types: 1 (non-IDR slice) and 5 (IDR slice). */
            nut = buf[payload] & 0x1F;
            is_slice = (nut == 1 || nut == 5);
        }
        size_t next_sc = 0, next_pl = 0;
        if (!FindStartCode(buf, len, payload, &next_sc, &next_pl)) next_sc = len;

        if (!have_au) { au_start = sc_start; have_au = true; }
        if (is_slice) {
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
    case V4L2_H264_SLICE_TYPE_P:  return 'P';
    case V4L2_H264_SLICE_TYPE_B:  return 'B';
    case V4L2_H264_SLICE_TYPE_I:  return 'I';
    case V4L2_H264_SLICE_TYPE_SP: return 'p'; /* lowercase = SP */
    case V4L2_H264_SLICE_TYPE_SI: return 'i';
    default:                      return '?';
    }
}

} /* anon */

static int RunH265(const std::vector<uint8_t> &buf,
                   const std::vector<AuRange> &aus,
                   const char *path)
{
    std::printf("# input=%s codec=h265 bytes=%zu aus=%zu\n",
                path, buf.size(), aus.size());

    H265DpbCtx dpb{};
    DpbPoolEntry pool[16];
    for (int i = 0; i < 16; i++) {
        pool[i].output_frame = (uint64_t)0x1000 + i;
        pool[i].colmv        = (uint64_t)0x2000 + i;
    }
    if (H265Dpb_Init(&dpb, pool, 16) != DPB_OK) {
        std::fprintf(stderr, "H265Dpb_Init failed\n");
        return 1;
    }

    H265ParseResult parsed{};
    H265ParseResultInit(&parsed);
    std::vector<uint8_t> scratch(2u << 20);

    for (size_t i = 0; i < aus.size(); i++) {
        const uint8_t *p = buf.data() + aus[i].start;
        size_t          n = aus[i].end - aus[i].start;
        H265ParseStatus s = H265ParseAccessUnit(p, n,
                                                scratch.data(),
                                                scratch.size(),
                                                &parsed);
        if (s != H265_PARSE_OK) {
            std::fprintf(stderr, "au %zu: parser status=%d\n", i, (int)s);
            return 1;
        }
        if (!parsed.has_slice) continue;

        H265DpbSelection sel{};
        if (H265Dpb_Select(&dpb, &parsed, &sel) != DPB_OK) {
            std::fprintf(stderr, "au %zu: H265Dpb_Select failed\n", i);
            return 1;
        }

        std::printf("au=%zu nut=%u idr=%u irap=%u nri=%u poc=%d cur_slot=%u valid_mask=0x%04x",
                    i,
                    parsed.slice_nal_unit_type,
                    parsed.is_idr,
                    parsed.is_irap,
                    parsed.nal_ref_flag,
                    parsed.poc,
                    sel.current_slot,
                    sel.ref_valid_mask);

        std::printf(" dpb=[");
        bool first = true;
        for (int k = 0; k < 16; k++) {
            if (!(sel.ref_valid_mask & (1u << k))) continue;
            std::printf("%s%d/poc=%d", first ? "" : ",", k, sel.ref_pocs[k]);
            first = false;
        }
        std::printf("]\n");

        H265Dpb_OnDecodeComplete(&dpb);
    }
    return 0;
}

static bool EndsWith(const std::string &s, const char *suf)
{
    size_t sl = std::strlen(suf);
    return s.size() >= sl &&
           _stricmp(s.c_str() + (s.size() - sl), suf) == 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.{h264,h265,265,264}>\n", argv[0]);
        return 2;
    }

    auto buf = ReadAll(argv[1]);
    if (buf.empty()) {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 2;
    }

    std::string path = argv[1];
    bool h265 = EndsWith(path, ".h265") || EndsWith(path, ".265") ||
                EndsWith(path, ".hevc");

    auto aus = SliceAus(buf.data(), buf.size(), h265);

    if (h265) {
        return RunH265(buf, aus, argv[1]);
    }

    std::printf("# input=%s codec=h264 bytes=%zu aus=%zu\n",
                argv[1], buf.size(), aus.size());

    DpbCtx dpb{};
    DpbPoolEntry pool[16];
    for (int i = 0; i < 16; i++) {
        pool[i].output_frame = (uint64_t)0x1000 + i;
        pool[i].colmv        = (uint64_t)0x2000 + i;
    }
    if (Dpb_Init(&dpb, pool, 16) != DPB_OK) {
        std::fprintf(stderr, "Dpb_Init failed\n");
        return 1;
    }

    H264ParseResult parsed{};
    std::vector<uint8_t> scratch(2u << 20);

    for (size_t i = 0; i < aus.size(); i++) {
        const uint8_t *p = buf.data() + aus[i].start;
        size_t          n = aus[i].end - aus[i].start;
        H264ParseStatus s = H264ParseAccessUnit(p, n,
                                                scratch.data(),
                                                scratch.size(),
                                                &parsed);
        if (s != H264_PARSE_OK) {
            std::fprintf(stderr, "au %zu: parser status=%d\n", i, (int)s);
            return 1;
        }
        if (!parsed.has_slice) continue;

        DpbSelection sel{};
        if (Dpb_Select(&dpb, &parsed, &sel) != DPB_OK) {
            std::fprintf(stderr, "au %zu: Dpb_Select failed\n", i);
            return 1;
        }

        bool is_idr = (parsed.decode.flags &
                       V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;

        std::printf("au=%zu type=%c nri=%u idr=%d fn=%u poc=%d cur_slot=%u",
                    i,
                    SliceTypeChar(parsed.slice.slice_type),
                    parsed.decode.nal_ref_idc,
                    is_idr ? 1 : 0,
                    parsed.decode.frame_num,
                    parsed.decode.top_field_order_cnt,
                    sel.current_slot);

        /* Compact DPB summary — only valid (active ref) entries. */
        std::printf(" dpb=[");
        bool first = true;
        for (int k = 0; k < 16; k++) {
            const auto &e = sel.dpb_entries[k];
            if (!(e.flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)) continue;
            int lt = (e.flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) ? 1 : 0;
            std::printf("%s%d/fn=%u/poc=%d/lt=%d",
                        first ? "" : ",", k,
                        e.frame_num, e.top_field_order_cnt, lt);
            first = false;
        }
        std::printf("]");

        for (int li = 0; li < 3; li++) {
            std::printf(" l%d=[", li);
            bool fi = true;
            for (int k = 0; k < 32; k++) {
                if (!(sel.ref_lists[li][k].fields & V4L2_H264_FRAME_REF))
                    continue;
                std::printf("%s%u",
                            fi ? "" : ",",
                            sel.ref_lists[li][k].index);
                fi = false;
            }
            std::printf("]");
        }
        std::printf("\n");

        /* No real decode happens — just simulate it by calling
         * OnDecodeComplete to advance DPB state for sliding window /
         * MMCO bookkeeping. */
        Dpb_OnDecodeComplete(&dpb);
    }
    return 0;
}
