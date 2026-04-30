/* tests/harness/mp4_extract/main.cpp
 *
 * Minimal MP4 → H.264 Annex-B extractor.  Pulls the first keyframe of
 * the video track out of an avc1 .mp4 and writes:
 *   start_code + SPS + start_code + PPS + start_code + IDR-NAL[s]
 *
 * Just enough box parsing to find:
 *   moov/trak[video]/mdia/minf/stbl/stsd:avc1/avcC : SPS+PPS+lengthSize
 *   moov/trak[video]/tkhd                          : width / height (for our printout)
 *   moov/trak[video]/mdia/minf/stbl/stss            : keyframe sample index (1-based)
 *   moov/trak[video]/mdia/minf/stbl/stsz            : per-sample sizes
 *   moov/trak[video]/mdia/minf/stbl/stco|co64       : chunk file offsets
 *   moov/trak[video]/mdia/minf/stbl/stsc            : sample-to-chunk map
 *
 * Then walks samples 1..stss[0] to compute the first keyframe's file
 * offset, reads its bytes, demuxes length-prefixed NALs into Annex-B.
 *
 * Usage:
 *   mp4_extract --in input.mp4 --out first_idr.h264
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <string>

static FILE *g_f = nullptr;
static long  g_size = 0;

static uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3];
}
static uint64_t rd_u64(const uint8_t *p) {
    return ((uint64_t)rd_u32(p) << 32) | rd_u32(p + 4);
}
static uint16_t rd_u16(const uint8_t *p) {
    return (uint16_t)((p[0]<<8) | p[1]);
}

static int read_at(uint64_t off, void *buf, size_t n) {
    if (_fseeki64(g_f, (int64_t)off, SEEK_SET) != 0) return 1;
    return fread(buf, 1, n, g_f) == n ? 0 : 1;
}

/* Parse one box header at file offset `off`.  Returns the body offset and
 * total size (header+body).  64-bit largesize handled. */
struct Box {
    uint32_t type;
    uint64_t size;       /* incl header */
    uint64_t body_off;   /* first byte after header */
    uint64_t body_size;
};
static int read_box(uint64_t off, Box *b) {
    uint8_t hdr[16];
    if (read_at(off, hdr, 8) != 0) return 1;
    uint32_t s = rd_u32(hdr);
    b->type = rd_u32(hdr + 4);
    if (s == 1) {
        if (read_at(off + 8, hdr + 8, 8) != 0) return 1;
        b->size = rd_u64(hdr + 8);
        b->body_off = off + 16;
    } else {
        b->size = s;
        b->body_off = off + 8;
    }
    b->body_size = b->size - (b->body_off - off);
    return 0;
}

#define FOURCC(a,b,c,d) (((uint32_t)(a)<<24)|((uint32_t)(b)<<16)|((uint32_t)(c)<<8)|(d))

static std::vector<uint8_t> g_avcc;          /* full avcC body */
static uint8_t              g_nal_lensize = 4;
static uint16_t             g_width = 0, g_height = 0;
static std::vector<uint32_t> g_stsz;         /* sample sizes (1-indexed conceptually) */
static std::vector<uint64_t> g_chunk_off;    /* stco / co64 */
struct StscRun { uint32_t first_chunk; uint32_t samples_per_chunk; };
static std::vector<StscRun>  g_stsc;
static std::vector<uint32_t> g_stss;         /* sync sample indices, 1-based */
static int g_in_video_trak = 0;

static int parse_avcC(uint64_t off, uint64_t sz) {
    g_avcc.resize(sz);
    return read_at(off, g_avcc.data(), sz);
}

static int parse_stsd(uint64_t off, uint64_t sz) {
    /* full-box: 1 byte version + 3 bytes flags + 4 bytes count + entries */
    std::vector<uint8_t> body(sz);
    if (read_at(off, body.data(), sz) != 0) return 1;
    uint32_t count = rd_u32(body.data() + 4);
    uint64_t p = 8;
    for (uint32_t i = 0; i < count && p + 8 <= sz; i++) {
        uint32_t entry_size = rd_u32(body.data() + p);
        uint32_t entry_type = rd_u32(body.data() + p + 4);
        if (entry_type == FOURCC('a','v','c','1') ||
            entry_type == FOURCC('a','v','c','3')) {
            /* SampleEntry: 6 reserved + 2 dataref → 8 bytes
             * VisualSampleEntry: pre_defined(2)+reserved(2)+pre_defined(12)
             *   = 16 bytes  → total 24
             * width(2) height(2) horizres(4) vertres(4) reserved(4)
             *   frame_count(2) compressorname(32) depth(2) pre_defined(2)
             *   = 56 bytes → total 80
             * children boxes follow at offset 80 from entry start. */
            const uint8_t *e = body.data() + p;
            g_width  = rd_u16(e + 8 + 16 + 8);
            g_height = rd_u16(e + 8 + 16 + 10);
            uint64_t child_off = p + 8 + 16 + 8 + 8 + 32 + 2 + 2 + 4;
            /* recompute via known offsets: 86 is the standard offset */
            child_off = p + 86;
            while (child_off + 8 <= p + entry_size) {
                uint32_t cs = rd_u32(body.data() + child_off);
                uint32_t ct = rd_u32(body.data() + child_off + 4);
                if (ct == FOURCC('a','v','c','C')) {
                    g_avcc.assign(body.data() + child_off + 8,
                                  body.data() + child_off + cs);
                    g_nal_lensize = (g_avcc[4] & 0x03) + 1;
                }
                if (cs == 0) break;
                child_off += cs;
            }
        }
        p += entry_size;
    }
    return 0;
}

static int parse_stsz(uint64_t off, uint64_t sz) {
    std::vector<uint8_t> body(sz);
    if (read_at(off, body.data(), sz) != 0) return 1;
    uint32_t sample_size  = rd_u32(body.data() + 4);
    uint32_t sample_count = rd_u32(body.data() + 8);
    g_stsz.resize(sample_count);
    if (sample_size == 0) {
        for (uint32_t i = 0; i < sample_count; i++)
            g_stsz[i] = rd_u32(body.data() + 12 + i * 4);
    } else {
        for (uint32_t i = 0; i < sample_count; i++) g_stsz[i] = sample_size;
    }
    return 0;
}

static int parse_stco(uint64_t off, uint64_t sz, int is_co64) {
    std::vector<uint8_t> body(sz);
    if (read_at(off, body.data(), sz) != 0) return 1;
    uint32_t count = rd_u32(body.data() + 4);
    g_chunk_off.resize(count);
    for (uint32_t i = 0; i < count; i++) {
        if (is_co64) g_chunk_off[i] = rd_u64(body.data() + 8 + i * 8);
        else         g_chunk_off[i] = rd_u32(body.data() + 8 + i * 4);
    }
    return 0;
}

static int parse_stsc(uint64_t off, uint64_t sz) {
    std::vector<uint8_t> body(sz);
    if (read_at(off, body.data(), sz) != 0) return 1;
    uint32_t count = rd_u32(body.data() + 4);
    g_stsc.resize(count);
    for (uint32_t i = 0; i < count; i++) {
        g_stsc[i].first_chunk      = rd_u32(body.data() + 8 + i * 12 + 0);
        g_stsc[i].samples_per_chunk= rd_u32(body.data() + 8 + i * 12 + 4);
    }
    return 0;
}

static int parse_stss(uint64_t off, uint64_t sz) {
    std::vector<uint8_t> body(sz);
    if (read_at(off, body.data(), sz) != 0) return 1;
    uint32_t count = rd_u32(body.data() + 4);
    g_stss.resize(count);
    for (uint32_t i = 0; i < count; i++)
        g_stss[i] = rd_u32(body.data() + 8 + i * 4);
    return 0;
}

static int parse_hdlr(uint64_t off, uint64_t sz, int *is_video) {
    /* full-box(4) + pre_defined(4) + handler_type(4) + reserved(12) + name */
    if (sz < 24) { *is_video = 0; return 0; }
    std::vector<uint8_t> body(sz);
    if (read_at(off, body.data(), sz) != 0) return 1;
    uint32_t handler = rd_u32(body.data() + 8);
    *is_video = (handler == FOURCC('v','i','d','e'));
    return 0;
}

/* Recursively walks containers. */
static int walk(uint64_t off, uint64_t end, int depth) {
    while (off + 8 <= end) {
        Box b;
        if (read_box(off, &b) != 0) return 1;
        if (b.size < 8 || b.body_off + b.body_size > end) return 1;

        switch (b.type) {
        case FOURCC('m','o','o','v'):
        case FOURCC('m','d','i','a'):
        case FOURCC('m','i','n','f'):
        case FOURCC('s','t','b','l'):
            walk(b.body_off, b.body_off + b.body_size, depth + 1);
            break;
        case FOURCC('t','r','a','k'): {
            int saved = g_in_video_trak;
            g_in_video_trak = 0;
            walk(b.body_off, b.body_off + b.body_size, depth + 1);
            g_in_video_trak = saved;
            break;
        }
        case FOURCC('h','d','l','r'): {
            int v = 0;
            parse_hdlr(b.body_off, b.body_size, &v);
            if (v) g_in_video_trak = 1;
            break;
        }
        case FOURCC('s','t','s','d'):
            if (g_in_video_trak) parse_stsd(b.body_off, b.body_size);
            break;
        case FOURCC('s','t','s','z'):
            if (g_in_video_trak) parse_stsz(b.body_off, b.body_size);
            break;
        case FOURCC('s','t','c','o'):
            if (g_in_video_trak) parse_stco(b.body_off, b.body_size, 0);
            break;
        case FOURCC('c','o','6','4'):
            if (g_in_video_trak) parse_stco(b.body_off, b.body_size, 1);
            break;
        case FOURCC('s','t','s','c'):
            if (g_in_video_trak) parse_stsc(b.body_off, b.body_size);
            break;
        case FOURCC('s','t','s','s'):
            if (g_in_video_trak) parse_stss(b.body_off, b.body_size);
            break;
        default:
            break;
        }
        off = b.body_off + b.body_size;
    }
    return 0;
}

/* Compute file offset of the 1-based sample number `sample`, given the
 * stsc/stco arrays.  Walks samples sequentially — fine for first
 * keyframe at sample 1 or near it. */
static uint64_t sample_file_offset(uint32_t sample) {
    /* Determine which chunk this sample lives in, and its index inside
     * that chunk.  stsc compresses runs: entry i's first_chunk says at
     * which chunk the run begins; the run continues until the next
     * entry. */
    uint64_t accumulated = 0;     /* count of samples in earlier chunks */
    uint32_t chunk_idx_base = 1;  /* 1-based */
    for (size_t i = 0; i < g_stsc.size(); i++) {
        uint32_t first = g_stsc[i].first_chunk;       /* 1-based */
        uint32_t spc   = g_stsc[i].samples_per_chunk;
        uint32_t next  = (i + 1 < g_stsc.size()) ? g_stsc[i + 1].first_chunk
                                                  : (uint32_t)g_chunk_off.size() + 1;
        for (uint32_t c = first; c < next; c++) {
            if (sample <= accumulated + spc) {
                /* In this chunk c, at index (sample - accumulated - 1). */
                uint32_t in_chunk = sample - (uint32_t)accumulated - 1;
                uint64_t off = g_chunk_off[c - 1];
                for (uint32_t k = 0; k < in_chunk; k++) {
                    uint32_t idx = (uint32_t)accumulated + k;
                    if (idx < g_stsz.size()) off += g_stsz[idx];
                }
                return off;
            }
            accumulated += spc;
        }
        chunk_idx_base = next;
    }
    return UINT64_MAX;
}

int wmain(int argc, wchar_t **argv) {
    const wchar_t *in_path = nullptr, *out_path = nullptr;
    for (int i = 1; i < argc; i++) {
        if (!wcscmp(argv[i], L"--in")  && i + 1 < argc) in_path  = argv[++i];
        else if (!wcscmp(argv[i], L"--out") && i + 1 < argc) out_path = argv[++i];
    }
    if (!in_path || !out_path) {
        std::fprintf(stderr, "usage: mp4_extract --in input.mp4 --out output.h264\n");
        return 1;
    }
    if (_wfopen_s(&g_f, in_path, L"rb") != 0 || !g_f) {
        std::fprintf(stderr, "open input failed\n"); return 2;
    }
    _fseeki64(g_f, 0, SEEK_END);
    uint64_t fsize = _ftelli64(g_f);
    walk(0, fsize, 0);

    if (g_avcc.empty() || g_stsz.empty() || g_chunk_off.empty() ||
        g_stsc.empty() || g_stss.empty()) {
        std::fprintf(stderr, "incomplete MP4 metadata "
            "(avcC=%zu stsz=%zu stco=%zu stsc=%zu stss=%zu)\n",
            g_avcc.size(), g_stsz.size(), g_chunk_off.size(),
            g_stsc.size(), g_stss.size());
        return 3;
    }
    std::printf("video: %ux%u  lengthSize=%u  total samples=%zu  keyframes=%zu\n",
                (unsigned)g_width, (unsigned)g_height,
                (unsigned)g_nal_lensize, g_stsz.size(), g_stss.size());

    /* avcC parameter-set layout:
     *   1 configurationVersion
     *   1 AVCProfileIndication
     *   1 profile_compatibility
     *   1 AVCLevelIndication
     *   1 (reserved 6 bits | 2 bits lengthSizeMinusOne)
     *   1 (reserved 3 bits | 5 bits numOfSPS)
     *   for each SPS: 2 bytes length + SPS bytes
     *   1 numOfPPS
     *   for each PPS: 2 bytes length + PPS bytes
     */
    const uint8_t start_code[4] = { 0, 0, 0, 1 };
    FILE *o = nullptr;
    if (_wfopen_s(&o, out_path, L"wb") != 0 || !o) {
        std::fprintf(stderr, "open output failed\n"); return 4;
    }

    size_t p = 5;     /* lengthSizeMinusOne byte ends at index 4 */
    if (g_avcc[p] & 0x1F) {  /* numOfSPS in low 5 bits */
        p += 1;
        for (int i = 0; i < (g_avcc[5] & 0x1F); i++) {
            uint16_t len = rd_u16(&g_avcc[p]); p += 2;
            std::fwrite(start_code, 1, 4, o);
            std::fwrite(&g_avcc[p], 1, len, o);
            p += len;
        }
    }
    int numPps = g_avcc[p++];
    for (int i = 0; i < numPps; i++) {
        uint16_t len = rd_u16(&g_avcc[p]); p += 2;
        std::fwrite(start_code, 1, 4, o);
        std::fwrite(&g_avcc[p], 1, len, o);
        p += len;
    }

    /* First keyframe sample. */
    uint32_t first_idr = g_stss[0];   /* 1-based */
    uint64_t off = sample_file_offset(first_idr);
    uint32_t sz  = g_stsz[first_idr - 1];
    std::printf("first IDR: sample %u  size %u  off 0x%llx\n",
                first_idr, sz, (unsigned long long)off);

    std::vector<uint8_t> sample(sz);
    if (read_at(off, sample.data(), sz) != 0) {
        std::fprintf(stderr, "read sample failed\n"); return 5;
    }

    /* Demux length-prefixed NALs → Annex-B. */
    size_t q = 0;
    while (q + g_nal_lensize <= sample.size()) {
        uint32_t nlen = 0;
        for (int i = 0; i < g_nal_lensize; i++)
            nlen = (nlen << 8) | sample[q + i];
        q += g_nal_lensize;
        if (q + nlen > sample.size()) break;
        std::fwrite(start_code, 1, 4, o);
        std::fwrite(&sample[q], 1, nlen, o);
        q += nlen;
    }
    std::fclose(o);
    std::fclose(g_f);
    std::printf("wrote %ls\n", out_path);
    return 0;
}
