/* linux_mpp_replay.cpp — replay an mpv-captured input dump through the
 * shared DecodeEngine on Linux/rk.
 *
 *   linux_mpp_replay <mft_input_dump.bin> [--frames N]
 *
 * Consumes the dump file produced by DecoderMFT::DumpAuIfActive (format
 * documented in mft/dll/decoder_mft.cpp at that function).  Initializes
 * DecodeEngine with the captured codec/width/height/bit_depth, then
 * feeds each captured AU through DecodeEngine_SubmitFramed using the
 * same framing/length_size mpv submitted with.  Pops display-order
 * NV12 frames via DecodeEngine_PollFrame, releases them immediately
 * (mirrors the synchronous ProcessOutput→ReleaseFrame on the Windows
 * MFT path; doesn't simulate EVR's render-hold since we already
 * confirmed sample-hold timing isn't the wedge variable).
 *
 * Reports a clean tail line "Done: submitted=N decoded=N failures=F"
 * — exit code 0 if no decode failures, 1 otherwise.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "winshim.h"
#include "mft/engine/decode_engine.h"
#include "mft/engine/backend_linux.h"
#include "shared/rkmpp_ioctl.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <climits>
#include <string>
#include <vector>

/* Dump format constants — keep in sync with decoder_mft.cpp. */
static constexpr uint32_t kFileMagic = 0x504B4452;  /* "RKDP" little-endian */
static constexpr uint32_t kAuMagic   = 0x55414B52;  /* "RKAU" little-endian */
static constexpr uint32_t kVersion   = 1;

/* Mirror of the in-DLL CodecKind enum used as `codec_kind` in the dump. */
enum DumpCodecKind : uint32_t {
    DUMP_KIND_H264 = 0,
    DUMP_KIND_HEVC = 1,
    DUMP_KIND_AV1  = 2,
    DUMP_KIND_VP9  = 3,
};

struct DumpReader {
    const uint8_t *p;
    const uint8_t *end;

    bool read_u32(uint32_t *v) {
        if (end - p < 4) return false;
        memcpy(v, p, 4); p += 4; return true;
    }
    bool read_u64(uint64_t *v) {
        if (end - p < 8) return false;
        memcpy(v, p, 8); p += 8; return true;
    }
    bool read_i64(int64_t *v) {
        if (end - p < 8) return false;
        memcpy(v, p, 8); p += 8; return true;
    }
    bool read_bytes(size_t n, std::vector<uint8_t> *out) {
        if ((size_t)(end - p) < n) return false;
        out->assign(p, p + n); p += n; return true;
    }
    bool skip(size_t n) {
        if ((size_t)(end - p) < n) return false;
        p += n; return true;
    }
};

static const char *codec_kind_str(uint32_t k) {
    switch (k) {
    case DUMP_KIND_H264: return "H264";
    case DUMP_KIND_HEVC: return "HEVC";
    case DUMP_KIND_AV1:  return "AV1";
    case DUMP_KIND_VP9:  return "VP9";
    default:             return "?";
    }
}

static uint8_t *read_file(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return nullptr; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t *)malloc((size_t)n);
    if (!buf) { fclose(f); return nullptr; }
    size_t got = fread(buf, 1, (size_t)n, f);
    fclose(f);
    if (got != (size_t)n) { free(buf); return nullptr; }
    *out_len = (size_t)n;
    return buf;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <mft_input_dump.bin> [--frames N]\n", argv[0]);
        return 1;
    }
    int max_frames = INT_MAX;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            max_frames = atoi(argv[++i]);
    }

    size_t bin_len = 0;
    uint8_t *bin = read_file(argv[1], &bin_len);
    if (!bin) return 1;

    DumpReader r{bin, bin + bin_len};

    /* --- Parse file header. --- */
    uint32_t magic = 0, version = 0, kind = 0, length_size = 0;
    uint32_t width = 0, height = 0, fps_num = 0, fps_den = 0, bit_depth = 0;
    uint32_t extradata_len = 0;
    if (!r.read_u32(&magic) || magic != kFileMagic) {
        fprintf(stderr, "bad file magic 0x%08x (expected 0x%08x)\n",
                magic, kFileMagic);
        free(bin); return 1;
    }
    if (!r.read_u32(&version) || version != kVersion) {
        fprintf(stderr, "unsupported version %u\n", version); free(bin); return 1;
    }
    if (!r.read_u32(&kind)        ||
        !r.read_u32(&length_size) ||
        !r.read_u32(&width)       ||
        !r.read_u32(&height)      ||
        !r.read_u32(&fps_num)     ||
        !r.read_u32(&fps_den)     ||
        !r.read_u32(&bit_depth)   ||
        !r.read_u32(&extradata_len)) {
        fprintf(stderr, "truncated file header\n"); free(bin); return 1;
    }
    /* Capture extradata bytes (already Annex-B in the dump format) so we
     * can FeedExtradata into the engine before any slice — without this
     * the parser has no SPS/PPS when the first slice arrives. */
    std::vector<uint8_t> extradata_bytes;
    if (extradata_len > 0) {
        if (!r.read_bytes(extradata_len, &extradata_bytes)) {
            fprintf(stderr, "truncated extradata (%u bytes)\n", extradata_len);
            free(bin); return 1;
        }
    }

    fprintf(stderr,
        "replay: codec=%s width=%u height=%u length_size=%u bit_depth=%u "
        "fps=%u/%u extradata=%u bytes\n",
        codec_kind_str(kind), width, height, length_size, bit_depth,
        fps_num, fps_den, extradata_len);

    if (kind != DUMP_KIND_H264 && kind != DUMP_KIND_HEVC) {
        fprintf(stderr, "replay: only H.264/H.265 supported via DecodeEngine "
                        "(kind=%s); AV1/VP9 would need separate harnesses\n",
                codec_kind_str(kind));
        free(bin); return 2;
    }

    /* Map dump's length_size to NalFraming + AvccLenSize.  The NalFraming
     * enum only distinguishes ANNEXB vs AVCC; the length-field width
     * (1/2/4 byte) is carried by AvccLenSize when framing == AVCC. */
    NalFraming  framing = FRAMING_ANNEXB;
    AvccLenSize ls      = AVCC_LEN_4;
    if (length_size == 0)            { framing = FRAMING_ANNEXB; }
    else if (length_size == 1)       { framing = FRAMING_AVCC4;  ls = AVCC_LEN_1; }
    else if (length_size == 2)       { framing = FRAMING_AVCC4;  ls = AVCC_LEN_2; }
    else if (length_size == 4)       { framing = FRAMING_AVCC4;  ls = AVCC_LEN_4; }
    else {
        fprintf(stderr, "bad length_size=%u\n", length_size);
        free(bin); return 1;
    }

    /* --- Init DecodeEngine over LinuxBackend. --- */
    DecodeEngine eng{};
    DecodeEngineBackend *be = LinuxBackend_New(width, height);
    if (!be) { fprintf(stderr, "LinuxBackend_New OOM\n"); free(bin); return 1; }
    Codec codec = (kind == DUMP_KIND_H264) ? Codec::H264 : Codec::H265;
    if (DecodeEngine_InitWithBackend(&eng, be, codec, width, height) != 0) {
        fprintf(stderr, "DecodeEngine_InitWithBackend failed\n");
        LinuxBackend_Free(be); free(bin); return 1;
    }

    /* Feed extradata first — extradata is already Annex-B in the dump
     * format, so framing=ANNEXB regardless of the per-AU length_size.
     * Mirrors how the Windows MFT parses extradata in SetInputType. */
    if (!extradata_bytes.empty()) {
        int rc = DecodeEngine_FeedExtradata(&eng, FRAMING_ANNEXB, AVCC_LEN_4,
                                            extradata_bytes.data(),
                                            extradata_bytes.size());
        if (rc != 0) {
            fprintf(stderr, "replay: FeedExtradata failed rc=%d (extradata "
                            "may be malformed)\n", rc);
        }
    }

    int submitted = 0, decoded = 0, failures = 0;
    auto pop_ready = [&]() {
        DecodedFrame f;
        while (DecodeEngine_PollFrame(&eng, &f) > 0) {
            DecodeEngine_ReleaseFrame(&eng, &f);
            decoded++;
        }
    };

    /* --- Replay every AU record. --- */
    int rec_idx = 0;
    while (r.p < r.end && submitted < max_frames) {
        uint32_t rec_magic = 0;
        if (!r.read_u32(&rec_magic)) break;
        if (rec_magic != kAuMagic) {
            fprintf(stderr,
                "replay: bad AU magic 0x%08x at byte %zu (rec %d)\n",
                rec_magic, (size_t)(r.p - 4 - bin), rec_idx);
            failures++;
            break;
        }
        uint64_t sample_no = 0; int64_t pts_hns = 0; uint32_t size = 0;
        if (!r.read_u64(&sample_no) || !r.read_i64(&pts_hns) ||
            !r.read_u32(&size)) {
            fprintf(stderr, "truncated AU header at rec %d\n", rec_idx);
            failures++;
            break;
        }
        if ((size_t)(r.end - r.p) < size) {
            fprintf(stderr,
                "truncated AU body (need %u, have %zu) at rec %d\n",
                size, (size_t)(r.end - r.p), rec_idx);
            failures++;
            break;
        }
        const uint8_t *au_bytes = r.p;
        r.p += size;

        int rc = DecodeEngine_SubmitFramed(&eng, framing, ls,
                                           au_bytes, size, pts_hns,
                                           /*epoch=*/0);
        if (rc != 0) {
            failures++;
            if (failures <= 8 || (failures % 100) == 0) {
                fprintf(stderr,
                    "replay: SubmitFramed failed rc=%d at sample_no=%llu "
                    "rec=%d size=%u\n",
                    rc, (unsigned long long)sample_no, rec_idx, size);
            }
        }
        submitted++;
        rec_idx++;
        pop_ready();
    }
    DecodeEngine_Drain(&eng);
    pop_ready();

    fprintf(stderr,
        "Done: submitted=%d decoded=%d failures=%d\n",
        submitted, decoded, failures);

    DecodeEngine_Shutdown(&eng);
    LinuxBackend_Free(be);
    free(bin);
    return failures == 0 ? 0 : 1;
}
