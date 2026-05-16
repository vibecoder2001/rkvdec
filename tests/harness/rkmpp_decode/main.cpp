/* tests/harness/rkmpp_decode/main.cpp
 *
 * H.264 / H.265 decode harness:
 *   rkmpp_decode --in stream.h264 --width W --height H [--out out.yuv]
 *   rkmpp_decode --codec h265 --in stream.h265 --width W --height H [--out]
 *
 * Reads an Annex-B-framed bitstream, splits into access units (one
 * VCL slice NAL each, prefixed by any VPS/SPS/PPS/SEI), and feeds each
 * AU to DecodeEngine.  Concatenates the decoded NV12 frames to --out.
 */
#include <windows.h>
#include <initguid.h>
#include "decode_engine.h"
#include "au_iter.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>

static int Usage() {
    std::fprintf(stderr,
        "usage: rkmpp_decode [--codec h264|h265] --in <file> "
        "--width <px> --height <px> [--out <file.yuv>]\n");
    return 1;
}

/* AU walker: thin pos-cursor adapter over mft/au_iter. */
static bool au_next(Codec codec, const uint8_t *buf, size_t len, size_t *pos,
                    size_t *au_off, size_t *au_len) {
    AuIter it = { buf, len, *pos };
    int ok = (codec == Codec::H265)
                ? H265AuNext(&it, au_off, au_len, nullptr)
                : H264AuNext(&it, au_off, au_len, nullptr);
    *pos = it.pos;
    return ok != 0;
}

int wmain(int argc, wchar_t **argv) {
    const wchar_t *in_path = nullptr, *out_path = nullptr;
    uint32_t width = 0, height = 0;
    Codec codec = Codec::H264;
    for (int i = 1; i < argc; i++) {
        if (!wcscmp(argv[i], L"--in")     && i + 1 < argc) in_path  = argv[++i];
        else if (!wcscmp(argv[i], L"--out")    && i + 1 < argc) out_path = argv[++i];
        else if (!wcscmp(argv[i], L"--width")  && i + 1 < argc) width    = _wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--height") && i + 1 < argc) height   = _wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--codec")  && i + 1 < argc) {
            const wchar_t *c = argv[++i];
            if (!wcscmp(c, L"h264") || !wcscmp(c, L"H264")) codec = Codec::H264;
            else if (!wcscmp(c, L"h265") || !wcscmp(c, L"H265") ||
                     !wcscmp(c, L"hevc") || !wcscmp(c, L"HEVC")) codec = Codec::H265;
            else return Usage();
        }
        else return Usage();
    }
    if (!in_path || !width || !height) return Usage();

    /* Read the bitstream file. */
    FILE *f = nullptr;
    if (_wfopen_s(&f, in_path, L"rb") != 0 || !f) {
        std::fprintf(stderr, "open input failed\n");
        return 2;
    }
    std::fseek(f, 0, SEEK_END);
    long len = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> bs(len);
    std::fread(bs.data(), 1, len, f);
    std::fclose(f);
    std::printf("input: %ls (%ld bytes) codec=%s\n", in_path, len,
                codec == Codec::H265 ? "H265" : "H264");

    DecodeEngine eng;
    if (DecodeEngine_Init(&eng, codec, width, height) != 0) {
        std::fprintf(stderr, "engine init failed\n");
        return 3;
    }

    FILE *o = nullptr;
    if (out_path) {
        if (_wfopen_s(&o, out_path, L"wb") != 0 || !o) {
            std::fprintf(stderr, "open output failed\n");
            DecodeEngine_Shutdown(&eng);
            return 5;
        }
    }

    size_t pos = 0;
    int submitted = 0;
    int emitted = 0;
    auto drain_ready = [&]() {
        DecodedFrame f;
        while (DecodeEngine_PollFrame(&eng, &f) > 0) {
            std::printf("emit frame %d: POC=%d pts=%lld bytes=%zu\n",
                        emitted, f.poc, (long long)f.pts_hns, f.yuv.size());
            if (o && !f.yuv.empty()) {
                std::fwrite(f.yuv.data(), 1, f.yuv.size(), o);
            }
            emitted++;
            /* Release the DPB CONSUMER hold the engine took for this frame.
             * Without this, the pool fills up after kPoolSize frames and
             * Dpb_Select returns DPB_FULL.  See dpb.h Dpb_AddHold rationale. */
            DecodeEngine_ReleaseFrame(&eng, &f);
        }
    };
    while (true) {
        size_t au_off, au_len;
        if (!au_next(codec, bs.data(), bs.size(), &pos, &au_off, &au_len)) break;
        std::printf("AU %d: off=%zu len=%zu\n", submitted, au_off, au_len);

        int rc = DecodeEngine_Submit(&eng, bs.data() + au_off, au_len, -1);
        if (rc != 0) {
            std::fprintf(stderr, "Submit failed at AU %d\n", submitted);
            break;
        }
        submitted++;
        drain_ready();
    }
    /* End of stream → drain reorder window in POC order. */
    DecodeEngine_Drain(&eng);
    drain_ready();
    std::printf("submitted %d AUs, emitted %d frames in display order\n",
                submitted, emitted);

    if (o) std::fclose(o);
    DecodeEngine_Shutdown(&eng);
    return 0;
}
