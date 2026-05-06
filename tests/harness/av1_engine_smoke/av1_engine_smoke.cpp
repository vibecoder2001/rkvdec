/* tests/harness/av1_engine_smoke — drive Av1DecodeEngine over an IVF file.
 *
 * Verifies the user-mode AV1 decode pipeline (dav1d → regbuilder_av1 →
 * RkmppAv1Dpb → NV12 output) works on the dev machine without hardware.
 * Software mode produces dav1d's NV12 output as the engine's frames; this
 * doubles as the ground-truth oracle for the future hardware-mode bring-up.
 *
 * Usage: av1_engine_smoke <file.ivf> [--dump-yuv <prefix>]
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
/* Instantiate GUID_DEVINTERFACE_RKMPP exactly once (referenced by the
 * device-open path in decode_engine_av1.cpp). */
#include <windows.h>
#include <initguid.h>
#include "../../../shared/rkmpp_ioctl.h"

#include "../rkmpp_decode/decode_engine_av1.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

static bool slurp(const char *p, std::vector<uint8_t> &out) {
    FILE *f = std::fopen(p, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    out.resize(n);
    bool ok = std::fread(out.data(), 1, n, f) == (size_t)n;
    std::fclose(f);
    return ok;
}

static int next_ivf(const uint8_t *b, size_t l, size_t &pos,
                    const uint8_t *&out, size_t &out_len) {
    if (pos + 12 > l) return 0;
    uint32_t sz = (uint32_t)b[pos] | ((uint32_t)b[pos+1] << 8) |
                  ((uint32_t)b[pos+2] << 16) | ((uint32_t)b[pos+3] << 24);
    pos += 12;
    if (pos + sz > l) return 0;
    out = b + pos; out_len = sz; pos += sz;
    return 1;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr,
            "usage: %s <file.ivf> [--dump-yuv <prefix>]\n", argv[0]);
        return 2;
    }
    const char *ivf_path = argv[1];
    const char *yuv_prefix = nullptr;
    for (int i = 2; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--dump-yuv")) yuv_prefix = argv[i + 1];
    }

    std::vector<uint8_t> buf;
    if (!slurp(ivf_path, buf) || buf.size() < 32 ||
        std::memcmp(buf.data(), "DKIF", 4) != 0) {
        std::fprintf(stderr, "not a valid IVF file\n");
        return 1;
    }
    /* IVF header at bytes 12..14 = width, 14..16 = height. */
    uint16_t W = (uint16_t)(buf[12] | (buf[13] << 8));
    uint16_t H = (uint16_t)(buf[14] | (buf[15] << 8));

    Av1DecodeEngine eng;
    if (Av1DecodeEngine_Init(&eng, Av1EngineMode::Software, W, H) != 0) {
        std::fprintf(stderr, "engine init failed\n");
        return 1;
    }

    size_t pos = 32;
    int submitted = 0, polled = 0;
    while (pos < buf.size()) {
        const uint8_t *fb; size_t fsz;
        if (!next_ivf(buf.data(), buf.size(), pos, fb, fsz)) break;
        if (Av1DecodeEngine_Submit(&eng, fb, fsz, /*pts=*/-1) != 0) {
            std::fprintf(stderr, "submit failed at au #%d\n", submitted);
            break;
        }
        submitted++;

        Av1DecodedFrame f;
        while (Av1DecodeEngine_PollFrame(&eng, &f)) {
            std::printf("frame %d: %ux%u, %zu yuv bytes, pts=%lld%s\n",
                        polled, f.width, f.height, f.yuv.size(),
                        (long long)f.pts_hns,
                        f.has_film_grain ? " (film-grain)" : "");
            if (yuv_prefix) {
                char path[512];
                std::snprintf(path, sizeof(path),
                              "%s_%04d_%ux%u.nv12",
                              yuv_prefix, polled, f.width, f.height);
                FILE *fp = std::fopen(path, "wb");
                if (fp) {
                    std::fwrite(f.yuv.data(), 1, f.yuv.size(), fp);
                    std::fclose(fp);
                }
            }
            Av1DecodeEngine_ReleaseFrame(&eng, &f);
            polled++;
        }
    }
    Av1DecodeEngine_Drain(&eng);
    Av1DecodedFrame f;
    while (Av1DecodeEngine_PollFrame(&eng, &f)) {
        std::printf("frame %d (drain): %ux%u, %zu yuv bytes\n",
                    polled, f.width, f.height, f.yuv.size());
        Av1DecodeEngine_ReleaseFrame(&eng, &f);
        polled++;
    }
    std::fprintf(stderr,
                 "av1_engine_smoke: submitted=%d polled=%d\n",
                 submitted, polled);
    Av1DecodeEngine_Shutdown(&eng);
    return polled > 0 ? 0 : 1;
}
