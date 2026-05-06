/* av1_hw_inject — drive the rkvdec_av1 hardware via our regbuilder.
 *
 * Reads an IVF file, runs each frame through our parser+regbuilder,
 * and writes the resulting per-frame register array to the kernel's
 * AV1SHIM debugfs override interface.  Then triggers ffmpeg-rkmpp
 * to do an actual hardware decode (which the kernel patches by
 * applying our overrides AFTER its standard fill).  Compares output
 * YUV against dav1d's bit-exact reference.
 *
 * Requires the patched kernel (mpp_av1dec.c with AV1SHIM debugfs
 * hooks) to be loaded.
 *
 * Usage: av1_hw_inject <file.ivf> [override_kick_idx]
 *   override_kick_idx — which frame's regbuilder output to inject
 *                       (default 0 = first inter-frame after key)
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

extern "C" {
#include <dav1d/dav1d.h>
}

#include "regbuilder_av1.h"

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

static int write_blob(const char *path, const void *data, size_t len) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) { perror(path); return -1; }
    ssize_t n = write(fd, data, len);
    close(fd);
    if (n != (ssize_t)len) { fprintf(stderr, "short write to %s: %zd/%zu\n", path, n, len); return -1; }
    return 0;
}

static int write_text(const char *path, const char *s) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) { perror(path); return -1; }
    ssize_t n = write(fd, s, strlen(s));
    close(fd);
    return (int)n;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.ivf> [override_kick_idx]\n", argv[0]);
        return 2;
    }
    int target_kick = (argc >= 3) ? atoi(argv[2]) : 0;

    std::vector<uint8_t> buf;
    if (!slurp(argv[1], buf) || buf.size() < 32 ||
        std::memcmp(buf.data(), "DKIF", 4) != 0) {
        std::fprintf(stderr, "not a valid IVF\n");
        return 1;
    }

    Dav1dContext  *c = nullptr;
    Dav1dSettings  s;
    dav1d_default_settings(&s);
#if defined(DAV1D_API_VERSION_MAJOR) && DAV1D_API_VERSION_MAJOR >= 6
    s.n_threads = 1;
    s.max_frame_delay = 1;
#else
    s.n_frame_threads = 1;
    s.n_tile_threads = 1;
    s.n_postfilter_threads = 1;
#endif
    if (dav1d_open(&c, &s) != 0) { std::fprintf(stderr, "dav1d_open failed\n"); return 1; }

    RkmppAv1Buffers bufs{};
    bufs.output_y_fd = 0; bufs.bitstream_fd = 0;

    RkmppAv1Dpb dpb;
    rkmpp_av1_dpb_init(&dpb);

    size_t pos = 32;
    int kick_idx = -1;     /* -1 because we skip the first (keyframe) */
    bool injected = false;

    auto drain = [&]() -> bool {
        for (;;) {
            Dav1dPicture p{};
            int r = dav1d_get_picture(c, &p);
            if (r != 0) return injected;
            kick_idx++;
            if (kick_idx == 0) {
                /* keyframe — DPB updates only, skip injection */
                rkmpp_av1_dpb_post_decode(&dpb, p.seq_hdr, p.frame_hdr);
                dav1d_picture_unref(&p);
                continue;
            }
            if (kick_idx == target_kick + 1) {
                /* This is the frame we want to inject (kick_idx is 1-based
                 * from inter frames, so target_kick=0 → kick_idx=1). */
                VdpuAv1dRegSet regs{};
                std::memset(&regs, 0, sizeof(regs));
                rkmpp_av1_build_regs(p.seq_hdr, p.frame_hdr, &dpb, &bufs, &regs);

                /* Build (val, mask) blobs.  We inject ONLY swregs that
                 * we explicitly set (non-zero in our output).  This
                 * lets the kernel use BSP's value for everything else. */
                const uint32_t *r32 = reinterpret_cast<const uint32_t *>(&regs);
                std::vector<uint32_t> val(512, 0);
                std::vector<uint32_t> mask(16, 0);
                size_t total = sizeof(regs) / sizeof(uint32_t);
                if (total > 512) total = 512;
                for (size_t i = 1; i < total; i++) {  /* skip swreg0 = version reg */
                    if (r32[i]) {
                        val[i]            = r32[i];
                        mask[i / 32]     |= (1u << (i % 32));
                    }
                }
                /* Skip the keyframe (kick 0) — our reg array is built
                 * for the first inter frame, so applying it to the
                 * keyframe corrupts the reference frame.  Skip count =
                 * target_kick (0 by default = skip 1 kick = the
                 * keyframe; override fires on kick 1 = first inter). */
                /* Kernel kicks index from 0 (keyframe).  To override the
                 * first inter (target_kick=0), kernel must skip 1 kick. */
                char skip_buf[16];
                snprintf(skip_buf, sizeof(skip_buf), "%d", target_kick + 1);
                if (write_blob("/sys/kernel/debug/av1shim/regs", val.data(), 512 * 4) ||
                    write_blob("/sys/kernel/debug/av1shim/mask", mask.data(), 16 * 4) ||
                    write_text("/sys/kernel/debug/av1shim/skip", skip_buf) < 0 ||
                    write_text("/sys/kernel/debug/av1shim/enable", "Y") < 0) {
                    fprintf(stderr, "failed to program AV1SHIM debugfs\n");
                    dav1d_picture_unref(&p);
                    return false;
                }
                /* Count nonzero overrides for log */
                int nz = 0;
                for (size_t i = 0; i < 512; i++)
                    if (val[i]) nz++;
                fprintf(stderr, "av1_hw_inject: programmed %d overrides for kick_idx=%d\n",
                        nz, kick_idx);
                injected = true;
            }
            rkmpp_av1_dpb_post_decode(&dpb, p.seq_hdr, p.frame_hdr);
            dav1d_picture_unref(&p);
        }
    };

    while (pos < buf.size()) {
        const uint8_t *fb; size_t fsz;
        if (!next_ivf(buf.data(), buf.size(), pos, fb, fsz)) break;
        Dav1dData d{};
        if (dav1d_data_wrap(&d, fb, fsz, [](const uint8_t*, void*){}, nullptr) != 0) break;
        for (;;) {
            int r = dav1d_send_data(c, &d);
            if (r == 0) { drain(); break; }
            if (r == DAV1D_ERR(EAGAIN)) { drain(); continue; }
            break;
        }
        dav1d_data_unref(&d);
    }
    drain();

    dav1d_close(&c);
    return injected ? 0 : 1;
}
