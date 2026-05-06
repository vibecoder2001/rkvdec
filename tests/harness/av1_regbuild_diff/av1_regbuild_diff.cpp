/* av1_regbuild_diff — reg-array diff harness for the AV1 bring-up.
 *
 * Reads an IVF file, parses each frame via dav1d, runs our regbuilder,
 * and emits the produced register array in BSP-shim format
 * (`AV1SHIM r[0][NNN]=VVVVVVVV`) for diffing against captured BSP
 * traces (e.g. docs/av1_trace_720p.log).
 *
 * Mirrors winreplay_h264_diff for the rkvdec2 codecs.
 *
 * Usage: av1_regbuild_diff <file.ivf>
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

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

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.ivf>\n", argv[0]);
        return 2;
    }
    std::vector<uint8_t> buf;
    if (!slurp(argv[1], buf) || buf.size() < 32 ||
        std::memcmp(buf.data(), "DKIF", 4) != 0) {
        std::fprintf(stderr, "not a valid IVF file\n");
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
    if (dav1d_open(&c, &s) != 0) {
        std::fprintf(stderr, "dav1d_open failed\n");
        return 1;
    }

    /* Dummy buffer FDs — we just emit the regs, don't kick hardware
     * here.  Kernel-side IOMMU translation isn't running, so the *_msb
     * fields will stay zero in our output.  Diff harness should mask
     * the *_lsb / *_msb DMA entries before byte-comparing. */
    RkmppAv1Buffers bufs{};
    bufs.output_y_fd  = 0x10;
    bufs.output_uv_fd = 0x10;
    bufs.bitstream_fd = 0x11;
    for (int i = 0; i < 7; i++) {
        bufs.ref_y_fd[i]  = 0x20 + i;
        bufs.ref_uv_fd[i] = 0x20 + i;
    }
    /* Optional: skip first N output pictures so kick numbering aligns
     * with BSP traces where the keyframe kick was missed by dmesg -c.
     * Defaults to 0; set RKMPP_AV1_SKIP_KEY=1 to drop the keyframe. */
    int skip = 0;
    if (const char *e = std::getenv("RKMPP_AV1_SKIP")) skip = atoi(e);

    size_t pos = 32;
    int kick = 0;
    /* Track most-recent IVF frame size; passed as bitstream_length to
     * regbuilder.  For our test streams (1 frame per TU) this matches the
     * value the kernel programs on each kick. */
    uint32_t last_fsz = 0;

    RkmppAv1Dpb dpb;
    rkmpp_av1_dpb_init(&dpb);

    auto drain = [&]() {
        for (;;) {
            Dav1dPicture p{};
            int r = dav1d_get_picture(c, &p);
            if (r != 0) return r;
            /* Update DPB BEFORE skip check so DPB tracks even
             * skipped-frame state (matters for the keyframe). */
            if (skip > 0) {
                rkmpp_av1_dpb_post_decode(&dpb, p.seq_hdr, p.frame_hdr);
                std::printf("# skipped frame: type=%d w=%dx%d show=%d "
                            "refresh=0x%02x\n",
                            p.frame_hdr->frame_type,
                            p.frame_hdr->width[0], p.frame_hdr->height,
                            p.frame_hdr->show_frame,
                            p.frame_hdr->refresh_frame_flags);
                skip--;
                dav1d_picture_unref(&p);
                continue;
            }
            /* Show-existing-frame OBUs don't decode anything; HW isn't
             * kicked for them.  Skip them but still update DPB state.
             * Note: dav1d does not surface this flag in Dav1dPicture::frame_hdr
             * for the resulting picture (the hdr reflects the *original*
             * decoded frame, not the show_existing pointer), so this branch
             * rarely fires.  Kept for streams where dav1d does set it. */
            if (p.frame_hdr->show_existing_frame) {
                rkmpp_av1_dpb_post_decode(&dpb, p.seq_hdr, p.frame_hdr);
                dav1d_picture_unref(&p);
                continue;
            }
            VdpuAv1dRegSet regs{};
            std::memset(&regs, 0, sizeof(regs));
            bufs.bitstream_length = last_fsz;
            RkmppAv1Status st = rkmpp_av1_build_regs(p.seq_hdr, p.frame_hdr,
                                                     &dpb, &bufs, &regs);
            /* DPB updates happen AFTER reg build so this frame's regs
             * reflect the state BEFORE its own decode (the refs it
             * reads from), then the post-decode call refreshes slots
             * marked by refresh_frame_flags for the NEXT frame. */
            rkmpp_av1_dpb_post_decode(&dpb, p.seq_hdr, p.frame_hdr);
            std::printf("# kick %d: build_regs=%d type=%d w=%dx%d show=%d "
                        "qp_y=%d filt=(%d,%d,%d,%d)\n",
                        kick, (int)st, p.frame_hdr->frame_type,
                        p.frame_hdr->width[0], p.frame_hdr->height,
                        p.frame_hdr->show_frame,
                        p.frame_hdr->quant.yac,
                        p.frame_hdr->loopfilter.level_y[0],
                        p.frame_hdr->loopfilter.level_y[1],
                        p.frame_hdr->loopfilter.level_u,
                        p.frame_hdr->loopfilter.level_v);
            std::printf("# kick %d begin\n", kick);
            rkmpp_av1_dump_regs_shim(&regs, stdout);
            std::printf("# kick %d end\n", kick);
            kick++;
            dav1d_picture_unref(&p);
        }
    };

    while (pos < buf.size()) {
        const uint8_t *fb; size_t fsz;
        if (!next_ivf(buf.data(), buf.size(), pos, fb, fsz)) break;
        last_fsz = (uint32_t)fsz;
        Dav1dData d{};
        if (dav1d_data_wrap(&d, fb, fsz, [](const uint8_t*, void*){}, nullptr) != 0)
            break;
        for (;;) {
            int r = dav1d_send_data(c, &d);
            if (r == 0) { drain(); break; }
            if (r == DAV1D_ERR(EAGAIN)) { drain(); continue; }
            std::fprintf(stderr, "send_data err=%d\n", r);
            break;
        }
        dav1d_data_unref(&d);
    }
    drain();

    dav1d_close(&c);
    std::fprintf(stderr, "kicks=%d\n", kick);
    return kick > 0 ? 0 : 1;
}
