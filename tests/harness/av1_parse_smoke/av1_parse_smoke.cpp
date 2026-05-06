/* av1_parse_smoke — AV1 parse + reference-decode harness using dav1d.
 *
 * Reads an IVF file, runs each frame through a full dav1d decode (CPU
 * software path), and dumps:
 *   - Per-frame parsed Dav1dFrameHeader / Dav1dSequenceHeader summary
 *     to stdout (this becomes the input to our regbuilder later).
 *   - Reference I420 YUVs to <out_prefix>.yuv (concat of all frames).
 *
 * dav1d does the actual software decode here only as a means to:
 *   1. Cross-check the parser against a known-good implementation.
 *   2. Produce reference YUVs that we'll diff against the rkvdec_av1
 *      hardware output once the kernel-submission harness exists.
 *
 * We never ship dav1d in the Windows driver — it's a Linux/host-side
 * verification tool, mirroring the role winreplay_h264_diff plays for
 * the rkvdec2 codecs.
 *
 * Usage: av1_parse_smoke <path.ivf> [out_prefix]
 */
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

extern "C" {
#include <dav1d/dav1d.h>
}

static bool slurp(const char *path, std::vector<uint8_t> &out) {
    FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    out.resize(n);
    bool ok = std::fread(out.data(), 1, n, f) == (size_t)n;
    std::fclose(f);
    return ok;
}

/* IVF walker: skip 32-byte file header, then per-frame { 4B size, 8B pts, payload }. */
static int next_ivf_frame(const uint8_t *buf, size_t len, size_t &pos,
                          const uint8_t *&out, size_t &out_len, uint64_t &pts) {
    if (pos + 12 > len) return 0;
    uint32_t sz = (uint32_t)buf[pos]        | ((uint32_t)buf[pos+1] << 8) |
                  ((uint32_t)buf[pos+2]<<16)| ((uint32_t)buf[pos+3] << 24);
    pts = 0;
    for (int i = 0; i < 8; i++) pts |= (uint64_t)buf[pos + 4 + i] << (i * 8);
    pos += 12;
    if (pos + sz > len) return 0;
    out = buf + pos;
    out_len = sz;
    pos += sz;
    return 1;
}

static const char *frame_type_str(int t) {
    switch (t) {
        case DAV1D_FRAME_TYPE_KEY:        return "KEY";
        case DAV1D_FRAME_TYPE_INTER:      return "INTER";
        case DAV1D_FRAME_TYPE_INTRA:      return "INTRA";
        case DAV1D_FRAME_TYPE_SWITCH:     return "SWITCH";
        default:                          return "??";
    }
}

/* Print one human-readable line per frame summarizing the header
 * fields that drive register state.  Indices match those captured in
 * the BSP register trace; the regbuilder will consume these. */
static void dump_header(int idx, const Dav1dPicture *p) {
    const Dav1dSequenceHeader *s = p->seq_hdr;
    const Dav1dFrameHeader    *h = p->frame_hdr;

    std::printf("frame[%d] type=%s coded=%dx%d render=%dx%d "
                "show=%d showable=%d existing=%d primary_ref=%d refresh=0x%02x "
                "tiles=%dx%d (cols_log2=%d rows_log2=%d) "
                "qp_y=%d qp_dc_y=%d qp_dc_uv=(%d,%d) qm=%d "
                "lf=(y=%d,%d u=%d v=%d sharpness=%d mr_delta=%d) "
                "cdef=(n=%d damping=%d) "
                "lr_type=(%d,%d,%d) "
                "seg=(en=%d update_map=%d update_data=%d temporal=%d) "
                "delta_q=%d delta_lf=%d "
                "txm=%d skip_mode=%d warp=%d "
                "film_grain=%d superres=(en=%d denom=%d) "
                "bpc=%d profile=%d hbd=%d mono=%d\n",
                idx,
                frame_type_str(h->frame_type),
                h->width[0], h->height,
                h->render_width, h->render_height,
                h->show_frame, h->showable_frame,
                h->show_existing_frame, h->primary_ref_frame, h->refresh_frame_flags,
                h->tiling.cols, h->tiling.rows,
                h->tiling.log2_cols, h->tiling.log2_rows,
                h->quant.yac, h->quant.ydc_delta,
                h->quant.udc_delta, h->quant.vdc_delta,
                h->quant.qm,
                h->loopfilter.level_y[0], h->loopfilter.level_y[1],
                h->loopfilter.level_u, h->loopfilter.level_v,
                h->loopfilter.sharpness, h->loopfilter.mode_ref_delta_enabled,
                h->cdef.n_bits, h->cdef.damping,
                (int)h->restoration.type[0], (int)h->restoration.type[1], (int)h->restoration.type[2],
                h->segmentation.enabled, h->segmentation.update_map,
                h->segmentation.update_data, h->segmentation.temporal,
                h->delta.q.present, h->delta.lf.present,
                (int)h->txfm_mode, h->skip_mode_enabled, h->warp_motion,
                h->film_grain.present,
                h->super_res.enabled, h->super_res.width_scale_denominator,
                p->p.bpc, s->profile, s->hbd, s->monochrome);
}

/* Append decoded picture as I420 (Y plane, then U, then V) to a single
 * .yuv stream file.  We assume 8 bpc 4:2:0 — matches our test stream. */
static bool dump_yuv(FILE *out, const Dav1dPicture *p) {
    if (!out || p->p.bpc != 8 || p->p.layout != DAV1D_PIXEL_LAYOUT_I420) return false;
    int w = p->p.w, h = p->p.h;
    /* Y */
    for (int y = 0; y < h; y++)
        std::fwrite((uint8_t *)p->data[0] + y * p->stride[0], 1, w, out);
    /* U, V */
    for (int plane = 1; plane <= 2; plane++)
        for (int y = 0; y < h / 2; y++)
            std::fwrite((uint8_t *)p->data[plane] + y * p->stride[1], 1, w / 2, out);
    return true;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.ivf> [out_prefix]\n", argv[0]);
        return 2;
    }

    std::vector<uint8_t> buf;
    if (!slurp(argv[1], buf)) {
        std::fprintf(stderr, "could not read %s\n", argv[1]);
        return 1;
    }
    if (buf.size() < 32 || std::memcmp(buf.data(), "DKIF", 4) != 0) {
        std::fprintf(stderr, "not an IVF file\n");
        return 1;
    }
    std::printf("dav1d version: %s\n", dav1d_version());

    /* Optional output YUV file. */
    FILE *yuv_out = nullptr;
    if (argc >= 3) {
        std::string yuv_path = std::string(argv[2]) + ".yuv";
        yuv_out = std::fopen(yuv_path.c_str(), "wb");
        if (!yuv_out)
            std::fprintf(stderr, "warn: cannot open %s for write\n", yuv_path.c_str());
        else
            std::printf("writing reference YUV to %s\n", yuv_path.c_str());
    }

    Dav1dContext  *c = nullptr;
    Dav1dSettings  s;
    dav1d_default_settings(&s);
    /* API drift: dav1d 1.x exposes a single n_threads + max_frame_delay,
     * while 0.9.x (Ubuntu 22.04 system package) has separate
     * n_frame_threads / n_tile_threads / n_postfilter_threads.  Set
     * whichever members the headers expose so we stay deterministic on
     * both Windows submodule (1.5.3+) and Linux system (0.9.x). */
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

    size_t pos = 32;
    int frames_in = 0, frames_out = 0;

    auto try_get_picture = [&]() {
        Dav1dPicture pic{};
        int r = dav1d_get_picture(c, &pic);
        if (r == 0) {
            dump_header(frames_out, &pic);
            if (yuv_out) dump_yuv(yuv_out, &pic);
            frames_out++;
            dav1d_picture_unref(&pic);
        } else if (r != DAV1D_ERR(EAGAIN)) {
            std::fprintf(stderr, "dav1d_get_picture err=%d\n", r);
        }
        return r;
    };

    while (pos < buf.size()) {
        const uint8_t *frame; size_t fsz; uint64_t pts;
        if (!next_ivf_frame(buf.data(), buf.size(), pos, frame, fsz, pts)) break;
        frames_in++;

        Dav1dData d{};
        if (dav1d_data_wrap(&d, frame, fsz, [](const uint8_t*, void*){}, nullptr) != 0) {
            std::fprintf(stderr, "dav1d_data_wrap failed\n");
            break;
        }
        d.m.timestamp = (int64_t)pts;

        for (;;) {
            int r = dav1d_send_data(c, &d);
            if (r == 0 || r == DAV1D_ERR(EAGAIN)) {
                /* try to drain any output ready */
                while (try_get_picture() == 0) { /* loop */ }
                if (r == 0) break;
            } else {
                std::fprintf(stderr, "dav1d_send_data err=%d\n", r);
                break;
            }
        }
        dav1d_data_unref(&d);
    }

    /* Flush remaining frames. */
    while (try_get_picture() == 0) { /* loop */ }

    dav1d_close(&c);
    if (yuv_out) std::fclose(yuv_out);

    std::printf("frames_in=%d frames_out=%d\n", frames_in, frames_out);
    return frames_out > 0 ? 0 : 1;
}
