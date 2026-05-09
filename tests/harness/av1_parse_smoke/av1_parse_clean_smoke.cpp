/* av1_parse_clean_smoke — Test harness for the clean-room AV1 parser.
 *
 * Reads an IVF file, passes each OBU through Av1ParseSeqHeader /
 * Av1ParseFrameHeader, and prints per-frame field summaries to stdout
 * in the same format used by av1_parse_smoke (dav1d) so the two outputs
 * can be diff'd directly.
 *
 * Does NOT call any dav1d functions — only includes <dav1d/headers.h>
 * for struct definitions.
 *
 * Build (Linux/host with project CMake):
 *   # add av1_parse_clean_smoke target to CMakeLists.txt for this directory
 *   cmake --build . --target av1_parse_clean_smoke
 *
 * Quick manual compile (no CMake):
 *   g++ -std=c++17 -O2 \
 *       -I../../mft \
 *       -I../../third_party/dav1d/include \
 *       av1_parse_clean_smoke.cpp \
 *       ../../mft/av1_parser.cpp \
 *       -o av1_parse_clean_smoke
 *
 * Usage:
 *   ./av1_parse_clean_smoke tests/data/av1/av1_720p.ivf
 *   ./av1_parse_clean_smoke tests/data/av1/av1_720p.ivf > clean.txt
 *   ./av1_parse_smoke       tests/data/av1/av1_720p.ivf > dav1d.txt
 *   diff dav1d.txt clean.txt  # should be empty for profile-0 8-bit streams
 */

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

extern "C" {
#include <dav1d/headers.h>
}
#include "av1_parser.h"

/* =========================================================================
 * IVF file reader
 * ========================================================================= */

static bool slurp(const char *path, std::vector<uint8_t> &out)
{
    FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    out.resize((size_t)n);
    bool ok = std::fread(out.data(), 1, (size_t)n, f) == (size_t)n;
    std::fclose(f);
    return ok;
}

static int next_ivf_frame(const uint8_t *buf, size_t len, size_t &pos,
                           const uint8_t *&out, size_t &out_len)
{
    if (pos + 12 > len) return 0;
    uint32_t sz = (uint32_t)buf[pos]         |
                  ((uint32_t)buf[pos + 1] <<  8) |
                  ((uint32_t)buf[pos + 2] << 16) |
                  ((uint32_t)buf[pos + 3] << 24);
    pos += 12; /* skip size(4) + pts(8) */
    if (pos + sz > len) return 0;
    out     = buf + pos;
    out_len = sz;
    pos    += sz;
    return 1;
}

/* =========================================================================
 * OBU framing: walk an AU (temporal unit) and dispatch to parser
 * ========================================================================= */

/* Read leb128 from a byte pointer; advance ptr. */
static uint32_t read_leb128(const uint8_t *&p, const uint8_t *end)
{
    uint64_t val = 0;
    for (int i = 0; i < 8; i++) {
        if (p >= end) return 0;
        uint8_t byte = *p++;
        val |= (uint64_t)(byte & 0x7f) << (i * 7);
        if (!(byte & 0x80)) break;
    }
    return (val > UINT32_MAX) ? 0 : (uint32_t)val;
}

static const char *frame_type_str(int t)
{
    switch (t) {
        case DAV1D_FRAME_TYPE_KEY:    return "KEY";
        case DAV1D_FRAME_TYPE_INTER:  return "INTER";
        case DAV1D_FRAME_TYPE_INTRA:  return "INTRA";
        case DAV1D_FRAME_TYPE_SWITCH: return "SWITCH";
        default:                      return "??";
    }
}

static void dump_header(int idx,
                        const Dav1dSequenceHeader *s,
                        const Dav1dFrameHeader    *h)
{
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
                "profile=%d hbd=%d mono=%d hdr_bytes=%u\n",
                idx,
                frame_type_str(h->frame_type),
                h->width[0], h->height,
                h->render_width, h->render_height,
                h->show_frame, h->showable_frame,
                h->show_existing_frame, h->primary_ref_frame,
                h->refresh_frame_flags,
                h->tiling.cols, h->tiling.rows,
                h->tiling.log2_cols, h->tiling.log2_rows,
                h->quant.yac, h->quant.ydc_delta,
                h->quant.udc_delta, h->quant.vdc_delta,
                h->quant.qm,
                h->loopfilter.level_y[0], h->loopfilter.level_y[1],
                h->loopfilter.level_u, h->loopfilter.level_v,
                h->loopfilter.sharpness, h->loopfilter.mode_ref_delta_enabled,
                h->cdef.n_bits, h->cdef.damping,
                (int)h->restoration.type[0],
                (int)h->restoration.type[1],
                (int)h->restoration.type[2],
                h->segmentation.enabled, h->segmentation.update_map,
                h->segmentation.update_data, h->segmentation.temporal,
                h->delta.q.present, h->delta.lf.present,
                (int)h->txfm_mode, h->skip_mode_enabled, h->warp_motion,
                h->film_grain.present,
                h->super_res.enabled, h->super_res.width_scale_denominator,
                s->profile, s->hbd, s->monochrome,
                h->frame_hdr_obu_size_bytes);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.ivf>\n", argv[0]);
        return 2;
    }

    std::vector<uint8_t> buf;
    if (!slurp(argv[1], buf)) {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    if (buf.size() < 32 || std::memcmp(buf.data(), "DKIF", 4) != 0) {
        std::fprintf(stderr, "not an IVF file\n");
        return 1;
    }

    Dav1dSequenceHeader seq{};
    bool              have_seq = false;
    Av1SavedFrameState saved[8]{};
    int frame_idx = 0;

    size_t pos = 32; /* skip IVF file header */
    while (pos < buf.size()) {
        const uint8_t *au;
        size_t au_len;
        if (!next_ivf_frame(buf.data(), buf.size(), pos, au, au_len))
            break;

        /* Walk OBUs inside this access unit. */
        const uint8_t *p   = au;
        const uint8_t *end = au + au_len;

        while (p < end) {
            /* OBU header byte */
            if (p >= end) break;
            uint8_t hdr = *p++;
            /* forbidden bit = bit 7; type = bits 6:3; ext = bit 2; size = bit 1; reserved = bit 0 */
            int obu_type     = (hdr >> 3) & 0xf;
            int has_ext      = (hdr >> 2) & 1;
            int has_size     = (hdr >> 1) & 1;

            if (has_ext) p++; /* skip extension byte */

            const uint8_t *payload;
            size_t payload_len;

            if (has_size) {
                uint32_t sz = read_leb128(p, end);
                if (p + sz > end) break;
                payload     = p;
                payload_len = sz;
                p          += sz;
            } else {
                payload     = p;
                payload_len = (size_t)(end - p);
                p           = end;
            }

            if (obu_type == DAV1D_OBU_SEQ_HDR) {
                int rc = Av1ParseSeqHeader(payload, payload_len, &seq);
                if (rc < 0) {
                    std::fprintf(stderr, "Av1ParseSeqHeader failed\n");
                    continue;
                }
                have_seq = true;

            } else if ((obu_type == DAV1D_OBU_FRAME_HDR ||
                        obu_type == DAV1D_OBU_FRAME       ||
                        obu_type == DAV1D_OBU_REDUNDANT_FRAME_HDR) && have_seq)
            {
                bool is_frame = (obu_type == DAV1D_OBU_FRAME);
                Dav1dFrameHeader fh{};
                int rc = Av1ParseFrameHeader(
                    payload, payload_len, is_frame,
                    &seq, saved, &fh);
                if (rc < 0) {
                    std::fprintf(stderr,
                        "Av1ParseFrameHeader failed at frame[%d]\n",
                        frame_idx);
                    frame_idx++;
                    continue;
                }

                if (!fh.show_existing_frame || obu_type == DAV1D_OBU_REDUNDANT_FRAME_HDR) {
                    dump_header(frame_idx, &seq, &fh);
                    Av1UpdateSavedStates(saved, &fh);
                }
                frame_idx++;
            }
        }
    }

    std::printf("total_frames=%d\n", frame_idx);
    return frame_idx > 0 ? 0 : 1;
}
