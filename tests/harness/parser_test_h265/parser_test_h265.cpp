/* tests/harness/parser_test_h265/parser_test_h265.cpp — sanity test for
 * the minimal H.265 parser_glue against the BSP capture .h265 streams.
 *
 * Both Z:\drivers-arm\bsp_capture\first_idr.h265 and \multi.h265 are
 * 1280x720 Main Profile streams (no tiles, no WPP, no LTR).  We parse
 * the first AU and validate VPS/SPS/PPS/slice fields the regbuilder
 * will consume.
 */
#include "parser_glue_h265.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); ok = 0; } \
} while (0)

static bool slurp(const char *path, std::vector<uint8_t> *out) {
    std::FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n <= 0) { std::fclose(f); return false; }
    out->resize((size_t)n);
    size_t got = std::fread(out->data(), 1, (size_t)n, f);
    std::fclose(f);
    return got == (size_t)n;
}

/* Find the byte offset of the second start code in `buf` (i.e. the end
 * of the first AU, approximately — close enough that the parser will
 * see VPS+SPS+PPS+IDR before the next IDR).  Our streams have one AU
 * per file segment; in practice we feed the whole file. */
static int run_one(const char *path) {
    std::vector<uint8_t> stream;
    if (!slurp(path, &stream)) {
        std::printf("FAIL: could not open %s\n", path);
        return 1;
    }
    std::printf("== %s (%zu bytes) ==\n", path, stream.size());

    std::vector<uint8_t> scratch(stream.size() * 2 + 16);
    H265ParseResult r;
    H265ParseResultInit(&r);

    /* Feed the entire file as one AU.  For multi-frame streams the
     * parser will end up reflecting the LAST slice it saw, but the
     * VPS/SPS/PPS arrays are sticky so all earlier ones remain valid. */
    H265ParseStatus s = H265ParseAccessUnit(stream.data(), stream.size(),
                                            scratch.data(), scratch.size(), &r);
    int ok = 1;
    EXPECT(s == H265_PARSE_OK || s == H265_PARSE_NEED_MORE,
           "parse status %d", (int)s);

    /* Expect VPS 0, SPS 0, PPS 0 from any well-formed Main stream. */
    EXPECT(r.vps[0].valid, "VPS[0] not parsed");
    EXPECT(r.sps[0].valid, "SPS[0] not parsed");
    EXPECT(r.pps[0].valid, "PPS[0] not parsed");

    if (r.sps[0].valid) {
        const H265Sps &sp = r.sps[0];
        std::printf("  SPS: %ux%u  chroma=%u  bd_luma+8=%u  bd_chroma+8=%u\n",
                    sp.pic_width_in_luma_samples, sp.pic_height_in_luma_samples,
                    sp.chroma_format_idc,
                    (unsigned)(sp.bit_depth_luma_minus8 + 8),
                    (unsigned)(sp.bit_depth_chroma_minus8 + 8));
        std::printf("       CtbLog2=%u CtbSizeY=%u  ctbsW=%u ctbsH=%u\n",
                    sp.ctb_log2_size_y, sp.ctb_size_y,
                    sp.pic_width_in_ctbs_y, sp.pic_height_in_ctbs_y);
        std::printf("       num_strps=%u amp=%u sao=%u pcm=%u tmvp=%u sis=%u\n",
                    sp.num_short_term_ref_pic_sets, sp.amp_enabled_flag,
                    sp.sample_adaptive_offset_enabled_flag, sp.pcm_enabled_flag,
                    sp.sps_temporal_mvp_enabled_flag,
                    sp.strong_intra_smoothing_enabled_flag);
        EXPECT(sp.pic_width_in_luma_samples  == 1280, "width=%u",  sp.pic_width_in_luma_samples);
        EXPECT(sp.pic_height_in_luma_samples == 720,  "height=%u", sp.pic_height_in_luma_samples);
        EXPECT(sp.chroma_format_idc == 1, "chroma=%u", sp.chroma_format_idc);
        EXPECT(sp.bit_depth_luma_minus8 == 0, "bd luma=%u", sp.bit_depth_luma_minus8);
    }
    if (r.pps[0].valid) {
        const H265Pps &pp = r.pps[0];
        std::printf("  PPS: init_qp=%d cb_off=%d cr_off=%d tiles=%u wpp=%u "
                    "deblock_ctrl=%u dep_slice=%u\n",
                    pp.init_qp_minus26 + 26, pp.pps_cb_qp_offset, pp.pps_cr_qp_offset,
                    pp.tiles_enabled_flag, pp.entropy_coding_sync_enabled_flag,
                    pp.deblocking_filter_control_present_flag,
                    pp.dependent_slice_segments_enabled_flag);
        std::printf("       weighted_pred=%u weighted_bipred=%u  "
                    "lt_present(sps)=%u\n",
                    pp.weighted_pred_flag, pp.weighted_bipred_flag,
                    r.sps[0].long_term_ref_pics_present_flag);
        EXPECT(pp.tiles_enabled_flag == 0, "tiles enabled");
    }
    if (r.has_slice) {
        const H265SliceHeader &sh = r.slice;
        std::printf("  slice: nal_type=%u idr=%u irap=%u type=%u  poc=%d "
                    "first=%u addr=%u  qp_delta=%d  hdr_bits=%u\n",
                    r.slice_nal_unit_type, r.is_idr, r.is_irap, sh.slice_type,
                    r.poc, sh.first_slice_segment_in_pic_flag,
                    sh.slice_segment_address, sh.slice_qp_delta,
                    sh.header_bit_size);
        std::printf("         data=%p size=%zu\n",
                    (const void*)r.slice_data, r.slice_data_size);
        EXPECT(r.slice_data != nullptr, "slice data ptr null");
        EXPECT(r.slice_data_size > 0,   "slice data size 0");
    } else {
        std::printf("  (no slice parsed)\n");
    }
    std::printf(ok ? "  -> OK\n" : "  -> FAIL\n");
    return ok ? 0 : 1;
}

int main(int argc, char **argv) {
    /* Allow an override path for CI. */
    const char *paths[2] = {
        "Z:\\drivers-arm\\bsp_capture\\first_idr.h265",
        "Z:\\drivers-arm\\bsp_capture\\multi.h265",
    };
    if (argc > 1) paths[0] = argv[1];
    if (argc > 2) paths[1] = argv[2];

    int rc = 0;
    rc |= run_one(paths[0]);
    rc |= run_one(paths[1]);
    if (rc == 0) std::printf("parser_test_h265 OK\n");
    return rc;
}
