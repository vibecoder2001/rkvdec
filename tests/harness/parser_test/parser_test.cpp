/* tests/harness/parser_test/parser_test.cpp — sanity test for the
 * minimal H.264 parser_glue.  Hand-crafted minimal Baseline 16x16 stream:
 * one SPS + one PPS + one IDR slice, all with hand-computed RBSP payloads.
 *
 * The hand-crafted nature is intentional: this test pins down our
 * bit-reader / Exp-Golomb / RBSP-unescape code without dragging in a
 * full encoder.  A separate harness will swap in real FFmpeg-encoded
 * fixtures once Task 12 (test-bitstream pipeline) lands.
 */
#include "parser_glue.h"

#include <cstdio>
#include <cstring>
#include <cstdint>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

/* SPS RBSP — Baseline profile, 16x16 frame, 1 ref, frame-mbs-only.
 * Bit layout (annotated):
 *   profile_idc            = 66       (0x42)
 *   constraint_set_flags   = 0xC0     (set0=1, set1=1)
 *   level_idc              = 10       (0x0A)
 *   seq_parameter_set_id   = ue(0)    "1"
 *   log2_max_frame_num_minus4 = ue(0) "1"
 *   pic_order_cnt_type     = ue(2)    "011"
 *   max_num_ref_frames     = ue(1)    "010"
 *   gaps_in_frame_num      = u1(0)    "0"
 *   pic_width_in_mbs_minus1     = ue(0) "1"
 *   pic_height_in_map_units     = ue(0) "1"
 *   frame_mbs_only_flag    = u1(1)    "1"
 *   direct_8x8_inference   = u1(1)    "1"
 *   frame_cropping_flag    = u1(0)    "0"
 *   vui_parameters_present = u1(0)    "0"
 *   rbsp_trailing_bits     = "1" + zero pad
 */
static const uint8_t kSps[] = {
    0x67,                       /* NAL header: nal_ref_idc=3, type=7 (SPS) */
    0x42, 0xC0, 0x0A, 0xDA, 0x79
};

/* PPS RBSP — single slice group, CAVLC, no scaling matrix, deblocking
 * control present.  Field-by-field:
 *   pic_parameter_set_id   = ue(0) "1"
 *   seq_parameter_set_id   = ue(0) "1"
 *   entropy_coding_mode    = u1(0) "0"
 *   bottom_field_pic_order_in_frame_present = u1(0) "0"
 *   num_slice_groups_minus1     = ue(0) "1"
 *   num_ref_idx_l0_default_active_minus1 = ue(0) "1"
 *   num_ref_idx_l1_default_active_minus1 = ue(0) "1"
 *   weighted_pred_flag     = u1(0) "0"
 *   weighted_bipred_idc    = u(2) "00"
 *   pic_init_qp_minus26    = se(0) "1"
 *   pic_init_qs_minus26    = se(0) "1"
 *   chroma_qp_index_offset = se(0) "1"
 *   deblocking_filter_control_present = u1(1) "1"
 *   constrained_intra_pred = u1(0) "0"
 *   redundant_pic_cnt_present = u1(0) "0"
 *   rbsp_trailing = "1" + pad
 */
static const uint8_t kPps[] = {
    0x68,                       /* NAL header: nal_ref_idc=3, type=8 (PPS) */
    0xCE, 0x3C, 0x80
};

/* Concatenate Annex-B-framed SPS + PPS into a buffer and parse. */
int main()
{
    uint8_t bitstream[64];
    size_t  n = 0;
    /* start code */
    bitstream[n++] = 0; bitstream[n++] = 0; bitstream[n++] = 0; bitstream[n++] = 1;
    std::memcpy(&bitstream[n], kSps, sizeof(kSps)); n += sizeof(kSps);
    bitstream[n++] = 0; bitstream[n++] = 0; bitstream[n++] = 0; bitstream[n++] = 1;
    std::memcpy(&bitstream[n], kPps, sizeof(kPps)); n += sizeof(kPps);

    uint8_t scratch[256];
    H264ParseResult r;
    H264ParseStatus s = H264ParseAccessUnit(bitstream, n,
                                            scratch, sizeof(scratch), &r);

    /* No slice in this stream → NEED_MORE is the expected status. */
    EXPECT(s == H264_PARSE_NEED_MORE, "got status %d", (int)s);
    EXPECT(r.has_sps, "SPS not parsed");
    EXPECT(r.has_pps, "PPS not parsed");

    EXPECT(r.sps.profile_idc == 66, "profile_idc=%u", r.sps.profile_idc);
    EXPECT(r.sps.level_idc   == 10, "level_idc=%u",   r.sps.level_idc);
    EXPECT(r.sps.seq_parameter_set_id == 0,
           "sps_id=%u", r.sps.seq_parameter_set_id);
    EXPECT(r.sps.log2_max_frame_num_minus4 == 0,
           "log2_max_frame_num_minus4=%u", r.sps.log2_max_frame_num_minus4);
    EXPECT(r.sps.pic_order_cnt_type == 2,
           "pic_order_cnt_type=%u", r.sps.pic_order_cnt_type);
    EXPECT(r.sps.max_num_ref_frames == 1,
           "max_num_ref_frames=%u", r.sps.max_num_ref_frames);
    EXPECT(r.sps.pic_width_in_mbs_minus1 == 0, "width_mbs-1=%u",
           r.sps.pic_width_in_mbs_minus1);
    EXPECT(r.sps.pic_height_in_map_units_minus1 == 0, "height-1=%u",
           r.sps.pic_height_in_map_units_minus1);
    EXPECT(r.sps.flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY,
           "frame_mbs_only flag missing: 0x%x", r.sps.flags);
    EXPECT(r.sps.flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE,
           "direct_8x8 flag missing: 0x%x", r.sps.flags);

    EXPECT(r.pps.pic_parameter_set_id == 0, "pps_id=%u",
           r.pps.pic_parameter_set_id);
    EXPECT(r.pps.seq_parameter_set_id == 0, "pps.sps_id=%u",
           r.pps.seq_parameter_set_id);
    EXPECT(r.pps.num_slice_groups_minus1 == 0, "num_slice_groups-1=%u",
           r.pps.num_slice_groups_minus1);
    EXPECT(r.pps.flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT,
           "deblock-control flag missing: 0x%x", r.pps.flags);
    EXPECT(!(r.pps.flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE),
           "entropy-coding (CABAC) wrongly set");

    std::printf("parser_test OK\n");
    return 0;
}
