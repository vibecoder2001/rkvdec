/* tests/harness/packed_tables_test/packed_tables_test.cpp
 *
 * Sanity checks for the Rockchip-format H.264 packed tables.  Each
 * function gets a synthetic input, and we assert specific bytes of
 * the resulting buffer match what `mpp/hal/rkdec/h264d/...` would
 * produce.
 */
#include "h264_packed_tables.h"

#include <cstdio>
#include <cstring>
#include <cstdint>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

static int test_cabac()
{
    size_t n = 0;
    const uint32_t *t = H264GetCabacInitTable(&n);
    EXPECT(n == 928, "cabac word count = %zu", n);
    /* Spot-check first and last entries from hal_h264d_com.c:16,172. */
    EXPECT(t[0]   == 0x3602f114u, "cabac[0]   = 0x%08x", t[0]);
    EXPECT(t[1]   == 0xf1144a03u, "cabac[1]   = 0x%08x", t[1]);
    EXPECT(t[927] == 0x00000000u, "cabac[927] = 0x%08x", t[927]);
    EXPECT(t[925] == 0x430e241du, "cabac[925] = 0x%08x", t[925]);
    return 0;
}

static int test_spspps()
{
    /* Synthetic Baseline 16x16 SPS+PPS, all flags off except
     * frame_mbs_only and direct_8x8 (matches parser_test fixture). */
    v4l2_ctrl_h264_sps sps{};
    sps.profile_idc = 66;
    sps.level_idc   = 10;
    sps.chroma_format_idc          = 1;
    sps.bit_depth_luma_minus8      = 0;
    sps.bit_depth_chroma_minus8    = 0;
    sps.log2_max_frame_num_minus4  = 0;
    sps.pic_order_cnt_type         = 2;
    sps.log2_max_pic_order_cnt_lsb_minus4 = 0;
    sps.max_num_ref_frames         = 1;
    sps.pic_width_in_mbs_minus1    = 0;   /* 16 px */
    sps.pic_height_in_map_units_minus1 = 0;
    sps.flags = V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY |
                V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;

    v4l2_ctrl_h264_pps pps{};
    pps.flags = V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

    v4l2_h264_dpb_entry dpb[16] = {};

    uint8_t out[RKH264_SPSPPS_UNIT_SIZE];
    H264PackSpsPpsUnit(out, &sps, &pps, dpb, /*field_pic=*/0);

    /* Read back as u64 LE words. */
    uint64_t w[6];
    std::memcpy(w, out, sizeof(w));

    /* SPS section (bit-by-bit; see prepare_spspps lines 145..164):
     *   bits 0..12  = 0x1FFF
     *   bits 13..14 = chroma_format_idc = 1     -> bit 13 set
     *   bits 15..17 = bit_depth_luma_minus8 = 0
     *   bits 18..20 = bit_depth_chroma_minus8 = 0
     *   bit  21     = qpprime_y_zero = 0
     *   bits 22..25 = log2_max_frame_num_minus4 = 0
     *   bits 26..30 = max_num_ref_frames = 1     -> bit 26 set
     *   bits 31..32 = pic_order_cnt_type = 2     -> bit 32 set
     *   bits 33..36 = log2_max_poc_lsb_minus4 = 0
     *   bit  37     = delta_pic_order_always_zero = 0
     *   bits 38..49 = (width_mbs-1)+1 = 1        -> bit 38 set
     *   bits 50..61 = (height_mbs-1)+1 = 1       -> bit 50 set
     *   bit  62     = frame_mbs_only = 1
     *   bit  63     = MbaffFrameFlag = 0
     */
    uint64_t expect_w0 =
        0x1FFFull
        | (1ull << 13)   /* chroma_format_idc */
        | (1ull << 26)   /* max_num_ref_frames */
        | (2ull << 31)   /* pic_order_cnt_type */
        | (1ull << 38)   /* width */
        | (1ull << 50)   /* height */
        | (1ull << 62);  /* frame_mbs_only */
    EXPECT(w[0] == expect_w0,
        "spspps w[0] = 0x%016llx  want 0x%016llx",
        (unsigned long long)w[0], (unsigned long long)expect_w0);

    /* word[1] bit 0 = direct_8x8, bit 1 = mvc_extension_enable, bit 2 =
     * num_views_minus1+1=1 (low bit of value 1). All else through bit 67
     * is zero (MVC ref counts), then align(128) pads to bit 128. */
    EXPECT((w[1] & 0xFFull) == 0x07ull,
        "spspps w[1] low byte = 0x%02x", (unsigned)(w[1] & 0xFFull));

    /* SPS section runs to bit 132 (65 SPS bits + 67 MVC bits); align(128)
     * pads to bit 256.  So PPS section starts at word[4] (bit 256). */
    EXPECT((w[4] & 0x1FFFull) == 0x1FFFull,
        "spspps w[4] low 13 bits = 0x%04llx",
        (unsigned long long)(w[4] & 0x1FFFull));

    /* DPB tail: when dpb is all zero, the 32-bit tail word is 0. */
    /* Tail lands at the very end (bit 344+). Already covered by zero
     * init of buf — check final byte is zero. */
    EXPECT(out[RKH264_SPSPPS_UNIT_SIZE - 1] == 0,
        "spspps trailing byte = 0x%02x",
        out[RKH264_SPSPPS_UNIT_SIZE - 1]);

    /* DPB tail with a long-term ref at slot 3 should set bit 3 of the
     * 32-bit tail word.  Re-pack and verify. */
    dpb[3].flags = V4L2_H264_DPB_ENTRY_FLAG_VALID |
                   V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
    H264PackSpsPpsUnit(out, &sps, &pps, dpb, 0);
    /* Tail is at bit position 344 (43 bytes * 8 = 344, then 32-bit tail
     * brings it to 376, then align(64) → 384 = 48 bytes). */
    uint32_t tail = 0;
    std::memcpy(&tail, out + 43, 4);
    EXPECT(tail & (1u << 3),
        "DPB long-term tail bit 3 not set: 0x%08x", tail);
    return 0;
}

static int test_rps_idr()
{
    /* IDR with no refs → all 16 dpb entries invalid, all ref_lists
     * entries invalid → 384 bytes of zero. */
    v4l2_h264_dpb_entry dpb[16] = {};
    v4l2_h264_reference ref_lists[3][32] = {};

    uint8_t out[RKH264_RPS_SIZE];
    std::memset(out, 0xCD, sizeof(out));   /* poison */
    H264PackFrameRps(out, /*frame_num=*/0, /*log2_max_minus4=*/0,
                     dpb, ref_lists);
    for (size_t i = 0; i < RKH264_RPS_SIZE; i++) {
        EXPECT(out[i] == 0,
            "RPS byte %zu = 0x%02x (expected zero for IDR)",
            i, out[i]);
    }
    return 0;
}

static int test_rps_one_ref()
{
    /* Single valid ref at slot 0 (frame ref, dpb_idx 0) — the L0[0]
     * entry packs to:
     *   bits 0..3 = dpb_idx = 0
     *   bit  4    = dpb_valid = 1
     *   bit  5    = bottom_flag = 0
     *   bit  6    = voidx = 0
     *  → 7-bit entry = 0x10. */
    v4l2_h264_dpb_entry dpb[16] = {};
    dpb[0].flags     = V4L2_H264_DPB_ENTRY_FLAG_VALID |
                       V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
    dpb[0].frame_num = 5;

    v4l2_h264_reference ref_lists[3][32] = {};
    ref_lists[0][0].fields = V4L2_H264_FRAME_REF;
    ref_lists[0][0].index  = 0;

    uint8_t out[RKH264_RPS_SIZE];
    /* frame_num=10 keeps wrap path off (ref frame_num=5 < 10, so
     * wrap = 5 verbatim).  Pass max_frame_num_minus4=0 → max_frame_num=16. */
    H264PackFrameRps(out, /*frame_num=*/10, /*log2_max_minus4=*/0,
                     dpb, ref_lists);

    /* RefPicList[0][0] is the FIRST 7-bit entry after the head padding
     * (16 bytes head pad + 32 bytes frame_num_wrap + 4 bytes NULL/voidx).
     * Total preceding bits = 0 (head pad is no-op at offset 0)
     *                      + 256 (16 wraps)
     *                      + 16 (NULL)
     *                      + 16 (voidx bitmap)
     *                      = 288 bits = byte 36, bit 0.
     *
     * Read 8 bits at byte 36; only the low 7 should be set to 0x10. */
    uint8_t entry = out[36] & 0x7F;
    EXPECT(entry == 0x10, "L0[0] entry = 0x%02x  want 0x10", entry);

    /* frame_num_wrap[0] should be 5 (since dpb[0].frame_num=5 < frame_num=0
     * is FALSE, so wrap = frame_num_list = 5).  At byte 0, LE 16-bit. */
    uint16_t wrap0;
    std::memcpy(&wrap0, out + 0, 2);
    EXPECT(wrap0 == 5, "frame_num_wrap[0] = %u", wrap0);
    return 0;
}

static int test_scaling_list()
{
    v4l2_ctrl_h264_scaling_matrix sm{};
    /* Identity-ish 4x4 list 0: 0x10 in every position. */
    for (int j = 0; j < 16; j++) sm.scaling_list_4x4[0][j] = 0x10;
    /* 4x4 list 5 starts at offset 5*16 = 80; check end of 4x4 region. */
    for (int j = 0; j < 64; j++) sm.scaling_list_8x8[1][j] = 0x42;

    uint8_t out[RKH264_SCALING_LIST_SIZE];
    H264PackScalingList(out, &sm, /*enable=*/1);
    EXPECT(out[0]  == 0x10, "4x4[0][0] = 0x%02x", out[0]);
    EXPECT(out[15] == 0x10, "4x4[0][15] = 0x%02x", out[15]);
    EXPECT(out[16] == 0x00, "4x4[1][0] = 0x%02x", out[16]);
    /* 8x8 list 1 starts at 6*16 + 64 = 160. */
    EXPECT(out[160] == 0x42, "8x8[1][0] = 0x%02x", out[160]);
    EXPECT(out[223] == 0x42, "8x8[1][63] = 0x%02x", out[223]);

    /* Disabled → all zero. */
    H264PackScalingList(out, &sm, /*enable=*/0);
    for (size_t i = 0; i < RKH264_SCALING_LIST_SIZE; i++) {
        EXPECT(out[i] == 0, "disabled scaling byte %zu = 0x%02x",
               i, out[i]);
    }
    return 0;
}

static int test_rcb_sizes()
{
    /* For 16x16, dim*coeff for each row-RCB ≤ 64*22 = 1408 → max one
     * 64-byte chunk per row.  FILT_COL with coeff 67 yields
     * align64(16*67)=align64(1072)=1088 bytes. */
    H264RcbInfo info[RKH264_RCB_COUNT];
    uint32_t total = H264GetRcbBufferSizes(info, /*w=*/16, /*h=*/16);

    /* DBLK_ROW (idx 0, reg 139) at offset 0, size align64(16*22)=384. */
    EXPECT(info[0].reg_idx == 139, "DBLK reg = %u", info[0].reg_idx);
    EXPECT(info[0].offset  == 0,   "DBLK offset = %u", info[0].offset);
    EXPECT(info[0].size    == 384, "DBLK size = %u", info[0].size);

    /* INTRA_ROW (idx 1, reg 133) follows DBLK: offset=384, size=128. */
    EXPECT(info[1].reg_idx == 133, "INTRA reg = %u", info[1].reg_idx);
    EXPECT(info[1].offset  == 384, "INTRA offset = %u", info[1].offset);
    EXPECT(info[1].size    == 128, "INTRA size = %u", info[1].size);

    /* FILT_COL last (idx 9, reg 142): align64(16*67)=1088. */
    EXPECT(info[9].reg_idx == 142, "FILT_COL reg = %u", info[9].reg_idx);
    EXPECT(info[9].size    == 1088, "FILT_COL size = %u", info[9].size);

    /* Total = sum of aligned sizes. Hand-computed:
     * 384 + 128 + 64 + 64 + 128 + 128 + 192 + 64 + 64 + 1088 = 2304 */
    EXPECT(total == 2304, "rcb total for 16x16 = %u  want 2304", total);

    /* Also exercise a real-world frame size (1920x1088). */
    total = H264GetRcbBufferSizes(info, 1920, 1088);
    EXPECT(total > 0, "rcb total for 1080p = 0");
    return 0;
}

int main()
{
    if (test_cabac())         return 1;
    if (test_spspps())        return 1;
    if (test_rps_idr())       return 1;
    if (test_rps_one_ref())   return 1;
    if (test_scaling_list())  return 1;
    if (test_rcb_sizes())     return 1;
    std::printf("packed_tables_test OK\n");
    return 0;
}
