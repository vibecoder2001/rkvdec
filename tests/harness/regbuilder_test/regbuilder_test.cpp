/* tests/harness/regbuilder_test/regbuilder_test.cpp
 *
 * Synthetic V4L2 controls + canned buffer handles → register list.
 * Asserts the right registers get written with the right plain values
 * and that DMA address registers carry the expected BufferHandle for
 * iova substitution at submit time.
 */
#include <windows.h>
#include "regbuilder_h264.h"
#include "rkvdec2_h264_regs.h"

#include <cstdio>
#include <cstring>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

/* Find the first emitted write at `off`. */
static const RKMPP_REG_WRITE* FindReg(const H264RegWriteList &list, uint32_t off)
{
    for (uint32_t i = 0; i < list.count; i++) {
        if (list.entries[i].Offset == off) return &list.entries[i];
    }
    return nullptr;
}

int main()
{
    H264ParseResult parsed{};
    parsed.has_sps   = 1;
    parsed.has_pps   = 1;
    parsed.has_slice = 1;

    /* 16x16 Baseline, single MB, I-frame only.  Same shape as the
     * parser_test fixture. */
    parsed.sps.profile_idc                = 66;
    parsed.sps.level_idc                  = 10;
    parsed.sps.pic_width_in_mbs_minus1    = 0;   /* 16 px */
    parsed.sps.pic_height_in_map_units_minus1 = 0;
    parsed.sps.flags = V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY |
                       V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;

    parsed.pps.flags = V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

    parsed.slice.slice_type = V4L2_H264_SLICE_TYPE_I;
    parsed.slice.first_mb_in_slice = 0;

    parsed.decode.flags = V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;
    parsed.decode.top_field_order_cnt    = 0x12345678;
    parsed.decode.bottom_field_order_cnt = 0x12345678;

    H264BufferRefs bufs{};
    bufs.bitstream        = 0xAA00000000000001ULL;
    bufs.bitstream_offset = 0x10;
    bufs.bitstream_size   = 0x40;
    bufs.output_frame     = 0xAA00000000000002ULL;
    bufs.colmv_cur        = 0xAA00000000000003ULL;
    bufs.error_ref        = 0xAA00000000000004ULL;
    /* RCB scratch all zero — should be skipped. */

    H264RegWriteList list{};
    H264RegBuildStatus s = H264BuildRegisterList(&parsed, &bufs, /*pic_idx=*/0, &list);
    EXPECT(s == H264_REGBUILD_OK, "build status %d", (int)s);
    EXPECT(list.count > 0,         "no writes emitted");
    EXPECT(list.count <= RKMPP_MAX_REG_WRITES, "overflow: %u", list.count);

    /* Common-bank essentials. */
    auto *r = FindReg(list, RKVDEC2_REG_DEC_MODE);
    EXPECT(r && r->Value == 1u, "dec_mode wrong");

    r = FindReg(list, RKVDEC2_REG_STR_LEN);
    EXPECT(r && r->Value == 0x40u, "str_len got 0x%x", r ? r->Value : 0);

    r = FindReg(list, RKVDEC2_REG_ERROR_MODE);
    EXPECT(r && (r->Value & RKVDEC2_CUR_PIC_IS_IDR),
           "cur_pic_is_idr not set");

    /* Codec params. */
    r = FindReg(list, RKVDEC2_REG_H264_FLAGS);
    EXPECT(r && (r->Value & RKVDEC2_H264_FIRSTSLICE_FLAG),
           "firstslice flag missing");

    r = FindReg(list, RKVDEC2_REG_CUR_TOP_POC);
    EXPECT(r && r->Value == 0x12345678u, "top POC got 0x%x", r ? r->Value : 0);

    /* Address regs carry buffer handles for kernel iova substitution. */
    r = FindReg(list, RKVDEC2_REG_RLC_BASE);
    EXPECT(r && r->BufferHandle == bufs.bitstream &&
                r->IovaOffset   == bufs.bitstream_offset,
           "RLC handle/offset wrong: h=%llx off=%u",
           (unsigned long long)(r ? r->BufferHandle : 0),
           r ? r->IovaOffset : 0);

    r = FindReg(list, RKVDEC2_REG_DECOUT_BASE);
    EXPECT(r && r->BufferHandle == bufs.output_frame,
           "DECOUT handle wrong");

    r = FindReg(list, RKVDEC2_REG_COLMV_CUR_BASE);
    EXPECT(r && r->BufferHandle == bufs.colmv_cur, "COLMV_CUR handle wrong");

    /* RCB scratch buffers were zero — must NOT have been emitted. */
    EXPECT(FindReg(list, RKVDEC2_REG_RCB_BASE_FIRST) == nullptr,
           "RCB[0] emitted with zero handle");

    /* Optional tables not provided — their regs should be absent. */
    EXPECT(FindReg(list, RKVDEC2_REG_PPS_BASE)      == nullptr,
           "PPS emitted with zero handle");
    EXPECT(FindReg(list, RKVDEC2_REG_CABACTBL_BASE) == nullptr,
           "CABAC emitted with zero handle");

    /* INT_EN intentionally NOT written by the regbuilder — see
     * regbuilder_h264.cpp.  Hardware comes up with interrupts on; the
     * kernel poller acks status bits after each kick. */
    EXPECT(FindReg(list, RKVDEC2_REG_INT_EN) == nullptr,
           "INT_EN should not be emitted");
    /* Start bit. */
    r = FindReg(list, RKVDEC2_REG_START_EN);
    EXPECT(r && r->Value == RKVDEC2_DEC_E, "kick bit wrong");

    /* Kick must be the very last entry — hardware latches addresses
     * on the dec_e rising edge, so anything after it would be a bug. */
    EXPECT(list.entries[list.count - 1].Offset == RKVDEC2_REG_START_EN,
           "kick is not the final write (idx %u, off 0x%x)",
           list.count - 1, list.entries[list.count - 1].Offset);

    /* CABAC stream without cabac_init buffer should fail explicitly. */
    {
        H264ParseResult p2 = parsed;
        p2.pps.flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
        H264BufferRefs b2 = bufs;
        b2.cabac_init_table = 0;
        H264RegWriteList l2{};
        H264RegBuildStatus s2 = H264BuildRegisterList(&p2, &b2, 0, &l2);
        EXPECT(s2 == H264_REGBUILD_UNSUPPORTED,
               "CABAC w/o cabac_init didn't fail (got %d)", (int)s2);
    }

    std::printf("regbuilder_test OK (%u writes)\n", list.count);
    return 0;
}
