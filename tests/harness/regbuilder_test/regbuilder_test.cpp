/* tests/harness/regbuilder_test/regbuilder_test.cpp
 *
 * Synthetic V4L2 controls + canned buffer handles → dense bank.
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

/* Return the bank value at swreg byte offset `off`, or 0 if the offset
 * doesn't land in a covered bank. */
static uint32_t BankValueAt(const H26xDenseOutput &out, uint32_t off)
{
    uint32_t *p = H26xDenseSlotFor(const_cast<H26xDenseOutput *>(&out), off);
    return p ? *p : 0u;
}

/* Find the (first) iova slot targeting reg byte offset `off`. */
static const RKMPP_DENSE_IOVA_SLOT*
FindIovaSlot(const H26xDenseOutput &out, uint32_t off)
{
    uint32_t idx = off / 4u;
    for (uint32_t i = 0; i < out.IovaSlotCount; i++) {
        if (out.IovaSlots[i].RegIdx == idx) return &out.IovaSlots[i];
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
    parsed.sps.chroma_format_idc          = 1;
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

    H26xDenseOutput dense{};
    H264RegBuildStatus s = H264BuildDenseRegs(&parsed, &bufs, /*pic_idx=*/0, &dense);
    EXPECT(s == H264_REGBUILD_OK, "build status %d", (int)s);

    /* Common-bank essentials. */
    EXPECT(BankValueAt(dense, RKVDEC2_REG_DEC_MODE) == 1u, "dec_mode wrong");
    EXPECT(BankValueAt(dense, RKVDEC2_REG_STR_LEN) == 0x40u,
           "str_len got 0x%x", BankValueAt(dense, RKVDEC2_REG_STR_LEN));
    EXPECT((BankValueAt(dense, RKVDEC2_REG_ERROR_MODE) &
            RKVDEC2_CUR_PIC_IS_IDR) != 0u, "cur_pic_is_idr not set");
    /* Codec params.  reg64 stays at zero — BSP H.264 capture shows
     * reg64=0 (the FIRSTSLICE_FLAG isn't programmed from user-mode). */
    EXPECT(BankValueAt(dense, RKVDEC2_REG_H264_FLAGS) == 0u,
           "reg64 should be zero, got 0x%x",
           BankValueAt(dense, RKVDEC2_REG_H264_FLAGS));
    EXPECT(BankValueAt(dense, RKVDEC2_REG_CUR_TOP_POC) == 0x12345678u,
           "top POC got 0x%x", BankValueAt(dense, RKVDEC2_REG_CUR_TOP_POC));

    /* Address regs are recorded as iova slots — bank slot stays zero,
     * kernel substitutes at submit time. */
    auto *iv = FindIovaSlot(dense, RKVDEC2_REG_RLC_BASE);
    EXPECT(iv && iv->BufferHandle == bufs.bitstream &&
                  iv->IovaOffset   == bufs.bitstream_offset,
           "RLC handle/offset wrong: h=%llx off=%u",
           (unsigned long long)(iv ? iv->BufferHandle : 0),
           iv ? iv->IovaOffset : 0);
    EXPECT(BankValueAt(dense, RKVDEC2_REG_RLC_BASE) == 0u,
           "RLC bank slot should be 0 pre-substitution");

    iv = FindIovaSlot(dense, RKVDEC2_REG_DECOUT_BASE);
    EXPECT(iv && iv->BufferHandle == bufs.output_frame,
           "DECOUT handle wrong");

    iv = FindIovaSlot(dense, RKVDEC2_REG_COLMV_CUR_BASE);
    EXPECT(iv && iv->BufferHandle == bufs.colmv_cur,
           "COLMV_CUR handle wrong");

    /* RCB scratch buffers were zero — should not be recorded as iova slots. */
    EXPECT(FindIovaSlot(dense, RKVDEC2_REG_RCB_BASE_FIRST) == nullptr,
           "RCB[0] emitted with zero handle");

    /* Optional tables not provided — no iova slot recorded. */
    EXPECT(FindIovaSlot(dense, RKVDEC2_REG_PPS_BASE) == nullptr,
           "PPS emitted with zero handle");
    EXPECT(FindIovaSlot(dense, RKVDEC2_REG_CABACTBL_BASE) == nullptr,
           "CABAC emitted with zero handle");

    /* INT_EN intentionally NOT written by the regbuilder.  Hardware
     * comes up with interrupts on; the kernel poller acks status bits
     * after each kick. */
    EXPECT(BankValueAt(dense, RKVDEC2_REG_INT_EN) == 0u,
           "INT_EN should not be set in bank");

    /* Kick bit goes into KickValue, not into the common bank slot.
     * Common[idx 10] (i.e. byte offset 0x28) must stay zero so the
     * kernel's bulk-write doesn't kick prematurely. */
    EXPECT(dense.KickValue == RKVDEC2_DEC_E, "kick value wrong: 0x%x",
           dense.KickValue);
    EXPECT(BankValueAt(dense, RKVDEC2_REG_START_EN) == 0u,
           "kick slot in bank must be zero (kernel writes it last)");

    /* CABAC stream without cabac_init buffer should fail explicitly. */
    {
        H264ParseResult p2 = parsed;
        p2.pps.flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
        H264BufferRefs b2 = bufs;
        b2.cabac_init_table = 0;
        H26xDenseOutput d2{};
        H264RegBuildStatus s2 = H264BuildDenseRegs(&p2, &b2, 0, &d2);
        EXPECT(s2 == H264_REGBUILD_UNSUPPORTED,
               "CABAC w/o cabac_init didn't fail (got %d)", (int)s2);
    }

    std::printf("regbuilder_test OK (%u iova slots, kick=0x%08x)\n",
                dense.IovaSlotCount, dense.KickValue);
    return 0;
}
