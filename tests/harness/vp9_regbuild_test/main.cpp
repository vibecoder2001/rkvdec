// tests/harness/vp9_regbuild_test/main.cpp — dense-bank shape tests for
// the VP9 regbuilder.  B2 exercises the common bank; B3 adds codec_params
// checks (reg103 param_flags, POCs, scaling factors).
// B4 adds codec_addr iova-slot checks and Vp9Regbuilder_FillProbs tests.
#include "regbuilder_vp9.h"
#include "rkvdec2_vp9_regs.h"
#include <cstdio>
#include <cstring>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

int main() {
    vp9::PicParams pp{};
    pp.frame_type = 0;
    pp.profile = 0;
    pp.bit_depth = 8;
    pp.width = 1280;
    pp.height = 720;
    pp.refresh_frame_flags = 0xFF;
    pp.tile_cols_log2 = 0;
    pp.tile_rows_log2 = 0;
    pp.header_size = 80;
    pp.frame_size = 2048;
    pp.uncompressed_header_size = 16;

    vp9::DpbCtx dpb{};

    vp9::RegbuildInputs in{};
    in.pp = &pp;
    in.dpb = &dpb;
    in.bitstream_handle = 1;
    in.bitstream_bytes  = pp.frame_size;
    in.decout_frame_handle = 2;
    in.decout_colmv_handle = 3;
    in.prob_handle         = 4;
    in.prob_default_handle = 5;
    in.segid_last_handle   = 6;
    in.segid_cur_handle    = 7;
    for (int i = 0; i < 10; ++i) in.rcb_handles[i] = 100u + i;
    in.error_ref_handle    = 20;

    H26xDenseOutput out{};
    auto st = vp9::Vp9Regbuilder_Fill(in, &out);
    CHECK(st == vp9::RegBuildStatus::Ok);

    // dec_mode (idx 9) sits at common[1]. VP9 = 2.
    CHECK(out.Bank.Common[9u - RKMPP_DENSE_COMMON_FIRST] == RKVDEC2_DEC_MODE_VP9);

    // Kick value: bit 0 of REG_START_EN (idx 10) is dec_e.
    CHECK(out.KickValue == RKVDEC2_DEC_E);

    // Kick slot in the bank must stay zero — the kernel writes the
    // kick value separately as the very last MMIO write.
    CHECK(out.Bank.Common[RKMPP_DENSE_KICK_REG_IDX - RKMPP_DENSE_COMMON_FIRST] == 0);

    // Stream length plumbed into common[STR_LEN].
    // str_len = align16(bitstream_bytes) + 128 (BSP-matched padding).
    CHECK(out.Bank.Common[16u - RKMPP_DENSE_COMMON_FIRST] ==
          (((pp.frame_size + 15u) & ~15u) + 128u));

    // Keyframe iova slot count:
    //   reg128..132 common-addr     = 5
    //   reg133..142 RCB             = 10
    //   reg160 delta_prob           = 1
    //   reg161 pps_base (scratch)   = 1
    //   reg162 last_prob            = 1
    //   reg163 rps_base (scratch)   = 1
    //   reg164/165/166 self-ref     = 3
    //   reg167 count_prob           = 1
    //   reg168/169 segid last/cur   = 2
    //   reg170 ref_colmv self-ref   = 1
    //   reg171 intercmd_base        = 1
    //   reg172 update_prob_wr       = 1
    //   reg173..179 scratch         = 7
    //   reg181..196 per-slot colmv = 16 [stride-1, all scratch-filled]
    // Total = 51
    CHECK(out.IovaSlotCount == 51);

    // ---- B3 (post-B5 layout fix): reg103 keyframe ------------------
    // Bit layout starts at bit 20 (per BSP Vdpu34xVp9dParam.reg103).
    // Default-pp keyframe: refresh_frame_context=0, so prob_save_en
    // (bit 22) is NOT set; allow_high_precision_mv=0 (bit 29) not set;
    // last_intra_only=0 (bit 30) not set.
    //   bit 20 prob_update_en      = 0x00100000
    //   bit 21 refresh_en          = 0x00200000  (no err_resilient/parallel)
    //   bit 23 intra_only_flag     = 0x00800000
    //   bit 25 ref_mode_rfsh_en    = 0x02000000
    //   bit 26 single_ref_rfsh_en  = 0x04000000
    //   bit 27 comp_ref_rfsh_en    = 0x08000000
    //   Sum = 0x0EB00000
    {
        uint32_t reg103 = out.Bank.CodecParams[103u - RKMPP_DENSE_CPARAM_FIRST];
        CHECK(reg103 == 0x0EB00000u);
    }

    // cur_poc = dpb.next_poc + 1.  Default-constructed DpbCtx has
    // next_poc==0, so the first kick writes 1 — matching BSP capture
    // (regs_kf/regs_000.bin reg65 == 1).
    CHECK(out.Bank.CodecParams[65u - RKMPP_DENSE_CPARAM_FIRST] == 1u);

    // ---- B5: common-bank fills (BSP-captured values) ----------------
    // reg11 IMPORTANT_EN = 0x01000062.
    CHECK(out.Bank.Common[11u - RKMPP_DENSE_COMMON_FIRST] == 0x01000062u);
    // reg12 SECONDARY_EN = 0x82 (NO dec_global_en for VP9).
    CHECK(out.Bank.Common[12u - RKMPP_DENSE_COMMON_FIRST] == 0x00000082u);
    // reg13 ERROR_MODE: keyframe has CUR_PIC_IS_IDR set.
    CHECK(out.Bank.Common[13u - RKMPP_DENSE_COMMON_FIRST] == 0x01000001u);
    // reg26 BLOCK_GATING = 0x800FFFEF.
    CHECK(out.Bank.Common[26u - RKMPP_DENSE_COMMON_FIRST] == 0x800FFFEFu);
    // reg32 TIMEOUT_THRESH = 0x3FFFF.
    CHECK(out.Bank.Common[32u - RKMPP_DENSE_COMMON_FIRST] == 0x0003FFFFu);
    // reg18/19/20 strides (1280x720, 8-bit): hor = 80, y_vir = 57600.
    CHECK(out.Bank.Common[18u - RKMPP_DENSE_COMMON_FIRST] == 80u);
    CHECK(out.Bank.Common[19u - RKMPP_DENSE_COMMON_FIRST] == 80u);
    CHECK(out.Bank.Common[20u - RKMPP_DENSE_COMMON_FIRST] == 57600u);

    // reg77 intercmd_num = 0.
    CHECK(out.Bank.CodecParams[77u - RKMPP_DENSE_CPARAM_FIRST] == 0u);

    // ---- B3: inter-frame codec_params checks ------------------------
    {
        vp9::PicParams pp2{};
        pp2.frame_type = 1;                // INTER
        pp2.refresh_frame_context = 1;
        pp2.allow_high_precision_mv = 1;
        pp2.txmode = 4;                    // TX_MODE_SELECT  → bit 2
        pp2.interp_filter = 4;             // SWITCHABLE      → bit 3
        pp2.width = 1280; pp2.height = 720;
        pp2.frame_refs[0].index = 0;
        pp2.frame_refs[1].index = 1;
        pp2.frame_refs[2].index = 2;

        vp9::DpbCtx dpb2{};
        dpb2.slots[0] = { true, 0x1000, 0x2000, 1280, 720, 8, 0 };
        dpb2.slots[1] = { true, 0x1001, 0x2001, 1280, 720, 8, 1 };
        dpb2.slots[2] = { true, 0x1002, 0x2002, 1280, 720, 8, 2 };
        dpb2.next_poc = 3;

        vp9::RegbuildInputs in2{};
        in2.pp = &pp2; in2.dpb = &dpb2;
        in2.bitstream_handle = 1;
        in2.decout_frame_handle = 2;
        in2.decout_colmv_handle = 3;
        in2.prob_handle = 4; in2.prob_default_handle = 5;
        in2.segid_last_handle = 6; in2.segid_cur_handle = 7;
        for (int i = 0; i < 10; ++i) in2.rcb_handles[i] = 100u + i;
        in2.error_ref_handle = 20;

        H26xDenseOutput out2{};
        auto st2 = vp9::Vp9Regbuilder_Fill(in2, &out2);
        CHECK(st2 == vp9::RegBuildStatus::Ok);

        // reg103 inter frame, BSP-verified bit layout (starts at bit 20).
        // Inputs: txmode=4, interp_filter=4, refresh_frame_context=1,
        // allow_high_precision_mv=1, error_resilient_mode=0,
        // frame_parallel_decoding_mode=0, last_intra_only=0.
        //   bit 20 prob_update_en           = 0x00100000
        //   bit 21 refresh_en               = 0x00200000
        //   bit 22 prob_save_en             = 0x00400000
        //   bit 24 txfmmode_rfsh_en         = 0x01000000  (txmode==4)
        //   bit 25 ref_mode_rfsh_en         = 0x02000000
        //   bit 26 single_ref_rfsh_en       = 0x04000000
        //   bit 27 comp_ref_rfsh_en         = 0x08000000
        //   bit 28 interp_filter_switch_en  = 0x10000000  (ifilter==4)
        //   bit 29 allow_high_precision_mv  = 0x20000000
        //   Sum = 0x3F700000
        {
            uint32_t reg103_i = out2.Bank.CodecParams[103u - RKMPP_DENSE_CPARAM_FIRST];
            CHECK(reg103_i == 0x3F700000u);
        }

        // POC: cur_poc = dpb.next_poc = 3.
        // cur_poc = dpb.next_poc + 1 = 3 + 1 = 4.
        CHECK(out2.Bank.CodecParams[65u - RKMPP_DENSE_CPARAM_FIRST] == 4u);

        // Last POC = slot 0's poc = 0.
        CHECK(out2.Bank.CodecParams[95u - RKMPP_DENSE_CPARAM_FIRST] == 0u);

        // Per-ref scaling lives in reg88..reg93 (separate hor/ver u32 regs).
        // Identity scaling for slot 0 (ref dims == current dims): hscale=16384.
        uint32_t lref_hor = out2.Bank.CodecParams[88u - RKMPP_DENSE_CPARAM_FIRST];
        uint32_t lref_ver = out2.Bank.CodecParams[89u - RKMPP_DENSE_CPARAM_FIRST];
        CHECK(lref_hor == 16384u);
        CHECK(lref_ver == 16384u);
        // reg106/107 hold last-frame dims (not scale): 1280 / 720.
        CHECK(out2.Bank.CodecParams[106u - RKMPP_DENSE_CPARAM_FIRST] == 1280u);
        CHECK(out2.Bank.CodecParams[107u - RKMPP_DENSE_CPARAM_FIRST] == 720u);
        // reg104/105 must remain zero (BSP `_no_use` / count_update_en).
        CHECK(out2.Bank.CodecParams[104u - RKMPP_DENSE_CPARAM_FIRST] == 0u);
        CHECK(out2.Bank.CodecParams[105u - RKMPP_DENSE_CPARAM_FIRST] == 0u);

        // Per-ref strides for inter frame: reg79..84 = 80 (= 0x50)
        // and reg85..87 = 57600 (= 0xE100), matching BSP capture for
        // 1280x720 8-bit refs.
        CHECK(out2.Bank.CodecParams[79u - RKMPP_DENSE_CPARAM_FIRST] == 80u);
        CHECK(out2.Bank.CodecParams[80u - RKMPP_DENSE_CPARAM_FIRST] == 80u);
        CHECK(out2.Bank.CodecParams[81u - RKMPP_DENSE_CPARAM_FIRST] == 80u);
        CHECK(out2.Bank.CodecParams[82u - RKMPP_DENSE_CPARAM_FIRST] == 80u);
        CHECK(out2.Bank.CodecParams[83u - RKMPP_DENSE_CPARAM_FIRST] == 80u);
        CHECK(out2.Bank.CodecParams[84u - RKMPP_DENSE_CPARAM_FIRST] == 80u);
        CHECK(out2.Bank.CodecParams[85u - RKMPP_DENSE_CPARAM_FIRST] == 57600u);
        CHECK(out2.Bank.CodecParams[86u - RKMPP_DENSE_CPARAM_FIRST] == 57600u);
        CHECK(out2.Bank.CodecParams[87u - RKMPP_DENSE_CPARAM_FIRST] == 57600u);

        // reg13 inter ERROR_MODE = 0x1 (no CUR_PIC_IS_IDR on inter).
        CHECK(out2.Bank.Common[13u - RKMPP_DENSE_COMMON_FIRST] == 0x00000001u);

        // ---- B4 (post-B5 layout): iova slot count for inter case ------
        // pp.frame_refs[0..2].index = {0, 1, 2}; slots 0/1/2 valid.
        // pp.ref_frame_map[i] all default to index=0 → slot 0 valid.
        // Breakdown:
        // Inter slot count: same shape as keyframe.  Per-slot colmv
        // writes all 16 stride-1 entries (real handle for valid slots,
        // scratch for invalid).  Total = 51.
        CHECK(out2.IovaSlotCount == 51);

        auto has_slot = [&](uint32_t reg_idx) {
            for (uint32_t i = 0; i < out2.IovaSlotCount; ++i)
                if (out2.IovaSlots[i].RegIdx == reg_idx) return true;
            return false;
        };
        CHECK(has_slot(128));   // bitstream
        CHECK(has_slot(129));   // decout
        CHECK(has_slot(160));   // delta_prob
        CHECK(has_slot(162));   // last_prob
        CHECK(has_slot(164));   // ref_last
        CHECK(has_slot(165));   // ref_golden
        CHECK(has_slot(166));   // ref_altref
        CHECK(has_slot(167));   // count_prob
        CHECK(has_slot(168));   // segid_last
        CHECK(has_slot(169));   // segid_cur
        CHECK(has_slot(170));   // ref_colmv (last)
        CHECK(has_slot(172));   // update_prob_wr
        CHECK(has_slot(181));   // per-slot colmv[0]
    }

    // ---- B4: Vp9Regbuilder_FillProbs tests -------------------------

    // (c1) Keyframe: memset+return — buffer must stay all-zero.
    {
        uint8_t prob_buf[7088];
        memset(prob_buf, 0xAB, sizeof(prob_buf));   // poison

        vp9::PicParams kf_pp{};
        kf_pp.frame_type = 0;   // KEY

        vp9::ProbUpdates pu{};

        vp9::Vp9Regbuilder_FillProbs(kf_pp, pu, prob_buf);

        // B4 keyframe path: memset(0) then immediate return.
        // All bytes should be zero after the call.
        bool all_zero = true;
        for (size_t i = 0; i < sizeof(prob_buf); ++i) {
            if (prob_buf[i] != 0) { all_zero = false; break; }
        }
        CHECK(all_zero);
    }

    // (c2) Null pointer guard — must not crash.
    {
        vp9::PicParams dummy_pp{};
        vp9::ProbUpdates dummy_pu{};
        vp9::Vp9Regbuilder_FillProbs(dummy_pp, dummy_pu, nullptr);
        // reaches here → ok
    }

    // (c3) Inter frame with skip_present: skip[3] bytes appear at offset 0.
    {
        uint8_t prob_buf[7088];
        memset(prob_buf, 0, sizeof(prob_buf));

        vp9::PicParams inter_pp{};
        inter_pp.frame_type = 1;   // INTER

        vp9::ProbUpdates pu{};
        pu.skip_present = 1;
        pu.skip[0] = 0x11;
        pu.skip[1] = 0x22;
        pu.skip[2] = 0x33;

        vp9::Vp9Regbuilder_FillProbs(inter_pp, pu, prob_buf);

        // B4 placeholder layout: skip written first at offset 0.
        CHECK(prob_buf[0] == 0x11);
        CHECK(prob_buf[1] == 0x22);
        CHECK(prob_buf[2] == 0x33);
        // Byte just past skip should still be zero (nothing else set).
        CHECK(prob_buf[3] == 0x00);
    }

    return 0;
}
