/* tests/harness/dpb_test/dpb_test.cpp
 *
 * DPB selection scenarios:
 *   1. IDR → slot 0 picked, all dpb_entries cleared, all ref_lists zero
 *   2. IDR (ref) + P (ref) → P picks slot 1, slot 0 active in DPB
 *   3. IDR + non-ref P → P slot freed on completion
 *   4. Sliding window: refs spill out when pool fills up
 */
#include "dpb.h"
#include <cstdio>
#include <cstring>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

static H264ParseResult MakeIdr(uint16_t frame_num, uint8_t nal_ref_idc) {
    H264ParseResult r{};
    r.has_sps = r.has_pps = r.has_slice = 1;
    r.decode.flags        = V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;
    r.decode.frame_num    = frame_num;
    r.decode.nal_ref_idc  = nal_ref_idc;
    return r;
}
static H264ParseResult MakeP(uint16_t frame_num, uint8_t nal_ref_idc) {
    H264ParseResult r{};
    r.has_sps = r.has_pps = r.has_slice = 1;
    r.decode.flags        = V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
    r.decode.frame_num    = frame_num;
    r.decode.nal_ref_idc  = nal_ref_idc;
    r.slice.slice_type    = V4L2_H264_SLICE_TYPE_P;
    return r;
}

int main()
{
    DpbPoolEntry pool[4];
    for (int i = 0; i < 4; i++) {
        pool[i].output_frame = 0xF000000000000001ULL + (uint64_t)i;
        pool[i].colmv        = 0xC000000000000001ULL + (uint64_t)i;
    }

    DpbCtx ctx;
    EXPECT(Dpb_Init(&ctx, pool, 4) == DPB_OK, "init failed");

    /* --- 1. IDR -------------------------------------------------- */
    H264ParseResult idr = MakeIdr(0, 3);
    DpbSelection sel{};
    EXPECT(Dpb_Select(&ctx, &idr, &sel) == DPB_OK, "IDR select failed");
    EXPECT(sel.current_slot   == 0,                 "IDR slot=%u", sel.current_slot);
    EXPECT(sel.current_output == pool[0].output_frame, "IDR output handle wrong");
    EXPECT(sel.current_colmv  == pool[0].colmv,        "IDR colmv handle wrong");

    /* No refs: every entry of refs[]/dpb_entries/ref_lists must be zero. */
    for (int i = 0; i < DPB_MAX_SLOTS; i++) {
        EXPECT(sel.refs[i] == 0,      "IDR refs[%d]=0x%llx",
               i, (unsigned long long)sel.refs[i]);
        EXPECT(sel.ref_colmv[i] == 0, "IDR ref_colmv[%d]=0x%llx",
               i, (unsigned long long)sel.ref_colmv[i]);
        EXPECT(sel.dpb_entries[i].flags == 0,
               "IDR dpb_entries[%d].flags=0x%x", i, sel.dpb_entries[i].flags);
    }
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 32; i++) {
            EXPECT(sel.ref_lists[j][i].fields == 0 &&
                   sel.ref_lists[j][i].index  == 0,
                   "IDR ref_lists[%d][%d] non-zero", j, i);
        }
    }

    Dpb_OnDecodeComplete(&ctx);  /* IDR is ref → stays in slot 0 */

    /* --- 2. P after IDR ------------------------------------------ */
    H264ParseResult p1 = MakeP(1, 2);
    EXPECT(Dpb_Select(&ctx, &p1, &sel) == DPB_OK, "P select failed");
    EXPECT(sel.current_slot   == 1,                 "P slot=%u", sel.current_slot);
    EXPECT(sel.current_output == pool[1].output_frame, "P output handle");

    /* slot 0 (the IDR) should be exposed as a valid ref. */
    EXPECT(sel.refs[0]      == pool[0].output_frame, "ref[0] handle");
    EXPECT(sel.ref_colmv[0] == pool[0].colmv,         "ref_colmv[0] handle");
    EXPECT((sel.dpb_entries[0].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID),
           "ref[0] not marked VALID");
    EXPECT((sel.dpb_entries[0].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE),
           "ref[0] not marked ACTIVE");
    EXPECT(sel.dpb_entries[0].frame_num == 0,
           "ref[0] frame_num=%u", sel.dpb_entries[0].frame_num);

    Dpb_OnDecodeComplete(&ctx);  /* P is ref → stays in slot 1 */

    /* --- 3. Non-ref P frees its slot on completion --------------- */
    H264ParseResult p2 = MakeP(2, 0);
    EXPECT(Dpb_Select(&ctx, &p2, &sel) == DPB_OK, "non-ref P select");
    EXPECT(sel.current_slot == 2, "non-ref P slot=%u", sel.current_slot);
    Dpb_OnDecodeComplete(&ctx);  /* nal_ref_idc=0 → release slot 2 */

    /* Next pic should reuse slot 2. */
    H264ParseResult p3 = MakeP(3, 1);
    EXPECT(Dpb_Select(&ctx, &p3, &sel) == DPB_OK, "p3 select");
    EXPECT(sel.current_slot == 2, "p3 should reuse slot 2 (got %u)",
           sel.current_slot);
    Dpb_OnDecodeComplete(&ctx);  /* p3 is ref */

    /* --- 4. Sliding window evicts oldest short-term ref ---------- */
    /* Pool size is 4. After IDR(slot0) + p1(slot1) + p3(slot2) we have
     * 3 refs. Add one more — slot 3 is taken. Then a 5th picture has
     * to evict the oldest short-term (slot0, frame_num=0). */
    H264ParseResult p4 = MakeP(4, 1);
    EXPECT(Dpb_Select(&ctx, &p4, &sel) == DPB_OK, "p4 select");
    EXPECT(sel.current_slot == 3, "p4 slot=%u", sel.current_slot);
    Dpb_OnDecodeComplete(&ctx);

    H264ParseResult p5 = MakeP(5, 1);
    DpbSelection sel5{};
    EXPECT(Dpb_Select(&ctx, &p5, &sel5) == DPB_OK, "p5 select");
    /* slot 0 was the oldest short-term; should now be the chosen slot. */
    EXPECT(sel5.current_slot == 0, "p5 should evict slot 0 (got %u)",
           sel5.current_slot);
    /* sel5.refs[0] is the new pic itself → not a ref of itself; should be 0. */
    EXPECT(sel5.refs[0] == 0, "p5 refs[0] must be zero (it's current)");
    /* slots 1, 2, 3 should still be valid refs. */
    EXPECT(sel5.refs[1] == pool[1].output_frame, "p5 refs[1]");
    EXPECT(sel5.refs[2] == pool[2].output_frame, "p5 refs[2]");
    EXPECT(sel5.refs[3] == pool[3].output_frame, "p5 refs[3]");

    /* --- 5. New IDR flushes everything --------------------------- */
    H264ParseResult idr2 = MakeIdr(0, 3);
    EXPECT(Dpb_Select(&ctx, &idr2, &sel) == DPB_OK, "idr2 select");
    EXPECT(sel.current_slot == 0, "idr2 slot=%u", sel.current_slot);
    for (int i = 0; i < DPB_MAX_SLOTS; i++) {
        EXPECT(sel.refs[i] == 0, "idr2 refs[%d] not flushed", i);
    }

    std::printf("dpb_test OK\n");
    return 0;
}
