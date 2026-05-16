#include "dpb.h"
#include <cstdio>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

static H264ParseResult MakePic(uint16_t frame_num, uint8_t nal_ref_idc,
                               uint32_t flags)
{
    H264ParseResult r{};
    r.has_sps = r.has_pps = r.has_slice = 1;
    r.decode.flags        = flags;
    r.decode.frame_num    = frame_num;
    r.decode.nal_ref_idc  = nal_ref_idc;
    r.slice.slice_type    = V4L2_H264_SLICE_TYPE_P;
    r.sps.max_num_ref_frames = 4;
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
    DpbSelection sel{};
    EXPECT(Dpb_Init(&ctx, pool, 4) == DPB_OK, "init");

    H264ParseResult idr = MakePic(0, 3, V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC);
    EXPECT(Dpb_Select(&ctx, &idr, &sel) == DPB_OK, "IDR select");
    EXPECT(sel.current_slot == 0, "IDR slot=%u", sel.current_slot);
    Dpb_OnDecodeComplete(&ctx);

    H264ParseResult p1 = MakePic(1, 2, V4L2_H264_DECODE_PARAM_FLAG_PFRAME);
    EXPECT(Dpb_Select(&ctx, &p1, &sel) == DPB_OK, "P1 select");
    EXPECT(sel.current_slot == 1, "P1 slot=%u", sel.current_slot);
    Dpb_OnDecodeComplete(&ctx);

    H264ParseResult failed = MakePic(2, 1, V4L2_H264_DECODE_PARAM_FLAG_PFRAME);
    EXPECT(Dpb_Select(&ctx, &failed, &sel) == DPB_OK, "failed select");
    EXPECT(sel.current_slot == 2, "failed slot=%u", sel.current_slot);
    Dpb_OnDecodeFailed(&ctx);

    EXPECT(ctx.current_idx == -1, "current_idx=%d", ctx.current_idx);
    EXPECT(ctx.slots[0].in_use && ctx.slots[0].is_ref, "lost IDR ref");
    EXPECT(ctx.slots[1].in_use && ctx.slots[1].is_ref, "lost P1 ref");
    EXPECT(!ctx.slots[2].in_use && !ctx.slots[2].is_ref,
           "failed slot still active");

    H264ParseResult after = MakePic(2, 1, V4L2_H264_DECODE_PARAM_FLAG_PFRAME);
    EXPECT(Dpb_Select(&ctx, &after, &sel) == DPB_OK, "after select");
    EXPECT(sel.current_slot == 2, "after should reuse slot 2, got %u",
           sel.current_slot);
    EXPECT(sel.refs[0] == pool[0].output_frame, "after ref[0]");
    EXPECT(sel.refs[1] == pool[1].output_frame, "after ref[1]");

    std::printf("dpb_failed_current_test OK\n");
    return 0;
}
