// tests/harness/vp9_dpb_test/main.cpp
#include "vp9_dpb.h"
#include <cstdio>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

int main() {
    vp9::DpbCtx d{};
    vp9::PicParams pp{};

    // Frame 0: keyframe refreshes slot 0 only.
    pp.frame_type = 0;
    pp.refresh_frame_flags = 0x01;
    pp.width = 1280; pp.height = 720;
    vp9::Vp9Dpb_Update(d, pp, /*frame=*/0xAAA, /*colmv=*/0xBBB, 1280, 720, 8);
    CHECK(d.slots[0].valid);
    CHECK(d.slots[0].frame_handle == 0xAAA);
    CHECK(d.slots[0].colmv_handle == 0xBBB);
    CHECK(d.slots[0].width  == 1280);
    CHECK(d.slots[0].height == 720);
    CHECK(d.slots[0].bit_depth == 8);
    CHECK(d.slots[0].poc == 0);
    CHECK(!d.slots[1].valid);
    CHECK(d.next_poc == 1);

    // Frame 1: inter refreshes slot 1.
    pp.frame_type = 1;
    pp.refresh_frame_flags = 0x02;
    vp9::Vp9Dpb_Update(d, pp, /*frame=*/0xCCC, /*colmv=*/0xDDD, 1280, 720, 8);
    CHECK(d.slots[1].valid && d.slots[1].frame_handle == 0xCCC);
    CHECK(d.slots[1].poc == 1);
    CHECK(d.slots[0].frame_handle == 0xAAA);  // slot 0 unchanged
    CHECK(d.next_poc == 2);

    // Frame 2: refresh_frame_flags=0xFF (keyframe behaviour) writes
    // current into every slot.
    pp.refresh_frame_flags = 0xFF;
    vp9::Vp9Dpb_Update(d, pp, 0xEEE, 0xFFF, 1280, 720, 8);
    for (int i = 0; i < vp9::kNumRefFrames; ++i) {
        CHECK(d.slots[i].valid);
        CHECK(d.slots[i].frame_handle == 0xEEE);
        CHECK(d.slots[i].poc == 2);
    }

    // show_existing of a valid slot returns its handle.
    pp.show_existing_frame = 1;
    pp.show_existing_frame_idx = 3;
    CHECK(vp9::Vp9Dpb_ShowExisting(d, pp) == 0xEEE);

    // show_existing of an out-of-range slot returns 0.
    pp.show_existing_frame_idx = 9;
    CHECK(vp9::Vp9Dpb_ShowExisting(d, pp) == 0);

    // show_existing of an invalid slot returns 0.
    vp9::DpbCtx empty{};
    pp.show_existing_frame_idx = 0;
    CHECK(vp9::Vp9Dpb_ShowExisting(empty, pp) == 0);

    return 0;
}
