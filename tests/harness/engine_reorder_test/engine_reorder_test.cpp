/* tests/harness/engine_reorder_test/engine_reorder_test.cpp
 *
 * Host-side regression test for the multi-IDR reorder bug fixed in
 * commit 4c94bfa.  Drives DecodeEngine_OnDecodeComplete with synthetic
 * ReorderEntry streams (no hardware required) and verifies that every
 * submitted entry pops out via DecodeEngine_PollFrame in the expected
 * order, including across H.264 IDR boundaries where POC resets to 0.
 *
 * Pre-fix bug: DecodeEngine_OnDecodeComplete (then inlined in Submit)
 * sorted reorder_q purely by POC.  H.264 POC resets at every IDR, so
 * the previous GOP's high-POC tail entry got "stuck" in reorder_q
 * because every post-IDR frame had POC < the stuck entry's POC, so
 * bump_lowest never picked it.  It eventually popped many GOPs later
 * with stale pts.  This test simulates that exact scenario and
 * asserts the fix's IDR-spill keeps every entry in correct submit
 * order.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
/* `<initguid.h>` MUST precede the IOCTL header (transitively pulled in
 * by `decode_engine.h` → `regbuilder_h264.h` → `rkmpp_ioctl.h`) so the
 * `DEFINE_GUID(GUID_DEVINTERFACE_RKMPP, ...)` macro instantiates the
 * symbol the engine references via SetupDi. */
#include <windows.h>
#include <initguid.h>
#include "decode_engine.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); \
                   return 1; } \
} while (0)

namespace {

/* Build a minimal ReorderEntry suitable for queue-ordering tests.
 * Only fields the queue logic touches (poc, pts_hns, dur_hns, slot_idx,
 * epoch) are populated; yuv stays empty.  Caller threads pts so we can
 * verify post-pop ordering. */
DecodeEngine::ReorderEntry MakeEntry(int32_t poc, int64_t pts_hns,
                                     uint32_t epoch = 0) {
    DecodeEngine::ReorderEntry e;
    e.poc      = poc;
    e.pts_hns  = pts_hns;
    e.dur_hns  = 333333;            /* arbitrary; not used in ordering */
    e.slot_idx = -1;                /* skip the external_hold path */
    e.is_ref   = true;
    e.epoch    = epoch;
    return e;
}

/* Drain ready_q via PollFrame, returning popped pts values in order.
 * reorder_q residue (anything still pending) is NOT drained — the
 * caller drives the IDR boundary. */
std::vector<int64_t> PopReadyPtsAll(DecodeEngine *e) {
    std::vector<int64_t> out;
    DecodedFrame f;
    while (DecodeEngine_PollFrame(e, &f) == 1) {
        out.push_back(f.pts_hns);
    }
    return out;
}

/* ============================================================
 * Test 1 — single GOP, max_num_reorder_pics=1, no IDR boundary.
 * ============================================================
 * Simulates a non-bframe GOP: every Submit produces one bump.  Verifies
 * the steady-state behavior is unchanged by the IDR-spill fix.
 */
int test_single_gop()
{
    DecodeEngine eng{};
    eng.codec = Codec::H264;

    /* Submit POC 0, 2, 4, 6, 8 (synthetic IDR + 4 P-frames).  pts ramps
     * monotonically.  max_num_reorder_pics=1 means each submit bumps
     * the previous lowest-POC entry into ready_q. */
    for (int p = 0; p <= 8; p += 2) {
        bool is_idr = (p == 0);
        DecodeEngine_OnDecodeComplete(
            &eng, MakeEntry(p, p * 333333LL),
            /*max_num_reorder_pics=*/1, is_idr);
    }
    /* After 5 submits with cap=1: 4 bumped to ready_q, 1 left in
     * reorder_q (the most recent).  Drain it. */
    DecodeEngine_Drain(&eng);

    auto pts = PopReadyPtsAll(&eng);
    EXPECT(pts.size() == 5, "expected 5 popped, got %zu", pts.size());
    for (size_t i = 0; i < pts.size(); i++) {
        const int64_t expect = (int64_t)i * 2 * 333333;
        EXPECT(pts[i] == expect,
               "frame %zu: pts=%lld != expected %lld",
               i, (long long)pts[i], (long long)expect);
    }
    return 0;
}

/* ============================================================
 * Test 2 — multi-IDR, the regression case for commit 4c94bfa.
 * ============================================================
 * GOP A: POC 0, 2, ..., 494, 496, 498   (250 entries, ending at high POC)
 * GOP B starts with IDR (POC 0), continues 0, 2, ..., 498  (also 250)
 * GOP C starts with IDR (POC 0), continues 0, 2, ..., 498
 *
 * Without IDR-spill: GOP A's POC=498 stays stuck in reorder_q
 * (bump_lowest always prefers GOP B/C's lower POCs). It pops with
 * GOP-A pts after GOP B's POC climbs past 498 — i.e. nearly never
 * within the test window unless GOP B reaches the same POC.  In the
 * actual bug, content+pts of stuck entry was wildly stale.
 *
 * With IDR-spill: GOP A is fully drained to ready_q before GOP B's
 * IDR enters reorder_q.  ready_q sees GOP A in pts order, then GOP B,
 * then GOP C.  Every submit shows up exactly once, in submit order.
 */
int test_multi_idr()
{
    DecodeEngine eng{};
    eng.codec = Codec::H264;

    constexpr int kPocStep = 2;
    constexpr int kFramesPerGop = 250;        /* POC 0..498 */
    constexpr int kNumGops = 3;
    constexpr uint32_t kMaxReorder = 1;       /* non-bframe steady state */

    /* Track every submitted pts so we can confirm exactly one pop per. */
    std::vector<int64_t> submitted_pts;

    int64_t pts = 0;
    for (int gop = 0; gop < kNumGops; gop++) {
        for (int i = 0; i < kFramesPerGop; i++) {
            const int32_t poc = i * kPocStep;
            const bool is_idr = (i == 0);     /* IDR at start of every GOP */
            submitted_pts.push_back(pts);
            DecodeEngine_OnDecodeComplete(
                &eng, MakeEntry(poc, pts),
                kMaxReorder, is_idr);
            pts += 333333;
        }
    }
    /* End-of-stream: spill anything still pending in reorder_q. */
    DecodeEngine_Drain(&eng);

    auto popped = PopReadyPtsAll(&eng);
    EXPECT(popped.size() == submitted_pts.size(),
           "popped=%zu submitted=%zu (some entries got stuck or duplicated)",
           popped.size(), submitted_pts.size());

    /* Every submitted pts must appear exactly once, and they must come
     * out in submit order (== pts ascending, since we synthesized that
     * way).  Pre-fix: the IDR-boundary entries (POC=498 of GOP A, POC=498
     * of GOP B) would arrive out of order or with duplicates. */
    for (size_t i = 0; i < popped.size(); i++) {
        EXPECT(popped[i] == submitted_pts[i],
               "frame %zu: popped pts=%lld != submitted pts=%lld "
               "(reorder bug — IDR boundary skipped a stuck entry)",
               i, (long long)popped[i], (long long)submitted_pts[i]);
    }
    return 0;
}

/* ============================================================
 * Test 3 — POC-only sort fails without IDR spill.
 * ============================================================
 * Negative-control: explicitly call OnDecodeComplete with `is_idr=false`
 * across the GOP boundary and verify that the reorder_q does NOT spill
 * — so the test would FAIL on a regressed code path that ignores IDR.
 *
 * This protects the IDR-spill behavior from being silently removed:
 * if someone reverts the spill, test 2 fails; if someone changes the
 * spill condition (e.g. always-spill), this test catches that too.
 */
int test_no_spill_without_idr()
{
    DecodeEngine eng{};
    eng.codec = Codec::H264;

    /* Submit POC 100 (a "tail" frame from a hypothetical GOP) with
     * is_idr=false — should land in reorder_q. */
    DecodeEngine_OnDecodeComplete(&eng, MakeEntry(100, 1000000), 1, false);
    EXPECT(eng.reorder_q.size() == 1, "expected 1 in reorder_q (no bump yet)");
    EXPECT(eng.ready_q.size() == 0, "expected 0 in ready_q");

    /* Submit POC 0 (a hypothetical new-GOP IDR-shaped entry) but with
     * is_idr=false.  Without IDR-spill, the tail (POC=100) stays — it
     * gets stuck.  bump_lowest fires once because reorder_q size > 1,
     * picks POC=0 (lowest), pushes to ready_q. */
    DecodeEngine_OnDecodeComplete(&eng, MakeEntry(0, 2000000), 1, false);
    EXPECT(eng.reorder_q.size() == 1, "expected 1 in reorder_q (POC 100 stuck)");
    EXPECT(eng.ready_q.size() == 1, "expected 1 in ready_q (POC 0 bumped)");
    EXPECT(eng.reorder_q[0].poc == 100, "POC 100 should be the stuck one");
    EXPECT(eng.ready_q[0].poc == 0, "POC 0 should have been bumped");

    /* Now confirm the IDR-spill flag fixes it: re-submit the same scene
     * with is_idr=true on POC 0.  The spill drains POC=100 (still in
     * reorder_q from above) into ready_q FIRST, then pushes POC 0. */
    DecodeEngine_OnDecodeComplete(&eng, MakeEntry(0, 3000000), 1, true);
    /* After spill: POC 100 → ready_q.  Then push POC 0 (the fresh IDR).
     * reorder_q now has [POC 0]; size=1 ≤ max=1; no bump. */
    EXPECT(eng.reorder_q.size() == 1, "expected 1 in reorder_q after IDR push");
    EXPECT(eng.reorder_q[0].poc == 0, "fresh IDR should be in reorder_q");
    /* ready_q should now contain (in order): the originally-bumped POC 0,
     * then the spilled POC 100. */
    EXPECT(eng.ready_q.size() == 2, "expected 2 in ready_q (orig + spilled)");
    EXPECT(eng.ready_q[0].poc == 0, "first ready entry should be the orig POC 0");
    EXPECT(eng.ready_q[1].poc == 100, "second ready entry should be spilled POC 100");

    return 0;
}

/* ============================================================
 * Test 4 — H.265 path bypasses IDR spill.
 * ============================================================
 * The fix is H.264-specific: the engine's caller passes is_idr=false
 * for non-H.264 codecs.  Verify that even with a fake "high POC" entry
 * pre-existing, an entry submitted with is_idr=false (as H.265 path
 * does) does NOT spill.
 */
int test_h265_path_no_spill()
{
    DecodeEngine eng{};
    eng.codec = Codec::H265;

    /* Pre-load reorder_q with a "stuck" high-POC entry. */
    DecodeEngine_OnDecodeComplete(&eng, MakeEntry(500, 1000000), 1, false);
    EXPECT(eng.reorder_q.size() == 1, "stuck-entry preload");

    /* H.265 caller always passes is_idr_h264_boundary=false (it has its
     * own POC-management semantics).  Submitting POC=0 with is_idr=false
     * should NOT spill the high-POC entry. */
    DecodeEngine_OnDecodeComplete(&eng, MakeEntry(0, 2000000), 1, false);

    /* POC 0 is lowest → bumped to ready_q.  POC 500 stays. */
    EXPECT(eng.reorder_q.size() == 1, "expected POC 500 still in reorder_q");
    EXPECT(eng.reorder_q[0].poc == 500, "POC 500 should remain stuck");
    EXPECT(eng.ready_q.size() == 1, "expected POC 0 in ready_q");
    EXPECT(eng.ready_q[0].poc == 0, "POC 0 should have been bumped");
    return 0;
}

/* ============================================================
 * Test 5 — input capacity must exceed reorder threshold.
 * ============================================================
 * ow-rickroll_1080p.mp4 advertises max_num_ref_frames=4.  The MFT
 * used to hard-cap H.264 input at queue depth 4.  That deadlocked:
 * four decoded frames sat in reorder_q, ready_q was empty, and the
 * fifth input (the one that would bump the first output) was refused.
 */
int test_input_capacity_allows_reorder_bump()
{
    DecodeEngine eng{};
    eng.codec = Codec::H264;

    constexpr uint32_t kMaxReorder = 4;
    for (int i = 0; i < 4; i++) {
        DecodeEngine_OnDecodeComplete(
            &eng, MakeEntry(i * 2, i * 333333LL),
            kMaxReorder, i == 0);
    }

    EXPECT(eng.reorder_q.size() == 4, "expected 4 frames waiting in reorder_q");
    EXPECT(eng.ready_q.empty(), "expected no ready frame before fifth input");
    EXPECT(DecodeEngine_QueueDepth(&eng) == 4, "expected total queue depth 4");

    const size_t cap = DecodeEngine_InputQueueCapacity(&eng);
    EXPECT(cap > DecodeEngine_QueueDepth(&eng),
           "capacity=%zu must allow fifth input at depth=%zu",
           cap, DecodeEngine_QueueDepth(&eng));

    DecodeEngine_OnDecodeComplete(
        &eng, MakeEntry(8, 4 * 333333LL),
        kMaxReorder, false);
    EXPECT(!eng.ready_q.empty(), "fifth input should bump one output ready");
    return 0;
}

}  /* anon ns */

int main()
{
    int failed = 0;
    struct { const char *name; int (*fn)(); } tests[] = {
        {"single_gop",          test_single_gop},
        {"multi_idr",           test_multi_idr},
        {"no_spill_without_idr",test_no_spill_without_idr},
        {"h265_path_no_spill",  test_h265_path_no_spill},
        {"input_capacity_allows_reorder_bump",
                                test_input_capacity_allows_reorder_bump},
    };
    for (auto &t : tests) {
        std::printf("=== %s ===\n", t.name);
        int rc = t.fn();
        if (rc != 0) {
            std::printf("FAILED\n");
            failed++;
        } else {
            std::printf("PASS\n");
        }
    }
    std::printf("\n%d failed of %zu\n",
                failed, sizeof(tests)/sizeof(tests[0]));
    return failed ? 1 : 0;
}
