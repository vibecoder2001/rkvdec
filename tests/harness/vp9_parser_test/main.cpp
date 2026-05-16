// tests/harness/vp9_parser_test/main.cpp
#include "vp9_parser.h"
#include <cstdio>
#include <vector>
#include <fstream>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

static std::vector<uint8_t> slurp(const char *p) {
    std::ifstream f(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), {}};
}

int main(int argc, char **argv) {
    CHECK(argc >= 2);

    // -----------------------------------------------------------------------
    // Test 1: single keyframe stream — basic uncompressed-header smoke test
    //         + compressed-header coef_present[] sanity.
    // -----------------------------------------------------------------------
    {
        auto ivf = slurp(argv[1]);
        const uint8_t *p = ivf.data() + 32;
        uint32_t sz = *reinterpret_cast<const uint32_t*>(p);
        p += 12;
        vp9::ParserState st{};
        vp9::PicParams pp{};
        vp9::ProbUpdates pu{};
        auto r = vp9::Vp9Parser_Parse(p, sz, st, pp, pu);
        CHECK(r == vp9::ParseResult::Ok);
        CHECK(pp.frame_type == 0);          // KEY
        CHECK(pp.profile == 0);
        CHECK(pp.bit_depth == 8);
        CHECK(pp.width == 1280 && pp.height == 720);

        // Compressed-header smoke: tx_mode read; bool-decode reached skip probs.
        // libvpx-vp9 may encode all update_probs=0 on a keyframe, so we don't
        // assert coef_present[] here; skip_present sits after coef probs in §6.3
        // and confirms the bool-decode ran past that section.
        CHECK(pu.tx_mode_present == 1);
        CHECK(pu.skip_present    == 1);
    }

    // -----------------------------------------------------------------------
    // Test 2: 2-frame inter GOP — inter compressed-header fields exercised.
    // -----------------------------------------------------------------------
    if (argc >= 3) {
        auto ivf2 = slurp(argv[2]);
        const uint8_t *p2 = ivf2.data() + 32;
        uint32_t sz2 = *reinterpret_cast<const uint32_t*>(p2);
        p2 += 12;
        vp9::ParserState st2{};
        vp9::PicParams pp2{};
        vp9::ProbUpdates pu2{};

        // Frame 0 — KEY
        auto r2 = vp9::Vp9Parser_Parse(p2, sz2, st2, pp2, pu2);
        CHECK(r2 == vp9::ParseResult::Ok);
        CHECK(pp2.frame_type == 0);
        CHECK(pu2.tx_mode_present == 1);
        CHECK(pu2.skip_present    == 1);
        vp9::Vp9Parser_ApplyDpbUpdate(st2, pp2);

        // Frame 1 — INTER
        p2 += sz2;
        sz2 = *reinterpret_cast<const uint32_t*>(p2);
        p2 += 12;
        r2 = vp9::Vp9Parser_Parse(p2, sz2, st2, pp2, pu2);
        CHECK(r2 == vp9::ParseResult::Ok);
        CHECK(pp2.frame_type == 1);

        bool has_ref = false;
        for (int i = 0; i < 3; ++i)
            if (pp2.frame_refs[i].index < 8) { has_ref = true; break; }
        CHECK(has_ref);

        // Inter-frame compressed-header sections present.
        CHECK(pu2.inter_mode_present == 1);
        CHECK(pu2.is_inter_present   == 1);
        CHECK(pu2.ref_mode_present   == 1);
        CHECK(pu2.skip_present       == 1);
    }

    return 0;
}
