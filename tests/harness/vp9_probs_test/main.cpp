/* tests/harness/vp9_probs_test/main.cpp
 *
 * Parses the first VP9 frame from an IVF and runs the regbuilder's
 * prob-buffer fill, then byte-diffs the result against a BSP-captured
 * reference probe blob.  Validates Vp9Regbuilder_FillProbs without
 * needing real hardware.
 *
 * Usage: vp9_probs_test <stream.ivf> <reference_probe.bin>
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "vp9_parser.h"
#include "vp9_dpb.h"
#include "regbuilder_vp9.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static std::vector<uint8_t> read_file(const char *p) {
    FILE *f = std::fopen(p, "rb");
    if (!f) { std::fprintf(stderr, "open %s: errno=%d\n", p, errno); return {}; }
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> v((size_t)n);
    if (std::fread(v.data(), 1, (size_t)n, f) != (size_t)n) v.clear();
    std::fclose(f);
    return v;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr,
            "usage: %s <stream.ivf> <reference.bin> [--frame N]\n"
            "  parses N=0 (default = first frame) through the parser and\n"
            "  byte-diffs FillProbs output vs the reference blob.\n",
            argv[0]);
        return 2;
    }

    int target_frame = 0;
    for (int i = 3; i < argc; i++) {
        if (!std::strcmp(argv[i], "--frame") && i + 1 < argc)
            target_frame = std::atoi(argv[++i]);
    }

    auto ivf = read_file(argv[1]);
    auto ref = read_file(argv[2]);
    if (ivf.empty() || ref.empty()) {
        std::fprintf(stderr, "FAIL: unable to read inputs\n");
        return 2;
    }
    if (ivf.size() < 32 || std::memcmp(ivf.data(), "DKIF", 4) != 0) {
        std::fprintf(stderr, "FAIL: %s is not an IVF file\n", argv[1]);
        return 2;
    }

    /* Walk IVF frames up to target_frame, parsing each so the parser
     * state (and our pu) reflects the chosen frame. */
    vp9::ParserState st{};
    vp9::PicParams   pp{};
    vp9::ProbUpdates pu{};
    size_t pos = 32;
    int    seen = -1;
    while (pos + 12 <= ivf.size()) {
        uint32_t fsz;
        std::memcpy(&fsz, ivf.data() + pos, 4);
        pos += 12;
        if (pos + fsz > ivf.size()) {
            std::fprintf(stderr, "FAIL: truncated IVF\n");
            return 2;
        }
        const uint8_t *frame = ivf.data() + pos;
        pos += fsz;
        seen++;

        const uint8_t *frames[8] = {};
        size_t         sizes[8]  = {};
        int nf = vp9::Vp9Parser_SuperframeSplit(frame, fsz, frames, sizes, 8);
        if (nf <= 0) { std::fprintf(stderr, "FAIL: superframe split %d\n", nf); return 2; }
        pu = vp9::ProbUpdates{};
        vp9::ParseResult prc = vp9::Vp9Parser_Parse(frames[0], sizes[0], st, pp, pu);
        if (prc != vp9::ParseResult::Ok) {
            std::fprintf(stderr, "FAIL: parse frame %d result=%d\n", seen, (int)prc);
            return 2;
        }
        if (seen == target_frame) break;
        vp9::Vp9Parser_ApplyDpbUpdate(st, pp);
    }
    if (seen != target_frame) {
        std::fprintf(stderr, "FAIL: stream only has %d frame(s)\n", seen + 1);
        return 2;
    }

    constexpr size_t kProbSize = 4864;
    std::vector<uint8_t> ours(kProbSize, 0);
    vp9::Vp9Regbuilder_FillProbs(pp, pu, ours.data());

    if (ref.size() < kProbSize) {
        std::fprintf(stderr, "FAIL: reference shorter than %zu bytes (%zu)\n",
                     kProbSize, ref.size());
        return 2;
    }

    /* Byte-diff first 4864 bytes (the CPU-write region; codec writes
     * the count area at 0x2000+). */
    size_t first_diff = SIZE_MAX;
    size_t diff_count = 0;
    for (size_t i = 0; i < kProbSize; i++) {
        if (ours[i] != ref[i]) {
            if (first_diff == SIZE_MAX) first_diff = i;
            diff_count++;
        }
    }

    if (diff_count == 0) {
        std::fprintf(stderr, "PASS: %zu bytes byte-exact vs reference\n", kProbSize);
        return 0;
    }

    std::fprintf(stderr,
        "FAIL: %zu / %zu bytes differ, first at offset 0x%zx\n",
        diff_count, kProbSize, first_diff);

    /* Print first 32 bytes from each side near the divergence. */
    size_t start = first_diff & ~(size_t)0xF;
    size_t end   = start + 64;
    if (end > kProbSize) end = kProbSize;
    std::fprintf(stderr, "off   ours                                            ref\n");
    for (size_t r = start; r < end; r += 16) {
        std::fprintf(stderr, "%04zx ", r);
        for (size_t c = 0; c < 16 && r + c < end; c++)
            std::fprintf(stderr, "%02x ", ours[r + c]);
        std::fprintf(stderr, " ");
        for (size_t c = 0; c < 16 && r + c < end; c++)
            std::fprintf(stderr, "%02x ", ref[r + c]);
        std::fprintf(stderr, "\n");
    }
    return 1;
}
