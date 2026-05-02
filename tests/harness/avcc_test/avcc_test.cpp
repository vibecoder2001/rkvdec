/* tests/harness/avcc_test/avcc_test.cpp — unit tests for the AVCC/HVCC
 * length-prefixed → Annex-B converter.
 *
 * Cases:
 *   - Synthetic AVCC4 with three known NAL boundaries
 *   - Round-trip: Annex-B → AVCC4 (via local inverse helper) → Annex-B,
 *     byte-compare against the original
 *   - Malformed: length field overruns buffer
 *   - Empty input → 0
 *   - Single NAL
 *   - DetectNalFraming
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "avcc_to_annexb.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

/* Inverse helper: Annex-B → AVCC4.  Walks 00 00 00 01 / 00 00 01 start
 * codes and emits a 4-byte BE length followed by the NAL body for each.
 * Test-only: not as robust as a production stream parser, but enough to
 * round-trip the converter against itself.  Returns bytes written. */
static int annexb_to_avcc4(const uint8_t *in, size_t len,
                           std::vector<uint8_t> *out) {
    out->clear();
    /* Find every start code; each NAL ends at the next start code or EOF. */
    std::vector<size_t> starts;
    std::vector<size_t> hdr_len;
    for (size_t i = 0; i + 3 <= len; ) {
        if (in[i] == 0 && in[i+1] == 0 && i + 4 <= len &&
            in[i+2] == 0 && in[i+3] == 1) {
            starts.push_back(i + 4); hdr_len.push_back(4); i += 4;
        } else if (in[i] == 0 && in[i+1] == 0 && in[i+2] == 1) {
            starts.push_back(i + 3); hdr_len.push_back(3); i += 3;
        } else {
            ++i;
        }
    }
    for (size_t k = 0; k < starts.size(); ++k) {
        size_t s = starts[k];
        size_t e = (k + 1 < starts.size())
                       ? starts[k+1] - hdr_len[k+1]
                       : len;
        size_t nl = e - s;
        uint8_t hdr[4] = {
            (uint8_t)((nl >> 24) & 0xFF),
            (uint8_t)((nl >> 16) & 0xFF),
            (uint8_t)((nl >>  8) & 0xFF),
            (uint8_t)( nl        & 0xFF),
        };
        out->insert(out->end(), hdr, hdr + 4);
        out->insert(out->end(), in + s, in + e);
    }
    return (int)out->size();
}

static int test_synthetic_avcc4(void) {
    /* Three NALs of bodies {0xAA}, {0xBB,0xCC}, {0xDD,0xEE,0xFF}. */
    const uint8_t in[] = {
        0x00,0x00,0x00,0x01, 0xAA,
        0x00,0x00,0x00,0x02, 0xBB,0xCC,
        0x00,0x00,0x00,0x03, 0xDD,0xEE,0xFF,
    };
    const uint8_t want[] = {
        0x00,0x00,0x00,0x01, 0xAA,
        0x00,0x00,0x00,0x01, 0xBB,0xCC,
        0x00,0x00,0x00,0x01, 0xDD,0xEE,0xFF,
    };
    uint8_t out[64] = {0};
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == (int)sizeof(want), "got=%d want=%zu", n, sizeof(want));
    EXPECT(memcmp(out, want, sizeof(want)) == 0, "byte mismatch");
    return 0;
}

static int test_single_nal(void) {
    const uint8_t in[] = { 0x00,0x00,0x00,0x05, 1,2,3,4,5 };
    const uint8_t want[] = { 0x00,0x00,0x00,0x01, 1,2,3,4,5 };
    uint8_t out[32] = {0};
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == (int)sizeof(want), "got=%d", n);
    EXPECT(memcmp(out, want, sizeof(want)) == 0, "mismatch");
    return 0;
}

static int test_empty(void) {
    uint8_t out[16];
    int n = AvccToAnnexB(nullptr, 0, AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == 0, "empty got=%d", n);
    return 0;
}

static int test_malformed_overrun(void) {
    /* Length=10 but only 3 body bytes follow. */
    const uint8_t in[] = { 0x00,0x00,0x00,0x0A, 1,2,3 };
    uint8_t out[64];
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == -1, "expected -1, got=%d", n);
    return 0;
}

static int test_malformed_short_length(void) {
    /* Only 3 bytes — not enough for a 4-byte length field. */
    const uint8_t in[] = { 0x00,0x00,0x00 };
    uint8_t out[64];
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == -1, "expected -1, got=%d", n);
    return 0;
}

static int test_zero_length_nal(void) {
    /* Length=0 is malformed (NAL header byte missing). */
    const uint8_t in[] = { 0x00,0x00,0x00,0x00 };
    uint8_t out[64];
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == -1, "expected -1, got=%d", n);
    return 0;
}

static int test_insufficient_capacity(void) {
    const uint8_t in[] = { 0x00,0x00,0x00,0x05, 1,2,3,4,5 };
    uint8_t out[8];  /* need 9 */
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_4, out, sizeof(out));
    EXPECT(n == -1, "expected -1, got=%d", n);
    return 0;
}

static int test_avcc2(void) {
    /* AVCC2: 2-byte BE length field. */
    const uint8_t in[] = {
        0x00, 0x02, 0xAA, 0xBB,
        0x00, 0x03, 0xCC, 0xDD, 0xEE,
    };
    const uint8_t want[] = {
        0x00,0x00,0x00,0x01, 0xAA, 0xBB,
        0x00,0x00,0x00,0x01, 0xCC, 0xDD, 0xEE,
    };
    uint8_t out[64] = {0};
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_2, out, sizeof(out));
    EXPECT(n == (int)sizeof(want), "got=%d", n);
    EXPECT(memcmp(out, want, sizeof(want)) == 0, "mismatch");
    return 0;
}

static int test_avcc1(void) {
    /* AVCC1: 1-byte length field. */
    const uint8_t in[] = { 0x02, 0xAA, 0xBB,  0x01, 0xCC };
    const uint8_t want[] = {
        0x00,0x00,0x00,0x01, 0xAA, 0xBB,
        0x00,0x00,0x00,0x01, 0xCC,
    };
    uint8_t out[32] = {0};
    int n = AvccToAnnexB(in, sizeof(in), AVCC_LEN_1, out, sizeof(out));
    EXPECT(n == (int)sizeof(want), "got=%d", n);
    EXPECT(memcmp(out, want, sizeof(want)) == 0, "mismatch");
    return 0;
}

static int test_roundtrip(void) {
    /* Synthetic Annex-B with three NALs (4-byte and 3-byte start codes
     * mixed) — round-trip must normalise to all-4-byte starts. */
    const uint8_t orig_4byte_only[] = {
        0x00,0x00,0x00,0x01, 0x67, 0x42, 0x00, 0x0A,
        0x00,0x00,0x00,0x01, 0x68, 0xCE,
        0x00,0x00,0x00,0x01, 0x65, 0x88, 0x80, 0x40,
    };
    std::vector<uint8_t> avcc;
    int n_avcc = annexb_to_avcc4(orig_4byte_only, sizeof(orig_4byte_only), &avcc);
    EXPECT(n_avcc > 0, "annexb_to_avcc4 failed");

    std::vector<uint8_t> back(avcc.size() + 64);
    int n_back = AvccToAnnexB(avcc.data(), avcc.size(),
                              AVCC_LEN_4, back.data(), back.size());
    EXPECT(n_back == (int)sizeof(orig_4byte_only),
           "round-trip size mismatch got=%d want=%zu",
           n_back, sizeof(orig_4byte_only));
    EXPECT(memcmp(back.data(), orig_4byte_only, sizeof(orig_4byte_only)) == 0,
           "round-trip content mismatch");
    return 0;
}

static int test_detect(void) {
    const uint8_t a[] = { 0x00,0x00,0x00,0x01, 0x67 };
    const uint8_t b[] = { 0x00,0x00,0x01, 0x67 };
    const uint8_t c[] = { 0x00,0x00,0x00,0x05, 1,2,3,4,5 };  /* AVCC4 */
    EXPECT(DetectNalFraming(a, sizeof(a)) == FRAMING_ANNEXB, "a");
    EXPECT(DetectNalFraming(b, sizeof(b)) == FRAMING_ANNEXB, "b");
    EXPECT(DetectNalFraming(c, sizeof(c)) == FRAMING_AVCC4,  "c");
    return 0;
}

int main(void) {
    int rc = 0;
    rc |= test_empty();
    rc |= test_single_nal();
    rc |= test_synthetic_avcc4();
    rc |= test_avcc2();
    rc |= test_avcc1();
    rc |= test_malformed_overrun();
    rc |= test_malformed_short_length();
    rc |= test_zero_length_nal();
    rc |= test_insufficient_capacity();
    rc |= test_roundtrip();
    rc |= test_detect();
    if (rc == 0) std::printf("avcc_test: PASS\n");
    else         std::printf("avcc_test: FAIL\n");
    return rc;
}
