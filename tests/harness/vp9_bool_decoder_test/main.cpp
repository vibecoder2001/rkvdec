// VP9 bool decoder unit test.
//
// Drives the decoder over the compressed-header bytes of the
// committed keyframe-only 720p IVF test stream.  Spec-correct VP9
// parsing of that stream yields tx_mode = 4 (TX_MODE_SELECT, the
// libvpx default).  Verified against BSP MPP `vp9d.c` on rk3588.
//
// Replaces the earlier {0x82,0x12} synthetic vector: that test fit
// our pre-fix 8-bit-buffer decoder model but didn't match the actual
// spec algorithm (16-bit refill, 24-bit value preload).
#include "vp9_bool_decoder.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <keyframe_only_720p.ivf>\n", argv[0]);
        return 2;
    }
    std::ifstream f(argv[1], std::ios::binary);
    std::vector<uint8_t> all{std::istreambuf_iterator<char>(f), {}};
    CHECK(all.size() > 256);

    // IVF: 32B file hdr + 12B frame hdr; uncompressed header is 18 bytes
    // for the committed test stream (verified against BSP capture).
    // first_partition_size = 226 (compressed-header bytes).
    const uint8_t *comp = all.data() + 32 + 12 + 18;
    size_t comp_len = 226;

    vp9::BoolDecoder bd;
    bd.init(comp, comp_len);
    CHECK(bd.ok());

    // tx_mode read: 2-bit literal; if value == 3, read one more bit and add.
    // Expected: tx_mode = 4 (TX_MODE_SELECT) — first 2 bits decode to 3,
    // then the extra bit is 1.
    int tx2 = bd.decode_literal(2);
    CHECK(tx2 == 3);
    int extra = bd.decode_bool(128);
    CHECK(extra == 1);
    int tx_mode = tx2 + extra;
    CHECK(tx_mode == 4);

    return 0;
}
