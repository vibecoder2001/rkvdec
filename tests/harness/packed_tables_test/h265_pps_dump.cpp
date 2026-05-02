/* tests/harness/packed_tables_test/h265_pps_dump.cpp
 *
 * Host-side helper that parses an .h265 file, runs H265PackPPS, and
 * writes the first PPS slot to a file.  Used to regenerate
 * win_h265_pps.bin for byte-for-byte comparison against the BSP gold
 * dump (Z:\drivers-arm\bsp_capture\hevc_dma_capture\bsp_pps.bin).
 *
 * Usage: h265_pps_dump <input.h265> <output.bin>
 */
#include "parser_glue_h265.h"
#include "h265_packed_tables.h"

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

static bool slurp(const char *path, std::vector<uint8_t> *out) {
    std::FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n <= 0) { std::fclose(f); return false; }
    out->resize((size_t)n);
    size_t got = std::fread(out->data(), 1, (size_t)n, f);
    std::fclose(f);
    return got == (size_t)n;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <input.h265> <output.bin>\n", argv[0]);
        return 1;
    }
    std::vector<uint8_t> stream;
    if (!slurp(argv[1], &stream)) {
        std::fprintf(stderr, "failed to read %s\n", argv[1]);
        return 1;
    }
    std::vector<uint8_t> scratch(stream.size() * 2 + 16);

    H265ParseResult r;
    H265ParseResultInit(&r);
    H265ParseAccessUnit(stream.data(), stream.size(),
                        scratch.data(), scratch.size(), &r);

    if (!r.sps[0].valid || !r.pps[0].valid) {
        std::fprintf(stderr, "SPS or PPS not parsed\n");
        return 1;
    }
    const H265Vps *vps = r.vps[0].valid ? &r.vps[0] : nullptr;
    const H265Sps *sps = &r.sps[0];
    const H265Pps *pps = &r.pps[0];

    std::vector<uint8_t> out(RKH265_SPSPPS_UNIT_SIZE);
    int n = H265PackPPS(vps, sps, pps, out.data(), out.size());
    if (n <= 0) {
        std::fprintf(stderr, "H265PackPPS failed\n");
        return 1;
    }

    std::FILE *f = std::fopen(argv[2], "wb");
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", argv[2]);
        return 1;
    }
    /* The BSP debug-dump (`fwrite(pps_ptr, 1, 80 * 64, fp)` at
     * hal_h265d_vdpu34x.c:434) writes the first 5120 bytes of the
     * 7168-byte (= 64 * 112) PPS buffer.  That captures 45 full slots +
     * the first 80 bytes of slot 45.  Match that exactly so `cmp`
     * succeeds against Z:\drivers-arm\bsp_capture\hevc_dma_capture\bsp_pps.bin. */
    constexpr size_t kBspDumpBytes = 80u * 64u;   // 5120
    std::fwrite(out.data(), 1, kBspDumpBytes, f);
    std::fclose(f);
    std::printf("wrote %s (%zu bytes; slot=%d, full unit=%u)\n",
                argv[2], kBspDumpBytes,
                (int)RKH265_SPSPPS_SLOT_SIZE, (unsigned)RKH265_SPSPPS_UNIT_SIZE);
    return 0;
}
