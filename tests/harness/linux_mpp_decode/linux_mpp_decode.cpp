/* linux_mpp_decode.cpp — H.264 decode harness for /dev/mpp_service.
 *
 *   linux_mpp_decode <stream.h264> <width> <height> [out.yuv] [--frames N]
 *
 * Thin DecodeEngine consumer: hands every Annex-B AU to
 * DecodeEngine_Submit; pops display-order NV12 frames via
 * DecodeEngine_PollFrame.  Replaces the standalone parse → DPB →
 * regbuilder → mpp_svc loop that lived here pre-2026-05-11 — the
 * standalone copy drifted from the Windows MFT engine twice (colmv
 * size + ref-iova mapping) and that drift is what motivated
 * decode-engine-backend-split.md.
 *
 * Reference compare: ffmpeg -i stream.h264 -f rawvideo -pix_fmt nv12 ref.yuv
 * Bit-exact match with ref.yuv ⇒ user-mode pipeline (parser → dpb →
 * regbuilder → packed_tables → engine) is correct end-to-end.
 */
#include "winshim.h"
#include "mft/au_iter.h"
#include "mft/engine/decode_engine.h"
#include "mft/engine/backend_linux.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>

static uint8_t *read_file(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return nullptr; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t *)malloc((size_t)n);
    if (!buf) { fclose(f); return nullptr; }
    size_t got = fread(buf, 1, (size_t)n, f);
    fclose(f);
    if (got != (size_t)n) { free(buf); return nullptr; }
    *out_len = (size_t)n;
    return buf;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <stream.h264> <width> <height> [out.yuv] [--frames N]\n",
                argv[0]);
        return 1;
    }
    uint32_t W = (uint32_t)atoi(argv[2]);
    uint32_t H = (uint32_t)atoi(argv[3]);
    FILE *out_fp = nullptr;
    int max_frames = INT_MAX;
    int first_opt = 4;
    if (argc >= 5 && argv[4][0] != '-') {
        out_fp = fopen(argv[4], "wb");
        if (!out_fp) { perror(argv[4]); return 1; }
        first_opt = 5;
    }
    for (int i = first_opt; i < argc; i++) {
        if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            max_frames = atoi(argv[++i]);
    }

    size_t bs_len = 0;
    uint8_t *bs = read_file(argv[1], &bs_len);
    if (!bs) return 1;

    DecodeEngine eng{};
    DecodeEngineBackend *be = LinuxBackend_New(W, H);
    if (!be) { fprintf(stderr, "LinuxBackend_New OOM\n"); free(bs); return 1; }
    if (DecodeEngine_InitWithBackend(&eng, be, Codec::H264, W, H) != 0) {
        LinuxBackend_Free(be); free(bs); return 1;
    }

    int decoded = 0, submitted = 0;
    auto pop_ready = [&]() {
        DecodedFrame f;
        while (DecodeEngine_PollFrame(&eng, &f) > 0) {
            if (out_fp && !f.yuv.empty())
                fwrite(f.yuv.data(), 1, f.yuv.size(), out_fp);
            DecodeEngine_ReleaseFrame(&eng, &f);
            decoded++;
        }
    };

    AuIter it;
    AuIter_Init(&it, bs, bs_len);
    size_t au_off, au_len;
    while (submitted < max_frames && H264AuNext(&it, &au_off, &au_len, nullptr)) {
        if (DecodeEngine_Submit(&eng, bs + au_off, au_len, /*pts_hns=*/-1) != 0) {
            fprintf(stderr, "Submit failed at AU %d\n", submitted);
            break;
        }
        submitted++;
        pop_ready();
    }
    DecodeEngine_Drain(&eng);
    pop_ready();

    fprintf(stderr, "Done: submitted=%d decoded=%d\n", submitted, decoded);
    DecodeEngine_Shutdown(&eng);
    LinuxBackend_Free(be);
    if (out_fp) fclose(out_fp);
    free(bs);
    return decoded == submitted ? 0 : 1;
}
