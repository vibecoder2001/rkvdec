/* linux_mpp_decode_vp9.cpp — VP9 decode harness for /dev/mpp_service.
 *
 *   linux_mpp_decode_vp9 <stream.ivf> <width> <height> [out.yuv] [--frames N]
 *
 * Drives Vp9DecodeEngine over a Linux backend.  IVF demux only — no
 * WebM / MP4 support.  Compare output against:
 *     ffmpeg -i stream.ivf -pix_fmt nv12 ref.yuv
 *
 * Bit-exact match with ref.yuv ⇒ the parser → DPB → regbuilder →
 * backend pipeline is correct end-to-end on real hardware.
 */
#include "winshim.h"
#include "mft/engine/decode_engine_vp9.h"
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
        fprintf(stderr, "usage: %s <stream.ivf> <width> <height> [out.yuv] [--frames N]\n",
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
    const char *dump_prefix = nullptr;
    const char *probe_blob = nullptr;
    uint16_t header_size_override = 0;
    for (int i = first_opt; i < argc; i++) {
        if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            max_frames = atoi(argv[++i]);
        else if (strcmp(argv[i], "--dump-bank") == 0 && i + 1 < argc)
            dump_prefix = argv[++i];
        else if (strcmp(argv[i], "--header-size-override") == 0 && i + 1 < argc)
            header_size_override = (uint16_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "--probe-blob") == 0 && i + 1 < argc)
            probe_blob = argv[++i];
    }

    size_t bs_len = 0;
    uint8_t *bs = read_file(argv[1], &bs_len);
    if (!bs) return 1;
    if (bs_len < 32) {
        fprintf(stderr, "%s: not an IVF (too short)\n", argv[1]);
        free(bs); return 1;
    }
    if (memcmp(bs, "DKIF", 4) != 0) {
        fprintf(stderr, "%s: missing DKIF magic\n", argv[1]);
        free(bs); return 1;
    }

    Vp9DecodeEngine eng{};
    DecodeEngineBackend *be = LinuxBackend_New(W, H);
    if (!be) { fprintf(stderr, "LinuxBackend_New OOM\n"); free(bs); return 1; }
    if (Vp9DecodeEngine_InitWithBackend(&eng, be, W, H) != 0) {
        LinuxBackend_Free(be); free(bs); return 1;
    }
    eng.dump_prefix = dump_prefix;
    eng.header_size_override = header_size_override;
    eng.probe_blob_path = probe_blob;

    size_t pos = 32;
    int decoded = 0;
    while (pos + 12 <= bs_len && decoded < max_frames) {
        uint32_t sz;
        memcpy(&sz, bs + pos, 4);
        pos += 12;
        if (pos + sz > bs_len) {
            fprintf(stderr, "truncated frame at pos=%zu sz=%u\n", pos, sz);
            break;
        }

        Vp9DecodedFrame f;
        int r = Vp9DecodeEngine_DecodeOne(&eng, bs + pos, sz, /*pts=*/decoded, &f);
        if (r != 0) {
            fprintf(stderr, "decode_one returned %d on frame %d\n", r, decoded);
            break;
        }
        if (f.show && out_fp && !f.yuv.empty())
            fwrite(f.yuv.data(), 1, f.yuv.size(), out_fp);
        Vp9DecodeEngine_ReleaseFrame(&eng, &f);

        pos += sz;
        ++decoded;
    }

    Vp9DecodeEngine_Shutdown(&eng);
    LinuxBackend_Free(be);
    free(bs);
    if (out_fp) fclose(out_fp);
    fprintf(stderr, "linux_mpp_decode_vp9: decoded %d frames\n", decoded);
    return decoded > 0 ? 0 : 2;
}
