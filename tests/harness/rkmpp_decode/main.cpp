/* tests/harness/rkmpp_decode/main.cpp
 *
 * Phase 3b first-decode harness:
 *   rkmpp_decode --in stream.h264 --width W --height H --out out.yuv
 *
 * Reads the entire .h264 file (Annex-B framed), feeds it as ONE access
 * unit to DecodeEngine, dumps the resulting NV12 frame to --out.
 *
 * For first-IDR test the stream is just one SPS + one PPS + one IDR
 * slice; the engine isn't streaming yet.
 */
#include <windows.h>
#include <initguid.h>     /* DEFINE_GUID for GUID_DEVINTERFACE_RKMPP */
#include "decode_engine.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>

static int Usage() {
    std::fprintf(stderr,
        "usage: rkmpp_decode --in <file.h264> --width <px> --height <px> "
        "[--out <file.yuv>]\n");
    return 1;
}

int wmain(int argc, wchar_t **argv) {
    const wchar_t *in_path = nullptr, *out_path = nullptr;
    uint32_t width = 0, height = 0;
    for (int i = 1; i < argc; i++) {
        if (!wcscmp(argv[i], L"--in")     && i + 1 < argc) in_path  = argv[++i];
        else if (!wcscmp(argv[i], L"--out")    && i + 1 < argc) out_path = argv[++i];
        else if (!wcscmp(argv[i], L"--width")  && i + 1 < argc) width    = _wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--height") && i + 1 < argc) height   = _wtoi(argv[++i]);
        else return Usage();
    }
    if (!in_path || !width || !height) return Usage();

    /* Read the bitstream file. */
    FILE *f = nullptr;
    if (_wfopen_s(&f, in_path, L"rb") != 0 || !f) {
        std::fprintf(stderr, "open input failed\n");
        return 2;
    }
    std::fseek(f, 0, SEEK_END);
    long len = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> au(len);
    std::fread(au.data(), 1, len, f);
    std::fclose(f);
    std::printf("input: %ls (%ld bytes)\n", in_path, len);

    DecodeEngine eng;
    if (DecodeEngine_Init(&eng, width, height) != 0) {
        std::fprintf(stderr, "engine init failed\n");
        return 3;
    }

    std::vector<uint8_t> yuv;
    int rc = DecodeEngine_DecodeOne(&eng, au.data(), au.size(),
                                    out_path ? &yuv : nullptr);
    if (rc != 0) {
        DecodeEngine_Shutdown(&eng);
        return 4;
    }

    if (out_path && !yuv.empty()) {
        FILE *o = nullptr;
        if (_wfopen_s(&o, out_path, L"wb") == 0 && o) {
            std::fwrite(yuv.data(), 1, yuv.size(), o);
            std::fclose(o);
            std::printf("wrote %ls (%zu bytes)\n", out_path, yuv.size());
        }
    }

    DecodeEngine_Shutdown(&eng);
    return 0;
}
