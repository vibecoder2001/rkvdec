/* tests/harness/mft_decode/mft_decode.cpp — Phase 2B end-to-end test.
 *
 * Loads rkmpp_decoder_mft.dll directly via LoadLibrary +
 * DllGetClassObject (no regsvr32 / no registry dependency), drives an
 * IMFTransform instance through a real Annex-B bitstream, and writes
 * the decoded NV12 frames to disk.
 *
 *   mft_decode --codec h264|h265 --in stream.{h264,h265} --out frames.yuv
 *
 * Width/height are inferred from the first SPS in the input by scanning
 * AUs; for streams where that's awkward we accept --width / --height
 * overrides.  Default 1920x1080 matches the BSP capture set.
 *
 * Requires rkmpp.sys + an RKCP3550 codec on the system — i.e. only
 * runs on the Windows ARM64 board.  CI build verifies compilation.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <windows.h>
#include <initguid.h>  /* instantiate CLSID_RkmppXxxDecoder symbols */
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mftransform.h>
#include <unknwn.h>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "guids.h"

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

#define HR(call) do { \
    HRESULT _hr = (call); \
    if (FAILED(_hr)) { std::printf("FAIL %s:%d: %s -> 0x%08x\n", \
                                   __FILE__, __LINE__, #call, (unsigned)_hr); \
                       return 1; } \
} while (0)

typedef HRESULT (__stdcall *PFN_DllGetClassObject)(REFCLSID, REFIID, void **);

enum class CodecCli { H264, H265, AV1 };

/* -------- Annex-B AU walker (same logic as rkmpp_decode/main.cpp) -------- */

static size_t find_start_code(const uint8_t *buf, size_t len, size_t from) {
    for (size_t i = from; i + 3 <= len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1)
            return i + 3;
        if (i + 4 <= len &&
            buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1)
            return i + 4;
    }
    return SIZE_MAX;
}

static bool au_next_h264(const uint8_t *buf, size_t len, size_t *pos,
                         size_t *au_off, size_t *au_len) {
    if (*pos >= len) return false;
    size_t first_sc = find_start_code(buf, len, *pos);
    if (first_sc == SIZE_MAX) return false;

    bool found_slice = false;
    size_t after_slice = len;
    size_t nh_off = first_sc;
    while (nh_off < len) {
        uint8_t nut = buf[nh_off] & 0x1F;
        if (nut == 1 || nut == 5) {
            found_slice = true;
            size_t next_sc = find_start_code(buf, len, nh_off + 1);
            after_slice = (next_sc == SIZE_MAX) ? len : (next_sc - 3);
            break;
        }
        size_t next_sc = find_start_code(buf, len, nh_off + 1);
        if (next_sc == SIZE_MAX || next_sc >= len) break;
        nh_off = next_sc;
    }
    if (!found_slice) return false;

    size_t sc_start = (first_sc >= 3) ? first_sc - 3 : 0;
    *au_off = sc_start;
    *au_len = after_slice - sc_start;
    *pos    = after_slice;
    return true;
}

static bool au_next_h265(const uint8_t *buf, size_t len, size_t *pos,
                         size_t *au_off, size_t *au_len) {
    if (*pos >= len) return false;
    size_t first_sc = find_start_code(buf, len, *pos);
    if (first_sc == SIZE_MAX) return false;

    bool found_slice = false;
    size_t after_slice = len;
    size_t nh_off = first_sc;
    while (nh_off < len) {
        uint8_t nut = (buf[nh_off] >> 1) & 0x3F;
        if (nut < 32) {
            found_slice = true;
            size_t next_sc = find_start_code(buf, len, nh_off + 1);
            after_slice = (next_sc == SIZE_MAX) ? len : (next_sc - 3);
            break;
        }
        size_t next_sc = find_start_code(buf, len, nh_off + 1);
        if (next_sc == SIZE_MAX || next_sc >= len) break;
        nh_off = next_sc;
    }
    if (!found_slice) return false;

    size_t sc_start = (first_sc >= 3) ? first_sc - 3 : 0;
    *au_off = sc_start;
    *au_len = after_slice - sc_start;
    *pos    = after_slice;
    return true;
}

/* IVF temporal-unit walker: each frame in an IVF stream is preceded by
 * a 12-byte header (4-byte LE size, 8-byte LE pts) followed by the OBU
 * bytes.  The 32-byte file header (DKIF...) is skipped on the first call
 * by setting *pos = 32 before invoking.  Returns the OBU payload range. */
static bool au_next_av1(const uint8_t *b, size_t l, size_t *pos,
                        size_t *off, size_t *len) {
    if (*pos + 12 > l) return false;
    uint32_t sz = (uint32_t)b[*pos] | ((uint32_t)b[*pos+1] << 8) |
                  ((uint32_t)b[*pos+2] << 16) | ((uint32_t)b[*pos+3] << 24);
    *pos += 12;
    if (*pos + sz > l) return false;
    *off = *pos;
    *len = sz;
    *pos += sz;
    return true;
}

static bool au_next(CodecCli c, const uint8_t *b, size_t l, size_t *pos,
                    size_t *off, size_t *len) {
    if (c == CodecCli::AV1)  return au_next_av1 (b, l, pos, off, len);
    if (c == CodecCli::H265) return au_next_h265(b, l, pos, off, len);
    return au_next_h264(b, l, pos, off, len);
}

/* -------- IMFSample helpers -------- */

static HRESULT MakeSampleFromBytes(const uint8_t *data, size_t n,
                                   LONGLONG pts_hns, LONGLONG dur_hns,
                                   IMFSample **out) {
    IMFMediaBuffer *mbuf = nullptr;
    HRESULT hr = MFCreateMemoryBuffer((DWORD)n, &mbuf);
    if (FAILED(hr)) return hr;

    BYTE *dst = nullptr; DWORD cap = 0, cur = 0;
    hr = mbuf->Lock(&dst, &cap, &cur);
    if (FAILED(hr)) { mbuf->Release(); return hr; }
    std::memcpy(dst, data, n);
    mbuf->Unlock();
    mbuf->SetCurrentLength((DWORD)n);

    IMFSample *s = nullptr;
    hr = MFCreateSample(&s);
    if (FAILED(hr)) { mbuf->Release(); return hr; }
    s->AddBuffer(mbuf);
    mbuf->Release();
    s->SetSampleTime(pts_hns);
    s->SetSampleDuration(dur_hns);
    *out = s;
    return S_OK;
}

/* Pump ProcessOutput until NEED_MORE_INPUT.  Each frame produced gets
 * its NV12 data appended to *out_file. */
static int DrainOutputs(IMFTransform *mft, FILE *out_file, int *frame_count) {
    for (;;) {
        MFT_OUTPUT_DATA_BUFFER ob{}; DWORD st = 0;
        HRESULT hr = mft->ProcessOutput(0, 1, &ob, &st);
        if (hr == MF_E_TRANSFORM_NEED_MORE_INPUT) return 0;
        if (FAILED(hr)) {
            std::printf("ProcessOutput failed 0x%08x\n", (unsigned)hr);
            return 1;
        }
        if (!ob.pSample) {
            std::printf("ProcessOutput returned S_OK with null sample\n");
            return 1;
        }
        IMFMediaBuffer *mbuf = nullptr;
        if (SUCCEEDED(ob.pSample->ConvertToContiguousBuffer(&mbuf))) {
            BYTE *p = nullptr; DWORD cap = 0, cur = 0;
            if (SUCCEEDED(mbuf->Lock(&p, &cap, &cur))) {
                if (out_file) {
                    std::fwrite(p, 1, cur, out_file);
                    std::fflush(out_file);
                }
                LONGLONG pts = 0; ob.pSample->GetSampleTime(&pts);
                std::printf("  frame %d: %u bytes pts=%lld\n",
                            *frame_count, cur, (long long)pts);
                std::fflush(stdout);
                mbuf->Unlock();
            }
            mbuf->Release();
        }
        ob.pSample->Release();
        (*frame_count)++;
    }
}

/* -------- main -------- */

int wmain(int argc, wchar_t **argv) {
    const wchar_t *in_path = nullptr;
    const wchar_t *out_path = nullptr;
    CodecCli codec = CodecCli::H264;
    UINT32 width = 1920, height = 1080;

    for (int i = 1; i < argc; i++) {
        if      (!wcscmp(argv[i], L"--in")     && i+1 < argc) in_path  = argv[++i];
        else if (!wcscmp(argv[i], L"--out")    && i+1 < argc) out_path = argv[++i];
        else if (!wcscmp(argv[i], L"--width")  && i+1 < argc) width    = (UINT32)_wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--height") && i+1 < argc) height   = (UINT32)_wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--codec")  && i+1 < argc) {
            const wchar_t *c = argv[++i];
            if      (!wcscmp(c, L"h264") || !wcscmp(c, L"H264")) codec = CodecCli::H264;
            else if (!wcscmp(c, L"h265") || !wcscmp(c, L"H265") ||
                     !wcscmp(c, L"hevc") || !wcscmp(c, L"HEVC")) codec = CodecCli::H265;
            else if (!wcscmp(c, L"av1")  || !wcscmp(c, L"AV1"))  codec = CodecCli::AV1;
            else {
                std::printf("unknown codec: %ls\n", c);
                return 1;
            }
        } else {
            std::printf("usage: mft_decode --codec h264|h265|av1 --in <file> "
                        "--out <yuv> [--width W --height H]\n"
                        "  h264/h265: input is Annex-B bitstream\n"
                        "  av1:       input is an IVF file (DKIF header + OBU TUs)\n");
            return 1;
        }
    }
    if (!in_path) {
        std::printf("missing --in\n");
        return 1;
    }

    /* Load the input bitstream. */
    FILE *f = nullptr;
    if (_wfopen_s(&f, in_path, L"rb") != 0 || !f) {
        std::printf("FAIL: open %ls\n", in_path);
        return 2;
    }
    std::fseek(f, 0, SEEK_END);
    long fsize = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> bs(fsize);
    std::fread(bs.data(), 1, fsize, f);
    std::fclose(f);
    const char *codec_str = (codec == CodecCli::AV1)  ? "av1"
                          : (codec == CodecCli::H265) ? "h265"
                                                      : "h264";
    std::printf("input: %ls (%ld bytes) codec=%s %ux%u\n", in_path, fsize,
                codec_str, width, height);

    HR(CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED));
    HR(MFStartup(MF_VERSION, MFSTARTUP_LITE));

    HMODULE dll = LoadLibraryW(L"rkmpp_decoder_mft.dll");
    if (!dll) {
        std::printf("FAIL: LoadLibrary rkmpp_decoder_mft.dll -> %lu\n",
                    GetLastError());
        return 3;
    }
    auto get_class_obj = (PFN_DllGetClassObject)
        GetProcAddress(dll, "DllGetClassObject");
    if (!get_class_obj) {
        std::printf("FAIL: GetProcAddress DllGetClassObject\n");
        return 3;
    }

    REFCLSID clsid = (codec == CodecCli::AV1)  ? CLSID_RkmppAv1Decoder
                   : (codec == CodecCli::H265) ? CLSID_RkmppHevcDecoder
                                               : CLSID_RkmppH264Decoder;
    GUID input_subtype = (codec == CodecCli::AV1)  ? MFVideoFormat_AV1
                       : (codec == CodecCli::H265) ? MFVideoFormat_HEVC
                                                   : MFVideoFormat_H264;

    IClassFactory *cf = nullptr;
    HR(get_class_obj(clsid, IID_IClassFactory, (void**)&cf));
    IMFTransform *mft = nullptr;
    HR(cf->CreateInstance(nullptr, IID_IMFTransform, (void**)&mft));
    cf->Release();

    /* Type negotiation. */
    IMFMediaType *in_type = nullptr;
    HR(MFCreateMediaType(&in_type));
    HR(in_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    HR(in_type->SetGUID(MF_MT_SUBTYPE, input_subtype));
    HR(MFSetAttributeSize(in_type, MF_MT_FRAME_SIZE, width, height));
    HR(MFSetAttributeRatio(in_type, MF_MT_FRAME_RATE, 30, 1));
    HR(mft->SetInputType(0, in_type, 0));

    IMFMediaType *out_avail = nullptr;
    HR(mft->GetOutputAvailableType(0, 0, &out_avail));
    HR(mft->SetOutputType(0, out_avail, 0));
    out_avail->Release();
    in_type->Release();

    HR(mft->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0));

    FILE *out = nullptr;
    if (out_path) {
        if (_wfopen_s(&out, out_path, L"wb") != 0 || !out) {
            std::printf("FAIL: open output %ls\n", out_path);
            return 4;
        }
    }

    /* Push AUs.  Each AU = one IMFSample with PTS = idx*(10M/30) HNS.
     * AV1 input is IVF: skip the 32-byte DKIF header.  Pull the actual
     * picture dimensions from the IVF header so the MFT's output type
     * matches the bitstream regardless of --width/--height defaults. */
    size_t pos = 0;
    if (codec == CodecCli::AV1) {
        if (bs.size() < 32 || std::memcmp(bs.data(), "DKIF", 4) != 0) {
            std::printf("FAIL: not an IVF file (missing DKIF header)\n");
            return 7;
        }
        pos = 32;
    }
    int au_idx = 0, frames_out = 0;
    while (true) {
        size_t au_off = 0, au_len = 0;
        if (!au_next(codec, bs.data(), bs.size(), &pos, &au_off, &au_len)) break;

        LONGLONG pts = (LONGLONG)((uint64_t)au_idx * 10'000'000ULL / 30);
        LONGLONG dur = (LONGLONG)(10'000'000ULL / 30);

        IMFSample *s = nullptr;
        HRESULT hr = MakeSampleFromBytes(bs.data() + au_off, au_len,
                                         pts, dur, &s);
        if (FAILED(hr)) {
            std::printf("FAIL build sample at AU %d\n", au_idx);
            break;
        }

        /* If MFT is full, drain output first. */
        for (int retry = 0; retry < 4; retry++) {
            hr = mft->ProcessInput(0, s, 0);
            if (hr == MF_E_NOTACCEPTING) {
                if (DrainOutputs(mft, out, &frames_out) != 0) {
                    s->Release();
                    return 5;
                }
                continue;
            }
            break;
        }
        s->Release();
        if (FAILED(hr)) {
            std::printf("ProcessInput failed at AU %d: 0x%08x\n",
                        au_idx, (unsigned)hr);
            break;
        }

        /* Drain whatever's ready. */
        if (DrainOutputs(mft, out, &frames_out) != 0) return 5;
        au_idx++;
    }

    /* Final drain. */
    HR(mft->ProcessMessage(MFT_MESSAGE_COMMAND_DRAIN, 0));
    if (DrainOutputs(mft, out, &frames_out) != 0) return 5;

    HR(mft->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0));
    mft->Release();

    if (out) std::fclose(out);
    MFShutdown();
    CoUninitialize();

    std::printf("\n=== mft_decode summary ===\n");
    std::printf("AUs submitted: %d\n", au_idx);
    std::printf("frames out:    %d\n", frames_out);
    std::printf("output:        %ls\n", out_path ? out_path : L"(none)");
    return frames_out > 0 ? 0 : 6;
}
