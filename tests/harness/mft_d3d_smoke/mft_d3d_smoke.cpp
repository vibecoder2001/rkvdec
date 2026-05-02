/* tests/harness/mft_d3d_smoke/mft_d3d_smoke.cpp
 *
 * Phase 4 smoke test: drive rkmpp_decoder_mft.dll in D3D11 output mode
 * and verify the textures we emit decode to the same NV12 bytes as the
 * BSP gold (and as the system-memory MFT path).
 *
 * Pipeline:
 *   - Create a WARP ID3D11Device + IMFDXGIDeviceManager.
 *   - Load DLL, instantiate the H.264 or HEVC decoder.
 *   - SET_D3D_MANAGER, set types, BEGIN_STREAMING.
 *   - Walk Annex-B AUs, ProcessInput / ProcessOutput.
 *   - For each output sample: MFGetService(MR_BUFFER_SERVICE,
 *     ID3D11Texture2D), CopyResource into a STAGING texture, Map, write
 *     Y plane (height rows of width bytes) and UV plane (height/2 rows
 *     of width bytes) to the output YUV file.
 *
 * Run on the ARM64 board only — the DLL needs rkmpp.sys to do real
 * decode work.  Compares md5(output) against board_md5.txt out of band.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <windows.h>
#include <initguid.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mftransform.h>
#include <unknwn.h>
#include <evr.h>
#include <d3d11.h>
#include <dxgi.h>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "guids.h"

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")

#define HR(call) do { \
    HRESULT _hr = (call); \
    if (FAILED(_hr)) { std::printf("FAIL %s:%d: %s -> 0x%08x\n", \
                                   __FILE__, __LINE__, #call, (unsigned)_hr); \
                       return 1; } \
} while (0)

typedef HRESULT (__stdcall *PFN_DllGetClassObject)(REFCLSID, REFIID, void **);

enum class CodecCli { H264, H265 };

/* -------- Annex-B AU walker (lifted from mft_decode) -------- */

static size_t find_start_code(const uint8_t *buf, size_t len, size_t from) {
    for (size_t i = from; i + 3 <= len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1) return i + 3;
        if (i + 4 <= len &&
            buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1)
            return i + 4;
    }
    return SIZE_MAX;
}

static bool au_next(CodecCli c, const uint8_t *buf, size_t len, size_t *pos,
                    size_t *au_off, size_t *au_len) {
    if (*pos >= len) return false;
    size_t first_sc = find_start_code(buf, len, *pos);
    if (first_sc == SIZE_MAX) return false;
    bool found_slice = false;
    size_t after_slice = len, nh_off = first_sc;
    while (nh_off < len) {
        bool is_slice = false;
        if (c == CodecCli::H264) {
            uint8_t nut = buf[nh_off] & 0x1F;
            is_slice = (nut == 1 || nut == 5);
        } else {
            uint8_t nut = (buf[nh_off] >> 1) & 0x3F;
            is_slice = (nut < 32);
        }
        if (is_slice) {
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

/* -------- D3D11 readback of an NV12 sample's texture -------- */

static int WriteNv12FromSample(IMFSample *sample, ID3D11Device *dev,
                               ID3D11DeviceContext *ctx,
                               UINT32 width, UINT32 height,
                               FILE *out) {
    IMFMediaBuffer *mbuf = nullptr;
    if (FAILED(sample->GetBufferByIndex(0, &mbuf))) return 1;

    /* DXGI surface buffers expose the underlying ID3D11Texture2D via
     * IMFDXGIBuffer::GetResource, not MFGetService(MR_BUFFER_SERVICE). */
    IMFDXGIBuffer   *dxgi = nullptr;
    ID3D11Texture2D *tex  = nullptr;
    UINT             subresource = 0;
    HRESULT hr = mbuf->QueryInterface(IID_PPV_ARGS(&dxgi));
    if (FAILED(hr)) {
        std::printf("FAIL: IMFDXGIBuffer QI -> 0x%08x\n", (unsigned)hr);
        mbuf->Release();
        return 1;
    }
    hr = dxgi->GetResource(IID_PPV_ARGS(&tex));
    if (FAILED(hr)) {
        std::printf("FAIL: IMFDXGIBuffer::GetResource -> 0x%08x\n",
                    (unsigned)hr);
        dxgi->Release(); mbuf->Release();
        return 1;
    }
    dxgi->GetSubresourceIndex(&subresource);
    dxgi->Release();

    D3D11_TEXTURE2D_DESC td = {};
    tex->GetDesc(&td);

    D3D11_TEXTURE2D_DESC sd = td;
    sd.Usage          = D3D11_USAGE_STAGING;
    sd.BindFlags      = 0;
    sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    sd.MiscFlags      = 0;
    ID3D11Texture2D *staging = nullptr;
    hr = dev->CreateTexture2D(&sd, nullptr, &staging);
    if (FAILED(hr)) {
        std::printf("FAIL: CreateTexture2D(staging) -> 0x%08x\n", (unsigned)hr);
        tex->Release(); mbuf->Release(); return 1;
    }
    ctx->CopySubresourceRegion(staging, 0, 0, 0, 0,
                               tex, subresource, nullptr);

    D3D11_MAPPED_SUBRESOURCE mapped = {};
    hr = ctx->Map(staging, 0, D3D11_MAP_READ, 0, &mapped);
    if (FAILED(hr)) {
        std::printf("FAIL: Map(staging) -> 0x%08x\n", (unsigned)hr);
        staging->Release(); tex->Release(); mbuf->Release(); return 1;
    }

    /* NV12 in D3D11: a single texture with two planes laid out in
     * memory as Y (height rows) followed by interleaved UV (height/2
     * rows), each at RowPitch.  pData points to plane 0; UV starts at
     * pData + RowPitch * height. */
    const uint8_t *base   = static_cast<const uint8_t *>(mapped.pData);
    const UINT     pitch  = mapped.RowPitch;
    for (UINT32 y = 0; y < height; y++)
        std::fwrite(base + y * pitch, 1, width, out);
    const uint8_t *uv = base + pitch * td.Height;
    for (UINT32 y = 0; y < height / 2; y++)
        std::fwrite(uv + y * pitch, 1, width, out);

    ctx->Unmap(staging, 0);
    staging->Release();
    tex->Release();
    mbuf->Release();
    return 0;
}

static int DrainOutputs(IMFTransform *mft, ID3D11Device *dev,
                        ID3D11DeviceContext *ctx, UINT32 w, UINT32 h,
                        FILE *out, int *frame_count) {
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
        if (WriteNv12FromSample(ob.pSample, dev, ctx, w, h, out) != 0) {
            ob.pSample->Release();
            return 1;
        }
        LONGLONG pts = 0; ob.pSample->GetSampleTime(&pts);
        std::printf("  frame %d: pts=%lld (D3D11 NV12)\n",
                    *frame_count, (long long)pts);
        ob.pSample->Release();
        (*frame_count)++;
    }
}

int wmain(int argc, wchar_t **argv) {
    const wchar_t *in_path = nullptr;
    const wchar_t *out_path = nullptr;
    CodecCli codec = CodecCli::H264;
    UINT32 width = 1280, height = 720;

    for (int i = 1; i < argc; i++) {
        if      (!wcscmp(argv[i], L"--in")     && i+1 < argc) in_path  = argv[++i];
        else if (!wcscmp(argv[i], L"--out")    && i+1 < argc) out_path = argv[++i];
        else if (!wcscmp(argv[i], L"--width")  && i+1 < argc) width    = (UINT32)_wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--height") && i+1 < argc) height   = (UINT32)_wtoi(argv[++i]);
        else if (!wcscmp(argv[i], L"--codec")  && i+1 < argc) {
            const wchar_t *c = argv[++i];
            if      (!wcscmp(c, L"h264")) codec = CodecCli::H264;
            else if (!wcscmp(c, L"h265") || !wcscmp(c, L"hevc")) codec = CodecCli::H265;
            else { std::printf("unknown codec: %ls\n", c); return 1; }
        } else {
            std::printf("usage: mft_d3d_smoke --codec h264|h265 --in <file> "
                        "--out <yuv> [--width W --height H]\n");
            return 1;
        }
    }
    if (!in_path) { std::printf("missing --in\n"); return 1; }

    /* Slurp the bitstream. */
    FILE *f = nullptr;
    if (_wfopen_s(&f, in_path, L"rb") != 0 || !f) {
        std::printf("FAIL: open %ls\n", in_path); return 2;
    }
    std::fseek(f, 0, SEEK_END);
    long fsize = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> bs(fsize);
    std::fread(bs.data(), 1, fsize, f);
    std::fclose(f);

    HR(CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED));
    HR(MFStartup(MF_VERSION, MFSTARTUP_LITE));

    /* Build a WARP D3D11 device — RK3588 has no native D3D11 driver,
     * but WARP gives us a fully functional ID3D11Device backed by CPU
     * memory, which is sufficient to exercise our SET_D3D_MANAGER
     * handshake and texture emission path. */
    ID3D11Device        *dev = nullptr;
    ID3D11DeviceContext *ctx = nullptr;
    D3D_FEATURE_LEVEL   fl   = D3D_FEATURE_LEVEL_11_0;
    /* WARP on Windows ARM64 rejects BGRA_SUPPORT|VIDEO_SUPPORT
     * (DXGI_ERROR_UNSUPPORTED).  We don't need either flag for the
     * STAGING readback path — pass 0 and let the runtime pick the
     * feature level. */
    HRESULT dhr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr,
                                    0, nullptr, 0, D3D11_SDK_VERSION,
                                    &dev, &fl, &ctx);
    if (FAILED(dhr)) {
        std::printf("WARP D3D11CreateDevice failed 0x%08x; trying HARDWARE\n",
                    (unsigned)dhr);
        dhr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
                                0, nullptr, 0, D3D11_SDK_VERSION,
                                &dev, &fl, &ctx);
    }
    if (FAILED(dhr)) {
        std::printf("FAIL: D3D11CreateDevice -> 0x%08x\n", (unsigned)dhr);
        return 1;
    }
    std::printf("D3D11 device: feature level 0x%x\n", fl);

    /* MF requires the device be marked thread-safe before being wrapped
     * in a DXGI device manager. */
    {
        ID3D10Multithread *mt = nullptr;
        if (SUCCEEDED(dev->QueryInterface(IID_PPV_ARGS(&mt)))) {
            mt->SetMultithreadProtected(TRUE);
            mt->Release();
        }
    }

    UINT reset_token = 0;
    IMFDXGIDeviceManager *mgr = nullptr;
    HR(MFCreateDXGIDeviceManager(&reset_token, &mgr));
    HR(mgr->ResetDevice(dev, reset_token));

    /* Load the MFT DLL out of the EXE's directory. */
    HMODULE dll = LoadLibraryW(L"rkmpp_decoder_mft.dll");
    if (!dll) {
        std::printf("FAIL: LoadLibrary -> %lu\n", GetLastError()); return 3;
    }
    auto get_class_obj = (PFN_DllGetClassObject)
        GetProcAddress(dll, "DllGetClassObject");
    if (!get_class_obj) { std::printf("FAIL: GetProcAddress\n"); return 3; }

    REFCLSID clsid = (codec == CodecCli::H265)
                     ? CLSID_RkmppHevcDecoder : CLSID_RkmppH264Decoder;
    GUID input_subtype = (codec == CodecCli::H265)
                         ? MFVideoFormat_HEVC : MFVideoFormat_H264;

    IClassFactory *cf = nullptr;
    HR(get_class_obj(clsid, IID_IClassFactory, (void**)&cf));
    IMFTransform *mft = nullptr;
    HR(cf->CreateInstance(nullptr, IID_IMFTransform, (void**)&mft));
    cf->Release();

    /* Verify the MFT advertises D3D11 awareness. */
    {
        IMFAttributes *a = nullptr;
        HR(mft->GetAttributes(&a));
        UINT32 aware = 0;
        a->GetUINT32(MF_SA_D3D11_AWARE, &aware);
        std::printf("MFT MF_SA_D3D11_AWARE = %u\n", aware);
        a->Release();
        if (!aware) {
            std::printf("FAIL: MFT did not advertise D3D11 awareness\n");
            return 4;
        }
    }

    /* Hand the device manager to the MFT.  ULONG_PTR cast of an
     * IUnknown* is the documented contract for SET_D3D_MANAGER. */
    HR(mft->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER,
                           reinterpret_cast<ULONG_PTR>(mgr)));

    /* Type negotiation. */
    IMFMediaType *in_type = nullptr;
    HR(MFCreateMediaType(&in_type));
    HR(in_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    HR(in_type->SetGUID(MF_MT_SUBTYPE, input_subtype));
    HR(MFSetAttributeSize(in_type, MF_MT_FRAME_SIZE, width, height));
    HR(MFSetAttributeRatio(in_type, MF_MT_FRAME_RATE, 30, 1));
    HR(mft->SetInputType(0, in_type, 0));
    in_type->Release();

    IMFMediaType *out_avail = nullptr;
    HR(mft->GetOutputAvailableType(0, 0, &out_avail));
    HR(mft->SetOutputType(0, out_avail, 0));
    out_avail->Release();

    HR(mft->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0));

    FILE *out = nullptr;
    if (out_path) {
        if (_wfopen_s(&out, out_path, L"wb") != 0 || !out) {
            std::printf("FAIL: open %ls\n", out_path); return 4;
        }
    }

    size_t pos = 0;
    int au_idx = 0, frames_out = 0;
    while (true) {
        size_t au_off = 0, au_len = 0;
        if (!au_next(codec, bs.data(), bs.size(), &pos, &au_off, &au_len)) break;

        IMFMediaBuffer *mbuf = nullptr;
        HR(MFCreateMemoryBuffer((DWORD)au_len, &mbuf));
        BYTE *dst = nullptr; DWORD cap = 0, cur = 0;
        HR(mbuf->Lock(&dst, &cap, &cur));
        std::memcpy(dst, bs.data() + au_off, au_len);
        mbuf->Unlock();
        mbuf->SetCurrentLength((DWORD)au_len);
        IMFSample *s = nullptr;
        HR(MFCreateSample(&s));
        s->AddBuffer(mbuf);
        mbuf->Release();
        s->SetSampleTime((LONGLONG)((uint64_t)au_idx * 10'000'000ULL / 30));
        s->SetSampleDuration((LONGLONG)(10'000'000ULL / 30));

        HRESULT hr = S_OK;
        for (int retry = 0; retry < 4; retry++) {
            hr = mft->ProcessInput(0, s, 0);
            if (hr == MF_E_NOTACCEPTING) {
                if (DrainOutputs(mft, dev, ctx, width, height, out, &frames_out) != 0) {
                    s->Release(); return 5;
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
        if (DrainOutputs(mft, dev, ctx, width, height, out, &frames_out) != 0)
            return 5;
        au_idx++;
    }

    HR(mft->ProcessMessage(MFT_MESSAGE_COMMAND_DRAIN, 0));
    if (DrainOutputs(mft, dev, ctx, width, height, out, &frames_out) != 0)
        return 5;

    HR(mft->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0));
    HR(mft->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, 0));
    mft->Release();

    if (out) std::fclose(out);
    mgr->Release();
    ctx->Release();
    dev->Release();
    MFShutdown();
    CoUninitialize();

    std::printf("\n=== mft_d3d_smoke summary ===\n");
    std::printf("AUs submitted: %d\n", au_idx);
    std::printf("frames out:    %d\n", frames_out);
    std::printf("output:        %ls\n", out_path ? out_path : L"(none)");
    return frames_out > 0 ? 0 : 6;
}
