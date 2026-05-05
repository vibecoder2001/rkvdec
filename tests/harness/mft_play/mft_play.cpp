/* tests/harness/mft_play/mft_play.cpp
 *
 * Minimal Media Foundation player that forces our rkmpp decoder MFT
 * into the topology.  Builds:
 *
 *   IMFMediaSource (MP4/etc.) -> rkmpp MFT -> EVR (HWND)
 *
 * No discovery/merit games — we CoCreateInstance the MFT by CLSID and
 * stick its instance on a TRANSFORM_NODE.  The source's video subtype
 * picks H.264 vs HEVC.
 *
 * Usage:  mft_play.exe --in <url-or-path> [--codec auto|h264|h265]
 *
 * Requires the DLL to be either next to the EXE (LoadLibrary path) or
 * registered via regsvr32 (CoCreateInstance via HKCR).  Tries
 * CoCreateInstance first; falls back to LoadLibrary + DllGetClassObject.
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
#include <evr.h>
#include <d3d11.h>
#include <d3d11_1.h>
#include <dxgi1_2.h>
#include <d3dcompiler.h>
#include <shlwapi.h>
#include <unknwn.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwchar>

#include "guids.h"

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "evr.lib")
#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "shlwapi.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

#define HRCK(call) do { \
    HRESULT _hr = (call); \
    if (FAILED(_hr)) { std::fprintf(stderr, "FAIL %s:%d: %s -> 0x%08x\n", \
                                    __FILE__, __LINE__, #call, (unsigned)_hr); \
                       return _hr; } \
} while (0)

typedef HRESULT (__stdcall *PFN_DllGetClassObject)(REFCLSID, REFIID, void **);

/* -------- args -------- */

enum class CodecPick { Auto, H264, H265 };

struct Args {
    const wchar_t *in_path   = nullptr;
    CodecPick      codec     = CodecPick::Auto;
    bool           no_render = false;
    bool           use_evr   = false;
};

static bool ParseArgs(int argc, wchar_t **argv, Args *out) {
    for (int i = 1; i < argc; i++) {
        if      (!wcscmp(argv[i], L"--in")    && i+1 < argc) out->in_path = argv[++i];
        else if (!wcscmp(argv[i], L"--codec") && i+1 < argc) {
            const wchar_t *c = argv[++i];
            if      (!wcscmp(c, L"auto")) out->codec = CodecPick::Auto;
            else if (!wcscmp(c, L"h264")) out->codec = CodecPick::H264;
            else if (!wcscmp(c, L"h265") || !wcscmp(c, L"hevc")) out->codec = CodecPick::H265;
            else return false;
        }
        else if (!wcscmp(argv[i], L"--no-render")) out->no_render = true;
        else if (!wcscmp(argv[i], L"--use-evr"))   out->use_evr   = true;
        else return false;
    }
    return out->in_path != nullptr;
}

/* D3D11VideoSink — replacement for EVR's video output node.
 *
 *   1. Owns a DXGI flip-model swap chain on the player's HWND.
 *   2. NV12 → BGRA conversion via a hand-rolled vertex+pixel shader
 *      pair (full-screen triangle, two SRVs on the source NV12
 *      texture: R8 luma view + R8G8 chroma view, BT.601 limited
 *      range matrix).  ID3D11VideoDevice / ID3D11VideoProcessor was
 *      tried first but WARP on ARM64 doesn't expose it (E_NOINTERFACE
 *      on QI), so we render via the regular pipeline instead.
 *   3. OnProcessSample receives packed NV12 bytes per frame; uploads
 *      to an NV12 texture, runs a single triangle through the shader,
 *      presents.
 *
 * Skips EVR's quality manager, vsync logic, sample queueing, drop_mode
 * scheduler — all handled here directly. */
static const char *kVsHlsl =
    "struct VOut { float4 pos : SV_Position; float2 uv : TEXCOORD; };\n"
    "VOut main(uint vid : SV_VertexID) {\n"
    "    VOut o;\n"
    "    o.uv = float2((vid << 1) & 2, vid & 2);\n"
    "    o.pos = float4(o.uv * 2.0 - 1.0, 0.0, 1.0);\n"
    "    o.pos.y = -o.pos.y;\n"
    "    return o;\n"
    "}\n";

static const char *kPsHlsl =
    "Texture2D<float>  ytex  : register(t0);\n"
    "Texture2D<float2> uvtex : register(t1);\n"
    "SamplerState samp       : register(s0);\n"
    "float4 main(float4 pos : SV_Position, float2 uv : TEXCOORD) : SV_Target {\n"
    "    float  y = ytex.Sample(samp, uv).r;\n"
    "    float2 c = uvtex.Sample(samp, uv).rg;\n"
    "    float Y = y - 0.0625;\n"
    "    float U = c.r - 0.5;\n"
    "    float V = c.g - 0.5;\n"
    "    float r = saturate(1.164 * Y + 1.596 * V);\n"
    "    float g = saturate(1.164 * Y - 0.391 * U - 0.813 * V);\n"
    "    float b = saturate(1.164 * Y + 2.018 * U);\n"
    "    return float4(r, g, b, 1.0);\n"
    "}\n";
class D3D11VideoSink : public IMFSampleGrabberSinkCallback {
public:
    D3D11VideoSink() : refs_(1) {}
    ~D3D11VideoSink() { Cleanup(); }

    HRESULT Init(HWND hwnd, ID3D11Device *device, UINT width, UINT height) {
        hwnd_   = hwnd;
        width_  = width;
        height_ = height;
        device_ = device; device_->AddRef();
        device_->GetImmediateContext(&context_);

        IDXGIDevice *dxgi_dev = nullptr;
        HRCK(device_->QueryInterface(IID_PPV_ARGS(&dxgi_dev)));
        IDXGIAdapter *adapter = nullptr;
        HRCK(dxgi_dev->GetAdapter(&adapter));
        IDXGIFactory2 *factory = nullptr;
        HRCK(adapter->GetParent(IID_PPV_ARGS(&factory)));
        adapter->Release();
        dxgi_dev->Release();

        RECT cr; GetClientRect(hwnd, &cr);
        UINT cw = (cr.right  > 0) ? (UINT)cr.right  : width;
        UINT ch = (cr.bottom > 0) ? (UINT)cr.bottom : height;

        DXGI_SWAP_CHAIN_DESC1 sd = {};
        sd.Width       = cw;
        sd.Height      = ch;
        sd.Format      = DXGI_FORMAT_B8G8R8A8_UNORM;
        sd.SampleDesc.Count = 1;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.BufferCount = 2;
        sd.SwapEffect  = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.AlphaMode   = DXGI_ALPHA_MODE_IGNORE;
        HRCK(factory->CreateSwapChainForHwnd(device_, hwnd, &sd,
                                             nullptr, nullptr, &swap_chain_));
        factory->Release();

        /* Compile shaders. */
        ID3DBlob *vsb = nullptr, *psb = nullptr, *err = nullptr;
        HRESULT hr = D3DCompile(kVsHlsl, std::strlen(kVsHlsl), "vs",
                                nullptr, nullptr, "main", "vs_4_0",
                                0, 0, &vsb, &err);
        if (FAILED(hr)) {
            std::fprintf(stderr, "D3DCompile(vs) failed 0x%08x: %s\n",
                (unsigned)hr, err ? (const char*)err->GetBufferPointer() : "?");
            if (err) err->Release();
            return hr;
        }
        if (err) { err->Release(); err = nullptr; }
        hr = D3DCompile(kPsHlsl, std::strlen(kPsHlsl), "ps",
                        nullptr, nullptr, "main", "ps_4_0",
                        0, 0, &psb, &err);
        if (FAILED(hr)) {
            std::fprintf(stderr, "D3DCompile(ps) failed 0x%08x: %s\n",
                (unsigned)hr, err ? (const char*)err->GetBufferPointer() : "?");
            if (err) err->Release();
            vsb->Release();
            return hr;
        }
        if (err) err->Release();

        HRCK(device_->CreateVertexShader(vsb->GetBufferPointer(),
                                          vsb->GetBufferSize(),
                                          nullptr, &vs_));
        HRCK(device_->CreatePixelShader(psb->GetBufferPointer(),
                                         psb->GetBufferSize(),
                                         nullptr, &ps_));
        vsb->Release(); psb->Release();

        D3D11_SAMPLER_DESC sds = {};
        sds.Filter   = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
        sds.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
        sds.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
        sds.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
        sds.MaxLOD   = D3D11_FLOAT32_MAX;
        HRCK(device_->CreateSamplerState(&sds, &samp_));

        /* Pre-allocate the NV12 source texture so we don't recreate
         * per frame.  USAGE_DEFAULT + UpdateSubresource is the
         * recommended pattern for streaming uploads. */
        D3D11_TEXTURE2D_DESC td = {};
        td.Width              = width_;
        td.Height             = height_;
        td.MipLevels          = 1;
        td.ArraySize          = 1;
        td.Format             = DXGI_FORMAT_NV12;
        td.SampleDesc.Count   = 1;
        td.Usage              = D3D11_USAGE_DEFAULT;
        td.BindFlags          = D3D11_BIND_SHADER_RESOURCE;
        HRCK(device_->CreateTexture2D(&td, nullptr, &nv12_tex_));

        D3D11_SHADER_RESOURCE_VIEW_DESC sd_y = {};
        sd_y.Format        = DXGI_FORMAT_R8_UNORM;
        sd_y.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
        sd_y.Texture2D.MipLevels = 1;
        HRCK(device_->CreateShaderResourceView(nv12_tex_, &sd_y, &y_srv_));

        D3D11_SHADER_RESOURCE_VIEW_DESC sd_uv = sd_y;
        sd_uv.Format = DXGI_FORMAT_R8G8_UNORM;
        HRCK(device_->CreateShaderResourceView(nv12_tex_, &sd_uv, &uv_srv_));

        std::fprintf(stderr,
            "D3D11VideoSink: swap chain %ux%u (window) BGRA, NV12 source %ux%u, "
            "shader pipeline ready\n", cw, ch, width_, height_);
        std::fflush(stderr);
        return S_OK;
    }

    void Cleanup() {
        if (uv_srv_)     { uv_srv_->Release();     uv_srv_     = nullptr; }
        if (y_srv_)      { y_srv_->Release();      y_srv_      = nullptr; }
        if (nv12_tex_)   { nv12_tex_->Release();   nv12_tex_   = nullptr; }
        if (samp_)       { samp_->Release();       samp_       = nullptr; }
        if (ps_)         { ps_->Release();         ps_         = nullptr; }
        if (vs_)         { vs_->Release();         vs_         = nullptr; }
        if (swap_chain_) { swap_chain_->Release(); swap_chain_ = nullptr; }
        if (context_)    { context_->Release();    context_    = nullptr; }
        if (device_)     { device_->Release();     device_     = nullptr; }
    }

    HRESULT RenderNV12(const BYTE *nv12, DWORD len) {
        const UINT y_size = width_ * height_;
        if (len < y_size + (y_size / 2)) return E_INVALIDARG;

        /* Upload NV12 bytes to the source texture (Y subresource 0,
         * UV subresource 1).  Both at row pitch == width. */
        const UINT row_pitch = width_;
        context_->UpdateSubresource(nv12_tex_, 0, nullptr,
                                    nv12, row_pitch, row_pitch * height_);
        context_->UpdateSubresource(nv12_tex_, 1, nullptr,
                                    nv12 + y_size, row_pitch,
                                    row_pitch * (height_ / 2));

        /* Bind back buffer as render target, run a fullscreen-triangle
         * pass through the YUV→BGRA pixel shader. */
        ID3D11Texture2D *bb = nullptr;
        HRESULT hr = swap_chain_->GetBuffer(0, IID_PPV_ARGS(&bb));
        if (FAILED(hr)) return hr;
        ID3D11RenderTargetView *rtv = nullptr;
        hr = device_->CreateRenderTargetView(bb, nullptr, &rtv);
        bb->Release();
        if (FAILED(hr)) return hr;

        D3D11_VIEWPORT vp = {};
        DXGI_SWAP_CHAIN_DESC1 cur_desc;
        swap_chain_->GetDesc1(&cur_desc);
        vp.Width  = (FLOAT)cur_desc.Width;
        vp.Height = (FLOAT)cur_desc.Height;
        vp.MaxDepth = 1.0f;

        ID3D11ShaderResourceView *srvs[2] = { y_srv_, uv_srv_ };
        context_->OMSetRenderTargets(1, &rtv, nullptr);
        context_->RSSetViewports(1, &vp);
        context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        context_->IASetInputLayout(nullptr);
        context_->VSSetShader(vs_, nullptr, 0);
        context_->PSSetShader(ps_, nullptr, 0);
        context_->PSSetShaderResources(0, 2, srvs);
        context_->PSSetSamplers(0, 1, &samp_);
        context_->Draw(3, 0);

        /* Unbind RTV so next GetBuffer doesn't trip "resource still bound". */
        ID3D11RenderTargetView *null_rtv = nullptr;
        context_->OMSetRenderTargets(1, &null_rtv, nullptr);
        rtv->Release();

        hr = swap_chain_->Present(1, 0);
        return hr;
    }

    /* IUnknown */
    STDMETHODIMP QueryInterface(REFIID iid, void **ppv) override {
        if (!ppv) return E_POINTER;
        if (iid == IID_IUnknown ||
            iid == IID_IMFSampleGrabberSinkCallback ||
            iid == IID_IMFClockStateSink) {
            *ppv = static_cast<IMFSampleGrabberSinkCallback*>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    STDMETHODIMP_(ULONG) AddRef() override  { return InterlockedIncrement(&refs_); }
    STDMETHODIMP_(ULONG) Release() override {
        long r = InterlockedDecrement(&refs_);
        if (r == 0) delete this;
        return r;
    }
    /* IMFClockStateSink — sink is clock-driven, but we render
     * immediately on each sample receive; clock state changes don't
     * gate us. */
    STDMETHODIMP OnClockStart(MFTIME, LONGLONG) override { return S_OK; }
    STDMETHODIMP OnClockStop(MFTIME)            override { return S_OK; }
    STDMETHODIMP OnClockPause(MFTIME)           override { return S_OK; }
    STDMETHODIMP OnClockRestart(MFTIME)         override { return S_OK; }
    STDMETHODIMP OnClockSetRate(MFTIME, float)  override { return S_OK; }
    /* IMFSampleGrabberSinkCallback */
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock*) override { return S_OK; }
    STDMETHODIMP OnProcessSample(REFGUID /*major_type*/, DWORD /*flags*/,
                                  LONGLONG /*sample_time*/, LONGLONG /*duration*/,
                                  const BYTE *buffer, DWORD len) override {
        return RenderNV12(buffer, len);
    }
    STDMETHODIMP OnShutdown() override { return S_OK; }

private:
    long                       refs_;
    HWND                       hwnd_       = nullptr;
    UINT                       width_      = 0;
    UINT                       height_     = 0;
    ID3D11Device              *device_     = nullptr;
    ID3D11DeviceContext       *context_    = nullptr;
    IDXGISwapChain1           *swap_chain_ = nullptr;
    ID3D11VertexShader        *vs_         = nullptr;
    ID3D11PixelShader         *ps_         = nullptr;
    ID3D11SamplerState        *samp_       = nullptr;
    ID3D11Texture2D           *nv12_tex_   = nullptr;
    ID3D11ShaderResourceView  *y_srv_      = nullptr;
    ID3D11ShaderResourceView  *uv_srv_     = nullptr;
};

/* No-op IMFSampleGrabberSinkCallback — used when --no-render is set.
 * Replaces the EVR output node with a sample-grabber sink that silently
 * accepts every video sample.  Decode + MFT run at full rate, no
 * presenter / vsync / GPU upload involved.  If playback finishes much
 * faster than realtime in this mode while EVR-bound playback stutters,
 * the bottleneck is EVR's render side, not our pipeline. */
class NoopGrabber : public IMFSampleGrabberSinkCallback {
    long refs_ = 1;
public:
    /* IUnknown */
    STDMETHODIMP QueryInterface(REFIID iid, void **ppv) override {
        if (!ppv) return E_POINTER;
        if (iid == IID_IUnknown ||
            iid == IID_IMFSampleGrabberSinkCallback ||
            iid == IID_IMFClockStateSink) {
            *ppv = static_cast<IMFSampleGrabberSinkCallback*>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    STDMETHODIMP_(ULONG) AddRef() override  { return InterlockedIncrement(&refs_); }
    STDMETHODIMP_(ULONG) Release() override {
        long r = InterlockedDecrement(&refs_);
        if (r == 0) delete this;
        return r;
    }
    /* IMFClockStateSink */
    STDMETHODIMP OnClockStart(MFTIME, LONGLONG) override { return S_OK; }
    STDMETHODIMP OnClockStop(MFTIME)            override { return S_OK; }
    STDMETHODIMP OnClockPause(MFTIME)           override { return S_OK; }
    STDMETHODIMP OnClockRestart(MFTIME)         override { return S_OK; }
    STDMETHODIMP OnClockSetRate(MFTIME, float)  override { return S_OK; }
    /* IMFSampleGrabberSinkCallback */
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock*) override { return S_OK; }
    STDMETHODIMP OnProcessSample(REFGUID, DWORD, LONGLONG, LONGLONG,
                                 const BYTE*, DWORD) override { return S_OK; }
    STDMETHODIMP OnShutdown() override { return S_OK; }
};

/* -------- MFT instantiation -------- */

static HRESULT CreateRkmppMft(REFCLSID clsid, IMFTransform **out) {
    /* Try registered first. */
    HRESULT hr = CoCreateInstance(clsid, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_IMFTransform, (void**)out);
    if (SUCCEEDED(hr)) return S_OK;

    /* Fall back to local DLL load (DLL next to mft_play.exe). */
    HMODULE dll = LoadLibraryW(L"rkmpp_decoder_mft.dll");
    if (!dll) {
        std::fprintf(stderr, "CoCreateInstance failed (0x%08x) and "
                     "LoadLibrary fallback failed (%lu)\n",
                     (unsigned)hr, GetLastError());
        return hr;
    }
    auto get = (PFN_DllGetClassObject)
        GetProcAddress(dll, "DllGetClassObject");
    if (!get) return E_FAIL;
    IClassFactory *cf = nullptr;
    hr = get(clsid, IID_IClassFactory, (void**)&cf);
    if (FAILED(hr)) return hr;
    hr = cf->CreateInstance(nullptr, IID_IMFTransform, (void**)out);
    cf->Release();
    return hr;
}

/* -------- pick a stream by major type (video or audio) -------- */

static HRESULT PickStreamByMajor(IMFPresentationDescriptor *pd,
                                 const GUID                &want_major,
                                 IMFStreamDescriptor       **out_sd,
                                 DWORD                     *out_index,
                                 GUID                      *out_subtype) {
    DWORD count = 0;
    HRCK(pd->GetStreamDescriptorCount(&count));
    for (DWORD i = 0; i < count; i++) {
        BOOL selected = FALSE;
        IMFStreamDescriptor *sd = nullptr;
        HRCK(pd->GetStreamDescriptorByIndex(i, &selected, &sd));
        IMFMediaTypeHandler *mth = nullptr;
        HRESULT hr = sd->GetMediaTypeHandler(&mth);
        GUID major = {};
        if (SUCCEEDED(hr)) hr = mth->GetMajorType(&major);
        if (FAILED(hr) || major != want_major) {
            if (mth) mth->Release();
            sd->Release();
            continue;
        }
        IMFMediaType *mt = nullptr;
        hr = mth->GetCurrentMediaType(&mt);
        mth->Release();
        if (FAILED(hr)) { sd->Release(); continue; }
        GUID sub = {};
        mt->GetGUID(MF_MT_SUBTYPE, &sub);
        mt->Release();
        if (!selected) pd->SelectStream(i);
        *out_sd      = sd;
        *out_index   = i;
        if (out_subtype) *out_subtype = sub;
        return S_OK;
    }
    return MF_E_NO_MORE_TYPES;
}

static HRESULT PickVideoStream(IMFPresentationDescriptor *pd,
                               IMFStreamDescriptor **out_sd,
                               DWORD *out_index, GUID *out_subtype) {
    return PickStreamByMajor(pd, MFMediaType_Video, out_sd, out_index, out_subtype);
}

static HRESULT PickAudioStream(IMFPresentationDescriptor *pd,
                               IMFStreamDescriptor **out_sd,
                               DWORD *out_index) {
    return PickStreamByMajor(pd, MFMediaType_Audio, out_sd, out_index, nullptr);
}

/* -------- topology builders -------- */

static HRESULT AddSourceNode(IMFTopology *topo, IMFMediaSource *src,
                             IMFPresentationDescriptor *pd,
                             IMFStreamDescriptor *sd,
                             IMFTopologyNode **out) {
    IMFTopologyNode *n = nullptr;
    HRCK(MFCreateTopologyNode(MF_TOPOLOGY_SOURCESTREAM_NODE, &n));
    HRCK(n->SetUnknown(MF_TOPONODE_SOURCE, src));
    HRCK(n->SetUnknown(MF_TOPONODE_PRESENTATION_DESCRIPTOR, pd));
    HRCK(n->SetUnknown(MF_TOPONODE_STREAM_DESCRIPTOR, sd));
    HRCK(topo->AddNode(n));
    *out = n;
    return S_OK;
}

static HRESULT AddTransformNode(IMFTopology *topo, IMFTransform *mft,
                                IMFTopologyNode **out) {
    IMFTopologyNode *n = nullptr;
    HRCK(MFCreateTopologyNode(MF_TOPOLOGY_TRANSFORM_NODE, &n));
    HRCK(n->SetObject(mft));
    HRCK(topo->AddNode(n));
    *out = n;
    return S_OK;
}

/* Create a hardware D3D11 device with video support, wrap it in an
 * IMFDXGIDeviceManager, and return the manager.  The manager is what
 * we'll try to push into EVR's topology so it forwards a D3D11 device
 * to our MFT via SET_D3D_MANAGER.
 *
 * EVR's default presenter on Windows is D3D9-based and does NOT honor
 * a DXGI device manager pushed via topology attributes — to actually
 * get D3D11 acceleration end-to-end you need a custom IMFVideoPresenter
 * (Microsoft has a sample) or to use MFMediaEngine instead of EVR.
 * Creating the manager here is harmless when the default presenter
 * ignores it, and gives us the plumbing for the eventual swap. */
static HRESULT CreateDxgiDeviceManager(IMFDXGIDeviceManager **out_mgr,
                                       ID3D11Device **out_dev) {
    *out_mgr = nullptr;
    *out_dev = nullptr;

    ID3D11Device        *dev = nullptr;
    ID3D11DeviceContext *ctx = nullptr;
    D3D_FEATURE_LEVEL    fl  = D3D_FEATURE_LEVEL_11_0;
    D3D_FEATURE_LEVEL    feature_levels[] = {
        D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0,
        D3D_FEATURE_LEVEL_10_1, D3D_FEATURE_LEVEL_10_0,
    };
    HRESULT hr = D3D11CreateDevice(
        nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
        D3D11_CREATE_DEVICE_VIDEO_SUPPORT |
        D3D11_CREATE_DEVICE_BGRA_SUPPORT,
        feature_levels, ARRAYSIZE(feature_levels),
        D3D11_SDK_VERSION, &dev, &fl, &ctx);
    if (FAILED(hr)) {
        std::fprintf(stderr,
            "mft_play: D3D11CreateDevice(HARDWARE) failed 0x%08x — "
            "trying WARP fallback\n", (unsigned)hr);
        hr = D3D11CreateDevice(
            nullptr, D3D_DRIVER_TYPE_WARP, nullptr,
            D3D11_CREATE_DEVICE_VIDEO_SUPPORT,
            feature_levels, ARRAYSIZE(feature_levels),
            D3D11_SDK_VERSION, &dev, &fl, &ctx);
        if (FAILED(hr)) {
            std::fprintf(stderr,
                "mft_play: D3D11CreateDevice(WARP) also failed 0x%08x\n",
                (unsigned)hr);
            return hr;
        }
    }

    /* Multi-thread protection is required when sharing the device
     * between MF threads and the MFT. */
    ID3D10Multithread *mt = nullptr;
    if (SUCCEEDED(dev->QueryInterface(IID_PPV_ARGS(&mt)))) {
        mt->SetMultithreadProtected(TRUE);
        mt->Release();
    }
    if (ctx) ctx->Release();

    UINT reset_token = 0;
    IMFDXGIDeviceManager *mgr = nullptr;
    hr = MFCreateDXGIDeviceManager(&reset_token, &mgr);
    if (FAILED(hr)) {
        std::fprintf(stderr,
            "mft_play: MFCreateDXGIDeviceManager failed 0x%08x\n",
            (unsigned)hr);
        dev->Release();
        return hr;
    }
    hr = mgr->ResetDevice(dev, reset_token);
    if (FAILED(hr)) {
        std::fprintf(stderr,
            "mft_play: IMFDXGIDeviceManager::ResetDevice failed 0x%08x\n",
            (unsigned)hr);
        mgr->Release();
        dev->Release();
        return hr;
    }
    std::fprintf(stderr,
        "mft_play: D3D11 device manager ready (feature_level 0x%04x)\n",
        (unsigned)fl);

    *out_mgr = mgr;
    *out_dev = dev;
    return S_OK;
}

static HRESULT AddOutputNode(IMFTopology *topo, HWND hwnd,
                             IMFDXGIDeviceManager *dxgi_mgr,
                             ID3D11Device *d3d11_dev,
                             const Args &args,
                             IMFMediaType *video_type,
                             IMFTopologyNode **out) {
    IMFActivate *output_activate = nullptr;
    if (args.no_render) {
        /* Sample-grabber sink with a no-op callback — diagnostic mode. */
        NoopGrabber *grabber = new NoopGrabber();
        HRESULT hr = MFCreateSampleGrabberSinkActivate(video_type, grabber,
                                                       &output_activate);
        grabber->Release();
        if (FAILED(hr)) {
            std::fprintf(stderr,
                "MFCreateSampleGrabberSinkActivate(noop) failed 0x%08x\n",
                (unsigned)hr);
            return hr;
        }
    } else if (args.use_evr) {
        HRCK(MFCreateVideoRendererActivate(hwnd, &output_activate));
        if (dxgi_mgr) {
            (void)output_activate->SetUINT32(MF_SA_D3D11_AWARE, TRUE);
        }
    } else {
        /* Default: replace EVR with our D3D11VideoSink wrapped via a
         * sample-grabber sink.  Skips EVR's quality manager / vsync /
         * sample queueing entirely — we render directly to a DXGI
         * swap chain on the player HWND.  Pull frame dims from the
         * grabber media type. */
        if (!d3d11_dev) {
            std::fprintf(stderr,
                "mft_play: D3D11 device unavailable, falling back to EVR\n");
            HRCK(MFCreateVideoRendererActivate(hwnd, &output_activate));
        } else {
            UINT64 frame_size = 0;
            HRCK(video_type->GetUINT64(MF_MT_FRAME_SIZE, &frame_size));
            UINT w = (UINT)(frame_size >> 32);
            UINT h = (UINT)(frame_size & 0xFFFFFFFF);

            D3D11VideoSink *sink = new D3D11VideoSink();
            HRESULT hr = sink->Init(hwnd, d3d11_dev, w, h);
            if (FAILED(hr)) {
                std::fprintf(stderr,
                    "D3D11VideoSink::Init failed 0x%08x — falling back to EVR\n",
                    (unsigned)hr);
                sink->Release();
                HRCK(MFCreateVideoRendererActivate(hwnd, &output_activate));
            } else {
                hr = MFCreateSampleGrabberSinkActivate(video_type, sink,
                                                      &output_activate);
                sink->Release();
                if (FAILED(hr)) {
                    std::fprintf(stderr,
                        "MFCreateSampleGrabberSinkActivate(D3D11) "
                        "failed 0x%08x\n", (unsigned)hr);
                    return hr;
                }
                std::fprintf(stderr,
                    "mft_play: video output = D3D11VideoSink (custom swap chain)\n");
            }
        }
    }

    IMFTopologyNode *n = nullptr;
    HRCK(MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &n));
    HRCK(n->SetObject(output_activate));
    HRCK(n->SetUINT32(MF_TOPONODE_STREAMID, 0));
    HRCK(n->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE));
    if (args.use_evr && dxgi_mgr) {
        (void)n->SetUINT32(MF_TOPONODE_D3DAWARE, TRUE);
    }
    HRCK(topo->AddNode(n));
    output_activate->Release();
    *out = n;
    return S_OK;
}

/* SAR (Streaming Audio Renderer) output node.  We attach the source's
 * audio stream straight to it; MF's topology resolver inserts the
 * appropriate audio decoder MFT (AAC/etc.) automatically when the
 * session is started. */
static HRESULT AddAudioOutputNode(IMFTopology *topo, IMFTopologyNode **out) {
    IMFActivate *sar = nullptr;
    HRCK(MFCreateAudioRendererActivate(&sar));
    IMFTopologyNode *n = nullptr;
    HRCK(MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &n));
    HRCK(n->SetObject(sar));
    HRCK(n->SetUINT32(MF_TOPONODE_STREAMID, 0));
    HRCK(n->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE));
    HRCK(topo->AddNode(n));
    sar->Release();
    *out = n;
    return S_OK;
}

/* -------- session event pump -------- */

static int RunSession(IMFMediaSession *sess, HWND hwnd) {
    PROPVARIANT start;  PropVariantInit(&start);
    HRESULT hr = sess->Start(&GUID_NULL, &start);
    PropVariantClear(&start);
    if (FAILED(hr)) {
        std::fprintf(stderr, "Start failed 0x%08x\n", (unsigned)hr);
        return 1;
    }

    bool ended = false;
    while (!ended) {
        /* Pump window messages so EVR can repaint. */
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) ended = true;
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }

        IMFMediaEvent *ev = nullptr;
        hr = sess->GetEvent(MF_EVENT_FLAG_NO_WAIT, &ev);
        if (hr == MF_E_NO_EVENTS_AVAILABLE) {
            Sleep(10);
            continue;
        }
        if (FAILED(hr)) break;

        MediaEventType type = MEUnknown;
        ev->GetType(&type);
        HRESULT status = S_OK;
        ev->GetStatus(&status);
        if (FAILED(status)) {
            std::fprintf(stderr, "session event %u failed 0x%08x\n",
                         (unsigned)type, (unsigned)status);
        }

        switch (type) {
        case MESessionEnded:
            std::printf("MESessionEnded\n");
            sess->Stop();
            break;
        case MESessionStopped:
            std::printf("MESessionStopped\n");
            sess->Close();
            break;
        case MESessionClosed:
            std::printf("MESessionClosed\n");
            ended = true;
            break;
        case MESessionTopologyStatus: {
            UINT32 topo_status = 0;
            ev->GetUINT32(MF_EVENT_TOPOLOGY_STATUS, &topo_status);
            std::printf("topology status=%u\n", topo_status);
            if (FAILED(status)) ended = true;
            break;
        }
        default:
            break;
        }
        ev->Release();
    }
    (void)hwnd;
    return 0;
}

/* -------- window -------- */

static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    switch (m) {
    case WM_CLOSE:   PostQuitMessage(0); return 0;
    case WM_DESTROY: PostQuitMessage(0); return 0;
    }
    return DefWindowProcW(h, m, w, l);
}

static HWND CreateRenderWindow() {
    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = WndProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, (LPCWSTR)IDC_ARROW);
    wc.lpszClassName = L"rkmpp_mft_play";
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    RegisterClassExW(&wc);
    HWND h = CreateWindowExW(0, wc.lpszClassName, L"rkmpp mft_play",
                             WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                             CW_USEDEFAULT, CW_USEDEFAULT,
                             1280, 720,
                             nullptr, nullptr, wc.hInstance, nullptr);
    return h;
}

/* -------- main -------- */

int wmain(int argc, wchar_t **argv) {
    Args args;
    if (!ParseArgs(argc, argv, &args)) {
        std::fwprintf(stderr,
            L"usage: mft_play --in <url-or-path> [--codec auto|h264|h265]\n");
        return 1;
    }

    HRCK(CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED));
    HRCK(MFStartup(MF_VERSION, MFSTARTUP_FULL));

    HWND hwnd = CreateRenderWindow();
    if (!hwnd) { std::fprintf(stderr, "CreateWindow failed\n"); return 2; }

    /* --- source --- */
    IMFSourceResolver *resolver = nullptr;
    HRCK(MFCreateSourceResolver(&resolver));
    MF_OBJECT_TYPE obj_type = MF_OBJECT_INVALID;
    IUnknown *src_unk = nullptr;
    HRCK(resolver->CreateObjectFromURL(args.in_path,
        MF_RESOLUTION_MEDIASOURCE
        | MF_RESOLUTION_CONTENT_DOES_NOT_HAVE_TO_MATCH_EXTENSION_OR_MIME_TYPE,
        nullptr, &obj_type, &src_unk));
    resolver->Release();
    IMFMediaSource *src = nullptr;
    HRCK(src_unk->QueryInterface(IID_PPV_ARGS(&src)));
    src_unk->Release();

    IMFPresentationDescriptor *pd = nullptr;
    HRCK(src->CreatePresentationDescriptor(&pd));

    /* Deselect every stream first so PickStreamByMajor takes ownership
     * of selection.  We re-select video unconditionally and audio if
     * present — A/V are both wanted so we can verify sync. */
    DWORD sd_count = 0;
    pd->GetStreamDescriptorCount(&sd_count);
    for (DWORD i = 0; i < sd_count; i++) pd->DeselectStream(i);

    IMFStreamDescriptor *sd = nullptr;
    DWORD video_idx = 0;
    GUID  video_sub = {};
    HRCK(PickVideoStream(pd, &sd, &video_idx, &video_sub));

    /* Audio is optional — Annex-B / .h264 / .h265 inputs are video-only
     * and PickAudioStream returns MF_E_NO_MORE_TYPES.  MP4 / MKV / etc.
     * carrying both video and audio will hit the audio branch and run
     * SAR alongside the video pipeline.  Both render against the same
     * MF presentation clock, so any drift between decoded video PTS
     * and SAR's playback position will manifest as A/V desync. */
    IMFStreamDescriptor *audio_sd  = nullptr;
    DWORD                audio_idx = 0;
    HRESULT              audio_hr  = PickAudioStream(pd, &audio_sd, &audio_idx);
    const bool           has_audio = SUCCEEDED(audio_hr);
    std::printf("audio stream: %s\n", has_audio ? "present" : "none");

    REFCLSID clsid = (args.codec == CodecPick::H265
                      || (args.codec == CodecPick::Auto && video_sub == MFVideoFormat_HEVC))
                     ? CLSID_RkmppHevcDecoder
                     : CLSID_RkmppH264Decoder;

    /* --- MFT --- */
    IMFTransform *mft = nullptr;
    HRCK(CreateRkmppMft(clsid, &mft));
    std::printf("rkmpp MFT instantiated for %s\n",
                clsid == CLSID_RkmppHevcDecoder ? "HEVC" : "H.264");

    /* --- D3D11 device manager (best-effort) --- */
    IMFDXGIDeviceManager *dxgi_mgr = nullptr;
    ID3D11Device         *d3d11_dev = nullptr;
    if (FAILED(CreateDxgiDeviceManager(&dxgi_mgr, &d3d11_dev))) {
        std::fprintf(stderr,
            "mft_play: continuing without D3D11 manager — sysmem path\n");
    }

    /* Push the manager directly to the MFT only when EVR is the sink.
     * Our D3D11VideoSink + sample-grabber path needs the MFT's output
     * as IMFMediaBuffer-Lockable bytes (sysmem), not DXGI surface
     * buffers; the sample grabber would otherwise see opaque GPU
     * surfaces it can't memcpy out of. */
    if (args.use_evr && dxgi_mgr && mft) {
        HRESULT hr_mgr = mft->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER,
                                             (ULONG_PTR)dxgi_mgr);
        std::fprintf(stderr,
            "mft_play: MFT SET_D3D_MANAGER returned 0x%08x\n",
            (unsigned)hr_mgr);
    }

    /* The sample-grabber sink (used in both default D3D11 mode and
     * --no-render mode) needs a media type matching the MFT's output:
     * NV12 at the source video dimensions. */
    IMFMediaType *grabber_type = nullptr;
    if (!args.use_evr) {
        IMFMediaTypeHandler *mth = nullptr;
        HRCK(sd->GetMediaTypeHandler(&mth));
        IMFMediaType *src_type = nullptr;
        HRCK(mth->GetCurrentMediaType(&src_type));
        UINT64 frame_size = 0;
        HRCK(src_type->GetUINT64(MF_MT_FRAME_SIZE, &frame_size));
        src_type->Release();
        mth->Release();

        HRCK(MFCreateMediaType(&grabber_type));
        HRCK(grabber_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
        HRCK(grabber_type->SetGUID(MF_MT_SUBTYPE,    MFVideoFormat_NV12));
        HRCK(grabber_type->SetUINT64(MF_MT_FRAME_SIZE, frame_size));
    }

    /* --- topology --- */
    IMFTopology *topo = nullptr;
    HRCK(MFCreateTopology(&topo));

    IMFTopologyNode *src_node = nullptr, *xform_node = nullptr, *out_node = nullptr;
    HRCK(AddSourceNode   (topo, src, pd, sd, &src_node));
    HRCK(AddTransformNode(topo, mft,         &xform_node));
    HRCK(AddOutputNode   (topo, hwnd, dxgi_mgr, d3d11_dev,
                          args, grabber_type, &out_node));
    if (grabber_type) grabber_type->Release();
    HRCK(src_node ->ConnectOutput(0, xform_node, 0));
    HRCK(xform_node->ConnectOutput(0, out_node, 0));

    /* Audio branch — source straight into SAR.  We deliberately don't
     * insert a decoder MFT; MF's topology loader does that for us
     * during SetTopology resolution based on the audio stream's
     * subtype (AAC / MP3 / PCM / ...).  This is a partially-resolved
     * branch, which works because MFCreateMediaSession's default
     * topology loader fills in missing transforms. */
    IMFTopologyNode *audio_src_node = nullptr, *audio_out_node = nullptr;
    if (has_audio) {
        HRCK(AddSourceNode     (topo, src, pd, audio_sd, &audio_src_node));
        HRCK(AddAudioOutputNode(topo, &audio_out_node));
        HRCK(audio_src_node->ConnectOutput(0, audio_out_node, 0));
    }

    /* --- session --- */
    IMFMediaSession *sess = nullptr;
    HRCK(MFCreateMediaSession(nullptr, &sess));
    HRCK(sess->SetTopology(0, topo));

    int rc = RunSession(sess, hwnd);

    sess->Release();
    topo->Release();
    src_node->Release();
    xform_node->Release();
    out_node->Release();
    if (audio_src_node) audio_src_node->Release();
    if (audio_out_node) audio_out_node->Release();
    mft->Release();
    sd->Release();
    if (audio_sd) audio_sd->Release();
    pd->Release();
    src->Shutdown();
    src->Release();
    if (dxgi_mgr)  dxgi_mgr->Release();
    if (d3d11_dev) d3d11_dev->Release();

    DestroyWindow(hwnd);
    MFShutdown();
    CoUninitialize();
    return rc;
}
