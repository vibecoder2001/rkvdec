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
    const wchar_t *in_path = nullptr;
    CodecPick      codec   = CodecPick::Auto;
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
        } else return false;
    }
    return out->in_path != nullptr;
}

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

/* -------- pick the video stream + codec -------- */

static HRESULT PickVideoStream(IMFPresentationDescriptor *pd,
                               IMFStreamDescriptor **out_sd,
                               DWORD *out_index, GUID *out_subtype) {
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
        if (FAILED(hr) || major != MFMediaType_Video) {
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
        *out_subtype = sub;
        return S_OK;
    }
    return MF_E_NO_MORE_TYPES;
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

static HRESULT AddOutputNode(IMFTopology *topo, HWND hwnd,
                             IMFTopologyNode **out) {
    IMFActivate *evr = nullptr;
    HRCK(MFCreateVideoRendererActivate(hwnd, &evr));
    IMFTopologyNode *n = nullptr;
    HRCK(MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &n));
    HRCK(n->SetObject(evr));
    HRCK(n->SetUINT32(MF_TOPONODE_STREAMID, 0));
    HRCK(n->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE));
    HRCK(topo->AddNode(n));
    evr->Release();
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

    /* Deselect every stream, then re-select only the video stream we
     * pick.  Skipping audio keeps the topology resolver from trying to
     * stand up an audio sink we don't care about. */
    DWORD sd_count = 0;
    pd->GetStreamDescriptorCount(&sd_count);
    for (DWORD i = 0; i < sd_count; i++) pd->DeselectStream(i);

    IMFStreamDescriptor *sd = nullptr;
    DWORD video_idx = 0;
    GUID  video_sub = {};
    HRCK(PickVideoStream(pd, &sd, &video_idx, &video_sub));

    REFCLSID clsid = (args.codec == CodecPick::H265
                      || (args.codec == CodecPick::Auto && video_sub == MFVideoFormat_HEVC))
                     ? CLSID_RkmppHevcDecoder
                     : CLSID_RkmppH264Decoder;

    /* --- MFT --- */
    IMFTransform *mft = nullptr;
    HRCK(CreateRkmppMft(clsid, &mft));
    std::printf("rkmpp MFT instantiated for %s\n",
                clsid == CLSID_RkmppHevcDecoder ? "HEVC" : "H.264");

    /* --- topology --- */
    IMFTopology *topo = nullptr;
    HRCK(MFCreateTopology(&topo));

    IMFTopologyNode *src_node = nullptr, *xform_node = nullptr, *out_node = nullptr;
    HRCK(AddSourceNode   (topo, src, pd, sd, &src_node));
    HRCK(AddTransformNode(topo, mft,         &xform_node));
    HRCK(AddOutputNode   (topo, hwnd,        &out_node));
    HRCK(src_node ->ConnectOutput(0, xform_node, 0));
    HRCK(xform_node->ConnectOutput(0, out_node, 0));

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
    mft->Release();
    sd->Release();
    pd->Release();
    src->Shutdown();
    src->Release();

    DestroyWindow(hwnd);
    MFShutdown();
    CoUninitialize();
    return rc;
}
