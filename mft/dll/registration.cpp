/* mft/dll/registration.cpp — DllRegisterServer / DllUnregisterServer
 * helpers.  Writes the standard HKCR\CLSID\{guid} entry plus the
 * MFTRegister entry (so MFTEnumEx can find us under
 * MFT_CATEGORY_VIDEO_DECODER).
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "registration.h"
#include "decoder_mft.h"
#include "guids.h"

#include <mfapi.h>
#include <mftransform.h>
#include <strsafe.h>

#pragma comment(lib, "mfplat.lib")

namespace rkmpp {

/* Stringify a CLSID as "{XXXX...}" (no braces? include them). */
static HRESULT ClsidToString(REFGUID g, wchar_t *out, size_t out_chars) {
    return StringCchPrintfW(out, out_chars,
        L"{%08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X}",
        g.Data1, g.Data2, g.Data3,
        g.Data4[0], g.Data4[1], g.Data4[2], g.Data4[3],
        g.Data4[4], g.Data4[5], g.Data4[6], g.Data4[7]);
}

static HRESULT WriteClsidRegistry(REFGUID clsid,
                                   const wchar_t *friendly,
                                   const wchar_t *dll_path) {
    wchar_t guid_str[64];
    HRESULT hr = ClsidToString(clsid, guid_str, ARRAYSIZE(guid_str));
    if (FAILED(hr)) return hr;

    wchar_t key_path[256];
    StringCchPrintfW(key_path, ARRAYSIZE(key_path), L"CLSID\\%s", guid_str);

    HKEY root;
    LONG s = RegCreateKeyExW(HKEY_CLASSES_ROOT, key_path, 0, nullptr,
                             0, KEY_WRITE, nullptr, &root, nullptr);
    if (s != ERROR_SUCCESS) return HRESULT_FROM_WIN32(s);
    RegSetValueExW(root, nullptr, 0, REG_SZ,
                   (const BYTE*)friendly,
                   (DWORD)((wcslen(friendly) + 1) * sizeof(wchar_t)));

    HKEY srv;
    s = RegCreateKeyExW(root, L"InprocServer32", 0, nullptr,
                       0, KEY_WRITE, nullptr, &srv, nullptr);
    if (s == ERROR_SUCCESS) {
        RegSetValueExW(srv, nullptr, 0, REG_SZ,
                       (const BYTE*)dll_path,
                       (DWORD)((wcslen(dll_path) + 1) * sizeof(wchar_t)));
        const wchar_t *both = L"Both";
        RegSetValueExW(srv, L"ThreadingModel", 0, REG_SZ,
                       (const BYTE*)both, (DWORD)(5 * sizeof(wchar_t)));
        RegCloseKey(srv);
    }
    RegCloseKey(root);
    return S_OK;
}

static HRESULT DeleteClsidRegistry(REFGUID clsid) {
    wchar_t guid_str[64];
    ClsidToString(clsid, guid_str, ARRAYSIZE(guid_str));
    wchar_t key_path[256];
    StringCchPrintfW(key_path, ARRAYSIZE(key_path),
                     L"CLSID\\%s\\InprocServer32", guid_str);
    RegDeleteKeyW(HKEY_CLASSES_ROOT, key_path);
    StringCchPrintfW(key_path, ARRAYSIZE(key_path), L"CLSID\\%s", guid_str);
    RegDeleteKeyW(HKEY_CLASSES_ROOT, key_path);
    return S_OK;
}

static HRESULT RegisterOne(CodecKind kind) {
    MFT_REGISTER_TYPE_INFO in  = { MFMediaType_Video, DecoderInputSubtype(kind) };
    /* Advertise both NV12 (8-bit streams) and P010 (10-bit High 10
     * H.264 / HEVC Main10).  The MFT picks per-stream which one to
     * surface from BuildOutputType once the SPS bit-depth is known. */
    MFT_REGISTER_TYPE_INFO out[] = {
        { MFMediaType_Video, MFVideoFormat_NV12 },
        { MFMediaType_Video, MFVideoFormat_P010 },
    };
    return MFTRegister(
        DecoderClsid(kind),
        MFT_CATEGORY_VIDEO_DECODER,
        const_cast<LPWSTR>(DecoderFriendlyName(kind)),
        MFT_ENUM_FLAG_SYNCMFT,
        1, &in,
        (UINT32)(sizeof(out) / sizeof(out[0])), out,
        nullptr);
}

HRESULT RegisterServer(HMODULE self) {
    wchar_t dll_path[MAX_PATH];
    if (!GetModuleFileNameW(self, dll_path, MAX_PATH)) return E_FAIL;

    HRESULT hr = WriteClsidRegistry(CLSID_RkmppH264Decoder,
                                    DecoderFriendlyName(CodecKind::H264),
                                    dll_path);
    if (FAILED(hr)) return hr;
    hr = WriteClsidRegistry(CLSID_RkmppHevcDecoder,
                            DecoderFriendlyName(CodecKind::HEVC),
                            dll_path);
    if (FAILED(hr)) return hr;
    hr = WriteClsidRegistry(CLSID_RkmppAv1Decoder,
                            DecoderFriendlyName(CodecKind::AV1),
                            dll_path);
    if (FAILED(hr)) return hr;

    hr = RegisterOne(CodecKind::H264);
    if (FAILED(hr)) return hr;
    hr = RegisterOne(CodecKind::HEVC);
    if (FAILED(hr)) return hr;
    hr = RegisterOne(CodecKind::AV1);
    return hr;
}

HRESULT UnregisterServer() {
    MFTUnregister(CLSID_RkmppH264Decoder);
    MFTUnregister(CLSID_RkmppHevcDecoder);
    MFTUnregister(CLSID_RkmppAv1Decoder);
    DeleteClsidRegistry(CLSID_RkmppH264Decoder);
    DeleteClsidRegistry(CLSID_RkmppHevcDecoder);
    DeleteClsidRegistry(CLSID_RkmppAv1Decoder);
    return S_OK;
}

} /* namespace rkmpp */
