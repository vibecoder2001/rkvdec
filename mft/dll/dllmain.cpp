/* mft/dll/dllmain.cpp — DLL entry, COM exports.
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include <windows.h>
#include <unknwn.h>
#include <initguid.h>  /* must precede guids.h so DEFINE_GUID instantiates */

#include "class_factory.h"
#include "decoder_mft.h"
#include "guids.h"
#include "registration.h"

/* Instantiate driver-side device-interface GUIDs that decode_engine
 * (linked from mft/engine/) references via SetupDi. */
#include "../../shared/rkmpp_ioctl.h"

static HMODULE g_self = nullptr;

BOOL APIENTRY DllMain(HMODULE m, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_self = m;
        DisableThreadLibraryCalls(m);
    }
    return TRUE;
}

STDAPI DllGetClassObject(REFCLSID rclsid, REFIID riid, void **ppv) {
    if (!ppv) return E_POINTER;
    *ppv = nullptr;

    rkmpp::CodecKind kind;
    if (rclsid == CLSID_RkmppH264Decoder)      kind = rkmpp::CodecKind::H264;
    else if (rclsid == CLSID_RkmppHevcDecoder) kind = rkmpp::CodecKind::HEVC;
    else if (rclsid == CLSID_RkmppAv1Decoder)  kind = rkmpp::CodecKind::AV1;
    else return CLASS_E_CLASSNOTAVAILABLE;

    rkmpp::ClassFactory *cf = new (std::nothrow) rkmpp::ClassFactory(kind);
    if (!cf) return E_OUTOFMEMORY;
    HRESULT hr = cf->QueryInterface(riid, ppv);
    cf->Release();
    return hr;
}

STDAPI DllCanUnloadNow(void) {
    return (rkmpp::g_dll_lock_count == 0) ? S_OK : S_FALSE;
}

STDAPI DllRegisterServer(void) {
    return rkmpp::RegisterServer(g_self);
}

STDAPI DllUnregisterServer(void) {
    return rkmpp::UnregisterServer();
}
