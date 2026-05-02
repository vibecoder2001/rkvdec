/* mft/dll/class_factory.cpp
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "class_factory.h"

namespace rkmpp {

ClassFactory::ClassFactory(CodecKind kind) : kind_(kind), refs_(1) {
    DllAddRef();
}

STDMETHODIMP ClassFactory::QueryInterface(REFIID iid, void **ppv) {
    if (!ppv) return E_POINTER;
    if (iid == IID_IUnknown || iid == IID_IClassFactory) {
        *ppv = static_cast<IClassFactory*>(this);
        AddRef();
        return S_OK;
    }
    *ppv = nullptr;
    return E_NOINTERFACE;
}

STDMETHODIMP_(ULONG) ClassFactory::AddRef() {
    return InterlockedIncrement(&refs_);
}
STDMETHODIMP_(ULONG) ClassFactory::Release() {
    long r = InterlockedDecrement(&refs_);
    if (r == 0) {
        DllRelease();
        delete this;
    }
    return r;
}

STDMETHODIMP ClassFactory::CreateInstance(IUnknown *outer, REFIID iid, void **ppv) {
    if (!ppv) return E_POINTER;
    *ppv = nullptr;
    if (outer) return CLASS_E_NOAGGREGATION;
    DecoderMFT *m = new (std::nothrow) DecoderMFT(kind_);
    if (!m) return E_OUTOFMEMORY;
    HRESULT hr = m->QueryInterface(iid, ppv);
    m->Release();
    return hr;
}

STDMETHODIMP ClassFactory::LockServer(BOOL lock) {
    if (lock) DllAddRef(); else DllRelease();
    return S_OK;
}

} /* namespace rkmpp */
