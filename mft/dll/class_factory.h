/* mft/dll/class_factory.h — single IClassFactory bound to one CLSID.
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <unknwn.h>
#include "decoder_mft.h"

namespace rkmpp {

class ClassFactory : public IClassFactory {
public:
    explicit ClassFactory(CodecKind kind);

    STDMETHODIMP         QueryInterface(REFIID iid, void **ppv) override;
    STDMETHODIMP_(ULONG) AddRef()  override;
    STDMETHODIMP_(ULONG) Release() override;

    STDMETHODIMP CreateInstance(IUnknown *outer, REFIID iid, void **ppv) override;
    STDMETHODIMP LockServer(BOOL lock) override;

private:
    CodecKind kind_;
    long      refs_;
};

} /* namespace rkmpp */
