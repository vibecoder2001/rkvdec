/* mft/dll/registration.h — MFTRegister / MFTUnregister + HKCR\CLSID
 * registry helpers, called from DllRegisterServer / DllUnregisterServer.
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <windows.h>

namespace rkmpp {

HRESULT RegisterServer(HMODULE self);
HRESULT UnregisterServer();

} /* namespace rkmpp */
