/* mft/engine/backend_linux.h — public surface of the Linux
 * DecodeEngineBackend implementation.  See backend_linux.cpp.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include "decode_engine_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate a Linux backend bound to a stream of `width` x `height`
 * pixels.  Returns NULL on OOM.  The returned pointer is suitable for
 * DecodeEngine_InitWithBackend and must be released with
 * LinuxBackend_Free after DecodeEngine_Shutdown. */
DecodeEngineBackend *LinuxBackend_New(uint32_t width, uint32_t height);

void LinuxBackend_Free(DecodeEngineBackend *be);

#ifdef __cplusplus
}  /* extern "C" */
#endif
