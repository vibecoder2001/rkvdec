#pragma once

/* Gated debug logging.  ERROR-level prints stay unconditional via direct
 * DbgPrintEx(DPFLTR_ERROR_LEVEL, ...) calls; INFO/WARNING bring-up chatter
 * routes through these macros and compiles out in release (DBG=0) builds. */

#include <ntddk.h>

#if DBG
#define RKMPP_LOG_INFO(...) \
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,    __VA_ARGS__)
#define RKMPP_LOG_WARN(...) \
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL, __VA_ARGS__)
#else
/* __noop is an MSVC intrinsic that accepts any arg list, evaluates none,
 * and counts each arg as "referenced" — so parameters that exist only
 * to be logged don't trip C4100 (unreferenced formal parameter) under
 * /W4 /WX in release builds. */
#define RKMPP_LOG_INFO(...) __noop(__VA_ARGS__)
#define RKMPP_LOG_WARN(...) __noop(__VA_ARGS__)
#endif
