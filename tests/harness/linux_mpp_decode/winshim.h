/* winshim.h — Windows integer typedef aliases for Linux builds of mft/ */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint8_t   UINT8;
typedef uint16_t  UINT16;
typedef uint32_t  UINT32;
typedef uint64_t  UINT64;
typedef int8_t    INT8;
typedef int16_t   INT16;
typedef int32_t   INT32;
typedef int64_t   INT64;
typedef uint8_t   BOOLEAN;
typedef void     *PVOID;
typedef uint32_t  NTSTATUS;
typedef size_t    SIZE_T;

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif
#ifndef STATUS_SUCCESS
#define STATUS_SUCCESS ((NTSTATUS)0)
#endif

/* DEFINE_GUID: rkmpp_ioctl.h declares the device interface GUID.
 * On Linux it generates no code. */
#define DEFINE_GUID(name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
    /* GUID stub: unused on Linux */
