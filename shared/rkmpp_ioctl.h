/* shared/rkmpp_ioctl.h — IOCTL surface for rkmpp.sys, included by both
 * the driver and user-mode clients. Phase 1: GET_CAPS only.
 */
#pragma once

#include <devioctl.h>

#define FILE_DEVICE_RKMPP 0x8086u  /* arbitrary, vendor-defined range */

#define IOCTL_RKMPP_GET_CAPS \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x800, METHOD_BUFFERED, FILE_READ_ACCESS)

/* Output of IOCTL_RKMPP_GET_CAPS. */
typedef struct _RKMPP_CAPS {
    UINT32 StructSize;          /* sizeof(RKMPP_CAPS) — versioning */
    UINT32 Hid;                 /* e.g. 0x3550 for RKCP3550 */
    UINT32 Uid;                 /* ACPI _UID, 0 for RVD0 */
    UINT32 RevisionWord;        /* raw value of the core's REVISION reg */
    UINT32 SupportedCodecs;     /* bitmap, see RKMPP_CODEC_* below */
    UINT32 Reserved[8];
} RKMPP_CAPS, *PRKMPP_CAPS;

/* SupportedCodecs bits */
#define RKMPP_CODEC_H264   (1u << 0)
#define RKMPP_CODEC_HEVC   (1u << 1)
#define RKMPP_CODEC_VP9    (1u << 2)
#define RKMPP_CODEC_AV1    (1u << 3)
#define RKMPP_CODEC_JPEG_D (1u << 4)
#define RKMPP_CODEC_AVS    (1u << 5)

/* Per-instance device-interface GUID. User mode enumerates instances of
 * this and opens each. Phase 1 v1 should see one entry: RVD0.
 */
DEFINE_GUID(GUID_DEVINTERFACE_RKMPP,
    0x9a1f4d11, 0x7c5e, 0x4ad7, 0xa4, 0x10, 0x1d, 0x21, 0x9e, 0x07, 0x6b, 0x10);

/* ---- Phase 2: buffer + job IOCTLs ---- */

#define IOCTL_RKMPP_ALLOC_BUFFER \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x801, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_FREE_BUFFER \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x802, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_SUBMIT_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x803, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_WAIT_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x804, METHOD_BUFFERED, FILE_WRITE_ACCESS)

typedef enum _RKMPP_BUFFER_USAGE {
    RkMppBufferUsageBitstreamInput = 1,
    RkMppBufferUsageReferenceFrame = 2,
    RkMppBufferUsageOutputFrame    = 3,
    RkMppBufferUsageScratch        = 4,  /* Phase 2 smoke test only */
} RKMPP_BUFFER_USAGE;

typedef struct _RKMPP_ALLOC_BUFFER_IN {
    UINT32 StructSize;
    UINT32 Size;            /* bytes */
    UINT32 Usage;           /* RKMPP_BUFFER_USAGE */
    UINT32 Reserved;
} RKMPP_ALLOC_BUFFER_IN;

typedef struct _RKMPP_ALLOC_BUFFER_OUT {
    UINT32 StructSize;
    UINT64 BufferHandle;    /* opaque to user; round-trips back via FREE/SUBMIT */
    UINT64 Iova;            /* device-visible iova; useful for telemetry */
    PVOID  UserVa;          /* user-mode VA mapped by the driver via
                             * MmMapLockedPagesSpecifyCache(UserMode); simpler
                             * than a section handle because the kernel maps
                             * directly into the calling process — no
                             * NtMapViewOfSection / DuplicateHandle needed. */
    UINT32 SizeRoundedUp;
    UINT32 Reserved;
} RKMPP_ALLOC_BUFFER_OUT;

typedef struct _RKMPP_FREE_BUFFER_IN {
    UINT64 BufferHandle;
} RKMPP_FREE_BUFFER_IN;

/* A register-list entry: write Value to register at Offset (relative to the
 * core's MMIO base). Phase 2 only supports plain writes. Phase 3 will add
 * buffer-handle substitution (so the user-mode register builder can name a
 * buffer by handle and the driver patches its iova in).
 */
typedef struct _RKMPP_REG_WRITE {
    UINT32 Offset;
    UINT32 Value;
} RKMPP_REG_WRITE;

#define RKMPP_MAX_REG_WRITES 256
#define RKMPP_MAX_BUF_REFS    16

typedef struct _RKMPP_BUFFER_REF {
    UINT64 BufferHandle;
    UINT32 Role;            /* opaque tag, e.g. role index in register list */
    UINT32 Reserved;
} RKMPP_BUFFER_REF;

typedef struct _RKMPP_SUBMIT_JOB_IN {
    UINT32 StructSize;
    UINT32 RegWriteCount;
    UINT32 BufRefCount;
    UINT32 TimeoutMs;
    RKMPP_REG_WRITE Writes[RKMPP_MAX_REG_WRITES];
    RKMPP_BUFFER_REF BufRefs[RKMPP_MAX_BUF_REFS];
} RKMPP_SUBMIT_JOB_IN;

typedef struct _RKMPP_SUBMIT_JOB_OUT {
    UINT64 JobId;
} RKMPP_SUBMIT_JOB_OUT;

typedef struct _RKMPP_WAIT_JOB_IN {
    UINT64 JobId;
    UINT32 TimeoutMs;
    UINT32 Reserved;
} RKMPP_WAIT_JOB_IN;

typedef struct _RKMPP_WAIT_JOB_OUT {
    NTSTATUS Status;        /* STATUS_SUCCESS, STATUS_DEVICE_HUNG, ... */
    UINT32   HardwareStatus;
    UINT64   ElapsedQpc;
} RKMPP_WAIT_JOB_OUT;
