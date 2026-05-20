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

/* Hardware-identifier constants for the RKMPP_CAPS.Hid field.  These
 * match the ACPI _HID assignments for the RK3588 video subsystem.
 * Code that opens a specific codec should test the symbolic constant
 * instead of the bare hex literal so that a future SoC variant can
 * be added in one place rather than across mft/, shared/, and the
 * .INF files.  See [[rkcp35xx_hid_map]]. */
#define RKMPP_HID_RKCP3550 0x3550u   /* rkvdec2 (H.264 / H.265 / VP9) */
#define RKMPP_HID_RKCP3560 0x3560u   /* rkav1d  (AV1) */

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
/* PEEK_JOB returns the post-substitution register list for a queued or
 * completed job — used by tests to verify iova-handle resolution before
 * the real hardware-kick path is in (Phase 3b Task 11). */
#define IOCTL_RKMPP_PEEK_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x805, METHOD_BUFFERED, FILE_WRITE_ACCESS)

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
 * core's MMIO base).
 *
 *   BufferHandle == 0
 *     Plain write — Value is stored verbatim.
 *
 *   BufferHandle != 0
 *     iova-substitution write — at submit time the driver looks up the
 *     iova of the buffer with the given handle (must belong to the same
 *     file object), adds IovaOffset, and stores the resulting 32-bit
 *     value in the register list before the kick.  Lets the user-mode
 *     register builder name a buffer by handle without round-tripping
 *     the iova through user space.  Value (if any) is overwritten.
 *
 *   IovaOffset is in bytes and must be < buffer.SizeRoundedUp; otherwise
 *   the whole submission is rejected with STATUS_INVALID_PARAMETER.
 */
typedef struct _RKMPP_REG_WRITE {
    UINT32 Offset;
    UINT32 Value;
    UINT64 BufferHandle;
    UINT32 IovaOffset;
    UINT32 Reserved;
} RKMPP_REG_WRITE;

/* AV1 emits up to 320 swregs (idx 0..319) per kick; rkvdec2 needs ~120.
 * Cap at 384 so AV1 always fits with headroom for future codec banks. */
#define RKMPP_MAX_REG_WRITES 384
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

typedef struct _RKMPP_PEEK_JOB_IN {
    UINT64 JobId;
} RKMPP_PEEK_JOB_IN;

typedef struct _RKMPP_PEEK_JOB_OUT {
    UINT32          RegWriteCount;
    UINT32          Reserved;
    RKMPP_REG_WRITE Writes[RKMPP_MAX_REG_WRITES];
} RKMPP_PEEK_JOB_OUT;

/* ---- Dense-bank job submission (rkvdec2: H.264 / H.265) -----------
 *
 * The sparse RKMPP_REG_WRITE list above lets us write a subset of swregs
 * per kick.  Combined with the kernel's PrevNonzeroMask bank-walk this
 * works, but has a first-kick footgun: any swreg slot that is zero this
 * kick AND was zero last kick is skipped — so on a fresh PnP boot, slots
 * the regbuilder happens to leave at zero are never written to MMIO at
 * all (they sit at hardware-reset value).
 *
 * Mirroring upstream Linux rkvdec-vdpu381-{h264,hevc}.c, the dense path
 * fixes this structurally: the regbuilder fills a zero-initialised bank
 * struct and the kernel `WRITE_REGISTER_BUFFER_ULONG`s each bank in
 * ascending order — every word in every covered bank, every kick.  Iova
 * substitution stays user-driven: the regbuilder records (reg_idx,
 * buffer_handle, iova_offset) triples in IovaSlots[] and the kernel
 * resolves them before the bulk write.
 *
 * Bank layout (matches the vdpu34x SWREG split):
 *   common[25]       idx   8..32   (idx 10 = DEC_E kick, written last)
 *   codec_params[49] idx  64..112
 *   common_addr[15]  idx 128..142
 *   codec_addr[40]   idx 160..199
 *   highpoc[5]       idx 200..204
 *   stat[22]         idx 256..277
 * Total: 156 u32 = 624 bytes per bank payload.
 *
 * AV1 keeps the sparse RKMPP_SUBMIT_JOB path; rkav1d's SWREG layout
 * doesn't match the vdpu34x banks (different MMIO windows). */

#define IOCTL_RKMPP_SUBMIT_DENSE_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x806, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_PEEK_DENSE_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x807, METHOD_BUFFERED, FILE_WRITE_ACCESS)

/* Bank index ranges + per-bank word counts.  Keep these in sync with the
 * kernel-side bulk-write loop.  Word counts are inclusive: e.g. common
 * covers idx 8..32 = 25 entries. */
#define RKMPP_DENSE_COMMON_FIRST   8u
#define RKMPP_DENSE_COMMON_LAST    32u
#define RKMPP_DENSE_COMMON_WORDS   25u
#define RKMPP_DENSE_CPARAM_FIRST   64u
#define RKMPP_DENSE_CPARAM_LAST    112u
#define RKMPP_DENSE_CPARAM_WORDS   49u
#define RKMPP_DENSE_CADDR_FIRST    128u
#define RKMPP_DENSE_CADDR_LAST     142u
#define RKMPP_DENSE_CADDR_WORDS    15u
#define RKMPP_DENSE_CODADDR_FIRST  160u
#define RKMPP_DENSE_CODADDR_LAST   199u
#define RKMPP_DENSE_CODADDR_WORDS  40u
#define RKMPP_DENSE_HIPOC_FIRST    200u
#define RKMPP_DENSE_HIPOC_LAST     204u
#define RKMPP_DENSE_HIPOC_WORDS    5u
#define RKMPP_DENSE_STAT_FIRST     256u
#define RKMPP_DENSE_STAT_LAST      277u
#define RKMPP_DENSE_STAT_WORDS     22u

/* Idx 10 in the common bank is DEC_E (kick).  Kernel zeros it during
 * the bulk write of common[] and writes the kick value separately as
 * the very last MMIO write.  Regbuilders should leave Common[2] == 0. */
#define RKMPP_DENSE_KICK_REG_IDX   10u

typedef struct _RKMPP_DENSE_BANK {
    UINT32 Common      [RKMPP_DENSE_COMMON_WORDS];   /* idx 8..32   */
    UINT32 CodecParams [RKMPP_DENSE_CPARAM_WORDS];   /* idx 64..112 */
    UINT32 CommonAddr  [RKMPP_DENSE_CADDR_WORDS];    /* idx 128..142*/
    UINT32 CodecAddr   [RKMPP_DENSE_CODADDR_WORDS];  /* idx 160..199*/
    UINT32 HighPoc     [RKMPP_DENSE_HIPOC_WORDS];    /* idx 200..204*/
    UINT32 Stat        [RKMPP_DENSE_STAT_WORDS];     /* idx 256..277*/
} RKMPP_DENSE_BANK;

/* One iova-substitution slot.  RegIdx must land inside an address-bank
 * range (common_addr 128..142 or selected entries in codec_addr).
 * Kernel resolves buffer_handle → iova (verifies same file-object owner
 * and IovaOffset < buffer.size), then writes (iova + IovaOffset)[31:0]
 * into the dense bank at RegIdx before issuing the bulk MMIO write. */
typedef struct _RKMPP_DENSE_IOVA_SLOT {
    UINT32 RegIdx;
    UINT32 IovaOffset;
    UINT64 BufferHandle;
} RKMPP_DENSE_IOVA_SLOT;

/* 15 common-addr + 16 ref + 16 colmv + PPS + RPS + scanlist + cabactbl
 * = 51 worst-case slots.  Round up to 64 for headroom. */
#define RKMPP_MAX_DENSE_IOVA_SLOTS 64

typedef struct _RKMPP_SUBMIT_DENSE_JOB_IN {
    UINT32                StructSize;
    UINT32                IovaSlotCount;
    UINT32                BufRefCount;
    UINT32                TimeoutMs;
    UINT32                KickValue;   /* value written to idx 10 last */
    UINT32                Reserved;
    RKMPP_DENSE_BANK      Bank;
    RKMPP_DENSE_IOVA_SLOT IovaSlots[RKMPP_MAX_DENSE_IOVA_SLOTS];
    RKMPP_BUFFER_REF      BufRefs[RKMPP_MAX_BUF_REFS];
} RKMPP_SUBMIT_DENSE_JOB_IN;

typedef struct _RKMPP_SUBMIT_DENSE_JOB_OUT {
    UINT64 JobId;
} RKMPP_SUBMIT_DENSE_JOB_OUT;

/* PEEK output: the post-substitution dense bank as the kernel will (or
 * did) write to MMIO.  Tests use this to verify iova resolution before
 * the kick reaches hardware. */
typedef struct _RKMPP_PEEK_DENSE_JOB_OUT {
    UINT32           StructSize;
    UINT32           KickValue;
    RKMPP_DENSE_BANK Bank;
} RKMPP_PEEK_DENSE_JOB_OUT;

/* Returns 1 if RegIdx is inside one of the address banks the kernel
 * iova-substitution path supports.  Shared between user-mode regbuilder
 * validation and kernel-side substitution. */
__inline int RkMppDenseIsAddressReg(UINT32 reg_idx)
{
    if (reg_idx >= RKMPP_DENSE_CADDR_FIRST && reg_idx <= RKMPP_DENSE_CADDR_LAST)
        return 1;
    /* HEVC codec-addr registers */
    if (reg_idx == 161u || reg_idx == 163u) return 1;        /* PPS, RPS */
    if (reg_idx >= 164u && reg_idx <= 179u)  return 1;       /* ref bases */
    if (reg_idx == 180u)                     return 1;       /* scanlist */
    if (reg_idx >= 181u && reg_idx <= 196u)  return 1;       /* colmv refs */
    if (reg_idx == 197u)                     return 1;       /* cabac init */
    /* VP9 codec-addr registers (Vdpu34xVp9dAddr, reg160..179) */
    if (reg_idx == 160u)                     return 1;       /* delta_prob_base */
    if (reg_idx == 162u)                     return 1;       /* last_prob_base */
    if (reg_idx == 167u)                     return 1;       /* count_prob_base */
    if (reg_idx == 168u)                     return 1;       /* segid_last_base */
    if (reg_idx == 169u)                     return 1;       /* segid_cur_base */
    if (reg_idx == 170u)                     return 1;       /* ref_colmv_base (last ref) */
    if (reg_idx >= 171u && reg_idx <= 178u)  return 1;       /* ref_frame_map[0..7] */
    if (reg_idx == 179u)                     return 1;       /* cur_colmv_base */
    return 0;
}

/* ---- Phase 3a: IOMMU fault injection scaffold ---- */

#define IOCTL_RKMPP_INJECT_IOMMU_FAULT \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x8FE, METHOD_BUFFERED, FILE_WRITE_ACCESS)

typedef struct _RKMPP_FAULT_RESULT {
    UINT32  Triggered;     /* 0 = no fault observed within timeout; 1 = observed */
    UINT32  StatusReg;     /* IOMMU INT_STATUS at fault time */
    UINT64  FaultIova;     /* iova captured by the IOMMU at fault time */
} RKMPP_FAULT_RESULT;
