/* mft/regbuilder_dense.h — shared dense-bank regbuilder output for the
 * rkvdec2 codecs (H.264 + H.265).
 *
 * Replaces the older sparse RKMPP_REG_WRITE[] output for the rkvdec2
 * codec paths.  Mirrors the upstream Linux rkvdec-vdpu381 model:
 *   - One zero-initialised bank struct covering every SWREG idx the
 *     codec uses (8..32, 64..112, 128..142, 160..199, 200..204, 256..277).
 *   - Per-codec regbuilders set named fields by index; the kernel
 *     `WRITE_REGISTER_BUFFER_ULONG`s each bank in ascending order every
 *     kick.  This eliminates the "register slot was zero last kick and
 *     this kick → skipped → still at hardware-reset value" footgun in
 *     the older PrevNonzeroMask bank-walk.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef _WIN32
#  ifndef _INC_WINDOWS
#    include <windows.h>
#  endif
#endif

#include "../shared/rkmpp_ioctl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Output of a dense regbuilder pass.  `Bank` holds the full register
 * payload; `IovaSlots[0..IovaSlotCount-1]` records the address-bank
 * indices the kernel must resolve via buffer-handle lookup (slot's
 * RegIdx value in `Bank` is left at 0 by the regbuilder; kernel writes
 * iova+IovaOffset into that slot before the bulk MMIO write).
 * `KickValue` is the value to write to idx 10 last (regbuilders set
 * this via emit_plain on RKVDEC2_REG_START_EN; the bulk write of the
 * Common bank leaves Common[2] (== idx 10) at zero). */
typedef struct H26xDenseOutput {
    RKMPP_DENSE_BANK      Bank;
    RKMPP_DENSE_IOVA_SLOT IovaSlots[RKMPP_MAX_DENSE_IOVA_SLOTS];
    uint32_t              IovaSlotCount;
    uint32_t              KickValue;
} H26xDenseOutput;

/* Resolve a swreg byte offset (idx * 4) into a pointer to its slot
 * inside the dense bank, or NULL if it falls in an uncovered range.
 * Used by emit_plain/emit_iova in both regbuilders.  Idx 10 (kick) is
 * intentionally NOT mapped: callers must route the kick value into
 * H26xDenseOutput.KickValue instead. */
static inline uint32_t *
H26xDenseSlotFor(H26xDenseOutput *out, uint32_t reg_offset)
{
    uint32_t idx = reg_offset / 4u;
    if (idx == RKMPP_DENSE_KICK_REG_IDX) return NULL;
    if (idx >= RKMPP_DENSE_COMMON_FIRST && idx <= RKMPP_DENSE_COMMON_LAST)
        return &out->Bank.Common[idx - RKMPP_DENSE_COMMON_FIRST];
    if (idx >= RKMPP_DENSE_CPARAM_FIRST && idx <= RKMPP_DENSE_CPARAM_LAST)
        return &out->Bank.CodecParams[idx - RKMPP_DENSE_CPARAM_FIRST];
    if (idx >= RKMPP_DENSE_CADDR_FIRST && idx <= RKMPP_DENSE_CADDR_LAST)
        return &out->Bank.CommonAddr[idx - RKMPP_DENSE_CADDR_FIRST];
    if (idx >= RKMPP_DENSE_CODADDR_FIRST && idx <= RKMPP_DENSE_CODADDR_LAST)
        return &out->Bank.CodecAddr[idx - RKMPP_DENSE_CODADDR_FIRST];
    if (idx >= RKMPP_DENSE_HIPOC_FIRST && idx <= RKMPP_DENSE_HIPOC_LAST)
        return &out->Bank.HighPoc[idx - RKMPP_DENSE_HIPOC_FIRST];
    if (idx >= RKMPP_DENSE_STAT_FIRST && idx <= RKMPP_DENSE_STAT_LAST)
        return &out->Bank.Stat[idx - RKMPP_DENSE_STAT_FIRST];
    return NULL;
}

/* H26xDenseIsAddressReg: thin alias around the kernel-shared helper in
 * shared/rkmpp_ioctl.h.  Kept here so existing user-mode call sites
 * don't have to rename. */
static inline int
H26xDenseIsAddressReg(uint32_t reg_idx)
{
    return RkMppDenseIsAddressReg((UINT32)reg_idx);
}

#ifdef __cplusplus
}  /* extern "C" */
#endif
