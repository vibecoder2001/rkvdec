/* mft/regbuilder_h265.h — parsed HEVC bitstream → RKMPP_REG_WRITE[]
 *
 * Phase 3b Task 4 (HEVC).  Sibling of regbuilder_h264.h.  Reads the
 * parsed VPS/SPS/PPS/slice produced by parser_glue_h265 + a DPB
 * selection (reference iovas, current pic iovas) and emits a register-
 * write list the kernel SUBMIT_JOB ioctl can submit to the rkvdec2
 * (vdpu34x) hardware in HEVC mode.
 *
 * The fill order mirrors hal_h265d_vdpu34x_gen_regs (rockchip-linux/mpp,
 * mpp/hal/rkdec/h265d/hal_h265d_vdpu34x.c:836) with codec-agnostic
 * stanzas (vdpu34x_setup_statistic, common-addr bank) shared with the
 * H.264 path.
 *
 * Out of scope (deferred):
 *   - Multi-layer / scalability fields in reg104 — left at 0
 *   - Tile RCB column buffers when tile_col_cut_num > 0 — RCB_INTER_COL
 *     and RCB_FILT_COL pass through whatever the caller's RCB sizer
 *     computed (h265_packed_tables exposes a sibling helper)
 *   - Long-term reference handling beyond what the SPS-side RPS table
 *     already encodes
 *
 * The Rockchip-packed PPS / RPS / scaling-list / CABAC tables are built
 * by h265_packed_tables.cpp (Task 3); this module just stamps their
 * iovas into reg161 / reg163 / reg180 / reg197.
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
#include "parser_glue_h265.h"
#include "h264_packed_tables.h"      /* H264RcbInfo (struct reused for HEVC) */

#ifdef __cplusplus
extern "C" {
#endif

/* HEVC reuses the 10-region RCB layout the H.264 path already defines —
 * the regions, register indices, and sizing helpers are identical at
 * the vdpu34x level.  Alias the type for HEVC call sites so the BSP
 * cross-references read naturally. */
typedef H264RcbInfo H265RcbInfo;

/* Buffer handles + sizes the regbuilder needs to fill the address
 * registers.  All fields are RKMPP buffer cookies returned by
 * IOCTL_RKMPP_ALLOC_BUFFER; ones the harness hasn't allocated yet
 * should be left zero — the regbuilder will skip writing them. */
typedef struct H265BufferRefs {
    uint64_t bitstream;            /* required: input bitstream */
    uint32_t bitstream_offset;     /* byte offset into bitstream where slice data starts */
    uint32_t bitstream_size;       /* bytes of valid bitstream */

    uint64_t output_frame;         /* required: reconstructed picture (NV12) */
    uint64_t colmv_cur;            /* required: colmv for current pic */
    uint64_t error_ref;            /* required: fallback reference frame */

    /* RCB scratch — same 10 sub-regions as H.264 (codec-agnostic).
     * If rcb[i]==0 the corresponding register isn't written. */
    uint64_t rcb[10];
    uint32_t rcb_offset[10];

    /* Rockchip-packed tables.  All three live in one consolidated info
     * buffer the harness allocated; the iova-substitution path resolves
     * each buffer-handle cookie to the same backing memory but the
     * IovaOffset field differentiates the regions. */
    uint64_t pps_table;            /* reg161 — packed VPS/SPS/PPS unit, 80*64 bytes */
    uint64_t rps_table;            /* reg163 — SPS-side RPS table, 3200 bytes */
    uint64_t scanlist;             /* reg180 — packed scaling list, 1360 bytes (0 ok) */
    uint64_t cabac_init_table;     /* reg197 — required: HEVC CABAC init blob */

    /* Reference-picture frames (DPB).  One iova per slot; HEVC has at
     * most 16 active short-term refs in our scope.  Leave a slot at 0
     * to skip the write — the codec will read whatever is in the bank
     * which the BSP zeroes before fill (the SET_POC_HIGNBIT_INFO sentinel
     * for missing refs is set up in the high-bit POC bank, not here). */
    uint64_t refs[16];
    uint64_t ref_colmv[16];

    /* Per-ref POC values (low 32 bits) — used to populate reg67..82.
     * Sourced from H265DpbSelection in task 5; kept as a flat array here
     * so this header doesn't depend on the (yet-to-land) DPB struct. */
    int32_t  ref_poc[16];

    /* Per-ref high-bit nibbles for reg200..203 (4 bits per ref).  HEVC
     * has 32 slots (vs. 16 on H.264) — for now the regbuilder fills
     * all 32 slots with this array's first 16 entries; the second 16
     * (reg202/203 lower halves) stay at 0.  Task 5 may extend the DPB
     * to expose 32 slots if multilayer support is added. */
    uint8_t  ref_poc_high[16];

    /* Long-term-vs-short-term flags per ref slot.  Bit-level layout in
     * the BSP RPS table is built by h265_packed_tables; the regbuilder
     * doesn't consume this directly today but will when ref-pic-layer
     * remapping (reg103) gets per-ref granularity. */
    uint8_t  ref_is_long_term[16];
} H265BufferRefs;

/* Output: a packed register-write list ready to feed into
 * RKMPP_SUBMIT_JOB_IN.Writes[].  Identical shape to H264RegWriteList. */
typedef struct H265RegWriteList {
    RKMPP_REG_WRITE entries[RKMPP_MAX_REG_WRITES];
    uint32_t        count;
} H265RegWriteList;

typedef enum {
    H265_REGBUILD_OK              = 0,
    H265_REGBUILD_MISSING_INPUT   = 1,
    H265_REGBUILD_TOO_MANY_REGS   = 2,
    H265_REGBUILD_UNSUPPORTED     = 3,
} H265RegBuildStatus;

/* Build the register list.
 *
 * - `parsed`  — parsed access-unit; must have an active VPS/SPS/PPS and
 *               a slice header (parsed->has_slice).  The active set is
 *               looked up via parsed->active_*_id.
 * - `bufs`    — buffer handles for DMA addresses (see above).
 * - `current_pic_index` — slot index for the current picture in the
 *               caller's DPB pool, only used to seed reg028.sw_film_idx
 *               when fast-mode is enabled (we leave it at 0 today).
 * - `out`     — output list; caller zero-inits.
 *
 * Returns 0 on success.  Failure modes match the H.264 enum. */
H265RegBuildStatus H265BuildRegisterList(const H265ParseResult *parsed,
                                         const H265BufferRefs  *bufs,
                                         uint32_t               current_pic_index,
                                         H265RegWriteList      *out);

#ifdef __cplusplus
}
#endif
