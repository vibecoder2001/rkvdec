/* mft/regbuilder_h264.h — V4L2 H.264 controls → RKMPP_REG_WRITE[]
 *
 * Phase 3b Task 8.  Reads the parsed V4L2 stateless H.264 control
 * structs that parser_glue produces, plus a set of buffer handles
 * supplied by the harness, and emits a register-write list the kernel
 * SUBMIT_JOB ioctl can submit to the rkvdec2 hardware.
 *
 * For first-IDR decode we don't need the full builder — Rockchip-packed
 * PPS / RPS / CABAC / scaling tables (built by hal_h264d_com.c in the
 * BSP userspace stack) are deferred to a follow-up.  This file
 * implements:
 *   - hard-coded common-bank init (always-the-same control bits)
 *   - frame-geometry registers (width / height / strides)
 *   - codec params for an IDR slice (POC, first-slice flag, idr flag)
 *   - all DMA address registers (with BufferHandle set so the kernel
 *     does iova substitution per Phase 3b Task 4)
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* rkmpp_ioctl.h pulls in IOCTL plumbing — UINT32 etc. come from
 * <windows.h>; <devioctl.h> sits on top of it.  Consumers that
 * include this header from a user-mode build don't need to do
 * anything special; the ifdef shields kernel-mode users (none yet). */
#ifdef _WIN32
#  ifndef _INC_WINDOWS
#    include <windows.h>
#  endif
#endif

#include "../shared/rkmpp_ioctl.h"
#include "parser_glue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Buffer handles + sizes the regbuilder needs to fill the address
 * registers.  All fields are RKMPP buffer cookies returned by
 * IOCTL_RKMPP_ALLOC_BUFFER; ones the harness hasn't allocated yet
 * should be left zero — the regbuilder will skip writing them. */
typedef struct H264BufferRefs {
    uint64_t bitstream;            /* required: input bitstream */
    uint32_t bitstream_offset;     /* byte offset into bitstream where slice data starts */
    uint32_t bitstream_size;       /* bytes of valid bitstream */

    uint64_t output_frame;         /* required: reconstructed picture (NV12) */
    uint64_t colmv_cur;            /* required: colmv for current pic */
    uint64_t error_ref;            /* required: fallback reference frame */

    /* RCB scratch — 10 sub-regions inside one (or several) backing
     * buffers, sized via H264GetRcbBufferSizes(width, height). The
     * harness can either:
     *   - share one buffer across all ten by setting rcb[*] = same
     *     handle and rcb_offset[i] = per-region byte offset, or
     *   - pass a separate handle per region with offset 0.
     * If rcb[i]==0 the regbuilder skips that register, but the
     * hardware will fault if it tries to use that scratch — so for
     * real decode all 10 must be populated. */
    uint64_t rcb[10];
    uint32_t rcb_offset[10];

    /* Rockchip-packed tables (deferred until hal_h264d_com.c port). */
    uint64_t pps_table;            /* required (deferred) */
    uint64_t rps_table;            /* required (deferred) */
    uint64_t cabac_init_table;     /* required when entropy_coding_mode=1 */
    uint64_t scaling_list;         /* optional */

    /* Reference-picture frames (DPB).  refs[i] iova set in regs 164+i. */
    uint64_t refs[16];
    uint64_t ref_colmv[16];
} H264BufferRefs;

/* Output: a packed register-write list ready to feed into
 * RKMPP_SUBMIT_JOB_IN.Writes[]. */
typedef struct H264RegWriteList {
    RKMPP_REG_WRITE entries[RKMPP_MAX_REG_WRITES];
    uint32_t        count;
} H264RegWriteList;

/* Build the register list.
 *
 * - `parsed`     — parsed access-unit; must have has_sps/has_pps/has_slice.
 * - `bufs`       — buffer handles for DMA addresses.
 * - `current_pic_index` — which DPB slot this picture occupies (0..15).
 * - `out`        — output list; caller zero-inits.
 *
 * Returns:
 *   0 on success, non-zero on validation failure.  Failure reasons are
 *   distinct status codes in the caller-side enum below. */
typedef enum {
    H264_REGBUILD_OK              = 0,
    H264_REGBUILD_MISSING_INPUT   = 1,
    H264_REGBUILD_TOO_MANY_REGS   = 2,
    H264_REGBUILD_UNSUPPORTED     = 3,
} H264RegBuildStatus;

H264RegBuildStatus H264BuildRegisterList(const H264ParseResult *parsed,
                                         const H264BufferRefs  *bufs,
                                         uint32_t               current_pic_index,
                                         H264RegWriteList      *out);

#ifdef __cplusplus
}
#endif
