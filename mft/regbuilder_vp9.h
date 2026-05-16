/* mft/regbuilder_vp9.h — parsed VP9 → rkvdec2 dense bank.
 *
 * Sibling of regbuilder_h264.h / regbuilder_h265.h.  Consumes
 * vp9::PicParams + vp9::DpbCtx + per-frame buffer handles and emits
 * a fully populated H26xDenseOutput.  The Windows / Linux backends
 * adapt H26xDenseOutput into RKMPP_SUBMIT_DENSE_JOB_IN at submit
 * time (see backend_{windows,linux}.cpp).
 *
 * Fill order mirrors hal_vp9d_vdpu34x_gen_regs (rockchip-linux/mpp,
 * mpp/hal/rkdec/vp9d/hal_vp9d_vdpu34x.c:410).
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

#include "regbuilder_dense.h"
#include "vp9_types.h"
#include "vp9_dpb.h"

#ifdef __cplusplus
namespace vp9 {

/* Buffer handles + sizes the regbuilder needs.  All fields are RKMPP
 * buffer cookies returned by IOCTL_RKMPP_ALLOC_BUFFER; zero cookies
 * are skipped (no iova slot recorded).  Sticky fields (last_intra_only,
 * col_ref_poc, segid_ref_poc) are owned by the engine and threaded
 * through here so the regbuilder stays stateless. */
struct RegbuildInputs {
    const PicParams *pp;
    const DpbCtx    *dpb;

    uint64_t  bitstream_handle;
    uint32_t  bitstream_offset;     /* byte offset of compressed-header byte */
    uint32_t  bitstream_bytes;      /* full frame payload size */

    uint64_t  decout_frame_handle;
    uint64_t  decout_colmv_handle;

    uint64_t  probe_handle;         /* combined delta_prob + count_prob (reg160 + reg167) */
    uint64_t  prob_loop_handle;     /* per-frame_context_idx loop buffer (reg172 + reg162 on inter) */
    uint64_t  prob_default_handle;  /* session-wide vp9_default_probs blob (reg162 on keyframe) */

    uint64_t  segid_last_handle;
    uint64_t  segid_cur_handle;

    uint64_t  rcb_handles[10];
    uint32_t  rcb_offsets[10];      /* per-region byte offset within the shared RCB
                                     * buffer; must be set so reg133..142 don't all
                                     * point at offset 0 and corrupt each other. */

    uint64_t  error_ref_handle;

    uint8_t   last_intra_only;
    int32_t   col_ref_poc;
    int32_t   segid_ref_poc;

    /* Previous frame's tx_mode + reference_mode (reg76).  BSP fills this
     * from ls_info.tx_mode_pre / frame_reference_mode_pre — NOT the
     * current frame's values. */
    uint8_t   last_tx_mode;
    uint8_t   last_ref_mode;

    /* Per-frame_context_idx POC tracking — reg99.prob_ref_poc.  Set by
     * the engine when prob context i was last refreshed. */
    int32_t   prob_ref_poc[4];

    /* Per-context "context already initialized" flag.  Cold start: 0
     * (forces reg28 rd_prob_idx = 0, reg162 = prob_default).  Set to
     * 1 by the engine after a kick that updated context i. */
    uint8_t   prob_ctx_valid[4];

    /* Sticky state from the previous frame, used for reg75 fill. */
    uint8_t   last_show_frame;
    uint8_t   last_segmentation_enabled;
    uint8_t   last_widthheight_eqcur;
    uint8_t   last_color_space;
    int16_t   last_mode_deltas[2];           /* signed 7-bit each, packed into bits 0..13 */
    int8_t    last_lf_ref_deltas[4];         /* INTRA, LAST, GOLDEN, ALTREF — reg94 */
};

enum class RegBuildStatus {
    Ok = 0,
    BadInput = 1,
    UncoveredReg = 2,
    SlotOverflow = 3,
};

/* Fill the dense output for one VP9 kick.  Caller must zero `out`
 * before calling (the regbuilder writes into it, never reads). */
RegBuildStatus Vp9Regbuilder_Fill(const RegbuildInputs &in,
                                  H26xDenseOutput      *out);

/* Fill the per-frame_context_idx prob buffer in-place. Caller has the
 * user VA from AllocBuf. On keyframe/intra_only: write defaults from
 * vp9_default_probs.h. Otherwise: patch deltas from `pu` at known byte
 * offsets.
 *
 * BYTE LAYOUT WARNING: the exact byte offsets the hardware expects are
 * not in the spec — they come from BSP `hal_vp9d_prob_flag_delta` and
 * will be locked down by Task B5 shim diff. For B4 we ship a
 * best-effort layout in spec order; B5 will adjust offsets. */
void Vp9Regbuilder_FillProbs(const PicParams   &pp,
                              const ProbUpdates &pu,
                              uint8_t           *prob_buf);

} /* namespace vp9 */
#endif /* __cplusplus */
