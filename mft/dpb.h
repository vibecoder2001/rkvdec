/* mft/dpb.h — minimal H.264 decoded-picture-buffer manager.
 *
 * Phase 3b coverage:
 *   - Pre-allocated pool of N frame buffers + per-slot colmv buffers.
 *   - On IDR: flush all slots and take slot 0 for the current pic.
 *   - On non-IDR (later): pick any free slot, mark previous picture as
 *     reference per nal_ref_idc, run sliding-window short-term marking.
 *   - Build the H264BufferRefs view + dpb[16] + RefPicList[3][32]
 *     handed to regbuilder + RPS-packer.
 *
 * Out of scope (later tasks):
 *   - Adaptive memory-control marking (long-term, MMCO).
 *   - Field coding (we treat every picture as a frame).
 *   - Picture reordering for output (the harness reads each picture
 *     in decode order, not display order, until DPB output is wired).
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#include "parser_glue.h"
#include "regbuilder_h264.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DPB_MAX_SLOTS 16

/* One pool slot — a pair of buffer handles the harness allocated up
 * front via IOCTL_RKMPP_ALLOC_BUFFER. Both must be non-zero. */
typedef struct DpbPoolEntry {
    uint64_t output_frame;      /* NV12 reconstructed picture */
    uint64_t colmv;             /* colmv scratch */
} DpbPoolEntry;

typedef enum {
    DPB_OK            = 0,
    DPB_INVALID_INPUT = 1,
    DPB_FULL          = 2,      /* no free slot — pool too small */
} DpbStatus;

/* Result of Dpb_Select.  All buffer-handle fields suit feeding straight
 * into H264BufferRefs (output_frame / refs[] / ref_colmv[]).  The
 * dpb_entries / ref_lists arrays go into decode_params.dpb / RPS pack. */
typedef struct DpbSelection {
    uint32_t                       current_slot;
    uint64_t                       current_output;
    uint64_t                       current_colmv;
    uint64_t                       refs[DPB_MAX_SLOTS];        /* output handle per slot */
    uint64_t                       ref_colmv[DPB_MAX_SLOTS];   /* colmv handle per slot */
    struct v4l2_h264_dpb_entry     dpb_entries[DPB_MAX_SLOTS];
    struct v4l2_h264_reference     ref_lists[3][32];
} DpbSelection;

/* Opaque DPB context — caller declares in its struct. */
typedef struct DpbCtx {
    DpbPoolEntry pool[DPB_MAX_SLOTS];
    uint32_t     pool_size;
    int32_t      current_idx;   /* -1 when nothing is in flight */

    /* Per-slot state. */
    struct {
        uint8_t  in_use   : 1;
        uint8_t  is_ref   : 1;
        uint8_t  long_term: 1;
        uint8_t  fields;        /* V4L2_H264_*_REF */
        uint16_t frame_num;
        int32_t  top_poc;
        int32_t  bottom_poc;
    } slots[DPB_MAX_SLOTS];
} DpbCtx;

/* Initialise the DPB with a caller-allocated pool of N buffer pairs.
 * Each pool[i] entry must have both fields non-zero.  pool_size must be
 * 1..DPB_MAX_SLOTS. */
DpbStatus Dpb_Init(DpbCtx *ctx, const DpbPoolEntry *pool, uint32_t pool_size);

/* Pick a slot for the parsed access unit and fill the output selection.
 * IDR (V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) flushes the DPB first. */
DpbStatus Dpb_Select(DpbCtx *ctx, const H264ParseResult *parsed,
                     DpbSelection *out);

/* Mark the current picture as decoded.  If it carries nal_ref_idc != 0
 * and was selected by the most recent Dpb_Select, it stays in the DPB
 * as a short-term reference; if the DPB is full, the oldest short-term
 * ref is evicted (sliding window, H.264 8.2.5.3). */
void Dpb_OnDecodeComplete(DpbCtx *ctx);

#ifdef __cplusplus
}
#endif
