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
#include "parser_glue_h265.h"
#include "regbuilder_h264.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DPB_MAX_SLOTS 16

/* Sentinel POC for unused entries in H265DpbSelection::ref_pocs[].  Mirrors
 * the H.264 sentinel pattern; the regbuilder packs reg67..82 with these
 * values so the codec's own 'POC == ref_poc?' search produces a predictable
 * miss on empty slots. */
#define H265_DPB_REF_POC_SENTINEL  ((int32_t)0x33333333)

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
    /* Sliding-window bound captured at Select time so OnDecodeComplete
     * can apply H.264 8.2.5.3 eviction without re-parsing the SPS. */
    uint32_t     max_num_ref_frames;
    /* Captured for FrameNumWrap math during eviction (spec 8.2.4.1).
     * max_frame_num = 1 << (log2_max_frame_num_minus4 + 4); curr_pic_num
     * is the frame_num of the most recently decoded reference picture. */
    uint32_t     max_frame_num;
    uint16_t     curr_pic_num;

    /* MaxLongTermFrameIdx (spec 8.2.5.4 op 4); -1 means "no LT allowed".
     * Initial value is "no LT" until an MMCO 4 sets it (spec 8.2.5.2). */
    int32_t      max_long_term_frame_idx;

    /* Deferred MMCO state captured at Dpb_Select time and applied in
     * Dpb_OnDecodeComplete after the codec finishes decoding the pic
     * (spec 8.2.5.4 — MMCO runs after decode). */
    uint8_t      pending_adaptive;
    uint8_t      pending_n_mmco;
    H264Mmco     pending_mmco[H264_MAX_MMCO_OPS];
    /* Captured at Select time so OnDecodeComplete can resolve PicNumX. */
    uint16_t     pending_curr_pic_num;

    /* POC of the most recently evicted short-term reference, set by
     * Dpb_OnDecodeComplete when sliding-window or MMCO fires.  Reset to
     * INT32_MIN at the start of each Dpb_OnDecodeComplete call.
     * Exposed for parser_dump --trace to print bump order. */
    int32_t      last_bumped_poc;

    /* Per-slot state. */
    struct {
        uint8_t  in_use   : 1;
        uint8_t  is_ref   : 1;
        uint8_t  long_term: 1;
        /* External hold count — incremented by `Dpb_AddExternalHold`
         * when a downstream consumer (MFT reorder_q / ready_q entry)
         * pins the slot's contents.  Slot pickers treat external_hold>0
         * the same as in_use, so the codec won't write a new picture
         * over data the consumer is still reading.  Decremented via
         * `Dpb_ReleaseExternalHold` when the consumer is done. */
        uint8_t  external_hold : 4;
        uint8_t  fields;        /* V4L2_H264_*_REF */
        uint16_t frame_num;
        int32_t  top_poc;
        int32_t  bottom_poc;
        uint16_t long_term_frame_idx;
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

/* External-hold ref counting.  Lets a downstream consumer (the MFT
 * reorder/ready queue) pin a pool slot's contents while the codec
 * picks new slots for subsequent decodes.  Used to make a future
 * zero-copy readout path safe — without this, a slot can be reassigned
 * to a new decode while a queued frame still references its data. */
void Dpb_AddExternalHold     (DpbCtx *ctx, uint32_t slot_idx);
void Dpb_ReleaseExternalHold (DpbCtx *ctx, uint32_t slot_idx);

/* =====================================================================
 * H.265 (HEVC) DPB — RPS-driven reference marking.
 *
 * rkvdec2 runs HEVC in HW_RPS mode (reg012.wait_reset_en=1, reg64.bit1=0)
 * which means the codec re-derives the per-picture RPS from the slice
 * header bitstream itself.  The DPB's job here is therefore:
 *   1. Allocate slots for each new picture (current pic + active refs).
 *   2. Track which slots hold which POC so the regbuilder can fill
 *      reg65/66 (cur POC), reg67..82 (16 ref POCs), and reg99
 *      (hevc_ref_valid mask).
 *   3. Manage slot lifetime — eviction is RPS-driven (HEVC 8.3.2),
 *      NOT sliding-window like H.264 8.2.5.3.
 *
 * Out of scope (matches Task 2 parser stub):
 *   - Long-term ref handling (long_term_ref_pics_present_flag = 0 in our
 *     test streams).
 *   - Inter-RPS prediction beyond what the parser already resolves into
 *     H265ShortTermRPS::delta_poc[].
 *   - Field coding (HEVC frame coding only — top_poc == bottom_poc).
 * ===================================================================== */

/* Result of H265Dpb_Select.  Field shapes match the regbuilder's
 * H265BufferRefs view (refs[16], ref_colmv[16], ref_poc[16]) so the
 * caller can splat directly into that struct, plus the auxiliary
 * ref_valid_mask for reg99 and current-pic POC pair. */
typedef struct H265DpbSelection {
    uint32_t current_slot;
    uint64_t current_output_iova;
    uint64_t current_colmv_iova;

    /* refs[i] — output iova of the slot holding active ref `i`, or 0 if
     * slot `i` doesn't hold an active ref.  The regbuilder substitutes
     * its error_ref iova for any 0 entry. */
    uint64_t refs       [DPB_MAX_SLOTS];
    uint64_t ref_colmv  [DPB_MAX_SLOTS];

    /* ref_pocs[i] — POC of active ref in slot `i`, or
     * H265_DPB_REF_POC_SENTINEL when the slot is unused.  Goes straight
     * into reg67..82. */
    int32_t  ref_pocs   [DPB_MAX_SLOTS];

    /* 16-bit bitmap matching Vdpu34xRegH265d_t::reg99 layout — bit `i`
     * set iff slot `i` is an active reference for the current pic. */
    uint16_t ref_valid_mask;

    /* Current pic POC.  HEVC frame coding: cur_top_poc == cur_bot_poc. */
    int32_t  cur_top_poc;
    int32_t  cur_bot_poc;
} H265DpbSelection;

/* HEVC DPB context.  Kept separate from DpbCtx because the per-slot
 * tracking is leaner (no field-coding bookkeeping, no V4L2_H264_*_REF
 * fields) and the eviction policy is fundamentally different.  Callers
 * that handle both codecs should hold one of each. */
typedef struct H265DpbCtx {
    DpbPoolEntry pool[DPB_MAX_SLOTS];
    uint32_t     pool_size;
    int32_t      current_idx;   /* -1 when nothing is in flight */

    struct {
        uint8_t  in_use : 1;
        uint8_t  is_ref : 1;     /* "used for reference" — RPS-driven */
        /* See DpbCtx::slots::external_hold — same purpose: protects this
         * slot from re-use while a downstream consumer still references
         * its decoded contents. */
        uint8_t  external_hold : 4;
        int32_t  poc;            /* signed: HEVC POC may be negative */
    } slots[DPB_MAX_SLOTS];
} H265DpbCtx;

/* Initialise — same shape as the H.264 path.  No allocation; the caller
 * supplies a pre-allocated buffer pool. */
DpbStatus H265Dpb_Init(H265DpbCtx *ctx, const DpbPoolEntry *pool,
                       uint32_t pool_size);

/* Pick a slot for the parsed access unit and run RPS-driven reference
 * marking (HEVC 8.3.2):
 *   - On IDR (NAL type 19/20): clear all slots.
 *   - Otherwise: parse the active short-term RPS from the slice header,
 *     mark slots whose POC appears in PocStCurr*∪PocStFoll as is_ref=1,
 *     clear is_ref on all other slots (they become reusable next call),
 *     then take a free slot for the current picture. */
DpbStatus H265Dpb_Select(H265DpbCtx *ctx, const H265ParseResult *parsed,
                         H265DpbSelection *out);

/* External hold ref-counting — see Dpb_AddExternalHold for rationale. */
void H265Dpb_AddExternalHold     (H265DpbCtx *ctx, uint32_t slot_idx);
void H265Dpb_ReleaseExternalHold (H265DpbCtx *ctx, uint32_t slot_idx);

/* Finalise current pic.  For HEVC the RPS marking already evicted stale
 * slots; this just clears current_idx (and releases the slot if the
 * picture wasn't a reference, mirroring the H.264 non-ref path). */
void H265Dpb_OnDecodeComplete(H265DpbCtx *ctx);

#ifdef __cplusplus
}
#endif
