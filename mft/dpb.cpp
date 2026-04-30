/* mft/dpb.cpp — Phase 3b minimal H.264 DPB. */
#include "dpb.h"
#include <string.h>

namespace {

/* Helpers for the per-slot bitfield struct (the C ABI uses :1 fields,
 * so we go through tiny accessors to keep the implementation tidy). */
static inline void slot_clear(DpbCtx *ctx, int i) {
    ctx->slots[i].in_use    = 0;
    ctx->slots[i].is_ref    = 0;
    ctx->slots[i].long_term = 0;
    ctx->slots[i].fields    = 0;
    ctx->slots[i].frame_num = 0;
    ctx->slots[i].top_poc    = 0;
    ctx->slots[i].bottom_poc = 0;
}

static int find_free(const DpbCtx *ctx) {
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (!ctx->slots[i].in_use) return (int)i;
    }
    return -1;
}

/* Sliding-window eviction: drop the short-term reference with the
 * oldest frame_num (H.264 8.2.5.3). */
static void evict_oldest_short_term(DpbCtx *ctx) {
    int oldest = -1;
    uint16_t oldest_fn = 0;
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (!ctx->slots[i].in_use) continue;
        if (!ctx->slots[i].is_ref) continue;
        if (ctx->slots[i].long_term) continue;
        if (oldest < 0 || ctx->slots[i].frame_num < oldest_fn) {
            oldest    = (int)i;
            oldest_fn = ctx->slots[i].frame_num;
        }
    }
    if (oldest >= 0) slot_clear(ctx, oldest);
}

} /* anon */

extern "C"
DpbStatus Dpb_Init(DpbCtx *ctx, const DpbPoolEntry *pool, uint32_t pool_size)
{
    if (!ctx || !pool || pool_size == 0 || pool_size > DPB_MAX_SLOTS)
        return DPB_INVALID_INPUT;
    for (uint32_t i = 0; i < pool_size; i++) {
        if (pool[i].output_frame == 0 || pool[i].colmv == 0)
            return DPB_INVALID_INPUT;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->pool_size   = pool_size;
    ctx->current_idx = -1;
    for (uint32_t i = 0; i < pool_size; i++) {
        ctx->pool[i] = pool[i];
    }
    return DPB_OK;
}

extern "C"
DpbStatus Dpb_Select(DpbCtx *ctx, const H264ParseResult *parsed,
                     DpbSelection *out)
{
    if (!ctx || !parsed || !out) return DPB_INVALID_INPUT;
    if (!parsed->has_slice)      return DPB_INVALID_INPUT;

    /* IDR flushes everything before picking the new slot. */
    if (parsed->decode.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) {
        for (uint32_t i = 0; i < ctx->pool_size; i++) slot_clear(ctx, (int)i);
    }

    int slot = find_free(ctx);
    if (slot < 0) {
        /* Try to make room by evicting an old short-term ref. */
        evict_oldest_short_term(ctx);
        slot = find_free(ctx);
        if (slot < 0) return DPB_FULL;
    }

    /* Populate the chosen slot for the picture currently being decoded. */
    ctx->slots[slot].in_use     = 1;
    ctx->slots[slot].is_ref     = (parsed->decode.nal_ref_idc != 0);
    ctx->slots[slot].long_term  = 0;
    ctx->slots[slot].fields     = V4L2_H264_FRAME_REF;
    ctx->slots[slot].frame_num  = parsed->decode.frame_num;
    ctx->slots[slot].top_poc    = parsed->decode.top_field_order_cnt;
    ctx->slots[slot].bottom_poc = parsed->decode.bottom_field_order_cnt;
    ctx->current_idx            = slot;

    /* Fill caller-visible selection. */
    memset(out, 0, sizeof(*out));
    out->current_slot   = (uint32_t)slot;
    out->current_output = ctx->pool[slot].output_frame;
    out->current_colmv  = ctx->pool[slot].colmv;

    /* Reference views: every other in-use ref slot. */
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if ((int)i == slot)              continue;
        if (!ctx->slots[i].in_use)        continue;
        if (!ctx->slots[i].is_ref)        continue;

        out->refs[i]      = ctx->pool[i].output_frame;
        out->ref_colmv[i] = ctx->pool[i].colmv;

        v4l2_h264_dpb_entry &e = out->dpb_entries[i];
        e.flags     = V4L2_H264_DPB_ENTRY_FLAG_VALID |
                      V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
        if (ctx->slots[i].long_term)
            e.flags |= V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
        e.fields    = ctx->slots[i].fields;
        e.frame_num = ctx->slots[i].frame_num;
        e.pic_num   = ctx->slots[i].frame_num;     /* short-term: same as frame_num */
        e.top_field_order_cnt    = ctx->slots[i].top_poc;
        e.bottom_field_order_cnt = ctx->slots[i].bottom_poc;
    }

    /* Ref-pic-list build (H.264 8.2.4) — IDR has no refs, leave zero.
     * For P/B, ref_lists[0] (and [1] for B) get built by sorting active
     * short-term refs by frame_num.  Defer until non-IDR streams are
     * exercised. */
    return DPB_OK;
}

extern "C"
void Dpb_OnDecodeComplete(DpbCtx *ctx)
{
    if (!ctx || ctx->current_idx < 0) return;

    int cur = ctx->current_idx;
    ctx->current_idx = -1;

    if (!ctx->slots[cur].is_ref) {
        /* Non-reference picture — release the slot back to the pool. */
        slot_clear(ctx, cur);
        return;
    }

    /* Reference picture stays in the DPB.  If holding it pushes the
     * count of short-term refs over max_num_ref_frames, the oldest
     * gets evicted on the *next* Select via evict_oldest_short_term —
     * we don't know max_num_ref_frames here without the SPS, so we
     * rely on Select to make room when needed. */
}
