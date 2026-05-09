/* mft/dpb.cpp — Phase 3b minimal H.264 DPB. */
#include "dpb.h"
#include <string.h>

namespace {

/* Helpers for the per-slot bitfield struct (the C ABI uses :1 fields,
 * so we go through tiny accessors to keep the implementation tidy).
 * Note: external_hold is intentionally preserved here — it's a refcount
 * owned by a downstream consumer, and the DPB clearing this slot doesn't
 * release the consumer's reference to its data. */
static inline void slot_clear(DpbCtx *ctx, int i) {
    ctx->slots[i].in_use    = 0;
    ctx->slots[i].is_ref    = 0;
    ctx->slots[i].long_term = 0;
    ctx->slots[i].fields    = 0;
    ctx->slots[i].frame_num = 0;
    ctx->slots[i].top_poc    = 0;
    ctx->slots[i].bottom_poc = 0;
    ctx->slots[i].long_term_frame_idx = 0;
}

/* Find a short-term ref slot with PicNum == picNumX (frame coding:
 * PicNum == FrameNumWrap == frame_num for refs since wrap-aware). */
static int find_short_term_by_pic_num(const DpbCtx *ctx, uint32_t pic_num_x) {
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (!ctx->slots[i].in_use) continue;
        if (!ctx->slots[i].is_ref) continue;
        if (ctx->slots[i].long_term) continue;
        if ((uint32_t)ctx->slots[i].frame_num == pic_num_x) return (int)i;
    }
    return -1;
}

/* Find a long-term ref slot with LongTermPicNum == lt_pic_num (frame
 * coding: LongTermPicNum == LongTermFrameIdx). */
static int find_long_term_by_lt_pic_num(const DpbCtx *ctx, uint32_t lt_pic_num) {
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (!ctx->slots[i].in_use) continue;
        if (!ctx->slots[i].is_ref) continue;
        if (!ctx->slots[i].long_term) continue;
        if ((uint32_t)ctx->slots[i].long_term_frame_idx == lt_pic_num) return (int)i;
    }
    return -1;
}

static int find_long_term_by_frame_idx(const DpbCtx *ctx, uint32_t lt_idx) {
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (!ctx->slots[i].in_use) continue;
        if (!ctx->slots[i].is_ref) continue;
        if (!ctx->slots[i].long_term) continue;
        if ((uint32_t)ctx->slots[i].long_term_frame_idx == lt_idx) return (int)i;
    }
    return -1;
}

/* Apply ref_pic_list_modification (spec 8.2.4.3) to one of our
 * already-derived per-slice lists.  Inputs:
 *   list[]          : v4l2_h264_reference array, sized 32 (existing slot)
 *   rplm_ops[]      : parsed (op, value) pairs from slice header
 *   n_rplm          : count of ops
 *   compact_fn[]    : map compact slot idx -> short-term ref frame_num
 *   compact_lt[]    : map compact slot idx -> 1 if long-term, 0 if short
 *   compact_lt_idx[]: long_term_frame_idx (only valid when compact_lt[i]=1)
 *   n_compact       : number of compact entries
 *   curr_pic_num    : current picture's PicNum (frame coding: == frame_num)
 *   max_pic_num     : MaxFrameNum (PicNum modulus)
 *   active_count    : num_ref_idx_lX_active_minus1+1; truncate list to this
 *
 * The algorithm: for each op {0,1,2}, locate the target ref's compact
 * idx, then do shift-insert at refIdxLX (= number of ops processed so
 * far), removing any later duplicate so the list still contains each
 * ref at most once. */
static void apply_rplm(struct v4l2_h264_reference *list,
                       const H264ParseResult       *parsed,
                       int                          which,    /* 0 = L0, 1 = L1 */
                       const uint16_t              *compact_fn,
                       const uint8_t               *compact_lt,
                       const uint16_t              *compact_lt_idx,
                       uint32_t                     n_compact,
                       uint16_t                     curr_pic_num,
                       uint32_t                     max_pic_num,
                       uint32_t                     active_count)
{
    uint8_t  fl    = (which == 0) ? parsed->ref_pic_list_modification_flag_l0
                                  : parsed->ref_pic_list_modification_flag_l1;
    if (!fl) return;
    uint8_t  n_ops = (which == 0) ? parsed->n_rplm_l0 : parsed->n_rplm_l1;
    const H264RplmOp *ops = (which == 0) ? parsed->rplm_l0 : parsed->rplm_l1;
    if (n_ops == 0) return;

    /* picNumLXPred starts at CurrPicNum (spec 8.2.4.3.1). */
    int32_t  pic_num_pred = (int32_t)curr_pic_num;
    uint32_t ref_idx_lx   = 0;

    for (uint32_t k = 0; k < n_ops; k++) {
        uint8_t  op  = ops[k].op;
        uint32_t val = ops[k].value;
        int32_t  target_compact_idx = -1;

        if (op == 0 || op == 1) {
            int32_t pic_num_no_wrap;
            if (op == 0) {
                pic_num_no_wrap = pic_num_pred - (int32_t)(val + 1);
                if (pic_num_no_wrap < 0) pic_num_no_wrap += (int32_t)max_pic_num;
            } else {
                pic_num_no_wrap = pic_num_pred + (int32_t)(val + 1);
                if (pic_num_no_wrap >= (int32_t)max_pic_num)
                    pic_num_no_wrap -= (int32_t)max_pic_num;
            }
            pic_num_pred = pic_num_no_wrap;
            int32_t pic_num_lx = pic_num_no_wrap;
            if (pic_num_lx > (int32_t)curr_pic_num)
                pic_num_lx -= (int32_t)max_pic_num;
            /* Find compact idx of short-term ref with PicNum == pic_num_lx.
             * For frame coding PicNum = FrameNumWrap; matching by raw
             * frame_num works whether the value wrapped or not since the
             * spec arithmetic above already produced a candidate that
             * collapses to the same wrap class. */
            int32_t target_fn = pic_num_lx;
            if (target_fn < 0) target_fn += (int32_t)max_pic_num;
            for (uint32_t i = 0; i < n_compact; i++) {
                if (compact_lt[i]) continue;
                if ((int32_t)compact_fn[i] == target_fn) {
                    target_compact_idx = (int32_t)i;
                    break;
                }
            }
        } else if (op == 2) {
            /* Long-term modification: val == long_term_pic_num. */
            for (uint32_t i = 0; i < n_compact; i++) {
                if (!compact_lt[i]) continue;
                if ((uint32_t)compact_lt_idx[i] == val) {
                    target_compact_idx = (int32_t)i;
                    break;
                }
            }
        } else {
            /* op 3 = end (already filtered by parser) — break defensively. */
            break;
        }

        if (target_compact_idx < 0) {
            /* Modification points at a ref we don't have.  Best we can do
             * is leave the list alone and continue — applying nothing
             * matches the codec's would-be reference fallback. */
            continue;
        }

        /* Shift list[ref_idx_lx .. active_count-1] right by 1, then place
         * the target at list[ref_idx_lx].  Then walk the rest of the
         * (now-32-deep) list and remove the first later duplicate so each
         * ref appears at most once. */
        if (ref_idx_lx < 32) {
            for (int32_t j = 31; j > (int32_t)ref_idx_lx; j--) {
                list[j] = list[j - 1];
            }
            list[ref_idx_lx].index  = (uint8_t)target_compact_idx;
            list[ref_idx_lx].fields = V4L2_H264_FRAME_REF;
            for (uint32_t j = ref_idx_lx + 1; j < 32; j++) {
                if ((list[j].fields & V4L2_H264_FRAME_REF) &&
                    list[j].index == (uint8_t)target_compact_idx) {
                    /* collapse */
                    for (uint32_t m = j; m + 1 < 32; m++) list[m] = list[m + 1];
                    list[31].fields = 0;
                    list[31].index  = 0;
                    break;
                }
            }
        }
        ref_idx_lx++;
        if (ref_idx_lx >= active_count) break;
    }

    /* Truncate beyond active_count — rest stays valid 0-init or prior. */
    for (uint32_t i = active_count; i < 32; i++) {
        list[i].fields = 0;
        list[i].index  = 0;
    }
}

/* Apply the captured MMCO list (spec 8.2.5.4) to ctx after the current
 * picture has been decoded.  `cur` is the slot holding the current pic. */
static void apply_mmco_ops(DpbCtx *ctx, int cur) {
    uint16_t curr_pic_num = ctx->pending_curr_pic_num;
    for (uint32_t k = 0; k < ctx->pending_n_mmco; k++) {
        const H264Mmco &m = ctx->pending_mmco[k];
        switch (m.op) {
        case 1: { /* mark short-term as unused for reference */
            uint32_t pic_num_x = (uint32_t)curr_pic_num
                               - (m.difference_of_pic_nums_minus1 + 1);
            int idx = find_short_term_by_pic_num(ctx, pic_num_x);
            if (idx >= 0 && idx != cur) slot_clear(ctx, idx);
            break;
        }
        case 2: { /* mark long-term as unused for reference */
            int idx = find_long_term_by_lt_pic_num(ctx, m.long_term_pic_num);
            if (idx >= 0 && idx != cur) slot_clear(ctx, idx);
            break;
        }
        case 3: { /* assign LT idx to a short-term ref */
            uint32_t pic_num_x = (uint32_t)curr_pic_num
                               - (m.difference_of_pic_nums_minus1 + 1);
            /* If any LT ref already holds this idx, drop it. */
            int prev_lt = find_long_term_by_frame_idx(ctx, m.long_term_frame_idx);
            if (prev_lt >= 0 && prev_lt != cur) slot_clear(ctx, prev_lt);
            int idx = find_short_term_by_pic_num(ctx, pic_num_x);
            if (idx >= 0) {
                ctx->slots[idx].long_term           = 1;
                ctx->slots[idx].long_term_frame_idx = (uint16_t)m.long_term_frame_idx;
            }
            break;
        }
        case 4: { /* set MaxLongTermFrameIdx */
            int32_t new_max = (int32_t)m.max_long_term_frame_idx_plus1 - 1;
            ctx->max_long_term_frame_idx = new_max;
            /* Drop LT refs whose idx exceeds the new max. */
            for (uint32_t i = 0; i < ctx->pool_size; i++) {
                if (!ctx->slots[i].in_use) continue;
                if (!ctx->slots[i].is_ref) continue;
                if (!ctx->slots[i].long_term) continue;
                if (new_max < 0 ||
                    (int32_t)ctx->slots[i].long_term_frame_idx > new_max) {
                    if ((int)i != cur) slot_clear(ctx, (int)i);
                }
            }
            break;
        }
        case 5: { /* mark all as unused for reference */
            for (uint32_t i = 0; i < ctx->pool_size; i++) {
                if ((int)i != cur) slot_clear(ctx, (int)i);
            }
            ctx->max_long_term_frame_idx = -1;
            /* Spec 8.2.5.4.5: current pic is treated as having frame_num 0
             * after MMCO 5; reflect that in slot bookkeeping so subsequent
             * sliding-window math (none here, but parity) stays sane. */
            if (cur >= 0) {
                ctx->slots[cur].frame_num = 0;
            }
            break;
        }
        case 6: { /* mark current as long-term */
            int prev_lt = find_long_term_by_frame_idx(ctx, m.long_term_frame_idx);
            if (prev_lt >= 0 && prev_lt != cur) slot_clear(ctx, prev_lt);
            if (cur >= 0) {
                ctx->slots[cur].long_term           = 1;
                ctx->slots[cur].long_term_frame_idx = (uint16_t)m.long_term_frame_idx;
            }
            break;
        }
        default: break;
        }
    }
}

static int find_free(const DpbCtx *ctx) {
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (ctx->slots[i].in_use)         continue;
        if (ctx->slots[i].external_hold)  continue;
        return (int)i;
    }
    return -1;
}

extern "C" void Dpb_AddExternalHold(DpbCtx *ctx, uint32_t slot_idx) {
    if (!ctx || slot_idx >= ctx->pool_size) return;
    if (ctx->slots[slot_idx].external_hold < 0xF)
        ctx->slots[slot_idx].external_hold++;
}

extern "C" void Dpb_ReleaseExternalHold(DpbCtx *ctx, uint32_t slot_idx) {
    if (!ctx || slot_idx >= ctx->pool_size) return;
    if (ctx->slots[slot_idx].external_hold > 0)
        ctx->slots[slot_idx].external_hold--;
}

/* Sliding-window eviction: drop the short-term reference with the
 * smallest FrameNumWrap (H.264 8.2.5.3 + 8.2.4.1).
 *
 * FrameNumWrap = frame_num if frame_num <= CurrPicNum else
 *                frame_num - MaxFrameNum.
 *
 * Using raw frame_num here breaks across frame_num wrap: when CurrPicNum
 * wraps from MaxFrameNum-1 to 0, every still-live ref has frame_num
 * larger than CurrPicNum and FrameNumWrap negative, so the just-added
 * ref (with frame_num=0) ISN'T the smallest under the spec — but it IS
 * under raw-frame_num comparison, and would get incorrectly evicted. */
static void evict_oldest_short_term(DpbCtx *ctx) {
    int oldest = -1;
    int32_t oldest_fnw = 0;
    int32_t curr_fn    = (int32_t)ctx->curr_pic_num;
    int32_t max_fn     = (int32_t)(ctx->max_frame_num ? ctx->max_frame_num : 16);
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (!ctx->slots[i].in_use) continue;
        if (!ctx->slots[i].is_ref) continue;
        if (ctx->slots[i].long_term) continue;
        int32_t fn  = (int32_t)ctx->slots[i].frame_num;
        int32_t fnw = (fn > curr_fn) ? fn - max_fn : fn;
        if (oldest < 0 || fnw < oldest_fnw) {
            oldest      = (int)i;
            oldest_fnw  = fnw;
        }
    }
    if (oldest >= 0) {
        ctx->last_bumped_poc = ctx->slots[oldest].top_poc;
        slot_clear(ctx, oldest);
    }
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
    ctx->pool_size              = pool_size;
    ctx->current_idx            = -1;
    ctx->max_long_term_frame_idx = -1; /* "no LT" until MMCO 4 sets it */
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

    /* IDR flushes everything before picking the new slot.  Spec 8.2.5.1:
     * MaxLongTermFrameIdx becomes 0 if long_term_reference_flag set
     * (and the IDR pic itself is stored as LT idx 0); else "no LT". */
    const bool is_idr = (parsed->decode.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;
    if (is_idr) {
        for (uint32_t i = 0; i < ctx->pool_size; i++) slot_clear(ctx, (int)i);
        ctx->max_long_term_frame_idx = parsed->idr_long_term_reference_flag ? 0 : -1;
    }

    /* No pre-Select eviction.  Per H.264 8.2.5, sliding-window runs only
     * when ADDING a ref picture (handled in Dpb_OnDecodeComplete).  Non-
     * ref pictures don't change the active-ref count, so they never need
     * eviction.  An earlier "BSP-parity max-1" pre-eviction here was
     * wrong: it dropped a still-live ref before the B pic, leaving the
     * NEXT P pic with too few refs (parser_dump exposed this on
     * dancing.h264 between au=11 P fn=11 and au=13 P fn=12). */

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
    ctx->slots[slot].long_term  = (is_idr && parsed->idr_long_term_reference_flag) ? 1 : 0;
    ctx->slots[slot].fields     = V4L2_H264_FRAME_REF;
    ctx->slots[slot].frame_num  = parsed->decode.frame_num;
    ctx->slots[slot].top_poc    = parsed->decode.top_field_order_cnt;
    ctx->slots[slot].bottom_poc = parsed->decode.bottom_field_order_cnt;
    ctx->slots[slot].long_term_frame_idx = 0;   /* IDR LT idx is 0 by spec */
    ctx->current_idx            = slot;

    /* Capture the dec_ref_pic_marking surface for OnDecodeComplete to
     * apply.  IDR's long_term_reference_flag was handled above; non-IDR
     * MMCO ops are deferred (spec 8.2.5.4 — MMCO runs after pic decode). */
    ctx->pending_adaptive     = (!is_idr) ? parsed->adaptive_ref_pic_marking_mode_flag : 0;
    ctx->pending_n_mmco       = (!is_idr) ? parsed->n_mmco : 0;
    ctx->pending_curr_pic_num = parsed->decode.frame_num;
    if (ctx->pending_n_mmco > 0) {
        memcpy(ctx->pending_mmco, parsed->mmco,
               sizeof(H264Mmco) * ctx->pending_n_mmco);
    }

    /* Sliding-window ref marking (H.264 8.2.5.3) is DEFERRED to
     * Dpb_OnDecodeComplete.
     *
     * The spec runs sliding-window AS PART OF dec_ref_pic_marking which
     * the codec evaluates AFTER the current pic is decoded.  At Select
     * time (= decode setup) the DPB must hold the prior-AU state that
     * the codec will reference; running eviction here was effectively
     * trimming the DPB by one slot too early, producing fn-shifted
     * reg67..98 for every B/P AU after the first eviction fired.
     *
     * Capture max_num_ref_frames now so the post-decode hook can apply
     * the bound without re-parsing the SPS. */
    ctx->max_num_ref_frames = parsed->sps.max_num_ref_frames;
    if (ctx->max_num_ref_frames == 0) ctx->max_num_ref_frames = 1;
    /* Capture FrameNumWrap reference state for OnDecodeComplete's
     * sliding-window eviction (spec 8.2.4.1). */
    ctx->max_frame_num = 1u << (parsed->sps.log2_max_frame_num_minus4 + 4);
    ctx->curr_pic_num  = (uint16_t)parsed->decode.frame_num;

    /* Fill caller-visible selection. */
    memset(out, 0, sizeof(*out));
    out->current_slot   = (uint32_t)slot;
    out->current_output = ctx->pool[slot].output_frame;
    out->current_colmv  = ctx->pool[slot].colmv;

    /* ----- Compact active-ref view (BSP RefFrameList semantics) -------- *
     * BSP fills `pp->RefFrameList[0..N-1]` with active short-term refs in
     * decode-insertion order (= frame_num ascending, modulo 16-bit wrap),
     * then 0xff sentinels.  reg67..98 (POC pairs), reg99..102 (per-slot
     * ref-info nibble), and the RPS-table per-slot fields are all keyed
     * by this compact slot index — NOT our pool slot.  See
     * hal_h264d_vdpu34x.c set_registers and h264d_init.c
     * prepare_init_dpb_info / fill_picparams in /tmp/mpp-src.
     *
     * Walking our sparse pool directly into reg67..98 caused +1 shifts on
     * every B/P AU after the IDR slot got polluted into the head of the
     * emission.  Pack compactly here so the regbuilder can iterate
     * dpb_entries[0..15] verbatim and produce byte-exact register output.
     *
     * Long-term refs are skipped — bframe.h264 / multi.h264 carry only
     * short-term refs; long-term handling is a follow-up alongside MMCO
     * surfacing. */
    struct CompactRef {
        uint32_t pool_idx;        /* original sparse pool slot index */
        uint16_t frame_num;
        int32_t  top_poc;
        int32_t  bottom_poc;
        uint8_t  fields;
        uint8_t  long_term;
        uint16_t long_term_frame_idx;
    };
    CompactRef compact[DPB_MAX_SLOTS];
    uint32_t   n_compact = 0;
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if ((int)i == slot)              continue;
        if (!ctx->slots[i].in_use)        continue;
        if (!ctx->slots[i].is_ref)        continue;
        compact[n_compact].pool_idx   = i;
        compact[n_compact].frame_num  = ctx->slots[i].frame_num;
        compact[n_compact].top_poc    = ctx->slots[i].top_poc;
        compact[n_compact].bottom_poc = ctx->slots[i].bottom_poc;
        compact[n_compact].fields     = ctx->slots[i].fields;
        compact[n_compact].long_term  = ctx->slots[i].long_term;
        compact[n_compact].long_term_frame_idx = ctx->slots[i].long_term_frame_idx;
        n_compact++;
    }
    /* Sort by FrameNumWrap ascending — matches BSP fs[] decode-order
     * traversal in the wrap-aware case (h264d_init.c FrameNumWrap fixup).
     * Raw-frame_num ascending was an approximation that holds only when
     * no frame_num wrap occurred; post-wrap the orderings disagree
     * (e.g. cur=2, refs fn=0,1,15 → raw asc = [0,1,15] but FNW asc =
     * [15(=-1), 0, 1] since fn>cur becomes negative).  The compact slot
     * order seeds reg67..98 (POC bank) and the rps_table per-slot
     * fields; mismatch produces wrong inter prediction on post-wrap
     * P frames. */
    {
        int32_t cur_fn   = (int32_t)parsed->decode.frame_num;
        int32_t max_fn_w = (int32_t)(1u << (parsed->sps.log2_max_frame_num_minus4 + 4));
        auto    fnw      = [&](const CompactRef &c) -> int32_t {
            int32_t fn = (int32_t)c.frame_num;
            return (fn > cur_fn) ? fn - max_fn_w : fn;
        };
        for (uint32_t i = 1; i < n_compact; i++) {
            for (uint32_t j = i; j > 0 && fnw(compact[j-1]) > fnw(compact[j]); j--) {
                CompactRef t = compact[j-1]; compact[j-1] = compact[j]; compact[j] = t;
            }
        }
    }

    /* No L0/L1-active trim here: BSP set_registers emits ALL DPB short-
     * term refs (full p_Dpb->fs_ref[]) into reg67..98, NOT limited by the
     * slice's num_ref_idx_l(0|1)_active.  The slice's active counts only
     * affect RefPicListL0/L1 derivation (which the codec re-runs from
     * slice header bits).  Trim was a wrong hypothesis — see commit log. */

    /* Pack compactly into dpb_entries / refs / ref_colmv.  Slots N..15 stay
     * zero-initialised (flags=0) so the regbuilder emits zeros for empty
     * reg67..98 slots and 0 nibbles for reg99..102 — matching BSP. */
    for (uint32_t i = 0; i < n_compact; i++) {
        v4l2_h264_dpb_entry &e = out->dpb_entries[i];
        e.flags     = V4L2_H264_DPB_ENTRY_FLAG_VALID |
                      V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;
        if (compact[i].long_term)
            e.flags |= V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
        e.fields    = compact[i].fields;
        e.frame_num = compact[i].frame_num;
        /* For LT refs, pic_num conveys LongTermPicNum (frame coding:
         * == LongTermFrameIdx).  ST refs use raw frame_num — the codec
         * keys ref selection off POC (top_field_order_cnt) anyway, and
         * empirically rejects FrameNumWrap-encoded pic_num. */
        e.pic_num   = compact[i].long_term ? compact[i].long_term_frame_idx
                                           : compact[i].frame_num;
        e.top_field_order_cnt    = compact[i].top_poc;
        e.bottom_field_order_cnt = compact[i].bottom_poc;
    }

    /* Per-slot iova fill mirroring BSP set_registers (hal_h264d_vdpu34x.c
     * lines 256-284):
     *
     *   for (i = 0..15)
     *     if (RefFrameList[i] valid)
     *         near_index = ref_index = RefFrameList[i].Index7Bits;
     *     else
     *         ref_index = (near_index < 0) ? CurrPic.Index7Bits : near_index;
     *     ref_base[i]   = buf(ref_index);
     *     colmv_base[i] = colmv(ref_index);
     *
     * With our compact ordering, near_index propagates the LAST active
     * ref's pool slot to the trailing 0xff entries — the codec prefetches
     * via REF_BASE[0] even when reg99..102 marks slot 0 inactive, so a
     * non-zero iova is required everywhere.  When N=0 (IDR), use the
     * current pic's iova (matches BSP behavior at IDR time).
     *
     * The diff harness iova-masks reg160+ so this fill isn't directly
     * checked, but it is what the kernel actually programs to the codec. */
    {
        uint32_t fallback_pool = (n_compact > 0)
                                  ? compact[n_compact - 1].pool_idx
                                  : (uint32_t)slot;
        for (uint32_t i = 0; i < DPB_MAX_SLOTS; i++) {
            uint32_t use_idx;
            if (i < n_compact) {
                use_idx = compact[i].pool_idx;
            } else {
                use_idx = fallback_pool;
            }
            out->refs[i]      = ctx->pool[use_idx].output_frame;
            out->ref_colmv[i] = ctx->pool[use_idx].colmv;
        }
    }

    /* ----- RefPicList build — BSP three-list layout -------------------- *
     * BSP `slice_long.RefPicList[3][32]` is NOT V4L2's two-list (L0/L1).
     * Per `mpp/codec/dec/h264/h264d_fill.c` fill_slice_syntax + h264d_init.c
     * prepare_init_ref_info, the three lists are:
     *   [0] = listP[0]  — P-style ref list, ALWAYS populated for non-IDR
     *                     slices (sorted by FrameNumWrap descending), used
     *                     for P slices and as fallback for B slices.
     *   [1] = listB[0]  — B's L0 (8.2.4.2.3): past desc by POC ++ future
     *                     asc by POC. Empty for P slices.
     *   [2] = listB[1]  — B's L1: future asc by POC ++ past desc by POC.
     *                     Empty for P slices.
     *
     * `index` is the COMPACT slot index (0..n_compact-1).  H264PackFrameRps
     * iterates ref_lists[0..2] and packs them into the per-slice 7-bit
     * entries the codec reads from rps_base. */
    if (!(parsed->decode.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) &&
        n_compact > 0) {
        struct RefSort { uint32_t compact_idx; int32_t poc; uint16_t frame_num; };
        RefSort past[DPB_MAX_SLOTS];   uint32_t n_past = 0;
        RefSort future[DPB_MAX_SLOTS]; uint32_t n_fut  = 0;
        /* Use frame POC = MIN(top, bottom) as BSP `framepoc` does (see
         * h264d_init.c:77). Bare top_field_order_cnt mis-categorises
         * past-vs-future for B-pyramid streams where top/bottom diverge. */
        int32_t cur_poc = parsed->decode.top_field_order_cnt;
        if (parsed->decode.bottom_field_order_cnt < cur_poc)
            cur_poc = parsed->decode.bottom_field_order_cnt;
        for (uint32_t i = 0; i < n_compact; i++) {
            int32_t ref_poc = compact[i].top_poc;
            if (compact[i].bottom_poc < ref_poc) ref_poc = compact[i].bottom_poc;
            RefSort e = { i, ref_poc, compact[i].frame_num };
            /* BSP uses `>=` for past partitioning (h264d_init.c:1204). */
            if (ref_poc <= cur_poc) past[n_past++]   = e;
            else                    future[n_fut++] = e;
        }

        /* Sort past desc by POC, future asc by POC (used by B-list build). */
        for (uint32_t i = 1; i < n_past; i++)
            for (uint32_t j = i; j > 0 && past[j-1].poc < past[j].poc; j--)
                { auto t = past[j-1]; past[j-1] = past[j]; past[j] = t; }
        for (uint32_t i = 1; i < n_fut; i++)
            for (uint32_t j = i; j > 0 && future[j-1].poc > future[j].poc; j--)
                { auto t = future[j-1]; future[j-1] = future[j]; future[j] = t; }

        /* listP[0] → ref_lists[0]: ALL n_compact refs sorted by
         * FrameNumWrap descending (spec 8.2.4.2.1).  FrameNumWrap is
         * `frame_num <= CurrPicNum ? frame_num : frame_num - MaxFrameNum`,
         * giving a signed picture-distance.  Raw-frame_num desc is wrong
         * across frame_num wrap (e.g. fn=15 should sort BEFORE fn=0
         * after a wrap because 15-16=-1 < 0). */
        {
            int32_t cur_fn   = (int32_t)parsed->decode.frame_num;
            int32_t max_fn_w = (int32_t)(1u << (parsed->sps.log2_max_frame_num_minus4 + 4));
            struct RefP { uint32_t compact_idx; int32_t fnw; };
            RefP all[DPB_MAX_SLOTS];
            uint32_t n = 0;
            for (uint32_t i = 0; i < n_compact; i++) {
                int32_t fn  = (int32_t)compact[i].frame_num;
                int32_t fnw = (fn > cur_fn) ? fn - max_fn_w : fn;
                all[n].compact_idx = i;
                all[n].fnw         = fnw;
                n++;
            }
            for (uint32_t i = 1; i < n; i++)
                for (uint32_t j = i; j > 0 && all[j-1].fnw < all[j].fnw; j--)
                    { auto t = all[j-1]; all[j-1] = all[j]; all[j] = t; }
            for (uint32_t i = 0; i < n && i < 32; i++) {
                out->ref_lists[0][i].index  = (uint8_t)all[i].compact_idx;
                out->ref_lists[0][i].fields = V4L2_H264_FRAME_REF;
            }
        }

        const uint8_t st = parsed->slice.slice_type;
        if (st != V4L2_H264_SLICE_TYPE_I && st != V4L2_H264_SLICE_TYPE_SI) {
            /* listB[0] → ref_lists[1]: past desc POC ++ future asc POC.
             * listB[1] → ref_lists[2]: future asc POC ++ past desc POC.
             * BSP populates these for ALL non-I slices including P; size is
             * n_compact (number of available short-term refs), not the
             * slice-header num_ref_idx_lX_active_minus1 (which is undefined
             * for P-slice L1 anyway). */
            uint32_t li = 0;
            for (uint32_t i = 0; i < n_past && li < 32; i++, li++) {
                out->ref_lists[1][li].index  = (uint8_t)past[i].compact_idx;
                out->ref_lists[1][li].fields = V4L2_H264_FRAME_REF;
            }
            for (uint32_t i = 0; i < n_fut && li < 32; i++, li++) {
                out->ref_lists[1][li].index  = (uint8_t)future[i].compact_idx;
                out->ref_lists[1][li].fields = V4L2_H264_FRAME_REF;
            }
            li = 0;
            for (uint32_t i = 0; i < n_fut && li < 32; i++, li++) {
                out->ref_lists[2][li].index  = (uint8_t)future[i].compact_idx;
                out->ref_lists[2][li].fields = V4L2_H264_FRAME_REF;
            }
            for (uint32_t i = 0; i < n_past && li < 32; i++, li++) {
                out->ref_lists[2][li].index  = (uint8_t)past[i].compact_idx;
                out->ref_lists[2][li].fields = V4L2_H264_FRAME_REF;
            }
            /* H.264 8.2.4.2.3 step 3: if listB[1] == listB[0] and size > 1,
             * swap [0]/[1] so the two lists differ. */
            uint32_t lb_size = (n_past + n_fut < 32u) ? (n_past + n_fut) : 32u;
            if (lb_size > 1) {
                bool same = true;
                for (uint32_t i = 0; i < lb_size; i++) {
                    if (out->ref_lists[1][i].index != out->ref_lists[2][i].index) {
                        same = false; break;
                    }
                }
                if (same) {
                    auto t = out->ref_lists[2][0];
                    out->ref_lists[2][0] = out->ref_lists[2][1];
                    out->ref_lists[2][1] = t;
                }
            }
        }

        /* ref_pic_list_modification (spec 8.2.4.3) — applied AFTER the
         * default lists are built.  For P slices (and SP), modifies
         * ref_lists[0] (= listP).  For B slices, modifies ref_lists[1]
         * (B's L0) and ref_lists[2] (B's L1).  Without applying these,
         * the codec uses our default POC/frame_num-sorted lists which
         * disagree with what the slice header asks for → wrong inter
         * prediction → cascading garbage from the first RPLM-using AU. */
        if (parsed->ref_pic_list_modification_flag_l0 ||
            parsed->ref_pic_list_modification_flag_l1) {
            uint16_t compact_fn_arr [DPB_MAX_SLOTS];
            uint8_t  compact_lt_arr [DPB_MAX_SLOTS];
            uint16_t compact_lti_arr[DPB_MAX_SLOTS];
            for (uint32_t i = 0; i < n_compact; i++) {
                compact_fn_arr [i] = compact[i].frame_num;
                compact_lt_arr [i] = compact[i].long_term;
                compact_lti_arr[i] = compact[i].long_term_frame_idx;
            }
            uint32_t max_pic_num = ctx->max_frame_num
                                   ? ctx->max_frame_num : 16u;
            uint16_t curr_pic_num = (uint16_t)parsed->decode.frame_num;
            const uint8_t st_local = parsed->slice.slice_type;

            uint32_t l0_active = (uint32_t)parsed->slice.num_ref_idx_l0_active_minus1 + 1u;
            uint32_t l1_active = (uint32_t)parsed->slice.num_ref_idx_l1_active_minus1 + 1u;
            if (l0_active > 32) l0_active = 32;
            if (l1_active > 32) l1_active = 32;

            if (st_local == V4L2_H264_SLICE_TYPE_P ||
                st_local == V4L2_H264_SLICE_TYPE_SP) {
                /* P slice: modify listP (= ref_lists[0]). */
                apply_rplm(out->ref_lists[0], parsed, /*which=*/0,
                           compact_fn_arr, compact_lt_arr, compact_lti_arr,
                           n_compact, curr_pic_num, max_pic_num,
                           l0_active);
            } else if (st_local == V4L2_H264_SLICE_TYPE_B) {
                /* B slice: modify both B-list halves. */
                apply_rplm(out->ref_lists[1], parsed, /*which=*/0,
                           compact_fn_arr, compact_lt_arr, compact_lti_arr,
                           n_compact, curr_pic_num, max_pic_num,
                           l0_active);
                apply_rplm(out->ref_lists[2], parsed, /*which=*/1,
                           compact_fn_arr, compact_lt_arr, compact_lti_arr,
                           n_compact, curr_pic_num, max_pic_num,
                           l1_active);
            }
        }
    }
    return DPB_OK;
}

extern "C"
void Dpb_OnDecodeComplete(DpbCtx *ctx)
{
    if (!ctx) return;
    ctx->last_bumped_poc = INT32_MIN;
    if (ctx->current_idx < 0) return;

    int cur = ctx->current_idx;
    ctx->current_idx = -1;

    if (!ctx->slots[cur].is_ref) {
        /* Non-reference picture — release the slot back to the pool. */
        slot_clear(ctx, cur);
        ctx->pending_adaptive = 0;
        ctx->pending_n_mmco   = 0;
        return;
    }

    /* Spec 8.2.5.4: if adaptive_ref_pic_marking_mode_flag was set, MMCO
     * ops fully replace sliding-window.  Otherwise run sliding-window
     * (8.2.5.3). */
    if (ctx->pending_adaptive) {
        apply_mmco_ops(ctx, cur);
    } else {
        uint32_t max_refs = ctx->max_num_ref_frames ? ctx->max_num_ref_frames : 1;
        for (;;) {
            uint32_t cnt = 0;
            for (uint32_t i = 0; i < ctx->pool_size; i++) {
                if (ctx->slots[i].in_use && ctx->slots[i].is_ref &&
                    !ctx->slots[i].long_term) cnt++;
            }
            if (cnt <= max_refs) break;
            evict_oldest_short_term(ctx);
        }
    }

    ctx->pending_adaptive = 0;
    ctx->pending_n_mmco   = 0;
}

/* =====================================================================
 * HEVC DPB — RPS-driven reference marking (HEVC spec 8.3.2).
 *
 * The codec runs in HW_RPS mode so it consumes the slice header's STRPS
 * directly; we only need to keep the slot-level POC bookkeeping aligned
 * with what the codec is going to derive, and propagate the "is this slot
 * still useful?" bit so we can recycle slots that fall out of the RPS.
 * ===================================================================== */

namespace {

static inline void h265_slot_clear(H265DpbCtx *ctx, int i) {
    /* external_hold deliberately not touched — see DpbCtx::slot_clear. */
    ctx->slots[i].in_use = 0;
    ctx->slots[i].is_ref = 0;
    ctx->slots[i].poc    = 0;
}

static int h265_find_free(const H265DpbCtx *ctx) {
    /* First reuse a slot that's no longer a ref (RPS marking already
     * cleared its is_ref bit) AND isn't held by a downstream consumer.
     * Then try truly-empty slots that also aren't held. */
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (ctx->slots[i].external_hold)              continue;
        if (ctx->slots[i].in_use && !ctx->slots[i].is_ref) return (int)i;
    }
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (ctx->slots[i].external_hold) continue;
        if (!ctx->slots[i].in_use)       return (int)i;
    }
    return -1;
}

extern "C" void H265Dpb_AddExternalHold(H265DpbCtx *ctx, uint32_t slot_idx) {
    if (!ctx || slot_idx >= ctx->pool_size) return;
    if (ctx->slots[slot_idx].external_hold < 0xF)
        ctx->slots[slot_idx].external_hold++;
}

extern "C" void H265Dpb_ReleaseExternalHold(H265DpbCtx *ctx, uint32_t slot_idx) {
    if (!ctx || slot_idx >= ctx->pool_size) return;
    if (ctx->slots[slot_idx].external_hold > 0)
        ctx->slots[slot_idx].external_hold--;
}

/* Resolve the active short-term RPS for the current slice (HEVC 7.4.7.1):
 * either the slice carries one inline, or it references st_rps[idx] in
 * the active SPS. */
static const H265ShortTermRPS *h265_active_strps(const H265ParseResult *p)
{
    const H265SliceHeader &sh = p->slice;
    if (!sh.short_term_ref_pic_set_sps_flag) return &sh.st_rps_slice;
    if (p->active_sps_id < 0) return nullptr;
    const H265Sps &sps = p->sps[p->active_sps_id];
    if (!sps.valid) return nullptr;
    if (sh.short_term_ref_pic_set_idx >= sps.num_short_term_ref_pic_sets)
        return nullptr;
    return &sps.st_rps[sh.short_term_ref_pic_set_idx];
}

} /* anon */

extern "C"
DpbStatus H265Dpb_Init(H265DpbCtx *ctx, const DpbPoolEntry *pool,
                       uint32_t pool_size)
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
    for (uint32_t i = 0; i < pool_size; i++) ctx->pool[i] = pool[i];
    return DPB_OK;
}

extern "C"
DpbStatus H265Dpb_Select(H265DpbCtx *ctx, const H265ParseResult *parsed,
                         H265DpbSelection *out)
{
    if (!ctx || !parsed || !out)   return DPB_INVALID_INPUT;
    if (!parsed->has_slice)        return DPB_INVALID_INPUT;

    /* ----- step 1: prep output (sentinel-fill ref_pocs[]) ------------ */
    memset(out, 0, sizeof(*out));
    for (int i = 0; i < DPB_MAX_SLOTS; i++) {
        out->ref_pocs[i] = H265_DPB_REF_POC_SENTINEL;
    }
    out->cur_top_poc = parsed->poc;
    out->cur_bot_poc = parsed->poc;

    /* ----- step 2: IDR flush (HEVC 8.3.2 paragraph 1) ---------------- *
     * For an IDR pic, all PocStCurr-star / PocStFoll lists are empty by
     * definition: every previously-decoded picture is marked "unused
     * for reference".  Clear all slots before allocating the new pic. */
    if (parsed->is_idr) {
        for (uint32_t i = 0; i < ctx->pool_size; i++) h265_slot_clear(ctx, (int)i);
    } else {
        /* ----- step 3: walk the active STRPS and remark slots ------- *
         * HEVC 8.3.2: for each entry in the RPS,
         *   PocLt = (used_by_curr_pic_*_flag != 0) ? PocStCurr* : PocStFoll
         * The union of all four PocStCurr-star / PocStFoll lists is the set of
         * POCs that must remain in the DPB for this picture.  Any slot
         * whose POC isn't in that union becomes "unused for reference".
         *
         * Long-term refs are out of scope (parser stubbed them); we only
         * walk the short-term half. */
        const H265ShortTermRPS *rps = h265_active_strps(parsed);

        /* Build a union set of "POCs needed by this picture".  At most
         * H265_MAX_REFS entries since num_delta_pocs <= H265_MAX_REFS. */
        int32_t  needed_pocs[H265_MAX_REFS];
        uint32_t n_needed = 0;

        if (rps) {
            /* Spec 8.3.2 derivation of PocStCurr{Before,After} and
             * PocStFoll: the RPS array is laid out as [neg... | pos...]
             * with delta_poc[i] already absolute (not delta-of-delta) per
             * the parser's resolution of inter_ref_pic_set_prediction.
             * For each entry the absolute POC is `parsed->poc + dpoc`
             * (negatives have dpoc<0, positives dpoc>0). */
            const uint32_t n_neg = rps->num_negative_pics;
            const uint32_t n_pos = rps->num_positive_pics;
            for (uint32_t i = 0; i < n_neg + n_pos &&
                                 i < H265_MAX_REFS; i++) {
                int32_t abs_poc = parsed->poc + (int32_t)rps->delta_poc[i];
                /* Whether or not used_by_curr_pic_flag[i] is set, this
                 * POC is part of the union — used_by_curr_pic distinguishes
                 * PocStCurr* from PocStFoll, but BOTH must be retained. */
                needed_pocs[n_needed++] = abs_poc;
            }
        }

        /* Apply marking: slots whose POC appears in needed_pocs[] keep
         * is_ref; everyone else loses it. */
        for (uint32_t i = 0; i < ctx->pool_size; i++) {
            if (!ctx->slots[i].in_use) continue;
            bool needed = false;
            for (uint32_t k = 0; k < n_needed; k++) {
                if (ctx->slots[i].poc == needed_pocs[k]) { needed = true; break; }
            }
            if (needed) {
                ctx->slots[i].is_ref = 1;
            } else {
                /* Slot drops out of the active set — the codec won't
                 * reference it for this pic.  Free it for reuse. */
                ctx->slots[i].is_ref = 0;
                /* in_use stays 1 until h265_find_free recycles it; this
                 * keeps the post-marking iteration deterministic and
                 * matches the BSP's "first reusable, then empty" pick
                 * order. */
            }
        }
    }

    /* ----- step 4: allocate a slot for the current picture ----------- */
    int slot = h265_find_free(ctx);
    if (slot < 0) return DPB_FULL;

    h265_slot_clear(ctx, slot);
    ctx->slots[slot].in_use = 1;
    /* `is_ref` for the current pic isn't known until we see the next
     * picture's RPS (which is when the codec decides if this pic is a
     * ref).  Optimistically mark it a ref; H265Dpb_OnDecodeComplete
     * may demote it for non-ref NALs (TRAIL_N etc.). */
    ctx->slots[slot].is_ref = parsed->nal_ref_flag ? 1 : 0;
    ctx->slots[slot].poc    = parsed->poc;
    ctx->current_idx        = slot;

    /* ----- step 5: pack the selection output ------------------------- *
     * The regbuilder consumes refs[]/ref_colmv[]/ref_pocs[] indexed by
     * DPB slot order.  Per the task brief: pack active refs into
     * refs[0..N-1] in POC order so the codec's own RefPicListL0/L1
     * derivation lands on the same slot indices.  Slot indices in the
     * H265DpbCtx itself are NOT renumbered — just the output view. */
    out->current_slot       = (uint32_t)slot;
    out->current_output_iova= ctx->pool[slot].output_frame;
    out->current_colmv_iova = ctx->pool[slot].colmv;

    /* Pack refs INDEXED BY DPB SLOT — not RefPicList order.  hal_h265d_vdpu34x.c
     * (BSP, line 1049) iterates dxva.RefPicList[i] for i in 0..15 and treats
     * `i` as the OUTPUT-bank slot; the DXVA upper layer fills RefPicList[i]
     * with the picture currently held in DPB slot `i` (Index7Bits == i for
     * valid entries, bPicEntry == 0xff for empty slots).  reg67_82_ref_poc[i]
     * therefore = POC of DPB slot i, NOT the i-th active ref in RefPicListTemp0
     * order.
     *
     * For B-frame streams with multiple active refs, slot order ≠ POC order,
     * so packing into slot index is required for byte-exact BSP parity.  For
     * single-ref P-only streams (multi.h265) the two orders coincide on slot 1.
     *
     * For HW_RPS this is functionally cosmetic — the codec re-derives L0/L1
     * from the slice header — but byte-exact reg67..82 parity keeps the
     * winreplay diff harness clean across all AUs. */
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if ((int)i == slot)              continue;
        if (!ctx->slots[i].in_use)        continue;
        if (!ctx->slots[i].is_ref)        continue;
        out->refs[i]       = ctx->pool[i].output_frame;
        out->ref_colmv[i]  = ctx->pool[i].colmv;
        out->ref_pocs[i]   = ctx->slots[i].poc;
        out->ref_valid_mask |= (uint16_t)(1u << i);
    }

    return DPB_OK;
}

extern "C"
void H265Dpb_OnDecodeComplete(H265DpbCtx *ctx)
{
    if (!ctx || ctx->current_idx < 0) return;
    int cur = ctx->current_idx;
    ctx->current_idx = -1;

    /* HEVC: RPS marking on the *next* Select handles eviction.  The only
     * thing left to do is release a non-reference picture (TRAIL_N /
     * TSA_N / STSA_N / RADL_N / RASL_N — sub-layer non-reference NALs
     * whose nal_ref_flag was 0 at Select time). */
    if (!ctx->slots[cur].is_ref) {
        h265_slot_clear(ctx, cur);
    }
}
