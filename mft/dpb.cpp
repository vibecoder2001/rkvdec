/* mft/dpb.cpp — Phase 3b minimal H.264 DPB. */
#include "dpb.h"
#include <string.h>
#include <cassert>

namespace {

/* True iff any lifecycle flag is set — slot is still "in use" for the
 * picker.  Mirrors the role the old `external_hold` refcount played. */
static inline bool slot_held(const DpbCtx *ctx, int i) {
    return ctx->slots[i].pending_reorder
        || ctx->slots[i].pending_ready
        || ctx->slots[i].held_by_consumer;
}
static inline bool h265_slot_held(const H265DpbCtx *ctx, int i) {
    return ctx->slots[i].pending_reorder
        || ctx->slots[i].pending_ready
        || ctx->slots[i].held_by_consumer;
}

/* Helpers for the per-slot bitfield struct (the C ABI uses :1 fields,
 * so we go through tiny accessors to keep the implementation tidy).
 * Note: lifecycle flags + epoch are intentionally preserved here — they
 * track downstream consumers, and the DPB clearing reference state for
 * a slot doesn't release the consumer's claim on its data. */
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
            /* H.264 7.4.3.3: long_term_frame_idx must be in
             * [0, MaxLongTermFrameIdx].  Reject out-of-range values so
             * an adversarial stream can't promote a slot to an LT idx
             * the codec never authorised — would later confuse
             * find_long_term_by_frame_idx and the codec's own LT match.
             * Defense-in-depth for the kernel (review parser Medium #6). */
            if (ctx->max_long_term_frame_idx < 0 ||
                (int32_t)m.long_term_frame_idx > ctx->max_long_term_frame_idx) {
                break;
            }
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
            /* Same range check as MMCO 3 — see spec 7.4.3.3.
             * Review parser Medium #6. */
            if (ctx->max_long_term_frame_idx < 0 ||
                (int32_t)m.long_term_frame_idx > ctx->max_long_term_frame_idx) {
                break;
            }
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
        if (ctx->slots[i].in_use)  continue;
        if (slot_held(ctx, (int)i)) continue;
        return (int)i;
    }
    return -1;
}

/* ----- H.264 hold API.  Refcount semantics replaced by named-flag
 *       single-set/single-clear with assertions on every transition. */

static inline uint8_t h264_flag_get(const DpbCtx *ctx, uint32_t slot_idx,
                                    DpbHoldReason r)
{
    const auto &s = ctx->slots[slot_idx];
    switch (r) {
    case DPB_HOLD_REORDER:  return s.pending_reorder;
    case DPB_HOLD_READY:    return s.pending_ready;
    case DPB_HOLD_CONSUMER: return s.held_by_consumer;
    }
    return 0;
}
static inline void h264_flag_set(DpbCtx *ctx, uint32_t slot_idx,
                                 DpbHoldReason r, uint8_t v)
{
    auto &s = ctx->slots[slot_idx];
    switch (r) {
    case DPB_HOLD_REORDER:  s.pending_reorder  = v; return;
    case DPB_HOLD_READY:    s.pending_ready    = v; return;
    case DPB_HOLD_CONSUMER: s.held_by_consumer = v; return;
    }
}

extern "C" void Dpb_AddHold(DpbCtx *ctx, uint32_t slot_idx,
                            DpbHoldReason reason, uint32_t entry_epoch)
{
    if (!ctx || slot_idx >= ctx->pool_size) return;
    /* RKMPP_VERIFY: consumer-hold-path invariants are load-bearing —
     * silently violating them corrupts slot lifetime tracking and the
     * kernel later DMAs into a slot a live consumer is still reading.
     * `assert()` compiles out under NDEBUG; use the always-evaluated
     * macro instead.  Review parser Low #11. */
    RKMPP_VERIFY(h264_flag_get(ctx, slot_idx, reason) == 0);
    RKMPP_VERIFY(ctx->slots[slot_idx].epoch == entry_epoch);
    (void)entry_epoch;  /* silence Release-build unused-param warning */
    h264_flag_set(ctx, slot_idx, reason, 1);
}

extern "C" void Dpb_ReleaseHold(DpbCtx *ctx, uint32_t slot_idx,
                                DpbHoldReason reason)
{
    if (!ctx || slot_idx >= ctx->pool_size) return;
    /* Double-release on the consumer path is load-bearing — see
     * Dpb_AddHold rationale.  Review parser Low #11. */
    RKMPP_VERIFY(h264_flag_get(ctx, slot_idx, reason) == 1);
    h264_flag_set(ctx, slot_idx, reason, 0);
}

extern "C" void Dpb_TransferHold(DpbCtx *ctx, uint32_t slot_idx,
                                 DpbHoldReason from, DpbHoldReason to,
                                 uint32_t entry_epoch)
{
    if (!ctx || slot_idx >= ctx->pool_size) return;
    RKMPP_VERIFY(from != to);
    RKMPP_VERIFY(h264_flag_get(ctx, slot_idx, from) == 1);
    RKMPP_VERIFY(h264_flag_get(ctx, slot_idx, to)   == 0);
    RKMPP_VERIFY(ctx->slots[slot_idx].epoch == entry_epoch);
    (void)entry_epoch;
    h264_flag_set(ctx, slot_idx, from, 0);
    h264_flag_set(ctx, slot_idx, to,   1);
}

extern "C" void Dpb_Flush(DpbCtx *ctx)
{
    if (!ctx) return;
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        /* Caller is responsible for releasing reorder/ready holds before
         * calling Flush — those are tied to engine queue entries the
         * caller knows about.  Consumer holds may legitimately survive a
         * flush and must keep their pre-flush epoch so a later
         * ReleaseHold(CONSUMER) still asserts cleanly.  RKMPP_VERIFY:
         * load-bearing — see Dpb_AddHold.  Review parser Low #11. */
        RKMPP_VERIFY(ctx->slots[i].pending_reorder == 0);
        RKMPP_VERIFY(ctx->slots[i].pending_ready   == 0);
        if (!ctx->slots[i].held_by_consumer) {
            slot_clear(ctx, (int)i);
        } else {
            /* Drop reference state but keep the slot reserved for the
             * consumer.  The consumer's eventual ReleaseHold will
             * compare against the pre-flush epoch on this slot. */
            ctx->slots[i].in_use    = 0;
            ctx->slots[i].is_ref    = 0;
            ctx->slots[i].long_term = 0;
        }
    }
    ctx->current_idx = -1;
    ctx->current_epoch++;
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

    /* Slot-state invariant: find_free guarantees in_use=0 and no hold,
     * but make it explicit at the picker boundary.  An assert here
     * means find_free or Flush left a slot in an unexpected state. */
    assert(!ctx->slots[slot].in_use);
    assert(!slot_held(ctx, slot));
    assert(!ctx->slots[slot].is_ref);
    assert(ctx->current_idx != slot);

    /* Populate the chosen slot for the picture currently being decoded. */
    ctx->slots[slot].in_use     = 1;
    ctx->slots[slot].is_ref     = (parsed->decode.nal_ref_idc != 0);
    ctx->slots[slot].long_term  = (is_idr && parsed->idr_long_term_reference_flag) ? 1 : 0;
    ctx->slots[slot].fields     = V4L2_H264_FRAME_REF;
    ctx->slots[slot].frame_num  = parsed->decode.frame_num;
    ctx->slots[slot].top_poc    = parsed->decode.top_field_order_cnt;
    ctx->slots[slot].bottom_poc = parsed->decode.bottom_field_order_cnt;
    ctx->slots[slot].long_term_frame_idx = 0;   /* IDR LT idx is 0 by spec */
    ctx->slots[slot].epoch      = ctx->current_epoch;
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
        /* V4L2 semantics: for LT refs, dpb[i].frame_num carries
         * long_term_frame_idx (not the original short-term frame_num).
         * H264PackFrameRps consumes e.frame_num to emit the wrap value
         * the codec uses to match LT refs at slice time; emitting the
         * original frame_num leaves LT slots unmatchable, causing
         * ref-list dereferences into junk slots once an LT-marked ref
         * is actually used. */
        e.frame_num = compact[i].long_term ? compact[i].long_term_frame_idx
                                           : compact[i].frame_num;
        /* pic_num mirrors frame_num here — for LT this is LongTermPicNum
         * (frame coding: == LongTermFrameIdx). */
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

        /* ref_pic_list_modification is HW-applied: rkvdec2 re-parses the
         * slice header and modifies the reflists internally.  Pre-applying
         * RPLM here causes a double-apply on B slices and produces
         * indices into slots without backing refs → post-seek H.264
         * timeouts.  See [[h264_v4l2_semantics]] and upstream
         * rkvdec-vdpu381-h264.c for the V4L2 reference. */
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

extern "C"
void Dpb_OnDecodeFailed(DpbCtx *ctx)
{
    if (!ctx) return;
    ctx->last_bumped_poc = INT32_MIN;

    if (ctx->current_idx >= 0) {
        int cur = ctx->current_idx;
        ctx->current_idx = -1;
        slot_clear(ctx, cur);
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
    /* Lifecycle flags + epoch deliberately not touched — see slot_clear. */
    ctx->slots[i].in_use = 0;
    ctx->slots[i].is_ref = 0;
    ctx->slots[i].poc    = 0;
}

static int h265_find_free(const H265DpbCtx *ctx) {
    /* First reuse a slot that's no longer a ref (RPS marking already
     * cleared its is_ref bit) AND isn't held by a downstream consumer.
     * Then try truly-empty slots that also aren't held. */
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (h265_slot_held(ctx, (int)i))               continue;
        if (ctx->slots[i].in_use && !ctx->slots[i].is_ref) return (int)i;
    }
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        if (h265_slot_held(ctx, (int)i)) continue;
        if (!ctx->slots[i].in_use)       return (int)i;
    }
    return -1;
}

static inline uint8_t h265_flag_get(const H265DpbCtx *ctx, uint32_t slot_idx,
                                    DpbHoldReason r)
{
    const auto &s = ctx->slots[slot_idx];
    switch (r) {
    case DPB_HOLD_REORDER:  return s.pending_reorder;
    case DPB_HOLD_READY:    return s.pending_ready;
    case DPB_HOLD_CONSUMER: return s.held_by_consumer;
    }
    return 0;
}
static inline void h265_flag_set(H265DpbCtx *ctx, uint32_t slot_idx,
                                 DpbHoldReason r, uint8_t v)
{
    auto &s = ctx->slots[slot_idx];
    switch (r) {
    case DPB_HOLD_REORDER:  s.pending_reorder  = v; return;
    case DPB_HOLD_READY:    s.pending_ready    = v; return;
    case DPB_HOLD_CONSUMER: s.held_by_consumer = v; return;
    }
}

extern "C" void H265Dpb_AddHold(H265DpbCtx *ctx, uint32_t slot_idx,
                                DpbHoldReason reason, uint32_t entry_epoch)
{
    if (!ctx || slot_idx >= ctx->pool_size) return;
    /* RKMPP_VERIFY — see Dpb_AddHold rationale.  Review parser Low #11. */
    RKMPP_VERIFY(h265_flag_get(ctx, slot_idx, reason) == 0);
    RKMPP_VERIFY(ctx->slots[slot_idx].epoch == entry_epoch);
    (void)entry_epoch;
    h265_flag_set(ctx, slot_idx, reason, 1);
}
extern "C" void H265Dpb_ReleaseHold(H265DpbCtx *ctx, uint32_t slot_idx,
                                    DpbHoldReason reason)
{
    if (!ctx || slot_idx >= ctx->pool_size) return;
    RKMPP_VERIFY(h265_flag_get(ctx, slot_idx, reason) == 1);
    h265_flag_set(ctx, slot_idx, reason, 0);
}
extern "C" void H265Dpb_TransferHold(H265DpbCtx *ctx, uint32_t slot_idx,
                                     DpbHoldReason from, DpbHoldReason to,
                                     uint32_t entry_epoch)
{
    if (!ctx || slot_idx >= ctx->pool_size) return;
    RKMPP_VERIFY(from != to);
    RKMPP_VERIFY(h265_flag_get(ctx, slot_idx, from) == 1);
    RKMPP_VERIFY(h265_flag_get(ctx, slot_idx, to)   == 0);
    RKMPP_VERIFY(ctx->slots[slot_idx].epoch == entry_epoch);
    (void)entry_epoch;
    h265_flag_set(ctx, slot_idx, from, 0);
    h265_flag_set(ctx, slot_idx, to,   1);
}
extern "C" void H265Dpb_Flush(H265DpbCtx *ctx)
{
    if (!ctx) return;
    for (uint32_t i = 0; i < ctx->pool_size; i++) {
        RKMPP_VERIFY(ctx->slots[i].pending_reorder == 0);
        RKMPP_VERIFY(ctx->slots[i].pending_ready   == 0);
        if (!ctx->slots[i].held_by_consumer) {
            h265_slot_clear(ctx, (int)i);
        } else {
            ctx->slots[i].in_use = 0;
            ctx->slots[i].is_ref = 0;
        }
    }
    ctx->current_idx = -1;
    ctx->current_epoch++;
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

extern "C"
void H265Dpb_OnDecodeFailed(H265DpbCtx *ctx)
{
    if (!ctx) return;
    if (ctx->current_idx >= 0) {
        int cur = ctx->current_idx;
        ctx->current_idx = -1;
        h265_slot_clear(ctx, cur);
    }
}
