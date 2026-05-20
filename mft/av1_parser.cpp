/* mft/av1_parser.cpp — Clean-room AV1 OBU header parser.
 *
 * Implements Av1ParseSeqHeader() and Av1ParseFrameHeader() from av1_parser.h.
 * Produces Dav1dSequenceHeader / Dav1dFrameHeader structs identical to what
 * dav1d would produce for the fields consumed by regbuilder_av1.cpp.
 *
 * Authoritative reference: AV1 Bitstream & Decoding Process Specification
 * v1.0.0 (https://aomediacodec.github.io/av1-spec/av1-spec.pdf).
 *   §4.10  — leb128, uvlc, su(n)
 *   §5.5   — sequence_header_obu()
 *   §5.9   — uncompressed_header() (frame header)
 *   §5.11  — various decode processes referenced inline
 *   §7.8   — get_relative_dist()
 *   §7.20  — reference frame update process
 *
 * No dav1d library functions are called.  <dav1d/headers.h> is included
 * only for the struct/enum definitions.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include "av1_parser.h"

#include <cstring>
#include <climits>
#include <cassert>

extern "C" {
#include <dav1d/headers.h>
}

/* =========================================================================
 * Internal bit-reader
 * Reads MSB-first within each byte (same convention as AV1 spec §4.10).
 * ========================================================================= */

struct BitReader {
    const uint8_t *buf;
    const uint8_t *end;
    uint64_t       state;     /* MSB-aligned shift register */
    int            bits_left; /* valid bits in state (0..63) */
    bool           error;

    BitReader(const uint8_t *p, size_t len)
        : buf(p), end(p + len), state(0), bits_left(0), error(false) {}
};

/* Refill the MSB-aligned shift register so at least n bits are available.
 * state holds valid bits in its top bits_left positions. New bytes are
 * inserted below the existing valid bits. */
static void br_refill(BitReader &r, int n)
{
    while (r.bits_left < n) {
        if (r.buf >= r.end) {
            r.error = true;
            return;
        }
        /* Append next byte below the currently valid bits */
        r.state |= (uint64_t)(*r.buf++) << (56 - r.bits_left);
        r.bits_left += 8;
    }
}

/* Read n unsigned bits (1 <= n <= 32).
 * State register is MSB-aligned: the top bits_left bits are valid.
 * After extraction the n consumed bits are shifted out (left). */
static uint32_t br_u(BitReader &r, int n)
{
    assert(n >= 1 && n <= 32);
    if (r.bits_left < n)
        br_refill(r, n);
    if (r.error) return 0;
    /* Top n bits of state */
    uint32_t v = (uint32_t)(r.state >> (64 - n));
    if (n < 32)
        v &= (1u << n) - 1u;
    r.state    <<= n;
    r.bits_left -= n;
    return v;
}

/* Read 1 bit. */
static inline uint32_t br_bit(BitReader &r) { return br_u(r, 1); }

/* Read n-bit signed integer via sign-extension (spec su(n)). */
static int32_t br_su(BitReader &r, int n)
{
    uint32_t v = br_u(r, n);
    int32_t  s = (int32_t)v;
    if (n < 32 && (v >> (n - 1)))
        s -= (1 << n);
    return s;
}

/* Read n-bit signed delta: f(1) flag + f(n) magnitude if flag is set. */
static int32_t br_delta_q(BitReader &r, int n)
{
    if (!br_bit(r)) return 0;
    return br_su(r, n);
}

/* Byte-align the reader (consume 0..7 trailing bits). */
static void br_byte_align(BitReader &r)
{
    int rem = r.bits_left & 7;
    if (rem) {
        br_u(r, rem);
    }
}

/* Return number of bytes consumed (rounded up from bit position). */
static size_t br_bytes_consumed(const BitReader &r, const uint8_t *start)
{
    /* buf points past last fully-consumed byte; bits_left are the
     * fractional bits we've pre-fetched but not consumed.  The byte
     * offset of the *current bit position* is:
     *   (buf - start) - ceil(bits_left / 8)  ... but we need to be
     * careful since bits_left can hold partial bytes. */
    size_t bytes_fetched = (size_t)(r.buf - start);
    size_t bits_pending  = (size_t)((r.bits_left + 7) / 8);
    if (bytes_fetched >= bits_pending)
        return bytes_fetched - bits_pending;
    return 0;
}

/* -------------------------------------------------------------------------
 * AV1 variable-length codes
 * ---------------------------------------------------------------------- */

/* leb128 (AV1 spec §4.10.5): reads a variable-length unsigned integer.
 * Used for OBU size fields. */
static uint32_t br_leb128(BitReader &r)
{
    uint64_t val = 0;
    for (int i = 0; i < 8; i++) {
        uint32_t byte = br_u(r, 8);
        val |= ((uint64_t)(byte & 0x7f)) << (i * 7);
        if (!(byte & 0x80)) break;
    }
    if (val > UINT32_MAX) { r.error = true; return 0; }
    return (uint32_t)val;
}

/* uvlc (AV1 spec §4.10.3): reads unsigned variable-length code.
 * On excessive leading-zero run (>= 32) sets r.error AND returns
 * UINT32_MAX — older call sites that check for UINT32_MAX keep
 * working, but new code can rely on the sticky error flag like the
 * H.264/H.265 br_failed pattern.  Without the sticky flag a parser
 * that sailed past a malformed uvlc would commit a poisoned value
 * to the regbuilder (which the kernel doesn't re-validate). */
static uint32_t br_uvlc(BitReader &r)
{
    int leading = 0;
    while (!br_bit(r)) {
        if (++leading == 32) { r.error = true; return UINT32_MAX; }
    }
    if (leading == 0) return 0;
    return ((1u << leading) - 1) + br_u(r, leading);
}

/* uniform(n) — spec §4.10.7: reads a value in [0, n-1] using the
 * minimum number of bits.  Callers must ensure n > 1; the prior
 * `assert` evaporated under -DNDEBUG.  Release-safe fail-closed
 * path latches r.error and returns 0 so a malformed caller can't
 * UB the shift below. */
static uint32_t br_uniform(BitReader &r, uint32_t n)
{
    if (n <= 1) { r.error = true; return 0; }
    /* floor(log2(n)) */
    int l = 0;
    for (uint32_t tmp = n; tmp > 1; tmp >>= 1) l++;
    l++; /* ceil(log2(n)) or floor+1 */
    uint32_t m = (1u << l) - n;
    uint32_t v = br_u(r, l - 1);
    return (v < m) ? v : ((v << 1) - m + br_bit(r));
}

/* subexp(ref, n) — spec §4.10.6 / NS(n) variant used for global MV.
 * Reads a value near `ref` in [-(1<<n), (1<<n)] using subexponential code.
 * Matches dav1d's dav1d_get_bits_subexp(). */
static uint32_t br_subexp_u(BitReader &r, uint32_t ref, uint32_t n)
{
    uint32_t v = 0;
    for (int i = 0; ; i++) {
        int b = i ? 3 + i - 1 : 3;
        uint32_t ub = (uint32_t)b;
        if (n < v + 3u * (1u << ub)) {
            v += br_uniform(r, n - v + 1);
            break;
        }
        if (!br_bit(r)) {
            v += br_u(r, b);
            break;
        }
        v += 1u << ub;
    }
    /* inv_recenter */
    auto inv_recenter = [](uint32_t r2, uint32_t v2) -> uint32_t {
        if (v2 > 2 * r2) return v2;
        if (v2 & 1)       return r2 - ((v2 + 1) >> 1);
        return r2 + (v2 >> 1);
    };
    return (ref * 2 <= n) ? inv_recenter(ref, v) :
                             n - inv_recenter(n - ref, v);
}

static int32_t br_subexp(BitReader &r, int32_t ref, uint32_t n)
{
    return (int32_t)br_subexp_u(r, (uint32_t)(ref + (1 << n)), 2u << n) -
           (int32_t)(1 << n);
}

/* =========================================================================
 * Sequence header parser  (AV1 spec §5.5)
 * ========================================================================= */

static int parse_seq_hdr(BitReader &r, Dav1dSequenceHeader *s)
{
    memset(s, 0, sizeof(*s));

    s->profile = (uint8_t)br_u(r, 3);
    if (s->profile > 2) return -1;

    s->still_picture             = (uint8_t)br_bit(r);
    s->reduced_still_picture_header = (uint8_t)br_bit(r);
    if (s->reduced_still_picture_header && !s->still_picture) return -1;

    if (s->reduced_still_picture_header) {
        s->num_operating_points = 1;
        s->operating_points[0].major_level = (uint8_t)br_u(r, 3);
        s->operating_points[0].minor_level = (uint8_t)br_u(r, 2);
        s->operating_points[0].initial_display_delay = 10;
        s->timing_info_present = 0;
        s->decoder_model_info_present = 0;
        s->display_model_info_present = 0;
    } else {
        s->timing_info_present = (uint8_t)br_bit(r);
        if (s->timing_info_present) {
            s->num_units_in_tick = br_u(r, 32);
            s->time_scale        = br_u(r, 32);
            s->equal_picture_interval = (uint8_t)br_bit(r);
            if (s->equal_picture_interval) {
                uint32_t ntpp = br_uvlc(r);
                if (ntpp == UINT32_MAX) return -1;
                s->num_ticks_per_picture = ntpp + 1;
            }
            s->decoder_model_info_present = (uint8_t)br_bit(r);
            if (s->decoder_model_info_present) {
                s->encoder_decoder_buffer_delay_length = (uint8_t)(br_u(r, 5) + 1);
                s->num_units_in_decoding_tick          = br_u(r, 32);
                s->buffer_removal_delay_length         = (uint8_t)(br_u(r, 5) + 1);
                s->frame_presentation_delay_length     = (uint8_t)(br_u(r, 5) + 1);
            }
        } else {
            s->decoder_model_info_present = 0;
        }

        s->display_model_info_present = (uint8_t)br_bit(r);
        s->num_operating_points = (uint8_t)(br_u(r, 5) + 1);
        for (int i = 0; i < s->num_operating_points; i++) {
            auto &op = s->operating_points[i];
            op.idc         = (uint16_t)br_u(r, 12);
            op.major_level = (uint8_t)(2 + br_u(r, 3));
            op.minor_level = (uint8_t)br_u(r, 2);
            if (op.major_level > 3)
                op.tier = (uint8_t)br_bit(r);
            if (s->decoder_model_info_present) {
                op.decoder_model_param_present = (uint8_t)br_bit(r);
                if (op.decoder_model_param_present) {
                    auto &opi = s->operating_parameter_info[i];
                    opi.decoder_buffer_delay =
                        br_u(r, s->encoder_decoder_buffer_delay_length);
                    opi.encoder_buffer_delay =
                        br_u(r, s->encoder_decoder_buffer_delay_length);
                    opi.low_delay_mode = (uint8_t)br_bit(r);
                }
            }
            if (s->display_model_info_present)
                op.display_model_param_present = (uint8_t)br_bit(r);
            op.initial_display_delay =
                op.display_model_param_present
                    ? (uint8_t)(br_u(r, 4) + 1)
                    : 10;
        }
    }

    s->width_n_bits  = (uint8_t)(br_u(r, 4) + 1);
    s->height_n_bits = (uint8_t)(br_u(r, 4) + 1);
    s->max_width     = (int)(br_u(r, s->width_n_bits)  + 1);
    s->max_height    = (int)(br_u(r, s->height_n_bits) + 1);

    if (!s->reduced_still_picture_header) {
        s->frame_id_numbers_present = (uint8_t)br_bit(r);
        if (s->frame_id_numbers_present) {
            s->delta_frame_id_n_bits = (uint8_t)(br_u(r, 4) + 2);
            s->frame_id_n_bits       = (uint8_t)(br_u(r, 3) + s->delta_frame_id_n_bits + 1);
        }
    }

    s->sb128           = (uint8_t)br_bit(r);
    s->filter_intra    = (uint8_t)br_bit(r);
    s->intra_edge_filter = (uint8_t)br_bit(r);

    if (s->reduced_still_picture_header) {
        s->screen_content_tools = DAV1D_ADAPTIVE;
        s->force_integer_mv     = DAV1D_ADAPTIVE;
        s->inter_intra          = 0;
        s->masked_compound      = 0;
        s->warped_motion        = 0;
        s->dual_filter          = 0;
        s->order_hint           = 0;
        s->jnt_comp             = 0;
        s->ref_frame_mvs        = 0;
        s->order_hint_n_bits    = 0;
    } else {
        s->inter_intra    = (uint8_t)br_bit(r);
        s->masked_compound = (uint8_t)br_bit(r);
        s->warped_motion  = (uint8_t)br_bit(r);
        s->dual_filter    = (uint8_t)br_bit(r);
        s->order_hint     = (uint8_t)br_bit(r);
        if (s->order_hint) {
            s->jnt_comp      = (uint8_t)br_bit(r);
            s->ref_frame_mvs = (uint8_t)br_bit(r);
        }
        /* screen_content_tools: f(1) adaptive flag, else f(1) value */
        if (br_bit(r)) {
            s->screen_content_tools = DAV1D_ADAPTIVE;
        } else {
            s->screen_content_tools =
                (Dav1dAdaptiveBoolean)br_bit(r);
        }
        /* force_integer_mv: only present when screen_content_tools != OFF */
        if (s->screen_content_tools) {
            if (br_bit(r)) {
                s->force_integer_mv = DAV1D_ADAPTIVE;
            } else {
                s->force_integer_mv = (Dav1dAdaptiveBoolean)br_bit(r);
            }
        } else {
            s->force_integer_mv = DAV1D_ADAPTIVE; /* matches dav1d: value 2 */
        }
        if (s->order_hint)
            s->order_hint_n_bits = (uint8_t)(br_u(r, 3) + 1);
    }

    s->super_res   = (uint8_t)br_bit(r);
    s->cdef        = (uint8_t)br_bit(r);
    s->restoration = (uint8_t)br_bit(r);

    /* color config */
    s->hbd = (uint8_t)br_bit(r);
    if (s->profile == 2 && s->hbd)
        s->hbd = (uint8_t)(1 + br_bit(r));   /* 0=8b, 1=10b, 2=12b */
    if (s->profile != 1)
        s->monochrome = (uint8_t)br_bit(r);
    s->color_description_present = (uint8_t)br_bit(r);
    if (s->color_description_present) {
        s->pri  = (Dav1dColorPrimaries)br_u(r, 8);
        s->trc  = (Dav1dTransferCharacteristics)br_u(r, 8);
        s->mtrx = (Dav1dMatrixCoefficients)br_u(r, 8);
    } else {
        s->pri  = DAV1D_COLOR_PRI_UNKNOWN;
        s->trc  = DAV1D_TRC_UNKNOWN;
        s->mtrx = DAV1D_MC_UNKNOWN;
    }

    if (s->monochrome) {
        s->color_range = (uint8_t)br_bit(r);
        s->layout = DAV1D_PIXEL_LAYOUT_I400;
        s->ss_hor = 1;
        s->ss_ver = 1;
        s->chr    = DAV1D_CHR_UNKNOWN;
    } else if (s->pri  == DAV1D_COLOR_PRI_BT709 &&
               s->trc  == DAV1D_TRC_SRGB &&
               s->mtrx == DAV1D_MC_IDENTITY) {
        s->layout = DAV1D_PIXEL_LAYOUT_I444;
        s->color_range = 1;
        /* profile 0 with RGB identity is non-conformant */
        if (s->profile != 1 && !(s->profile == 2 && s->hbd == 2))
            return -1;
    } else {
        s->color_range = (uint8_t)br_bit(r);
        switch (s->profile) {
        case 0:
            s->layout = DAV1D_PIXEL_LAYOUT_I420;
            s->ss_hor = 1; s->ss_ver = 1;
            break;
        case 1:
            s->layout = DAV1D_PIXEL_LAYOUT_I444;
            break;
        case 2:
            if (s->hbd == 2) {
                s->ss_hor = (uint8_t)br_bit(r);
                if (s->ss_hor)
                    s->ss_ver = (uint8_t)br_bit(r);
            } else {
                s->ss_hor = 1;
            }
            s->layout = s->ss_hor ? (s->ss_ver ? DAV1D_PIXEL_LAYOUT_I420
                                                : DAV1D_PIXEL_LAYOUT_I422)
                                  : DAV1D_PIXEL_LAYOUT_I444;
            break;
        }
        if (s->ss_hor & s->ss_ver)
            s->chr = (Dav1dChromaSamplePosition)br_u(r, 2);
        else
            s->chr = DAV1D_CHR_UNKNOWN;
    }

    if (!s->monochrome)
        s->separate_uv_delta_q = (uint8_t)br_bit(r);

    s->film_grain_present = (uint8_t)br_bit(r);

    if (r.error) return -1;
    return 0;
}

/* =========================================================================
 * Helpers shared between seq and frame parsing
 * ========================================================================= */

/* Helpers for IS_KEY_OR_INTRA / IS_INTER_OR_SWITCH (dav1d levels.h). */
static inline bool is_key_or_intra(const Dav1dFrameHeader *h)
{
    return h->frame_type == DAV1D_FRAME_TYPE_KEY ||
           h->frame_type == DAV1D_FRAME_TYPE_INTRA;
}

static inline bool is_inter_or_switch(const Dav1dFrameHeader *h)
{
    return h->frame_type == DAV1D_FRAME_TYPE_INTER ||
           h->frame_type == DAV1D_FRAME_TYPE_SWITCH;
}

/* AV1 spec §7.8 get_relative_dist(). */
static int32_t get_relative_dist(int bits, int32_t a, int32_t b)
{
    if (!bits) return 0;
    int32_t m = 1 << (bits - 1);
    int32_t diff = (a - b) & ((1 << bits) - 1);
    return diff - ((diff & m) << 1);
}

/* Tile size computation helper: smallest k such that (sz << k) >= tgt. */
static int tile_log2(int sz, int tgt)
{
    int k = 0;
    while ((sz << k) < tgt) k++;
    return k;
}

static inline int imax(int a, int b) { return a > b ? a : b; }
static inline int imin(int a, int b) { return a < b ? a : b; }
static inline uint8_t iclip_u8(int v) {
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

/* Default loop-filter mode/ref deltas (AV1 spec Table 4). */
static const Dav1dLoopfilterModeRefDeltas k_default_mode_ref_deltas = {
    /* mode_delta[2] */ { 0, 0 },
    /* ref_delta[8]  */ { 1, 0, 0, 0, -1, 0, -1, -1 },
};

/* =========================================================================
 * Frame header parser  (AV1 spec §5.9)
 * ========================================================================= */

static int parse_frame_size(BitReader &r,
                            const Dav1dSequenceHeader *seq,
                            Dav1dFrameHeader *h,
                            const Av1SavedFrameState prev[8],
                            bool use_ref)
{
    if (use_ref && prev) {
        for (int i = 0; i < 7; i++) {
            if (br_bit(r)) {
                int slot = h->refidx[i];
                if (slot < 0 || slot >= 8 || !prev[slot].valid)
                    return -1;
                const Dav1dFrameHeader *ref = &prev[slot].saved;
                h->width[1] = ref->width[1];
                h->height   = ref->height;
                /* super_res */
                h->super_res.enabled = seq->super_res && (uint8_t)br_bit(r);
                if (h->super_res.enabled) {
                    int d = (int)(9 + br_u(r, 3));
                    h->super_res.width_scale_denominator = (uint8_t)d;
                    h->width[0] = imax((h->width[1] * 8 + (d >> 1)) / d,
                                       imin(16, h->width[1]));
                } else {
                    h->super_res.width_scale_denominator = 8;
                    h->width[0] = h->width[1];
                }
                return 0;
            }
        }
    }

    if (h->frame_size_override) {
        h->width[1] = (int)(br_u(r, seq->width_n_bits)  + 1);
        h->height   = (int)(br_u(r, seq->height_n_bits) + 1);
    } else {
        h->width[1] = seq->max_width;
        h->height   = seq->max_height;
    }

    h->super_res.enabled = seq->super_res && (uint8_t)br_bit(r);
    if (h->super_res.enabled) {
        int d = (int)(9 + br_u(r, 3));
        h->super_res.width_scale_denominator = (uint8_t)d;
        h->width[0] = imax((h->width[1] * 8 + (d >> 1)) / d,
                           imin(16, h->width[1]));
    } else {
        h->super_res.width_scale_denominator = 8;
        h->width[0] = h->width[1];
    }

    h->have_render_size = (uint8_t)br_bit(r);
    if (h->have_render_size) {
        h->render_width  = (int)(br_u(r, 16) + 1);
        h->render_height = (int)(br_u(r, 16) + 1);
    } else {
        h->render_width  = h->width[1];
        h->render_height = h->height;
    }
    return 0;
}

/* Compute skip_mode_refs based on ref frame order hints.
 * prev[] required; if not available, skip_mode stays 0. */
static void compute_skip_mode(const Dav1dSequenceHeader *seq,
                               Dav1dFrameHeader *h,
                               const Av1SavedFrameState prev[8])
{
    if (!seq->order_hint || !prev) return;

    int ob = -1, oa = -1;
    int ob_idx = -1, oa_idx = -1;

    for (int i = 0; i < 7; i++) {
        int slot = h->refidx[i];
        if (slot < 0 || slot >= 8 || !prev[slot].valid) continue;
        int refpoc = prev[slot].saved.frame_offset;
        int diff = get_relative_dist(seq->order_hint_n_bits, refpoc,
                                     h->frame_offset);
        if (diff > 0) {
            /* Future ref: keep the CLOSEST one (smallest hint).
             * BSP av1d_codec.c read_skip_mode_params: update when
             * dist(refpoc, backward_hint) < 0  →  refpoc < current best. */
            if (oa < 0 ||
                get_relative_dist(seq->order_hint_n_bits, refpoc, oa) < 0)
            {
                oa = refpoc; oa_idx = i;
            }
        } else if (diff < 0) {
            /* Past ref: keep the CLOSEST one (largest hint = closer to
             * current).  Update when dist(refpoc, forward_hint) > 0. */
            if (ob < 0 ||
                get_relative_dist(seq->order_hint_n_bits, refpoc, ob) > 0)
            {
                ob = refpoc; ob_idx = i;
            }
        }
    }

    if ((ob | oa) >= 0) {
        h->skip_mode_refs[0] = (int8_t)imin(ob_idx, oa_idx);
        h->skip_mode_refs[1] = (int8_t)imax(ob_idx, oa_idx);
        h->skip_mode_allowed = 1;
    } else if (ob >= 0) {
        int ob2 = -1, ob2_idx = -1;
        for (int i = 0; i < 7; i++) {
            int slot = h->refidx[i];
            if (slot < 0 || slot >= 8 || !prev[slot].valid) continue;
            int refpoc = prev[slot].saved.frame_offset;
            if (get_relative_dist(seq->order_hint_n_bits, refpoc, ob) < 0) {
                if (ob2 < 0 ||
                    get_relative_dist(seq->order_hint_n_bits, refpoc, ob2) > 0)
                {
                    ob2 = refpoc; ob2_idx = i;
                }
            }
        }
        if (ob2 >= 0) {
            h->skip_mode_refs[0] = (int8_t)imin(ob_idx, ob2_idx);
            h->skip_mode_refs[1] = (int8_t)imax(ob_idx, ob2_idx);
            h->skip_mode_allowed = 1;
        }
    }
}

static int parse_frame_hdr(BitReader &r,
                            const Dav1dSequenceHeader *seq,
                            const Av1SavedFrameState   prev[8],
                            Dav1dFrameHeader *h)
{
    memset(h, 0, sizeof(*h));

    /* show_existing_frame */
    if (!seq->reduced_still_picture_header)
        h->show_existing_frame = (uint8_t)br_bit(r);

    if (h->show_existing_frame) {
        h->existing_frame_idx = (uint8_t)br_u(r, 3);
        /* frame_to_show_idx in dav1d parlance = existing_frame_idx */
        if (seq->decoder_model_info_present && !seq->equal_picture_interval)
            h->frame_presentation_delay = br_u(r, seq->frame_presentation_delay_length);
        if (seq->frame_id_numbers_present)
            h->frame_id = br_u(r, seq->frame_id_n_bits);

        /* Width/height/frame_offset come from the referenced slot. */
        int slot = h->existing_frame_idx;
        if (prev && slot >= 0 && slot < 8 && prev[slot].valid) {
            h->width[0]    = prev[slot].saved.width[0];
            h->width[1]    = prev[slot].saved.width[1];
            h->height      = prev[slot].saved.height;
            h->frame_offset = prev[slot].saved.frame_offset;
        }
        /* frame_hdr_obu_size_bytes will be set by caller from bit position. */
        if (r.error) return -1;
        return 0;
    }

    /* frame_type / show_frame */
    if (seq->reduced_still_picture_header) {
        h->frame_type = DAV1D_FRAME_TYPE_KEY;
        h->show_frame = 1;
    } else {
        h->frame_type = (Dav1dFrameType)br_u(r, 2);
        h->show_frame = (uint8_t)br_bit(r);
    }

    if (h->show_frame) {
        if (seq->decoder_model_info_present && !seq->equal_picture_interval)
            h->frame_presentation_delay = br_u(r, seq->frame_presentation_delay_length);
        h->showable_frame = (h->frame_type != DAV1D_FRAME_TYPE_KEY) ? 1 : 0;
    } else {
        h->showable_frame = (uint8_t)br_bit(r);
    }

    /* error_resilient_mode */
    bool forced_err_res = (h->frame_type == DAV1D_FRAME_TYPE_KEY && h->show_frame) ||
                          (h->frame_type == DAV1D_FRAME_TYPE_SWITCH) ||
                          seq->reduced_still_picture_header;
    h->error_resilient_mode = forced_err_res ? 1 : (uint8_t)br_bit(r);

    h->disable_cdf_update = (uint8_t)br_bit(r);

    /* allow_screen_content_tools */
    if (seq->screen_content_tools == DAV1D_ADAPTIVE)
        h->allow_screen_content_tools = (uint8_t)br_bit(r);
    else
        h->allow_screen_content_tools = (uint8_t)(int)seq->screen_content_tools;

    /* force_integer_mv */
    if (h->allow_screen_content_tools) {
        if (seq->force_integer_mv == DAV1D_ADAPTIVE)
            h->force_integer_mv = (uint8_t)br_bit(r);
        else
            h->force_integer_mv = (uint8_t)(int)seq->force_integer_mv;
    }
    if (is_key_or_intra(h))
        h->force_integer_mv = 1;

    /* frame_id */
    if (seq->frame_id_numbers_present)
        h->frame_id = br_u(r, seq->frame_id_n_bits);

    /* frame_size_override */
    if (!seq->reduced_still_picture_header) {
        h->frame_size_override =
            (h->frame_type == DAV1D_FRAME_TYPE_SWITCH)
                ? 1
                : (uint8_t)br_bit(r);
    }

    /* order hint / frame_offset */
    if (seq->order_hint)
        h->frame_offset = (uint8_t)br_u(r, seq->order_hint_n_bits);

    /* primary_ref_frame */
    h->primary_ref_frame =
        (!h->error_resilient_mode && is_inter_or_switch(h))
            ? (uint8_t)br_u(r, 3)
            : DAV1D_PRIMARY_REF_NONE;

    /* buffer_removal_time */
    if (seq->decoder_model_info_present) {
        h->buffer_removal_time_present = (uint8_t)br_bit(r);
        if (h->buffer_removal_time_present) {
            for (int i = 0; i < seq->num_operating_points; i++) {
                const auto &sop = seq->operating_points[i];
                if (sop.decoder_model_param_present) {
                    int in_tl = (sop.idc >> h->temporal_id) & 1;
                    int in_sl = (sop.idc >> (h->spatial_id + 8)) & 1;
                    if (!sop.idc || (in_tl && in_sl))
                        h->operating_points[i].buffer_removal_time =
                            br_u(r, seq->buffer_removal_delay_length);
                }
            }
        }
    }

    /* refresh_frame_flags */
    if (is_key_or_intra(h)) {
        h->refresh_frame_flags =
            (h->frame_type == DAV1D_FRAME_TYPE_KEY && h->show_frame)
                ? 0xff
                : (uint8_t)br_u(r, 8);
        /* error_resilient + order_hint: skip 8 order_hint values */
        if (h->refresh_frame_flags != 0xff && h->error_resilient_mode &&
            seq->order_hint)
        {
            for (int i = 0; i < 8; i++)
                br_u(r, seq->order_hint_n_bits);
        }
    } else {
        h->refresh_frame_flags =
            (h->frame_type == DAV1D_FRAME_TYPE_SWITCH)
                ? 0xff
                : (uint8_t)br_u(r, 8);
        if (h->error_resilient_mode && seq->order_hint)
            for (int i = 0; i < 8; i++)
                br_u(r, seq->order_hint_n_bits);
    }

    /* Reference frame indices for inter frames */
    if (is_inter_or_switch(h)) {
        /* frame_ref_short_signaling (§5.9.5) — only when order_hint enabled */
        if (seq->order_hint) {
            h->frame_ref_short_signaling = (uint8_t)br_bit(r);
            if (h->frame_ref_short_signaling) {
                /* Read LAST_FRAME and GOLDEN_FRAME indices explicitly;
                 * derive the rest via §7.8 get_relative_dist() ordering.
                 * This mirrors dav1d's algorithm. */
                h->refidx[0] = (int8_t)br_u(r, 3);  /* LAST */
                h->refidx[1] = h->refidx[2] = -1;
                h->refidx[3] = (int8_t)br_u(r, 3);  /* GOLDEN */

                /* Build per-slot frame_offset distance from current frame. */
                int frame_offs[8];
                int earliest_ref = -1;
                int earliest_off = INT_MAX;
                for (int i = 0; i < 8; i++) {
                    frame_offs[i] = INT_MAX; /* "not available" */
                    if (prev && prev[i].valid) {
                        frame_offs[i] = get_relative_dist(
                            seq->order_hint_n_bits,
                            prev[i].saved.frame_offset,
                            h->frame_offset);
                        if (frame_offs[i] < earliest_off) {
                            earliest_off = frame_offs[i];
                            earliest_ref = i;
                        }
                    }
                }
                /* Mark used slots so they aren't reused */
                frame_offs[h->refidx[0]] = INT_MIN;
                frame_offs[h->refidx[3]] = INT_MIN;

                /* ALTREF: pick latest */
                int ridx = -1;
                for (int i = 0, lo = 0; i < 8; i++) {
                    if (frame_offs[i] != INT_MIN && frame_offs[i] != INT_MAX &&
                        frame_offs[i] >= lo)
                    {
                        lo = frame_offs[i]; ridx = i;
                    }
                }
                if (ridx >= 0) { frame_offs[ridx] = INT_MIN; h->refidx[6] = (int8_t)ridx; }

                /* BWDREF (4), ALTREF2 (5): pick two smallest positive */
                for (int k = 4; k <= 5; k++) {
                    unsigned eo = UINT_MAX; ridx = -1;
                    for (int i = 0; i < 8; i++) {
                        if (frame_offs[i] != INT_MIN && frame_offs[i] != INT_MAX) {
                            unsigned hint = (unsigned)frame_offs[i];
                            if (hint < eo) { eo = hint; ridx = i; }
                        }
                    }
                    if (ridx >= 0) { frame_offs[ridx] = INT_MIN; h->refidx[k] = (int8_t)ridx; }
                }

                /* LAST2 (1), LAST3 (2): fill remaining with latest available */
                for (int k = 1; k < 7; k++) {
                    if (h->refidx[k] >= 0) continue;
                    unsigned lo = ~0u; ridx = -1;
                    for (int i = 0; i < 8; i++) {
                        if (frame_offs[i] != INT_MIN && frame_offs[i] != INT_MAX) {
                            unsigned hint = (unsigned)frame_offs[i];
                            if (hint >= lo) { lo = hint; ridx = i; }
                        }
                    }
                    if (ridx < 0) ridx = earliest_ref >= 0 ? earliest_ref : 0;
                    h->refidx[k] = (int8_t)ridx;
                    if (ridx >= 0) frame_offs[ridx] = INT_MIN;
                }
            }
        }

        /* Explicit refidx if not short-signaled */
        for (int i = 0; i < 7; i++) {
            if (!h->frame_ref_short_signaling)
                h->refidx[i] = (int8_t)br_u(r, 3);
            if (seq->frame_id_numbers_present)
                br_u(r, seq->delta_frame_id_n_bits); /* delta_frame_id; skip */
        }
    } else {
        /* Key/intra: no refs */
        for (int i = 0; i < 7; i++) h->refidx[i] = -1;
    }

    /* Frame size */
    if (is_key_or_intra(h)) {
        if (parse_frame_size(r, seq, h, prev, false) < 0) return -1;
        if (h->allow_screen_content_tools && !h->super_res.enabled)
            h->allow_intrabc = (uint8_t)br_bit(r);
    } else {
        bool use_ref = !h->error_resilient_mode && h->frame_size_override;
        if (parse_frame_size(r, seq, h, prev, use_ref) < 0) return -1;
        if (!h->force_integer_mv)
            h->hp = (uint8_t)br_bit(r);
        h->subpel_filter_mode = br_bit(r) ? DAV1D_FILTER_SWITCHABLE
                                           : (Dav1dFilterMode)br_u(r, 2);
        h->switchable_motion_mode = (uint8_t)br_bit(r);
        if (!h->error_resilient_mode && seq->ref_frame_mvs &&
            seq->order_hint && is_inter_or_switch(h))
        {
            h->use_ref_frame_mvs = (uint8_t)br_bit(r);
        }
    }

    /* refresh_context (disable_frame_end_update_cdf) */
    if (!seq->reduced_still_picture_header && !h->disable_cdf_update)
        h->refresh_context = (uint8_t)!br_bit(r);

    /* -----------------------------------------------------------------------
     * Tiling  (§5.9.15)
     * --------------------------------------------------------------------- */
    h->tiling.uniform = (uint8_t)br_bit(r);

    int sbsz_log2 = 6 + seq->sb128;
    int sbsz_min1 = (1 << sbsz_log2) - 1;
    int sbw = (h->width[0]  + sbsz_min1) >> sbsz_log2;
    int sbh = (h->height + sbsz_min1) >> sbsz_log2;
    int max_tile_width_sb  = 4096 >> sbsz_log2;
    int max_tile_area_sb   = (4096 * 2304) >> (2 * sbsz_log2);

    h->tiling.min_log2_cols = tile_log2(max_tile_width_sb, sbw);
    h->tiling.max_log2_cols = tile_log2(1, imin(sbw, DAV1D_MAX_TILE_COLS));
    h->tiling.max_log2_rows = tile_log2(1, imin(sbh, DAV1D_MAX_TILE_ROWS));
    int min_log2_tiles = imax(tile_log2(max_tile_area_sb, sbw * sbh),
                              h->tiling.min_log2_cols);

    if (h->tiling.uniform) {
        h->tiling.log2_cols = h->tiling.min_log2_cols;
        while (h->tiling.log2_cols < h->tiling.max_log2_cols && br_bit(r))
            h->tiling.log2_cols++;
        int tile_w = 1 + ((sbw - 1) >> h->tiling.log2_cols);
        h->tiling.cols = 0;
        for (int sbx = 0; sbx < sbw; sbx += tile_w, h->tiling.cols++)
            h->tiling.col_start_sb[h->tiling.cols] = (uint16_t)sbx;

        h->tiling.min_log2_rows =
            (uint8_t)imax(min_log2_tiles - h->tiling.log2_cols, 0);
        h->tiling.log2_rows = h->tiling.min_log2_rows;
        while (h->tiling.log2_rows < h->tiling.max_log2_rows && br_bit(r))
            h->tiling.log2_rows++;
        int tile_h = 1 + ((sbh - 1) >> h->tiling.log2_rows);
        h->tiling.rows = 0;
        for (int sby = 0; sby < sbh; sby += tile_h, h->tiling.rows++)
            h->tiling.row_start_sb[h->tiling.rows] = (uint16_t)sby;
    } else {
        h->tiling.cols = 0;
        int widest_tile = 0, max_ta = sbw * sbh;
        for (int sbx = 0; sbx < sbw && h->tiling.cols < DAV1D_MAX_TILE_COLS;
             h->tiling.cols++)
        {
            int tw = imin(sbw - sbx, max_tile_width_sb);
            int w  = (tw > 1) ? 1 + (int)br_uniform(r, (uint32_t)tw) : 1;
            h->tiling.col_start_sb[h->tiling.cols] = (uint16_t)sbx;
            sbx += w;
            widest_tile = imax(widest_tile, w);
        }
        h->tiling.log2_cols = (uint8_t)tile_log2(1, h->tiling.cols);
        if (min_log2_tiles) max_ta >>= min_log2_tiles + 1;
        int max_th = imax(max_ta / imax(widest_tile, 1), 1);

        h->tiling.rows = 0;
        for (int sby = 0; sby < sbh && h->tiling.rows < DAV1D_MAX_TILE_ROWS;
             h->tiling.rows++)
        {
            int th = imin(sbh - sby, max_th);
            int hh = (th > 1) ? 1 + (int)br_uniform(r, (uint32_t)th) : 1;
            h->tiling.row_start_sb[h->tiling.rows] = (uint16_t)sby;
            sby += hh;
        }
        h->tiling.log2_rows = (uint8_t)tile_log2(1, h->tiling.rows);
    }

    h->tiling.col_start_sb[h->tiling.cols] = (uint16_t)sbw;
    h->tiling.row_start_sb[h->tiling.rows] = (uint16_t)sbh;

    if (h->tiling.log2_cols || h->tiling.log2_rows) {
        h->tiling.update = (uint16_t)br_u(r, h->tiling.log2_cols + h->tiling.log2_rows);
        if (h->tiling.update >= (uint16_t)(h->tiling.cols * h->tiling.rows))
            return -1;
        h->tiling.n_bytes = (uint8_t)(br_u(r, 2) + 1);
    }

    /* -----------------------------------------------------------------------
     * Quantization  (§5.9.12)
     * --------------------------------------------------------------------- */
    h->quant.yac = (uint8_t)br_u(r, 8);
    h->quant.ydc_delta = (int8_t)br_delta_q(r, 7);
    if (!seq->monochrome) {
        int diff_uv = seq->separate_uv_delta_q ? (int)br_bit(r) : 0;
        h->quant.udc_delta = (int8_t)br_delta_q(r, 7);
        h->quant.uac_delta = (int8_t)br_delta_q(r, 7);
        if (diff_uv) {
            h->quant.vdc_delta = (int8_t)br_delta_q(r, 7);
            h->quant.vac_delta = (int8_t)br_delta_q(r, 7);
        } else {
            h->quant.vdc_delta = h->quant.udc_delta;
            h->quant.vac_delta = h->quant.uac_delta;
        }
    }
    h->quant.qm = (uint8_t)br_bit(r);
    if (h->quant.qm) {
        h->quant.qm_y = (uint8_t)br_u(r, 4);
        h->quant.qm_u = (uint8_t)br_u(r, 4);
        h->quant.qm_v = seq->separate_uv_delta_q
                            ? (uint8_t)br_u(r, 4)
                            : h->quant.qm_u;
    }

    /* -----------------------------------------------------------------------
     * Segmentation  (§5.9.14)
     * --------------------------------------------------------------------- */
    h->segmentation.enabled = (uint8_t)br_bit(r);
    if (h->segmentation.enabled) {
        if (h->primary_ref_frame == DAV1D_PRIMARY_REF_NONE) {
            h->segmentation.update_map  = 1;
            h->segmentation.update_data = 1;
        } else {
            h->segmentation.update_map = (uint8_t)br_bit(r);
            if (h->segmentation.update_map)
                h->segmentation.temporal = (uint8_t)br_bit(r);
            h->segmentation.update_data = (uint8_t)br_bit(r);
        }

        if (h->segmentation.update_data) {
            h->segmentation.seg_data.last_active_segid = -1;
            for (int i = 0; i < DAV1D_MAX_SEGMENTS; i++) {
                auto &seg = h->segmentation.seg_data.d[i];
                if (br_bit(r)) {
                    seg.delta_q = (int16_t)br_su(r, 9);
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                }
                if (br_bit(r)) {
                    seg.delta_lf_y_v = (int8_t)br_su(r, 7);
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                }
                if (br_bit(r)) {
                    seg.delta_lf_y_h = (int8_t)br_su(r, 7);
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                }
                if (br_bit(r)) {
                    seg.delta_lf_u = (int8_t)br_su(r, 7);
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                }
                if (br_bit(r)) {
                    seg.delta_lf_v = (int8_t)br_su(r, 7);
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                }
                if (br_bit(r)) {
                    seg.ref = (int8_t)br_u(r, 3);
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                    h->segmentation.seg_data.preskip = 1;
                } else {
                    seg.ref = -1;
                }
                if ((seg.skip = (uint8_t)br_bit(r))) {
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                    h->segmentation.seg_data.preskip = 1;
                }
                if ((seg.globalmv = (uint8_t)br_bit(r))) {
                    h->segmentation.seg_data.last_active_segid = (int8_t)i;
                    h->segmentation.seg_data.preskip = 1;
                }
            }
        } else {
            /* Inherit from primary_ref_frame slot */
            assert(h->primary_ref_frame != DAV1D_PRIMARY_REF_NONE);
            int pri_slot = h->refidx[h->primary_ref_frame];
            if (prev && pri_slot >= 0 && pri_slot < 8 && prev[pri_slot].valid) {
                h->segmentation.seg_data =
                    prev[pri_slot].saved.segmentation.seg_data;
            } else {
                /* No reference available — set all refs to -1 */
                for (int i = 0; i < DAV1D_MAX_SEGMENTS; i++)
                    h->segmentation.seg_data.d[i].ref = -1;
            }
        }
    } else {
        for (int i = 0; i < DAV1D_MAX_SEGMENTS; i++)
            h->segmentation.seg_data.d[i].ref = -1;
    }

    /* -----------------------------------------------------------------------
     * Delta Q / Delta LF  (§5.9.17)
     * --------------------------------------------------------------------- */
    if (h->quant.yac) {
        h->delta.q.present = (uint8_t)br_bit(r);
        if (h->delta.q.present) {
            h->delta.q.res_log2 = (uint8_t)br_u(r, 2);
            if (!h->allow_intrabc) {
                h->delta.lf.present = (uint8_t)br_bit(r);
                if (h->delta.lf.present) {
                    h->delta.lf.res_log2 = (uint8_t)br_u(r, 2);
                    h->delta.lf.multi    = (uint8_t)br_bit(r);
                }
            }
        }
    }

    /* -----------------------------------------------------------------------
     * Derive lossless flags
     * --------------------------------------------------------------------- */
    int delta_lossless = !h->quant.ydc_delta && !h->quant.udc_delta &&
                         !h->quant.uac_delta && !h->quant.vdc_delta &&
                         !h->quant.vac_delta;
    h->all_lossless = 1;
    for (int i = 0; i < DAV1D_MAX_SEGMENTS; i++) {
        h->segmentation.qidx[i] = h->segmentation.enabled
            ? iclip_u8(h->quant.yac + h->segmentation.seg_data.d[i].delta_q)
            : h->quant.yac;
        h->segmentation.lossless[i] = !h->segmentation.qidx[i] && delta_lossless;
        h->all_lossless &= h->segmentation.lossless[i];
    }

    /* -----------------------------------------------------------------------
     * Loop filter  (§5.9.11)
     * --------------------------------------------------------------------- */
    if (h->all_lossless || h->allow_intrabc) {
        h->loopfilter.mode_ref_delta_enabled = 1;
        h->loopfilter.mode_ref_delta_update  = 1;
        h->loopfilter.mode_ref_deltas = k_default_mode_ref_deltas;
    } else {
        h->loopfilter.level_y[0] = (uint8_t)br_u(r, 6);
        h->loopfilter.level_y[1] = (uint8_t)br_u(r, 6);
        if (!seq->monochrome &&
            (h->loopfilter.level_y[0] || h->loopfilter.level_y[1]))
        {
            h->loopfilter.level_u = (uint8_t)br_u(r, 6);
            h->loopfilter.level_v = (uint8_t)br_u(r, 6);
        }
        h->loopfilter.sharpness = (uint8_t)br_u(r, 3);

        /* Inherit mode_ref_deltas from primary_ref_frame */
        if (h->primary_ref_frame == DAV1D_PRIMARY_REF_NONE) {
            h->loopfilter.mode_ref_deltas = k_default_mode_ref_deltas;
        } else {
            int pri_slot = h->refidx[h->primary_ref_frame];
            if (prev && pri_slot >= 0 && pri_slot < 8 && prev[pri_slot].valid) {
                h->loopfilter.mode_ref_deltas =
                    prev[pri_slot].saved.loopfilter.mode_ref_deltas;
            } else {
                h->loopfilter.mode_ref_deltas = k_default_mode_ref_deltas;
            }
        }

        h->loopfilter.mode_ref_delta_enabled = (uint8_t)br_bit(r);
        if (h->loopfilter.mode_ref_delta_enabled) {
            h->loopfilter.mode_ref_delta_update = (uint8_t)br_bit(r);
            if (h->loopfilter.mode_ref_delta_update) {
                for (int i = 0; i < 8; i++)
                    if (br_bit(r))
                        h->loopfilter.mode_ref_deltas.ref_delta[i] =
                            (int8_t)br_su(r, 7);
                for (int i = 0; i < 2; i++)
                    if (br_bit(r))
                        h->loopfilter.mode_ref_deltas.mode_delta[i] =
                            (int8_t)br_su(r, 7);
            }
        }
    }

    /* -----------------------------------------------------------------------
     * CDEF  (§5.9.19)
     * --------------------------------------------------------------------- */
    if (!h->all_lossless && seq->cdef && !h->allow_intrabc) {
        h->cdef.damping = (uint8_t)(br_u(r, 2) + 3);
        h->cdef.n_bits  = (uint8_t)br_u(r, 2);
        for (int i = 0; i < (1 << h->cdef.n_bits); i++) {
            h->cdef.y_strength[i]  = (uint8_t)br_u(r, 6);
            if (!seq->monochrome)
                h->cdef.uv_strength[i] = (uint8_t)br_u(r, 6);
        }
    }

    /* -----------------------------------------------------------------------
     * Loop restoration  (§5.9.20)
     * --------------------------------------------------------------------- */
    if ((!h->all_lossless || h->super_res.enabled) &&
        seq->restoration && !h->allow_intrabc)
    {
        h->restoration.type[0] = (Dav1dRestorationType)br_u(r, 2);
        if (!seq->monochrome) {
            h->restoration.type[1] = (Dav1dRestorationType)br_u(r, 2);
            h->restoration.type[2] = (Dav1dRestorationType)br_u(r, 2);
        }

        if (h->restoration.type[0] || h->restoration.type[1] ||
            h->restoration.type[2])
        {
            h->restoration.unit_size[0] = (uint8_t)(6 + seq->sb128);
            if (br_bit(r)) {
                h->restoration.unit_size[0]++;
                if (!seq->sb128)
                    h->restoration.unit_size[0] += (uint8_t)br_bit(r);
            }
            h->restoration.unit_size[1] = h->restoration.unit_size[0];
            if ((h->restoration.type[1] || h->restoration.type[2]) &&
                seq->ss_hor == 1 && seq->ss_ver == 1)
            {
                h->restoration.unit_size[1] -= (uint8_t)br_bit(r);
            }
        } else {
            h->restoration.unit_size[0] = 8;
        }
    }

    /* -----------------------------------------------------------------------
     * TX mode  (§5.9.21)
     * --------------------------------------------------------------------- */
    if (!h->all_lossless)
        h->txfm_mode = br_bit(r) ? DAV1D_TX_SWITCHABLE : DAV1D_TX_LARGEST;

    /* -----------------------------------------------------------------------
     * Reference mode / skip mode  (§5.9.22 / §5.9.23)
     * --------------------------------------------------------------------- */
    if (is_inter_or_switch(h))
        h->switchable_comp_refs = (uint8_t)br_bit(r);

    if (h->switchable_comp_refs && is_inter_or_switch(h) && seq->order_hint)
        compute_skip_mode(seq, h, prev);

    if (h->skip_mode_allowed)
        h->skip_mode_enabled = (uint8_t)br_bit(r);

    /* warp_motion */
    if (!h->error_resilient_mode && is_inter_or_switch(h) && seq->warped_motion)
        h->warp_motion = (uint8_t)br_bit(r);

    h->reduced_txtp_set = (uint8_t)br_bit(r);

    /* -----------------------------------------------------------------------
     * Global motion vectors  (§5.9.24)
     * --------------------------------------------------------------------- */
    /* Default identity matrix: mat[2]=mat[5]=1<<16, rest zero */
    static const int32_t k_identity_mat[6] = { 0, 0, 1<<16, 0, 0, 1<<16 };
    for (int i = 0; i < 7; i++) {
        h->gmv[i].type = DAV1D_WM_TYPE_IDENTITY;
        memcpy(h->gmv[i].matrix, k_identity_mat, sizeof(k_identity_mat));
        memset(&h->gmv[i].u, 0, sizeof(h->gmv[i].u));
    }

    if (is_inter_or_switch(h)) {
        /* Reference GMV for primary_ref_frame-relative coding */
        for (int i = 0; i < 7; i++) {
            /* is_global */
            if (!br_bit(r)) continue;  /* stays IDENTITY */

            h->gmv[i].type = br_bit(r) ? DAV1D_WM_TYPE_ROT_ZOOM :
                             (br_bit(r) ? DAV1D_WM_TYPE_TRANSLATION
                                        : DAV1D_WM_TYPE_AFFINE);

            /* Fetch reference GMV matrix for relative coding */
            const int32_t *ref_mat = k_identity_mat;
            if (h->primary_ref_frame != DAV1D_PRIMARY_REF_NONE) {
                int pri_slot = h->refidx[h->primary_ref_frame];
                if (prev && pri_slot >= 0 && pri_slot < 8 &&
                    prev[pri_slot].valid)
                {
                    ref_mat = prev[pri_slot].saved.gmv[i].matrix;
                }
            }

            int32_t *mat = h->gmv[i].matrix;
            int bits, shift;

            if (h->gmv[i].type >= DAV1D_WM_TYPE_ROT_ZOOM) {
                mat[2] = (1 << 16) + 2 * br_subexp(r, (ref_mat[2] - (1 << 16)) >> 1, 12);
                mat[3] = 2 * br_subexp(r, ref_mat[3] >> 1, 12);
                bits  = 12;
                shift = 10;
            } else {
                bits  = 9 - !h->hp;
                shift = 13 + !h->hp;
            }

            if (h->gmv[i].type == DAV1D_WM_TYPE_AFFINE) {
                mat[4] = 2 * br_subexp(r, ref_mat[4] >> 1, 12);
                mat[5] = (1 << 16) + 2 * br_subexp(r, (ref_mat[5] - (1 << 16)) >> 1, 12);
            } else {
                mat[4] = -mat[3];
                mat[5] =  mat[2];
            }

            mat[0] = br_subexp(r, ref_mat[0] >> shift, (uint32_t)bits) * (1 << shift);
            mat[1] = br_subexp(r, ref_mat[1] >> shift, (uint32_t)bits) * (1 << shift);
        }
    }

    /* -----------------------------------------------------------------------
     * Film grain  (§5.9.30)
     * --------------------------------------------------------------------- */
    if (seq->film_grain_present && (h->show_frame || h->showable_frame)) {
        h->film_grain.present = (uint8_t)br_bit(r);
        if (h->film_grain.present) {
            unsigned seed = br_u(r, 16);
            h->film_grain.update =
                (h->frame_type != DAV1D_FRAME_TYPE_INTER) ? 1
                                                           : (uint8_t)br_bit(r);
            if (!h->film_grain.update) {
                /* Copy from a reference frame (3-bit index) */
                br_u(r, 3); /* refidx — we don't track the referenced grain */
                /* h->film_grain.data left zero */
                h->film_grain.data.seed = seed;
            } else {
                auto &fgd = h->film_grain.data;
                fgd.seed = seed;

                fgd.num_y_points = (int)br_u(r, 4);
                if (fgd.num_y_points > 14) return -1;
                for (int i = 0; i < fgd.num_y_points; i++) {
                    fgd.y_points[i][0] = (uint8_t)br_u(r, 8);
                    fgd.y_points[i][1] = (uint8_t)br_u(r, 8);
                }

                if (!seq->monochrome)
                    fgd.chroma_scaling_from_luma = (int)br_bit(r);
                if (seq->monochrome || fgd.chroma_scaling_from_luma ||
                    (seq->ss_hor == 1 && seq->ss_ver == 1 && !fgd.num_y_points))
                {
                    fgd.num_uv_points[0] = fgd.num_uv_points[1] = 0;
                } else {
                    for (int pl = 0; pl < 2; pl++) {
                        fgd.num_uv_points[pl] = (int)br_u(r, 4);
                        if (fgd.num_uv_points[pl] > 10) return -1;
                        for (int i = 0; i < fgd.num_uv_points[pl]; i++) {
                            fgd.uv_points[pl][i][0] = (uint8_t)br_u(r, 8);
                            fgd.uv_points[pl][i][1] = (uint8_t)br_u(r, 8);
                        }
                    }
                }

                fgd.scaling_shift = (int)br_u(r, 2) + 8;
                fgd.ar_coeff_lag  = (int)br_u(r, 2);
                int num_y_pos = 2 * fgd.ar_coeff_lag * (fgd.ar_coeff_lag + 1);
                /* Defensive bound: dav1d's ar_coeffs_y[] is sized 24,
                 * ar_coeffs_uv[2][25+3] (padding).  With ar_coeff_lag's
                 * 2-bit range, num_y_pos is at most 24 and num_uv at
                 * most 25 — both fit.  This assert guards against a
                 * future widening of the field silently corrupting
                 * adjacent struct memory.
                 * See [[critical_av1_filmgrain_bound]]. */
                if (num_y_pos < 0 || num_y_pos > 24) return -1;
                if (fgd.num_y_points)
                    for (int i = 0; i < num_y_pos; i++)
                        fgd.ar_coeffs_y[i] = (int8_t)((int)br_u(r, 8) - 128);
                for (int pl = 0; pl < 2; pl++) {
                    if (fgd.num_uv_points[pl] || fgd.chroma_scaling_from_luma) {
                        int num_uv = num_y_pos + !!fgd.num_y_points;
                        if (num_uv < 0 || num_uv > 25) return -1;
                        for (int i = 0; i < num_uv; i++)
                            fgd.ar_coeffs_uv[pl][i] = (int8_t)((int)br_u(r, 8) - 128);
                        if (!fgd.num_y_points)
                            fgd.ar_coeffs_uv[pl][num_uv] = 0;
                    }
                }
                fgd.ar_coeff_shift  = (uint64_t)br_u(r, 2) + 6;
                fgd.grain_scale_shift = (int)br_u(r, 2);
                for (int pl = 0; pl < 2; pl++) {
                    if (fgd.num_uv_points[pl]) {
                        fgd.uv_mult[pl]      = (int)br_u(r, 8) - 128;
                        fgd.uv_luma_mult[pl] = (int)br_u(r, 8) - 128;
                        fgd.uv_offset[pl]    = (int)br_u(r, 9) - 256;
                    }
                }
                fgd.overlap_flag          = (int)br_bit(r);
                fgd.clip_to_restricted_range = (int)br_bit(r);
            }
        }
    }

    if (r.error) return -1;
    return 0;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

int Av1ParseSeqHeader(const uint8_t *obu_payload, size_t obu_payload_len,
                      Dav1dSequenceHeader *out)
{
    if (!obu_payload || !out || !obu_payload_len) return -1;
    BitReader r(obu_payload, obu_payload_len);
    return parse_seq_hdr(r, out);
}

int Av1ParseFrameHeader(const uint8_t            *obu_payload,
                        size_t                    obu_payload_len,
                        bool                      obu_is_frame_type,
                        const Dav1dSequenceHeader *seq,
                        const Av1SavedFrameState   prev_states[8],
                        Dav1dFrameHeader          *out)
{
    if (!obu_payload || !seq || !out || !obu_payload_len) return -1;

    BitReader r(obu_payload, obu_payload_len);

    int rc = parse_frame_hdr(r, seq, prev_states, out);
    if (rc < 0) return -1;

    /* Byte-align after uncompressed_header() to find tile data offset. */
    br_byte_align(r);

    /* frame_hdr_obu_size_bytes = bytes consumed by uncompressed_header()
     * including trailing alignment bits.
     * For OBU_FRAME the tile_group payload starts immediately after this. */
    out->frame_hdr_obu_size_bytes =
        (uint32_t)br_bytes_consumed(r, obu_payload);

    (void)obu_is_frame_type; /* tile_group offset is always byte-aligned */
    return 0;
}

void Av1UpdateSavedStates(Av1SavedFrameState     states[8],
                          const Dav1dFrameHeader *cur)
{
    if (!states || !cur) return;
    for (int i = 0; i < 8; i++) {
        if ((cur->refresh_frame_flags >> i) & 1) {
            states[i].valid  = true;
            states[i].saved  = *cur;
        }
    }
}
