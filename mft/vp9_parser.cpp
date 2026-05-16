// mft/vp9_parser.cpp — VP9 parser (uncompressed + compressed headers).
//
// Implements VP9 spec §6.1.1 (superframe split), §6.2 (uncompressed header),
// and §6.3 (compressed header — probability delta updates).
//
// Authoritative reference: VP9 Bitstream & Decoding Process Specification
// v0.6 (https://www.webmproject.org/vp9/).
//   §6.1.1 — Superframe syntax
//   §6.2   — Uncompressed header syntax
//   §6.2.5 — frame_size_with_refs()
//   §6.2.8 — loop_filter_params()
//   §6.2.9 — quantization_params()
//   §6.2.10 — segmentation_params()
//   §6.2.11 — tile_info()
//   §6.3   — Compressed header syntax
//   §6.3.1 — read_tx_mode()
//   §6.3.2 — read_tx_mode_probs()
//   §6.3.5/6.3.6 — read_coef_probs() / decode_term_subexp()
//   §6.3.7 — read_skip_prob()
//   §6.3.8 — read_inter_mode_probs()
//   §6.3.9 — read_interp_filter_probs()
//   §6.3.10 — read_is_inter_probs() + read_frame_reference_mode()
//   §6.3.11 — read_frame_reference_mode_probs()
//   §6.3.12 — read_y_mode_probs()
//   §6.3.13 — read_partition_probs()
//   §6.3.16 — read_mv_probs()
//
// BSP reference: C:\Users\vibecoder\mpp\mpp\codec\dec\vp9\vp9d.c
// (used only to verify field ordering; no code copied).
//
// SPDX-License-Identifier: BSD-2-Clause-Patent

#include "vp9_parser.h"
#include "vp9_bool_decoder.h"
#include <cstring>
#include <cassert>

namespace vp9 {

/* =========================================================================
 * Internal bit-reader (MSB-first within each byte, same as VP9 spec)
 * ========================================================================= */

struct BitReader {
    const uint8_t *buf;      // next byte to fetch
    const uint8_t *start;    // first byte (for byte_pos())
    const uint8_t *end;      // one past last byte
    uint64_t       state;    // MSB-aligned shift register
    int            bits_left;// valid bits held in state (top bits_left bits)
    bool           error;
    uint32_t       used_bits;// total bits consumed (for tracing)

    BitReader(const uint8_t *p, size_t len)
        : buf(p), start(p), end(p + len),
          state(0), bits_left(0), error(false), used_bits(0) {}
};

// Refill so at least n bits are available.
static void br_refill(BitReader &r, int n)
{
    while (r.bits_left < n) {
        if (r.buf >= r.end) { r.error = true; return; }
        r.state    |= (uint64_t)(*r.buf++) << (56 - r.bits_left);
        r.bits_left += 8;
    }
}

// Read n unsigned bits (1 <= n <= 32).
static uint32_t br_u(BitReader &r, int n)
{
    assert(n >= 1 && n <= 32);
    if (r.bits_left < n) br_refill(r, n);
    if (r.error) return 0;
    uint32_t v = (uint32_t)(r.state >> (64 - n));
    if (n < 32) v &= (1u << n) - 1u;
    r.state    <<= n;
    r.bits_left -= n;
    r.used_bits += (uint32_t)n;
    return v;
}

static inline uint32_t br_bit(BitReader &r) { return br_u(r, 1); }

// Read VP9 signed value: spec s(n) reads n bits as magnitude followed by
// 1 bit as sign (NOT two's-complement).  Verified against BSP MPP
// mpp_read_signbits: `mag = read(n); sign = read(1); return sign ? -mag : mag`.
// Total bits consumed: n + 1.
static int32_t br_su(BitReader &r, int n)
{
    uint32_t mag = br_u(r, n);
    uint32_t sign = br_bit(r);
    return sign ? -(int32_t)mag : (int32_t)mag;
}

// Byte-align the reader (discard 0..7 already-fetched bits).
static void br_byte_align(BitReader &r)
{
    int rem = r.bits_left & 7;
    if (rem) {
        // Discard the fractional bits currently sitting in state.
        r.state    <<= rem;
        r.bits_left -= rem;
    }
}

// Byte position of the current bit cursor (bytes consumed, 0-based).
// After br_byte_align this equals the offset of the next unread byte.
static uint32_t br_byte_pos(const BitReader &r)
{
    // buf points past the last fetched byte; bits_left are pre-fetched.
    size_t fetched = (size_t)(r.buf - r.start);
    size_t pending = (size_t)(r.bits_left / 8);
    if (fetched >= pending)
        return (uint32_t)(fetched - pending);
    return 0;
}

/* =========================================================================
 * §6.2.2  color_config()
 * ========================================================================= */

// Color space enum values per VP9 spec Table 2.
// 0=UNKNOWN 1=BT601 2=BT709 3=SMPTE170 4=SMPTE240 5=BT2020 6=RESERVED 7=SRGB
static void read_color_config(BitReader &r, uint8_t profile, PicParams &pp)
{
    // bit_depth: profile 2/3 read 1 extra bit (10/12-bit).  12-bit isn't
    // wired through repack_yuv (NV15→P010 is 10-bit specific) and the
    // PP module isn't configured for 12-bit, so we tag it as 12 here and
    // let Vp9Parser_Parse turn that into ParseResult::Error below.
    if (profile >= 2) {
        uint32_t ten_or_twelve = br_bit(r);
        pp.bit_depth = (uint8_t)(ten_or_twelve ? 12 : 10);
    } else {
        pp.bit_depth = 8;
    }

    pp.color_space = (uint8_t)br_u(r, 3); // 3 bits: color_space

    if (pp.color_space != 7) {
        // Not SRGB
        pp.color_range = (uint8_t)br_bit(r);
        if (profile == 1 || profile == 3) {
            pp.subsampling_x = (uint8_t)br_bit(r);
            pp.subsampling_y = (uint8_t)br_bit(r);
            br_bit(r); // reserved_zero
        } else {
            // profile 0/2: 4:2:0
            pp.subsampling_x = 1;
            pp.subsampling_y = 1;
        }
    } else {
        // SRGB: full range, 4:4:4
        pp.color_range = 1;
        if (profile == 1 || profile == 3) {
            pp.subsampling_x = 0;
            pp.subsampling_y = 0;
            br_bit(r); // reserved_zero
        }
        // profile 0/2 with SRGB is not valid per spec, but we don't error
    }
}

/* =========================================================================
 * §6.2.3  frame_size()  and  §6.2.4  render_size()
 * ========================================================================= */

static void read_frame_size(BitReader &r, PicParams &pp)
{
    pp.width  = br_u(r, 16) + 1;
    pp.height = br_u(r, 16) + 1;
}

static void read_render_size(BitReader &r, PicParams &pp)
{
    uint32_t render_and_frame_size_different = br_bit(r);
    if (render_and_frame_size_different) {
        pp.render_width  = br_u(r, 16) + 1;
        pp.render_height = br_u(r, 16) + 1;
    } else {
        pp.render_width  = pp.width;
        pp.render_height = pp.height;
    }
}

/* =========================================================================
 * §6.2.5  frame_size_with_refs()
 * ========================================================================= */

static void read_frame_size_with_refs(BitReader &r, const ParserState &st,
                                      const PicParams &pp_partial,
                                      PicParams &pp)
{
    bool found = false;
    for (int i = 0; i < kRefsPerFrame; i++) {
        uint32_t use_ref = br_bit(r);
        if (use_ref) {
            uint8_t slot = pp_partial.frame_refs[i].index;
            if (slot < kNumRefFrames && st.valid[slot]) {
                pp.width  = st.ref_state[slot].width;
                pp.height = st.ref_state[slot].height;
            }
            found = true;
            break;
        }
    }
    if (!found) {
        read_frame_size(r, pp);
    }
    read_render_size(r, pp);
}

/* =========================================================================
 * §6.2.8  loop_filter_params()
 * ========================================================================= */

static void read_loop_filter_params(BitReader &r, const ParserState &st,
                                    bool is_key_or_intra,
                                    PicParams &pp)
{
    // Inherit from previous state, then overwrite with new values.
    if (is_key_or_intra) {
        // Reset to defaults on key/intra frames.
        memset(&pp.lf, 0, sizeof(pp.lf));
        pp.lf.mode_ref_delta_enabled = 1;
        pp.lf.ref_deltas[0] =  1; // INTRA
        pp.lf.ref_deltas[1] =  0; // LAST
        pp.lf.ref_deltas[2] = -1; // GOLDEN
        pp.lf.ref_deltas[3] = -1; // ALTREF
        pp.lf.mode_deltas[0] = 0;
        pp.lf.mode_deltas[1] = 0;
    } else {
        pp.lf = st.prev_lf;
    }

    pp.lf.level     = (uint8_t)br_u(r, 6);
    pp.lf.sharpness = (uint8_t)br_u(r, 3);

    pp.lf.mode_ref_delta_enabled = (uint8_t)br_bit(r);
    if (pp.lf.mode_ref_delta_enabled) {
        pp.lf.mode_ref_delta_update = (uint8_t)br_bit(r);
        if (pp.lf.mode_ref_delta_update) {
            for (int i = 0; i < kMaxRefLfDeltas; i++) {
                if (br_bit(r))
                    pp.lf.ref_deltas[i] = (int8_t)br_su(r, 6);
            }
            for (int i = 0; i < kMaxModeLfDeltas; i++) {
                if (br_bit(r))
                    pp.lf.mode_deltas[i] = (int8_t)br_su(r, 6);
            }
        }
    }
}

/* =========================================================================
 * §6.2.9  quantization_params()
 * ========================================================================= */

static void read_quantization_params(BitReader &r, PicParams &pp)
{
    pp.base_qindex = (uint8_t)br_u(r, 8);

    // delta_q: flag + signed 4-bit value
    pp.y_dc_delta_q  = br_bit(r) ? (int8_t)br_su(r, 4) : 0;
    pp.uv_dc_delta_q = br_bit(r) ? (int8_t)br_su(r, 4) : 0;
    pp.uv_ac_delta_q = br_bit(r) ? (int8_t)br_su(r, 4) : 0;

    pp.lossless = (pp.base_qindex == 0 &&
                   pp.y_dc_delta_q  == 0 &&
                   pp.uv_dc_delta_q == 0 &&
                   pp.uv_ac_delta_q == 0) ? 1 : 0;
}

/* =========================================================================
 * §6.2.10  segmentation_params()
 * ========================================================================= */

// Feature widths per spec Table 4: ALT_Q=8, ALT_LF=6, REF_FRAME=2, SKIP=0
static const int kSegFeatureBits[kSegLvlMax]   = { 8, 6, 2, 0 };
static const int kSegFeatureSigned[kSegLvlMax] = { 1, 1, 0, 0 };

static void read_segmentation_params(BitReader &r, const ParserState &st,
                                     PicParams &pp)
{
    pp.seg.enabled = (uint8_t)br_bit(r);
    if (!pp.seg.enabled) {
        memset(&pp.seg, 0, sizeof(pp.seg));
        // Preserve pred_probs at 255 (spec default) for inactive seg.
        for (int i = 0; i < 3; i++) pp.seg.pred_probs[i] = 255;
        return;
    }

    pp.seg.update_map = (uint8_t)br_bit(r);
    if (pp.seg.update_map) {
        for (int i = 0; i < 7; i++) {
            pp.seg.tree_probs[i] = 255;
            if (br_bit(r))
                pp.seg.tree_probs[i] = (uint8_t)br_u(r, 8);
        }
        pp.seg.temporal_update = (uint8_t)br_bit(r);
        if (pp.seg.temporal_update) {
            for (int i = 0; i < 3; i++) {
                pp.seg.pred_probs[i] = 255;
                if (br_bit(r))
                    pp.seg.pred_probs[i] = (uint8_t)br_u(r, 8);
            }
        } else {
            for (int i = 0; i < 3; i++)
                pp.seg.pred_probs[i] = 255;
        }
    } else {
        // Inherit from previous frame's seg state.
        for (int i = 0; i < 7; i++)
            pp.seg.tree_probs[i] = st.prev_seg.tree_probs[i];
        pp.seg.temporal_update = 0;
        for (int i = 0; i < 3; i++)
            pp.seg.pred_probs[i] = st.prev_seg.pred_probs[i];
    }

    pp.seg.update_data = (uint8_t)br_bit(r);
    if (pp.seg.update_data) {
        pp.seg.abs_delta = (uint8_t)br_bit(r);
        for (int i = 0; i < kMaxSegments; i++) {
            pp.seg.feature_mask[i] = 0;
            for (int j = 0; j < kSegLvlMax; j++) {
                uint32_t feature_enabled = br_bit(r);
                if (feature_enabled) {
                    pp.seg.feature_mask[i] |= (uint8_t)(1u << j);
                    int nbits = kSegFeatureBits[j];
                    if (nbits > 0) {
                        if (kSegFeatureSigned[j]) {
                            pp.seg.feature_data[i][j] =
                                (int16_t)br_su(r, nbits);
                        } else {
                            pp.seg.feature_data[i][j] =
                                (int16_t)br_u(r, nbits);
                        }
                    } else {
                        pp.seg.feature_data[i][j] = 0;
                    }
                } else {
                    pp.seg.feature_data[i][j] = 0;
                }
            }
        }
    } else {
        // Inherit feature data from previous frame.
        for (int i = 0; i < kMaxSegments; i++) {
            pp.seg.feature_mask[i] = st.prev_seg.feature_mask[i];
            for (int j = 0; j < kSegLvlMax; j++)
                pp.seg.feature_data[i][j] = st.prev_seg.feature_data[i][j];
        }
        pp.seg.abs_delta = st.prev_seg.abs_delta;
    }
}

/* =========================================================================
 * §6.2.11  tile_info()
 * ========================================================================= */

// Minimum log2 of tile columns: smallest k such that sb_cols >> k <= 64.
static uint32_t min_log2_tile_cols(uint32_t sb_cols)
{
    uint32_t k = 0;
    while ((sb_cols >> k) > 64) k++;
    return k;
}

// Maximum log2 of tile columns: largest k such that sb_cols >> k >= 4.
static uint32_t max_log2_tile_cols(uint32_t sb_cols)
{
    uint32_t k = 0;
    while (sb_cols >> (k + 1) >= 4) k++;
    return k;
}

static void read_tile_info(BitReader &r, const PicParams &pp,
                           uint8_t &tile_cols_log2, uint8_t &tile_rows_log2)
{
    // VP9 uses 64-pixel superblocks for tiling purposes.
    uint32_t sb_cols = (pp.width  + 63) >> 6;

    uint32_t min_col = min_log2_tile_cols(sb_cols);
    uint32_t max_col = max_log2_tile_cols(sb_cols);

    uint32_t log2_cols = min_col;
    while (log2_cols < max_col) {
        if (!br_bit(r)) break;
        log2_cols++;
    }
    tile_cols_log2 = (uint8_t)log2_cols;

    // tile rows: 1 bit; if 1, read one more bit
    uint32_t log2_rows = br_bit(r);
    if (log2_rows)
        log2_rows += br_bit(r);
    tile_rows_log2 = (uint8_t)log2_rows;
}

/* =========================================================================
 * §6.2  uncompressed_header()
 * ========================================================================= */

static ParseResult parse_uncompressed_header(const uint8_t *frame, size_t len,
                                             const ParserState &st,
                                             PicParams &pp)
{
    if (!frame || len == 0) return ParseResult::NeedMoreData;

    BitReader r(frame, len);
    memset(&pp, 0, sizeof(pp));

    // 1. frame_marker (2 bits, must be 0b10 = 2)
    uint32_t marker = br_u(r, 2);
    if (r.error) return ParseResult::NeedMoreData;
    if (marker != 2) return ParseResult::Error;

    // Sequence-sticky defaults — color_config is only sent on KEY and
    // (profile>0) intra-only frames, so inter frames must inherit
    // bit_depth/profile from the active sequence (spec §6.2 says these
    // persist across frames until the next color_config).
    pp.bit_depth = st.last_bit_depth ? st.last_bit_depth : 8u;

    // 2. profile: low bit, high bit, then reserved if profile == 3
    uint32_t profile_low  = br_bit(r);
    uint32_t profile_high = br_bit(r);
    pp.profile = (uint8_t)(profile_low | (profile_high << 1));
    if (pp.profile == 3) {
        uint32_t reserved = br_bit(r);
        if (r.error) return ParseResult::NeedMoreData;
        if (reserved != 0) return ParseResult::Error;
        // Profile 3 = 4:4:4 sampling, unsupported by the rkvdec2 PP
        // path (NV12/P010 only — no 4:4:4 packing exists in repack_yuv).
        return ParseResult::Error;
    }

    // 3. show_existing_frame
    pp.show_existing_frame = (uint8_t)br_bit(r);
    if (pp.show_existing_frame) {
        pp.show_existing_frame_idx = (uint8_t)br_u(r, 3);
        if (r.error) return ParseResult::NeedMoreData;
        // No further fields; compressed header size and tile data not present.
        pp.uncompressed_header_size = br_byte_pos(r);
        pp.header_size = 0;
        return r.error ? ParseResult::NeedMoreData : ParseResult::Ok;
    }

    // 4. frame_type (0=KEY, 1=INTER), show_frame, error_resilient_mode
    pp.frame_type            = (uint8_t)br_bit(r);
    pp.show_frame            = (uint8_t)br_bit(r);
    pp.error_resilient_mode  = (uint8_t)br_bit(r);

    bool is_keyframe  = (pp.frame_type == 0);
    bool is_intraonly = false;

    if (is_keyframe) {
        // 5a. KEY frame
        // frame_sync_code: 3 bytes 0x49 0x83 0x42 (read as 24 bits)
        uint32_t sync = br_u(r, 24);
        if (r.error) return ParseResult::NeedMoreData;
        if (sync != 0x498342u) return ParseResult::Error;

        read_color_config(r, pp.profile, pp);
        if (pp.bit_depth == 12) return ParseResult::Error;
        read_frame_size(r, pp);
        read_render_size(r, pp);

        pp.refresh_frame_flags = 0xFF;
    } else {
        // 5b. INTER or INTRA-ONLY frame

        // intra_only is only read when show_frame == 0
        if (!pp.show_frame) {
            is_intraonly = (br_bit(r) != 0);
            pp.intra_only = (uint8_t)is_intraonly;
        }

        // reset_frame_context (2 bits) unless error_resilient_mode
        if (!pp.error_resilient_mode) {
            pp.reset_frame_context = (uint8_t)br_u(r, 2);
        } else {
            pp.reset_frame_context = 0;
        }

        if (is_intraonly) {
            // frame_sync_code
            uint32_t sync = br_u(r, 24);
            if (r.error) return ParseResult::NeedMoreData;
            if (sync != 0x498342u) return ParseResult::Error;

            // color_config only for profile > 0 on intra-only
            if (pp.profile > 0) {
                read_color_config(r, pp.profile, pp);
                if (pp.bit_depth == 12) return ParseResult::Error;
            } else {
                // profile 0 intra-only: implied 8-bit 4:2:0 BT.601
                pp.bit_depth     = 8;
                pp.color_space   = 1; // BT.601
                pp.color_range   = 1; // full range (spec §6.2.2 intra-only profile 0 default)
                pp.subsampling_x = 1;
                pp.subsampling_y = 1;
            }

            pp.refresh_frame_flags = (uint8_t)br_u(r, 8);
            read_frame_size(r, pp);
            read_render_size(r, pp);

        } else {
            // INTER frame
            pp.refresh_frame_flags = (uint8_t)br_u(r, 8);

            // 3 reference frames: ref_frame_idx (3 bits) + sign_bias (1 bit)
            for (int i = 0; i < kRefsPerFrame; i++) {
                pp.frame_refs[i].index     = (uint8_t)br_u(r, 3);
                pp.frame_refs[i].sign_bias = (uint8_t)br_bit(r);
            }

            // frame_size_with_refs (§6.2.5)
            read_frame_size_with_refs(r, st, pp, pp);

            pp.allow_high_precision_mv = (uint8_t)br_bit(r);

            // interp_filter: 1 bit; if 1 → SWITCHABLE(4), else read 2 bits
            if (br_bit(r)) {
                pp.interp_filter = 4; // SWITCHABLE
            } else {
                pp.interp_filter = (uint8_t)br_u(r, 2);
            }
        }
    }

    if (r.error) return ParseResult::NeedMoreData;

    // 7. refresh_frame_context (unless error_resilient_mode)
    if (!pp.error_resilient_mode) {
        pp.refresh_frame_context = (uint8_t)br_bit(r);
    } else {
        pp.refresh_frame_context = 0;
    }

    // 8. frame_parallel_decoding_mode (unless error_resilient_mode)
    if (!pp.error_resilient_mode) {
        pp.frame_parallel_decoding_mode = (uint8_t)br_bit(r);
    } else {
        pp.frame_parallel_decoding_mode = 1;
    }

    // 9. frame_context_idx (2 bits)
    pp.frame_context_idx = (uint8_t)br_u(r, 2);

    // 10. loop_filter_params (§6.2.8)
    bool is_key_or_intra = is_keyframe || is_intraonly;
    /* TRACE: bit position at each phase (header_size debug) */
#ifdef VP9_TRACE_HEADER_SIZE
#define VP9_TRACE(name) do { \
    fprintf(stderr, "[vp9_trace] %-32s used_bits=%u\n", name, r.used_bits); \
} while (0)
#else
#define VP9_TRACE(name) ((void)0)
#endif
    VP9_TRACE("pre loop_filter");
    read_loop_filter_params(r, st, is_key_or_intra, pp);

    VP9_TRACE("pre quantization");
    // 11. quantization_params (§6.2.9)
    read_quantization_params(r, pp);

    VP9_TRACE("pre segmentation");
    // 12. segmentation_params (§6.2.10)
    read_segmentation_params(r, st, pp);

    VP9_TRACE("pre tile_info");
    // 13. tile_info (§6.2.11)
    read_tile_info(r, pp, pp.tile_cols_log2, pp.tile_rows_log2);

    VP9_TRACE("pre header_size");

    if (r.error) return ParseResult::NeedMoreData;

    // 14. header_size_in_bytes (16 bits) — size of compressed header
#ifdef VP9_TRACE_HEADER_SIZE
    {
        size_t fetched = (size_t)(r.buf - r.start);
        size_t pending = (size_t)(r.bits_left / 8);
        size_t byte_pos = fetched - pending;
        int    bit_in_byte = r.bits_left % 8;
        fprintf(stderr, "[vp9_trace] header_size read at byte=%zu bit_in=%d\n",
                byte_pos, bit_in_byte);
    }
#endif
    pp.header_size = br_u(r, 16);

    if (r.error) return ParseResult::NeedMoreData;

    // 15. byte_align, then record uncompressed_header_size
    br_byte_align(r);
    pp.uncompressed_header_size = br_byte_pos(r);

    pp.frame_size = (uint32_t)len;

    return r.error ? ParseResult::NeedMoreData : ParseResult::Ok;
}

/* =========================================================================
 * §6.3  Compressed header — probability delta helpers
 * ========================================================================= */

// §6.3.6 — decode_term_subexp(bd): bounded sub-exponential coded value.
static uint8_t vp9_decode_term_subexp(BoolDecoder &bd)
{
    if (bd.decode_bool(128) == 0) return (uint8_t)bd.decode_literal(4);
    if (bd.decode_bool(128) == 0) return (uint8_t)(bd.decode_literal(4) + 16);
    if (bd.decode_bool(128) == 0) return (uint8_t)(bd.decode_literal(5) + 32);
    int v = bd.decode_literal(7);
    if (v < 65) return (uint8_t)(v + 64);
    return (uint8_t)((v << 1) - 1 + bd.decode_literal(1));
}

// §6.3.6 — prob diff update.  The hardware ingests the *raw* delta
// (the post-`term_subexp` byte before inv_remap) and applies the
// inv_remap internally against its own ref-prob table.  We therefore
// stash the raw byte in pu's value array — that's the value the
// regbuilder writes into the prob buffer for the codec.
//
// Returns true if a delta was decoded (caller mirrors to a flag byte
// so the regbuilder can emit the per-context update bit).
// Returns 1 if a delta was decoded, 0 otherwise.  Plain `uint8_t` (not
// `bool`) because g++ on aarch64 miscompiles the
// `pu.X_flag[i] = vp9_decode_prob_diff_update(...)` pattern when the
// return is `bool` — the store of the converted byte appears to drop
// in front of the function's side effects on `*out_raw_delta`, leaving
// flag=0 even when the delta WAS written.  Returning uint8_t makes the
// store unambiguous and the parser bit-exact across MSVC and g++.
static uint8_t vp9_decode_prob_diff_update(BoolDecoder &bd, uint8_t *out_raw_delta)
{
    if (bd.decode_bool(252)) {
        *out_raw_delta = vp9_decode_term_subexp(bd);
        return 1;
    }
    return 0;
}

// §6.3.16 — MV probability update.  Unlike the other prob groups, the
// rkvdec2 hardware reads the *applied* probability for MV updates
// (the (v << 1) | 1 mapping), not the raw 7-bit literal — see BSP
// vp9d.c line 1127 where `prob_flag_delta.p_delta.mv_joint[i]` gets
// `(read_bits(7) << 1) | 1`.  Match that convention.
static bool vp9_decode_mv_prob_update(BoolDecoder &bd, uint8_t *out_byte)
{
    if (bd.decode_bool(252)) {
        uint8_t v = (uint8_t)bd.decode_literal(7);
        *out_byte = (uint8_t)((v << 1) | 1);
        return true;
    }
    return false;
}

// §6.3.1 — tx_mode_to_max_tx_size: highest tx size allowed for a given mode.
// tx_mode 0 → only 4x4, mode 1 → up to 8x8, mode 2 → up to 16x16,
// mode 3 → up to 32x32 (ALLOW_32x32), mode 4 → all sizes (TX_MODE_SELECT).
static int tx_mode_to_max_tx_size(int tx_mode)
{
    // tx_mode values 0..3 allow tx sizes 0..tx_mode.
    // TX_MODE_SELECT(4) allows all four sizes (0..3).
    return (tx_mode == 4) ? 3 : tx_mode;
}

/* =========================================================================
 * §6.3  Compressed header parse
 * ========================================================================= */

static void parse_compressed_header(const uint8_t *buf, size_t sz,
                                    const PicParams &pp,
                                    ProbUpdates &pu)
{
    BoolDecoder bd;
    bd.init(buf, sz);
    if (!bd.ok()) return;

    // §6.3.1  read_tx_mode
    int tx_mode;
    if (pp.lossless) {
        tx_mode = 0; // ONLY_4X4
    } else {
        tx_mode = bd.decode_literal(2);
        if (tx_mode == 3) {
            tx_mode += bd.decode_bool(128); // 3 = ALLOW_32x32, 4 = TX_MODE_SELECT
        }
        if (!bd.ok()) goto bail;
    }
    pu.tx_mode_present = 1;
    pu.tx_mode         = (uint8_t)tx_mode;
    // Also mirror into pp-side field via the ProbUpdates structure.

    // §6.3.2  read_tx_mode_probs (only when TX_MODE_SELECT)
    if (tx_mode == 4) {
        for (int c = 0; c < 2; ++c)
            pu.tx_size_8x8_flag[c][0] =
                vp9_decode_prob_diff_update(bd, &pu.tx_size_8x8[c][0]);
        for (int c = 0; c < 2; ++c)
            for (int k = 0; k < 2; ++k)
                pu.tx_size_16x16_flag[c][k] =
                    vp9_decode_prob_diff_update(bd, &pu.tx_size_16x16[c][k]);
        for (int c = 0; c < 2; ++c)
            for (int k = 0; k < 3; ++k)
                pu.tx_size_32x32_flag[c][k] =
                    vp9_decode_prob_diff_update(bd, &pu.tx_size_32x32[c][k]);
        if (!bd.ok()) goto bail;
        pu.tx_probs_present = 1;
    }

    // §6.3.5 / §6.3.6  read_coef_probs
    {
        int max_tx = tx_mode_to_max_tx_size(tx_mode);
        for (int tx = 0; tx <= max_tx; ++tx) {
            int update_probs = bd.decode_bool(128);
            if (!bd.ok()) goto bail;
            if (update_probs) {
                pu.coef_present[tx] = 1;
                // Spec §6.3.6 loop: block_type {0,1}, ref_type {0,1},
                // band 0..5, ctx 0..(band==0 ? 2 : 5), coef 0..2.
                for (int plane = 0; plane < 2; ++plane) {
                    for (int ref = 0; ref < 2; ++ref) {
                        for (int band = 0; band < 6; ++band) {
                            int ctx_max = (band == 0) ? 3 : 6;
                            for (int ctx = 0; ctx < ctx_max; ++ctx) {
                                for (int i = 0; i < 3; ++i) {
                                    if (!bd.ok()) goto bail;
                                    uint8_t *p = &pu.coef_values[tx][ref][plane][band][ctx][i];
                                    if (vp9_decode_prob_diff_update(bd, p))
                                        pu.coef_changed[tx][ref][plane][band][ctx][i] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!bd.ok()) goto bail;
    }

    // §6.3.7  read_skip_prob
    pu.skip_present = 1;
    for (int i = 0; i < 3; ++i)
        pu.skip_flag[i] = vp9_decode_prob_diff_update(bd, &pu.skip[i]);
    if (!bd.ok()) goto bail;

    // Inter-only sections: skip if keyframe or intra-only.
    if (pp.frame_type != 0 && !pp.intra_only) {
        // §6.3.8  read_inter_mode_probs: 7 contexts × 3 probs
        pu.inter_mode_present = 1;
        for (int c = 0; c < 7; ++c)
            for (int k = 0; k < 3; ++k)
                pu.inter_mode_flag[c][k] =
                    vp9_decode_prob_diff_update(bd, &pu.inter_mode[c][k]);
        if (!bd.ok()) goto bail;

        // §6.3.9  read_interp_filter_probs (only when interp_filter == SWITCHABLE=4)
        if (pp.interp_filter == 4) {
            pu.interp_filter_present = 1;
            for (int c = 0; c < 4; ++c)
                for (int k = 0; k < 2; ++k)
                    pu.interp_filter_flag[c][k] =
                        vp9_decode_prob_diff_update(bd, &pu.interp_filter[c][k]);
            if (!bd.ok()) goto bail;
        }

        // §6.3.10  read_is_inter_probs: 4 probs
        pu.is_inter_present = 1;
        for (int i = 0; i < 4; ++i)
            pu.is_inter_flag[i] =
                vp9_decode_prob_diff_update(bd, &pu.is_inter[i]);
        if (!bd.ok()) goto bail;

        // §6.3.10  read_frame_reference_mode: determines reference_mode.
        // The non_single/ref_select bits are only present when compound
        // reference is allowed — when at least two of the three frame_refs
        // have differing sign_bias values.  When all sign_biases match,
        // the encoder skips both bits and the decoder defaults to
        // SINGLE_REFERENCE.  Reading the bits unconditionally desyncs
        // the bool decoder for the entire rest of the compressed header.
        int reference_mode = 0;
        {
            uint8_t sb0 = pp.frame_refs[0].sign_bias;
            uint8_t sb1 = pp.frame_refs[1].sign_bias;
            uint8_t sb2 = pp.frame_refs[2].sign_bias;
            bool compound_allowed = (sb0 != sb1) || (sb0 != sb2);
            if (compound_allowed) {
                int non_single = bd.decode_bool(128);
                if (non_single) {
                    int ref_select = bd.decode_bool(128);
                    reference_mode = ref_select ? 2 : 1;
                }
            }
        }
        if (!bd.ok()) goto bail;
        pu.ref_mode_present = 1;
        pu.reference_mode   = (uint8_t)reference_mode;

        // §6.3.11  read_frame_reference_mode_probs
        if (reference_mode == 2) { // REFERENCE_MODE_SELECT
            pu.comp_mode_present = 1;
            for (int i = 0; i < 5; ++i)
                pu.comp_mode_flag[i] =
                    vp9_decode_prob_diff_update(bd, &pu.comp_mode[i]);
            if (!bd.ok()) goto bail;
        }
        if (reference_mode != 1) { // not COMPOUND_REFERENCE → have single refs
            pu.single_ref_present = 1;
            for (int i = 0; i < 5; ++i)
                for (int k = 0; k < 2; ++k)
                    pu.single_ref_flag[i][k] =
                        vp9_decode_prob_diff_update(bd, &pu.single_ref[i][k]);
            if (!bd.ok()) goto bail;
        }
        if (reference_mode != 0) { // not SINGLE_REFERENCE → have comp ref
            pu.comp_ref_present = 1;
            for (int i = 0; i < 5; ++i)
                pu.comp_ref_flag[i] =
                    vp9_decode_prob_diff_update(bd, &pu.comp_ref[i]);
            if (!bd.ok()) goto bail;
        }

        // §6.3.12  read_y_mode_probs: 4 × 9 probs
        pu.y_mode_present = 1;
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 9; ++k)
                pu.y_mode_flag[c][k] =
                    vp9_decode_prob_diff_update(bd, &pu.y_mode[c][k]);
        if (!bd.ok()) goto bail;

        // §6.3.13  read_partition_probs: 16 contexts × 3 probs
        pu.partition_present = 1;
        for (int c = 0; c < 16; ++c)
            for (int k = 0; k < 3; ++k)
                pu.partition_flag[c][k] =
                    vp9_decode_prob_diff_update(bd, &pu.partition[c][k]);
        if (!bd.ok()) goto bail;

        // §6.3.16  read_mv_probs
        pu.mv_present = 1;
        for (int i = 0; i < 3; ++i)
            pu.mv_joints_flag[i] =
                vp9_decode_mv_prob_update(bd, &pu.mv_joints[i]);
        if (!bd.ok()) goto bail;
        for (int comp = 0; comp < 2; ++comp) {
            pu.mv_sign_flag[comp] =
                vp9_decode_mv_prob_update(bd, &pu.mv_sign[comp]);
            for (int k = 0; k < 10; ++k)
                pu.mv_classes_flag[comp][k] =
                    vp9_decode_mv_prob_update(bd, &pu.mv_classes[comp][k]);
            pu.mv_class0_flag[comp] =
                vp9_decode_mv_prob_update(bd, &pu.mv_class0[comp]);
            for (int k = 0; k < 10; ++k)
                pu.mv_bits_flag[comp][k] =
                    vp9_decode_mv_prob_update(bd, &pu.mv_bits[comp][k]);
        }
        if (!bd.ok()) goto bail;
        for (int comp = 0; comp < 2; ++comp) {
            for (int c = 0; c < 2; ++c)
                for (int k = 0; k < 3; ++k)
                    pu.mv_class0_fp_flag[comp][c][k] =
                        vp9_decode_mv_prob_update(bd, &pu.mv_class0_fp[comp][c][k]);
            for (int k = 0; k < 3; ++k)
                pu.mv_fp_flag[comp][k] =
                    vp9_decode_mv_prob_update(bd, &pu.mv_fp[comp][k]);
        }
        if (!bd.ok()) goto bail;
        if (pp.allow_high_precision_mv) {
            for (int comp = 0; comp < 2; ++comp) {
                pu.mv_class0_hp_flag[comp] =
                    vp9_decode_mv_prob_update(bd, &pu.mv_class0_hp[comp]);
                pu.mv_hp_flag[comp] =
                    vp9_decode_mv_prob_update(bd, &pu.mv_hp[comp]);
            }
            if (!bd.ok()) goto bail;
        }
    }

    return;

bail:
    // Bool decoder error in compressed header is non-fatal — the uncompressed
    // header is still valid.  Leave any partially-populated ProbUpdates in
    // place; downstream consumers gate on individual `_present` flags and
    // pu was zero-initialized by Vp9Parser_Parse before we ran.
    return;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

ParseResult Vp9Parser_Parse(const uint8_t *frame, size_t len,
                            ParserState &st,
                            PicParams &pp,
                            ProbUpdates &pu)
{
    memset(&pu, 0, sizeof(pu));
    ParseResult res = parse_uncompressed_header(frame, len, st, pp);
    if (res != ParseResult::Ok) return res;

    // Skip compressed header parse for show_existing_frame or zero header_size.
    if (pp.show_existing_frame || pp.header_size == 0) return ParseResult::Ok;

    // Clamp compressed-header length to the bytes actually remaining in the
    // frame buffer.  pp.header_size is the boolean-decoder payload length
    // reported by the bitstream; the encoder may set it larger than the tile
    // data that follows (the bool decoder stops at its byte limit regardless).
    // We must never pass a length that would let bd.init() read past the end
    // of the caller's buffer.
    size_t avail = (pp.uncompressed_header_size < len)
                   ? (len - pp.uncompressed_header_size) : 0;
    if (avail == 0) return ParseResult::Ok;
    size_t hdr_len = ((size_t)pp.header_size < avail) ? (size_t)pp.header_size : avail;

    parse_compressed_header(frame + pp.uncompressed_header_size,
                            hdr_len,
                            pp, pu);

    // Mirror tx_mode into pp so the regbuilder can see it without going
    // through ProbUpdates.
    if (pu.tx_mode_present)
        pp.txmode = pu.tx_mode;

    // Mirror reference_mode into pp.
    if (pu.ref_mode_present)
        pp.reference_mode = pu.reference_mode;

    return ParseResult::Ok;
}

void Vp9Parser_ApplyDpbUpdate(ParserState &st, const PicParams &pp)
{
    for (int i = 0; i < kNumRefFrames; i++) {
        if ((pp.refresh_frame_flags >> i) & 1) {
            st.ref_state[i] = pp;
            st.valid[i]     = 1;
        }
    }
    st.last_width      = pp.width;
    st.last_height     = pp.height;
    st.last_profile    = pp.profile;
    st.last_bit_depth  = pp.bit_depth ? pp.bit_depth : 8u;
    st.prev_lf         = pp.lf;
    st.prev_seg        = pp.seg;
}

int Vp9Parser_SuperframeSplit(const uint8_t *buf, size_t len,
                              const uint8_t **frames, size_t *sizes,
                              int max_frames)
{
    if (!buf || len == 0 || !frames || !sizes || max_frames < 1)
        return -1;

    // Check for superframe marker in the last byte (spec §6.1.1).
    // Marker format: bits[7:5] = 0b110, bits[4:3] = bytes_per_framesize-1,
    //                bits[2:0] = frames_in_superframe-1.
    uint8_t last = buf[len - 1];
    if ((last & 0xE0) != 0xC0) {
        // Not a superframe — single frame spans entire buffer.
        frames[0] = buf;
        sizes[0]  = len;
        return 1;
    }

    int bytes_per_sz = (int)((last >> 3) & 0x3) + 1;
    int num_frames   = (int)(last & 0x7) + 1;

    if (num_frames > max_frames) return -1;

    // Index size: 2 * marker_byte + num_frames * bytes_per_sz
    size_t index_sz = 2 + (size_t)num_frames * (size_t)bytes_per_sz;
    if (index_sz > len) return -1;

    // The second marker byte (at end-of-index) must match the first.
    const uint8_t *index_start = buf + len - index_sz;
    if (index_start[0] != last) return -1;
    if (index_start[index_sz - 1] != last) return -1;

    // Decode frame sizes from the index.
    const uint8_t *p      = index_start + 1;
    const uint8_t *data   = buf;
    size_t         remain = len - index_sz;

    for (int i = 0; i < num_frames; i++) {
        uint32_t sz = 0;
        for (int b = 0; b < bytes_per_sz; b++)
            sz |= (uint32_t)(*p++) << (b * 8);

        if (sz > remain) return -1;
        frames[i] = data;
        sizes[i]  = sz;
        data   += sz;
        remain -= sz;
    }

    return num_frames;
}

} // namespace vp9
