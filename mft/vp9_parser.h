// mft/vp9_parser.h — VP9 uncompressed-header parser public API.
//
// Implements VP9 spec §6.1.1 (superframe split) and §6.2 (uncompressed header).
// Compressed header (§6.3) is handled by a separate task; ProbUpdates is left
// zeroed by this parser.
//
// SPDX-License-Identifier: BSD-2-Clause-Patent
#pragma once
#include "vp9_types.h"
#include <cstddef>

namespace vp9 {

struct ParserState {
    // Sticky across frames (spec §8.2 persistent state):
    PicParams         ref_state[kNumRefFrames];  // recorded dims per slot
    LoopFilterParams  prev_lf{};
    SegmentParams     prev_seg{};
    uint8_t           valid[kNumRefFrames] = {0};
    uint32_t          last_width = 0, last_height = 0;
    // Sequence-sticky: color_config (incl. bit_depth) only appears on
    // keyframes / intra_only-profile>0; inter frames inherit silently.
    uint8_t           last_profile   = 0;
    uint8_t           last_bit_depth = 8;
};

enum class ParseResult { Ok, NeedMoreData, Error };

// Parse a single superframe-already-split VP9 frame. Returns ParseResult,
// fills `pp` (always) and `pu` (for non-keyframe inter frames that
// updated probs). Does NOT advance `st` — call Vp9Parser_ApplyDpbUpdate
// after the frame is successfully kicked.
ParseResult Vp9Parser_Parse(const uint8_t *frame, size_t len,
                            ParserState &st,
                            PicParams &pp,
                            ProbUpdates &pu);

void Vp9Parser_ApplyDpbUpdate(ParserState &st, const PicParams &pp);

// Superframe split: VP9 streams may pack multiple frames; spec §6.1.1.
// Caller must split before invoking the parser.
int Vp9Parser_SuperframeSplit(const uint8_t *buf, size_t len,
                              const uint8_t **frames, size_t *sizes,
                              int max_frames);

} // namespace vp9
