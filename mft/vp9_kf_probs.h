/* mft/vp9_kf_probs.h — VP9 spec-mandated keyframe probability tables.
 *
 * These are the static default probabilities defined in the VP9
 * Bitstream and Decoding Process specification, used to initialise the
 * CDFs at the start of every keyframe / intra-only frame.  Values are
 * the same in every conforming VP9 decoder (libvpx, dav1d, ffmpeg,
 * rockchip-mpp, ...) and come from the spec, not from any one
 * implementation.
 *
 * Stored in spec order — outer dimension matches the spec's table
 * ordering (DC, V, H, D45, D135, D117, D153, D207, D63, TM).  The
 * partition table is stored in hardware-ascending order (8x8 → 64x64),
 * matching how the rkvdec2 hardware consumes it in the prob buffer.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once

#include <cstdint>

namespace vp9 {

/* §10.5 default_kf_y_mode_probs[above][left][k] */
extern const uint8_t kKfYModeProbs[10][10][9];

/* §10.5 default_kf_uv_mode_probs[y][k] */
extern const uint8_t kKfUvModeProbs[10][9];

/* §10.4 default_kf_partition_probs[bsize_ctx][k]
 * Stored in hardware-ascending block-size order: rows 0..3 = 8x8,
 * 4..7 = 16x16, 8..11 = 32x32, 12..15 = 64x64.  Spec order is the
 * reverse (64x64 first) — see Vp9Regbuilder_FillProbs for the
 * spec→hw reorder applied to the parser's pu.partition. */
extern const uint8_t kKfPartitionProbsHw[16][3];

}  // namespace vp9
