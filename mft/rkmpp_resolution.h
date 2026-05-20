/* mft/rkmpp_resolution.h — centralized rkvdec/rkav1d HW resolution gate.
 *
 * Every regbuilder enters this function with width/height extracted from
 * the parsed bitstream.  Reject anything beyond hardware capability (and
 * anything obviously nonsensical like zero) BEFORE the values flow into
 * MMIO-bound arithmetic — the kernel's iova-substitution path does NOT
 * re-validate dimensions, tile counts, or ref counts, so the parser/
 * regbuilder is the trust boundary.
 *
 * Rationale (from 2026-05-19 code review):
 *   - VP9 regbuilder_vp9.cpp:84-86 `pp.height * aligned_pitch` can
 *     overflow uint32 at extreme 10-bit resolutions.
 *   - regbuilder_h264.cpp:51-58 and regbuilder_h265.cpp:106-113
 *     compute `luma_stride * (mb_h * 16)` in a uint32 chain that
 *     overflows on attacker-controlled SPS dimensions.
 *   - H.265 tile-column path can mis-program the codec when uint8
 *     truncation silently passes a large value.
 *
 * Centralising the check in one place lets us bump the cap once if/
 * when later silicon supports >8K.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Conservative cap that covers rkvdec2 + rkav1d at every supported
 * codec/profile.  All current shipping streams fit by orders of
 * magnitude; profile-max streams (4K, 8K) fit comfortably. */
#define RKMPP_MAX_VIDEO_WIDTH   8192u
#define RKMPP_MAX_VIDEO_HEIGHT  8192u

#ifdef __cplusplus
extern "C" {
#endif

static inline bool RkmppValidateResolution(uint32_t w, uint32_t h)
{
    if (w == 0 || h == 0) return false;
    if (w > RKMPP_MAX_VIDEO_WIDTH)  return false;
    if (h > RKMPP_MAX_VIDEO_HEIGHT) return false;
    /* Reject odd dimensions — NV12/P010 surfaces require even,
     * codec stride math assumes 2-pixel alignment in chroma.  This
     * also catches buggy parsers that forgot to apply conformance-
     * window adjustments. */
    if ((w | h) & 1u) return false;
    return true;
}

#ifdef __cplusplus
}
#endif
