/* mft/au_iter.h — Annex-B / IVF access-unit iterator shared by all
 * decode harnesses and by mft/engine/decode_engine.cpp.
 *
 * Consolidates four near-identical implementations that used to live in:
 *   - mft/engine/decode_engine.cpp  (find_slice_nal_h264 / _h265 statics)
 *   - tests/harness/rkmpp_decode/main.cpp
 *   - tests/harness/mft_decode/mft_decode.cpp
 *   - tests/harness/linux_mpp_decode/linux_mpp_decode.c
 *   - tests/harness/winreplay_h264_diff.c
 *
 * Two API shapes are exposed:
 *   1. Stream walker: H264AuNext / H265AuNext over an AuIter cursor.
 *   2. Single-AU slice locator: H264FindSliceNal / H265FindSliceNal — used
 *      by DecodeEngine's per-AU staging path (caller already has one AU
 *      from the MFT input sample, just needs the slice offset within it).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct AuIter {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;
} AuIter;

void AuIter_Init(AuIter *it, const uint8_t *buf, size_t len);

/* Returns 1 if an AU was found, 0 at EOF.
 *   - `*au_off`  is the offset of the AU's leading start-code prefix
 *     (backed up to include a 4-byte SC's leading zero when present).
 *   - `*au_len`  spans from `*au_off` to the byte before the next AU's
 *     start code (or EOF).
 *   - `slice_off_opt` may be NULL; if non-NULL, receives the offset of
 *     the slice NAL's leading start-code prefix (same backup rule as au_off).
 *
 * H264AuNext: AU ends at the first slice NAL (type 1 or 5).
 * H265AuNext: AU ends at the first VCL NAL (nal_unit_type < 32).
 */
int H264AuNext(AuIter *it, size_t *au_off, size_t *au_len,
               size_t *slice_off_opt);
int H265AuNext(AuIter *it, size_t *au_off, size_t *au_len,
               size_t *slice_off_opt);

/* Slice-NAL locator within a single Annex-B AU buffer.
 *   - `*slice_off`  is the offset of the slice NAL's start-code prefix
 *     within `au` (3-byte SC offset; caller can probe `au[*slice_off+2]`
 *     to discover whether it's 3- or 4-byte and adjust).
 *   - `*slice_size` is the byte count from `*slice_off` to `len`.
 * Returns 0 on success, non-zero if no slice NAL was found.
 */
int H264FindSliceNal(const uint8_t *au, size_t len,
                     size_t *slice_off, size_t *slice_size);
int H265FindSliceNal(const uint8_t *au, size_t len,
                     size_t *slice_off, size_t *slice_size);

#ifdef __cplusplus
}  /* extern "C" */
#endif
