/* mft/avcc_to_annexb.h — AVCC/HVCC length-prefixed NAL → Annex-B converter.
 *
 * MP4 / fragmented MP4 / WebM containers deliver H.264/HEVC bitstream as
 * length-prefixed NALs (typically 4-byte big-endian length, no start
 * codes).  Our parser_glue.cpp / parser_glue_h265.cpp expect Annex-B
 * framing.  This module rewrites a per-sample buffer at the boundary so
 * the existing parser/regbuilder/engine paths stay unchanged.
 *
 * Scope:
 *   - Per-sample frame data only.  Container-level extradata boxes
 *     (avcC / hvcC) carry SPS/PPS separately and are parsed by the
 *     MFT shell from the input media type — not here.
 *   - Emulation-prevention bytes are present in BOTH AVCC and Annex-B
 *     raw NAL bodies; only the framing differs.  We do not touch NAL
 *     payload bytes.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AVCC_LEN_4 = 4,
    AVCC_LEN_2 = 2,
    AVCC_LEN_1 = 1,
} AvccLenSize;

/* Convert an AVCC-framed buffer (concatenation of length-prefixed NALs)
 * to an Annex-B-framed buffer (4-byte start-code-prefixed NALs).  Caller
 * owns `out` of size `out_capacity`; returns bytes written, or -1 on
 * malformed input or insufficient capacity.
 *
 * AVCC NAL framing:  [length:N bytes BE][NAL bytes:length] [length:N][NAL] ...
 * Annex-B framing:    00 00 00 01 [NAL bytes] 00 00 00 01 [NAL bytes] ...
 *
 * Size guarantee: out_capacity >= len + (max_nal_count * 4) is always
 * enough.  Practically out_capacity = len + 64 covers any real stream
 * (AVCC4 length field is the same width as the Annex-B start code, so
 * the output is exactly the same size as the input). */
int AvccToAnnexB(const uint8_t *in, size_t len,
                 AvccLenSize len_size,
                 uint8_t *out, size_t out_capacity);

/* Detect framing automatically from the first few bytes.  Returns
 * FRAMING_ANNEXB if input starts with a 3- or 4-byte start code, else
 * FRAMING_AVCC4.  Used in tests when the source doesn't say. */
typedef enum {
    FRAMING_ANNEXB = 0,
    FRAMING_AVCC4  = 1,
} NalFraming;

NalFraming DetectNalFraming(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
