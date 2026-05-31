/* mft/av1_parser.h — Clean-room AV1 OBU header parser.
 *
 * Parses sequence_header_obu and frame_header_obu / frame_obu payloads and
 * produces Dav1dSequenceHeader / Dav1dFrameHeader structs so that the
 * existing regbuilder_av1.cpp consumer requires zero changes.
 *
 * Only dav1d/headers.h is included for struct definitions — no dav1d
 * library functions are called.  The dav1d library link is NOT required.
 *
 * Spec reference: AV1 Bitstream & Decoding Process Specification v1.0.0
 * §5.5 (sequence_header_obu) / §5.9 (uncompressed_header).
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

/* C++ only header — dav1d/headers.h uses C linkage internally */
#ifdef __cplusplus

#include <stdbool.h>

extern "C" {
#include <dav1d/headers.h>
}

extern "C" {
#endif /* __cplusplus */

/* ---------------------------------------------------------------------------
 * Per-slot saved state for primary_ref_frame inheritance and
 * show_existing_frame width/height/order_hint propagation.
 * See AV1 spec §5.9.2 / §5.11.34.
 * ------------------------------------------------------------------------- */
typedef struct Av1SavedFrameState {
    bool             valid;
    Dav1dFrameHeader saved;  /* full frame_hdr snapshot for inheritance */
} Av1SavedFrameState;

/* ---------------------------------------------------------------------------
 * Av1ParseSeqHeader
 *
 * Parse one sequence_header_obu payload (after the OBU header byte(s) and
 * the optional leb128 size field have already been consumed by the caller).
 *
 *   obu_payload     — pointer to first byte of the OBU payload
 *   obu_payload_len — payload length in bytes
 *   out             — receives the parsed sequence header (zeroed on entry
 *                     by this function)
 *
 * Returns 0 on success, -1 on parse error or unsupported stream.
 * ------------------------------------------------------------------------- */
int Av1ParseSeqHeader(const uint8_t *obu_payload, size_t obu_payload_len,
                      Dav1dSequenceHeader *out);

/* ---------------------------------------------------------------------------
 * Av1ParseFrameHeader
 *
 * Parse one frame_header_obu or frame_obu payload.
 *
 *   obu_payload         — pointer to first byte of the OBU payload
 *   obu_payload_len     — payload length in bytes
 *   obu_is_frame_type   — true  = OBU_FRAME (type 6), i.e. the header is
 *                                 followed by tile_group_obu data in the
 *                                 same OBU payload.
 *                         false = OBU_FRAME_HDR (type 3) or
 *                                 OBU_REDUNDANT_FRAME_HDR (type 7).
 *   seq                 — current sequence header (must be non-NULL)
 *   prev_states[8]      — per-DPB-slot saved frame headers used for
 *                         primary_ref_frame inheritance.  May be NULL if
 *                         the caller does not maintain this state (fields
 *                         that would be inherited will be left at their
 *                         default values instead).
 *   out                 — receives the parsed frame header (zeroed first).
 *                         This is a pristine, unmodified Dav1dFrameHeader —
 *                         the parser does NOT depend on any local patch to
 *                         dav1d's headers.
 *   out_frame_hdr_obu_size_bytes
 *                       — optional (may be NULL).  On success receives the
 *                         byte count of the uncompressed_header() portion
 *                         (including trailing byte_alignment()), so the
 *                         caller can locate the tile_group payload within an
 *                         OBU_FRAME.  Kept separate from Dav1dFrameHeader so
 *                         this parser stays drop-in against an upstream,
 *                         unpatched dav1d (see "frame_tag_size" in MPP).
 *
 * Returns 0 on success, -1 on parse error or unsupported stream.
 * ------------------------------------------------------------------------- */
int Av1ParseFrameHeader(const uint8_t       *obu_payload,
                        size_t               obu_payload_len,
                        bool                 obu_is_frame_type,
                        const Dav1dSequenceHeader *seq,
                        const Av1SavedFrameState   prev_states[8],
                        Dav1dFrameHeader    *out,
                        uint32_t            *out_frame_hdr_obu_size_bytes);

/* ---------------------------------------------------------------------------
 * Av1UpdateSavedStates
 *
 * After a successful decode kick, propagate the current frame header into
 * every DPB slot whose bit is set in cur->refresh_frame_flags.
 *
 * Call this once per successfully decoded frame, after the hardware kick
 * completes (matching rkmpp_av1_dpb_post_decode semantics for the parser
 * layer).
 * ------------------------------------------------------------------------- */
void Av1UpdateSavedStates(Av1SavedFrameState       states[8],
                          const Dav1dFrameHeader   *cur);

#ifdef __cplusplus
} /* extern "C" */
#endif
