/* mft/avcc_to_annexb.cpp — see header.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "avcc_to_annexb.h"

#include <string.h>

/* Annex-B 4-byte start code.  We always emit the 4-byte form (00 00 00
 * 01) rather than the 3-byte form; the existing parsers accept both and
 * 4-byte gives byte-for-byte size parity with AVCC4 input. */
static const uint8_t kStartCode[4] = { 0x00, 0x00, 0x00, 0x01 };

/* Read a big-endian length field of `n` bytes (1, 2, or 4).  `n` MUST
 * be a validated AvccLenSize value — AvccToAnnexB's entry-guard rejects
 * anything else.  The default branch is kept only because some compilers
 * warn on a missing one even with a complete enum switch; it's a defensive
 * AVCC_LEN_4 read, but the caller never reaches it.  Review parser Medium #3. */
static uint32_t read_be_len(const uint8_t *p, AvccLenSize n) {
    switch (n) {
        case AVCC_LEN_1:
            return (uint32_t)p[0];
        case AVCC_LEN_2:
            return ((uint32_t)p[0] << 8) | (uint32_t)p[1];
        case AVCC_LEN_4:
            return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                   ((uint32_t)p[2] <<  8) | ((uint32_t)p[3]      );
        default:
            /* Should be unreachable — AvccToAnnexB's entry-guard rejects
             * invalid sizes.  Return 0 here so a bug that bypasses the
             * guard fails loudly with "zero-length NAL" rather than
             * silently mis-framing as AVCC_LEN_4.  Review parser Medium #3. */
            return 0;
    }
}

int AvccToAnnexB(const uint8_t *in, size_t len,
                 AvccLenSize len_size,
                 uint8_t *out, size_t out_capacity) {
    if (in == NULL && len != 0)            return -1;
    if (out == NULL && out_capacity != 0)  return -1;
    if (len_size != AVCC_LEN_1 &&
        len_size != AVCC_LEN_2 &&
        len_size != AVCC_LEN_4)            return -1;

    const size_t n = (size_t)len_size;
    size_t in_off  = 0;
    size_t out_off = 0;

    while (in_off < len) {
        /* Need n bytes for the length field. */
        if (len - in_off < n) return -1;

        uint32_t nal_len = read_be_len(in + in_off, len_size);
        in_off += n;

        /* Length must not overrun the buffer.  A zero-length NAL is
         * malformed (no NAL header byte). */
        if (nal_len == 0)               return -1;
        if (len - in_off < nal_len)     return -1;

        /* Emit start code + NAL body. */
        if (out_capacity - out_off < 4 + nal_len) return -1;
        memcpy(out + out_off, kStartCode, 4);
        out_off += 4;
        memcpy(out + out_off, in + in_off, nal_len);
        out_off += nal_len;

        in_off += nal_len;
    }

    /* size_t -> int return; AVCC sample buffers in MP4 are well under
     * INT_MAX in practice, but guard anyway. */
    if (out_off > 0x7FFFFFFFu) return -1;
    return (int)out_off;
}

NalFraming DetectNalFraming(const uint8_t *buf, size_t len) {
    /* Annex-B start code: 00 00 00 01 or 00 00 01 at offset 0. */
    if (len >= 4 && buf[0] == 0x00 && buf[1] == 0x00 &&
        buf[2] == 0x00 && buf[3] == 0x01) {
        return FRAMING_ANNEXB;
    }
    if (len >= 3 && buf[0] == 0x00 && buf[1] == 0x00 && buf[2] == 0x01) {
        return FRAMING_ANNEXB;
    }
    /* Anything else: assume AVCC4 (the dominant MP4 framing).  AVCC4
     * length fields for typical NALs (a few KB to ~MB) start with 0x00
     * 0x00 0x00 0x?? where ?? != 0x01, or 0x00 0x00 0x?? ?? for larger
     * NALs.  Distinguishing AVCC2/AVCC1 from AVCC4 from a buffer prefix
     * alone is unreliable; the MFT shell knows the size from hvcC/avcC. */
    return FRAMING_AVCC4;
}
