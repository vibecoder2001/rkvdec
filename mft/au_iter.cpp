/* mft/au_iter.cpp — see au_iter.h. */
#include "au_iter.h"

#include <limits.h>

/* Find next 3- or 4-byte Annex-B start code at or after `from`.
 * Returns offset of the first NAL header byte (= byte after start
 * code), or SIZE_MAX. */
static size_t find_start_code(const uint8_t *buf, size_t len, size_t from)
{
    for (size_t i = from; i + 3 <= len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1)
            return i + 3;
        if (i + 4 <= len &&
            buf[i] == 0 && buf[i+1] == 0 &&
            buf[i+2] == 0 && buf[i+3] == 1)
            return i + 4;
    }
    return (size_t)-1;
}

/* Back the start-code offset up by one byte if it's preceded by a zero —
 * collapses a 4-byte SC (00 00 00 01) into pointing at the leading zero. */
static size_t backup_to_4byte_sc(const uint8_t *buf, size_t sc_after_3)
{
    /* sc_after_3 is the offset of the 0x01 byte (i.e. nh - 3 + 2 + 1...). */
    /* Actually: caller passes nh - 3 (offset of first 0x00 in 3-byte view).
     * If buf[that-1] == 0, this is the leading zero of a 4-byte SC; back up. */
    if (sc_after_3 > 0 && buf[sc_after_3 - 1] == 0)
        return sc_after_3 - 1;
    return sc_after_3;
}

static int h264_nal_is_slice(uint8_t hdr0)
{
    uint8_t t = hdr0 & 0x1F;
    return t == 1 || t == 5;
}

/* HEVC VCL: nal_unit_type = (byte0 >> 1) & 0x3F, < 32. */
static int h265_nal_is_slice(uint8_t hdr0)
{
    return ((hdr0 >> 1) & 0x3F) < 32;
}

void AuIter_Init(AuIter *it, const uint8_t *buf, size_t len)
{
    it->buf = buf;
    it->len = len;
    it->pos = 0;
}

/* Shared walker parameterised by per-codec slice predicate. */
static int au_next_common(AuIter *it,
                          int (*is_slice)(uint8_t),
                          size_t *au_off, size_t *au_len,
                          size_t *slice_off_opt)
{
    if (it->pos >= it->len) return 0;
    size_t first_sc = find_start_code(it->buf, it->len, it->pos);
    if (first_sc == (size_t)-1) return 0;

    size_t sc_start = backup_to_4byte_sc(it->buf, first_sc - 3);

    size_t nh = first_sc;
    size_t end = it->len;
    size_t slice_nh = 0;
    int found = 0;
    while (nh < it->len) {
        if (is_slice(it->buf[nh])) {
            slice_nh = nh;
            size_t nxt = find_start_code(it->buf, it->len, nh + 1);
            if (nxt == (size_t)-1) {
                end = it->len;
            } else {
                end = backup_to_4byte_sc(it->buf, nxt - 3);
            }
            found = 1;
            break;
        }
        size_t nxt = find_start_code(it->buf, it->len, nh + 1);
        if (nxt == (size_t)-1) break;
        nh = nxt;
    }
    if (!found) return 0;

    *au_off = sc_start;
    *au_len = end - sc_start;
    if (slice_off_opt) {
        *slice_off_opt = backup_to_4byte_sc(it->buf, slice_nh - 3);
    }
    it->pos = end;
    return 1;
}

int H264AuNext(AuIter *it, size_t *au_off, size_t *au_len,
               size_t *slice_off_opt)
{
    return au_next_common(it, h264_nal_is_slice,
                          au_off, au_len, slice_off_opt);
}

int H265AuNext(AuIter *it, size_t *au_off, size_t *au_len,
               size_t *slice_off_opt)
{
    return au_next_common(it, h265_nal_is_slice,
                          au_off, au_len, slice_off_opt);
}

/* Single-AU slice locator.  `*slice_off` lands on the leading 0x00 of the
 * matched start code (3- or 4-byte); caller probes `au[*slice_off+2]` to
 * disambiguate.  Matches the pre-refactor decode_engine static
 * find_slice_nal_h264 contract verbatim. */
static int find_slice_nal_common(const uint8_t *au, size_t len,
                                 int (*is_slice)(uint8_t),
                                 size_t *out_off, size_t *out_size)
{
    for (size_t i = 0; i + 4 < len; i++) {
        int sc3 = (au[i] == 0 && au[i+1] == 0 && au[i+2] == 1);
        int sc4 = (au[i] == 0 && au[i+1] == 0 && au[i+2] == 0 && au[i+3] == 1);
        if (!sc3 && !sc4) continue;
        size_t hdr_off = i + (sc3 ? 3 : 4);
        if (hdr_off >= len) return 1;
        if (is_slice(au[hdr_off])) {
            *out_off  = i;
            *out_size = len - i;
            return 0;
        }
        i = hdr_off;   /* skip past this header byte */
    }
    return 1;
}

int H264FindSliceNal(const uint8_t *au, size_t len,
                     size_t *slice_off, size_t *slice_size)
{
    return find_slice_nal_common(au, len, h264_nal_is_slice,
                                 slice_off, slice_size);
}

int H265FindSliceNal(const uint8_t *au, size_t len,
                     size_t *slice_off, size_t *slice_size)
{
    return find_slice_nal_common(au, len, h265_nal_is_slice,
                                 slice_off, slice_size);
}
