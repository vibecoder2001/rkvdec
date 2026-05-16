// VP9 boolean arithmetic decoder — VP9 spec §9.2.1 / §9.2.2.
//
// State machine matches libvpx / BSP MPP `vp9d_reader_init` +
// `vp9d_read`: 24-bit value buffer pre-loaded at init via MPP_RB24,
// 16-bit refill triggered when `bits_left` goes non-negative, range
// renormalized via `norm_shift_table`.  Our earlier 8-bit-buffer
// implementation made the `bigsplit` comparison fire prematurely on
// real bitstreams (first decode_bool returned 0 where BSP returns 1).
#include "vp9_bool_decoder.h"

namespace vp9 {

// VP9 range-renormalization shift table (libvpx vp9_norm).
// Index by range; result is the number of left-shifts needed to bring
// range back into [128, 255].  range == 0 is invalid but we map to 7
// for safety.
static const uint8_t kNormShift[256] = {
    0, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

// 24-bit big-endian read; pads with zeros if buffer short.
static uint32_t rb24(const uint8_t *p, size_t avail) {
    uint32_t v = 0;
    if (avail >= 1) v |= ((uint32_t)p[0]) << 16;
    if (avail >= 2) v |= ((uint32_t)p[1]) <<  8;
    if (avail >= 3) v |= ((uint32_t)p[2]) <<  0;
    return v;
}

static uint32_t rb16(const uint8_t *p, size_t avail) {
    uint32_t v = 0;
    if (avail >= 1) v |= ((uint32_t)p[0]) << 8;
    if (avail >= 2) v |= ((uint32_t)p[1]) << 0;
    return v;
}

void BoolDecoder::fill() {
    /* Legacy entry point; unused now that decode_bool refills inline. */
}

void BoolDecoder::init(const uint8_t *buf, size_t sz) {
    buf_      = buf;
    sz_       = sz;
    pos_      = 0;
    range_    = 255;
    /* value_ holds the 24-bit pre-fetch; bits_left_ starts at -16 so
     * the first 16-bit refill triggers only after 16 shifts. */
    bits_left_ = -16;
    error_    = false;

    if (sz < 3) {
        value_ = (sz >= 1 ? ((uint32_t)buf[0] << 16) : 0)
               | (sz >= 2 ? ((uint32_t)buf[1] <<  8) : 0);
        pos_ = sz;
        if (sz < 3) error_ = true;
    } else {
        value_ = rb24(buf, sz);
        pos_ = 3;
    }

    /* Consume the mandatory marker bit (must be 0 per spec). */
    if (decode_bool(128) != 0) error_ = true;
}

int BoolDecoder::decode_bool(uint8_t prob) {
    if (error_) return 0;

    /* Renormalize range to [128, 255] using lookup table. */
    int shift = kNormShift[range_ & 0xFF];
    range_     <<= shift;
    value_     <<= shift;
    bits_left_  += shift;

    /* Refill when bits_left has caught up. */
    if (bits_left_ >= 0 && pos_ < sz_) {
        size_t avail = sz_ - pos_;
        uint32_t loaded = rb16(buf_ + pos_, avail);
        pos_ += (avail >= 2) ? 2 : avail;
        /* Pack the new bits into value_ at the position bits_left_
         * (the unused low bits, which after the shift above are
         * zeros).  See libvpx vpx_reader_fill. */
        value_     |= loaded << bits_left_;
        bits_left_ -= 16;
    }

    uint32_t split = 1u + (((range_ - 1u) * prob) >> 8);
    uint32_t low_shift = split << 16;

    int bit;
    if (value_ >= low_shift) {
        bit    = 1;
        range_ -= split;
        value_ -= low_shift;
    } else {
        bit    = 0;
        range_ = split;
    }
    return bit;
}

int BoolDecoder::decode_literal(int bits) {
    int v = 0;
    for (int i = 0; i < bits; ++i)
        v = (v << 1) | decode_bool(128);
    return v;
}

} // namespace vp9
