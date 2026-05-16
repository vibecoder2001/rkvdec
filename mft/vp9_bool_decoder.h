#pragma once
#include <cstdint>
#include <cstddef>

namespace vp9 {

// VP9 boolean arithmetic decoder — VP9 spec §9.2.
// State: (buf, sz, value, range, bits_left).
class BoolDecoder {
public:
    void init(const uint8_t *buf, size_t sz);
    int  decode_bool(uint8_t prob);    // spec read_bool(p)
    int  decode_literal(int bits);     // n successive prob=128 bools, MSB first
    bool   ok() const { return !error_; }
    /* Approximate bit position consumed — buffer bytes consumed minus
     * the un-shifted bits still in the working register.  Off by a few
     * bits but stable for cross-decoder bit-position diffing. */
    size_t bit_pos() const {
        return pos_ * 8 - (size_t)(bits_left_ >= 0 ? bits_left_ : 0);
    }

private:
    void fill();

    const uint8_t *buf_      = nullptr;
    size_t         sz_       = 0;
    size_t         pos_      = 0;
    uint32_t       value_    = 0;
    uint32_t       range_    = 255;
    int            bits_left_ = 0;
    bool           error_    = false;
};

} // namespace vp9
