/* mft/engine/repack_yuv.h
 *
 * Shared bit-depth-aware NV12 / NV15→P010 repack helper used by the
 * H.264, H.265, and AV1 decode paths.  Codec writes the user-visible
 * raster output to its PP module's buffer at the bit-depth-dependent
 * source stride (NV12 for 8-bit, NV15-packed for 10-bit on the
 * RK3588 vdpu34x / vdpu_av1d families); this helper unpacks/crops
 * into the contiguous, display-sized NV12 or P010 layout that the
 * MFT output sample expects.
 */
#pragma once
#include <cstdint>
#include <cstring>
#include <vector>

/* Source layout assumption (matches the BSP frame-allocator):
 *   Y plane:  src_stride * src_height_pad bytes
 *   UV plane: src_stride * (src_height_pad / 2) bytes immediately after
 * where src_stride is the bit-depth-dependent byte stride (== coded_w
 * for 8-bit NV12, == coded_w * 10/8 — 16-byte aligned — for 10-bit
 * NV15) and src_height_pad is the codec's vertically aligned height.
 *
 * Destination layout:
 *   8-bit  → packed display-height NV12:  width × height × 3/2 bytes
 *   10-bit → packed display-height P010:  width × 2 × height × 3/2,
 *            each 10-bit sample expanded into uint16 with the 10 valid
 *            bits in the upper 10 of 16 (sample16 = sample10 << 6).
 */
static inline void RepackCodecOutputToNV12orP010(
    const uint8_t *src,
    uint32_t       src_stride,
    uint32_t       src_height_pad,
    uint32_t       frame_width,
    uint32_t       frame_height,
    uint32_t       bit_depth,        /* 8 or 10 */
    std::vector<uint8_t> *out_yuv)
{
    const uint8_t *src_y  = src;
    const uint8_t *src_uv = src + (size_t)src_stride * src_height_pad;

    if (bit_depth == 8) {
        uint32_t y_disp  = frame_width * frame_height;
        uint32_t uv_disp = frame_width * (frame_height / 2u);
        out_yuv->resize(y_disp + uv_disp);
        uint8_t *dst = out_yuv->data();
        for (uint32_t r = 0; r < frame_height; r++)
            std::memcpy(dst + r * frame_width,
                        src_y + r * src_stride,
                        frame_width);
        for (uint32_t r = 0; r < frame_height / 2u; r++)
            std::memcpy(dst + y_disp + r * frame_width,
                        src_uv + r * src_stride,
                        frame_width);
        return;
    }

    /* NV15 → P010 unpack.  NV15 packs 4 little-endian 10-bit samples
     * into 5 bytes:
     *   b0 = s0[7:0]
     *   b1 = (s1[5:0] << 2) | s0[9:8]
     *   b2 = (s2[3:0] << 4) | s1[9:6]
     *   b3 = (s3[1:0] << 6) | s2[9:4]
     *   b4 = s3[9:2]
     * P010 stores each sample as uint16 with the 10 valid bits in the
     * upper 10 (sample16 = sample10 << 6). */
    uint32_t dst_stride = frame_width * 2u;
    uint32_t y_disp     = dst_stride * frame_height;
    uint32_t uv_disp    = dst_stride * (frame_height / 2u);
    out_yuv->resize(y_disp + uv_disp);
    uint16_t *dst_y  = reinterpret_cast<uint16_t *>(out_yuv->data());
    uint16_t *dst_uv = reinterpret_cast<uint16_t *>(out_yuv->data() + y_disp);
    auto unpack_row = [](const uint8_t *src_row, uint16_t *dst_row,
                         uint32_t samples) {
        uint32_t groups = samples / 4u;
        uint32_t tail   = samples - groups * 4u;
        for (uint32_t g = 0; g < groups; g++) {
            const uint8_t *s = src_row + g * 5u;
            uint16_t s0 =  (uint16_t)s[0]         | ((uint16_t)(s[1] & 0x03) << 8);
            uint16_t s1 = ((uint16_t)(s[1] >> 2)) | ((uint16_t)(s[2] & 0x0F) << 6);
            uint16_t s2 = ((uint16_t)(s[2] >> 4)) | ((uint16_t)(s[3] & 0x3F) << 4);
            uint16_t s3 = ((uint16_t)(s[3] >> 6)) | ((uint16_t)(s[4])        << 2);
            dst_row[g * 4 + 0] = (uint16_t)(s0 << 6);
            dst_row[g * 4 + 1] = (uint16_t)(s1 << 6);
            dst_row[g * 4 + 2] = (uint16_t)(s2 << 6);
            dst_row[g * 4 + 3] = (uint16_t)(s3 << 6);
        }
        if (tail) {
            const uint8_t *s = src_row + groups * 5u;
            uint16_t samp[4] = {
                (uint16_t)( (uint16_t)s[0]         | ((uint16_t)(s[1] & 0x03) << 8) ),
                (uint16_t)( ((uint16_t)(s[1] >> 2)) | ((uint16_t)(s[2] & 0x0F) << 6) ),
                (uint16_t)( ((uint16_t)(s[2] >> 4)) | ((uint16_t)(s[3] & 0x3F) << 4) ),
                0
            };
            for (uint32_t i = 0; i < tail; i++)
                dst_row[groups * 4 + i] = (uint16_t)(samp[i] << 6);
        }
    };
    for (uint32_t r = 0; r < frame_height; r++)
        unpack_row(src_y + r * src_stride,
                   dst_y + r * frame_width,
                   frame_width);
    for (uint32_t r = 0; r < frame_height / 2u; r++)
        unpack_row(src_uv + r * src_stride,
                   dst_uv + r * frame_width,
                   frame_width);
}

/* Byte stride emitted by the codec PP module for a given coded width
 * and bit depth.  Matches the rkmpp BSP frame allocator's hal_hor_align
 * pattern (mpp_buf_slot.c:300) — coded_w * depth/8, then 16-aligned. */
static inline uint32_t CodecOutputStride(uint32_t coded_w, uint32_t bit_depth)
{
    return ((coded_w * bit_depth + 7u) / 8u + 15u) & ~15u;
}
