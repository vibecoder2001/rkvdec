// mft/vp9_dpb.h — VP9 reference-frame map.
//
// VP9 has 8 reference slots (kNumRefFrames).  Each decoded frame can
// refresh any subset of those slots via the 8-bit refresh_frame_flags
// field in the uncompressed header (spec §8.2).  show_existing_frame
// re-emits a previously-decoded slot without running the codec.
#pragma once

#include "vp9_types.h"
#include <cstdint>

namespace vp9 {

struct DpbSlot {
    bool     valid;
    uint64_t frame_handle;   // user-pinned NV12 buffer cookie
    uint64_t colmv_handle;
    uint32_t width, height;  // coded dims of the frame in this slot
    uint8_t  bit_depth;
    int32_t  poc;            // synthesized frame order count
};

struct DpbCtx {
    DpbSlot slots[kNumRefFrames];
    int32_t next_poc;
};

// Refresh DPB slots per pp.refresh_frame_flags.  The just-decoded frame
// is written into every slot whose bit is set.  POC advances by one per
// call (regardless of how many slots are refreshed).
void Vp9Dpb_Update(DpbCtx &dpb, const PicParams &pp,
                   uint64_t cur_frame_handle, uint64_t cur_colmv_handle,
                   uint32_t cur_width, uint32_t cur_height,
                   uint8_t  cur_bit_depth);

// For show_existing_frame: return the frame_handle of the slot pointed
// to by pp.show_existing_frame_idx, or 0 if that slot is invalid /
// the index is out of range.
uint64_t Vp9Dpb_ShowExisting(const DpbCtx &dpb, const PicParams &pp);

} // namespace vp9
