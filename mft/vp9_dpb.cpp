// mft/vp9_dpb.cpp
#include "vp9_dpb.h"

namespace vp9 {

void Vp9Dpb_Update(DpbCtx &dpb, const PicParams &pp,
                   uint64_t cur_frame_handle, uint64_t cur_colmv_handle,
                   uint32_t cur_width, uint32_t cur_height,
                   uint8_t  cur_bit_depth)
{
    DpbSlot s{};
    s.valid        = true;
    s.frame_handle = cur_frame_handle;
    s.colmv_handle = cur_colmv_handle;
    s.width        = cur_width;
    s.height       = cur_height;
    s.bit_depth    = cur_bit_depth;
    /* Slot POC matches the value the regbuilder wrote to reg65 for this
     * frame (= next_poc + 1; BSP semantics start POCs at 1, not 0). */
    s.poc          = dpb.next_poc + 1;

    for (int i = 0; i < kNumRefFrames; ++i) {
        if ((pp.refresh_frame_flags >> i) & 1) {
            dpb.slots[i] = s;
        }
    }
    dpb.next_poc++;
}

uint64_t Vp9Dpb_ShowExisting(const DpbCtx &dpb, const PicParams &pp)
{
    uint8_t idx = pp.show_existing_frame_idx;
    if (idx >= kNumRefFrames) return 0;
    if (!dpb.slots[idx].valid) return 0;
    return dpb.slots[idx].frame_handle;
}

} // namespace vp9
