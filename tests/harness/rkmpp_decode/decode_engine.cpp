/* tests/harness/rkmpp_decode/decode_engine.cpp */
#include "decode_engine.h"
#include "../../../shared/rkmpp_ioctl.h"

#include <setupapi.h>

#include <cstdio>
#include <cstring>

static int Fail(const char *m, DWORD ec = 0) {
    std::fprintf(stderr, "decode_engine: %s (%lu)\n", m, ec);
    return 1;
}

static int OpenDevice(HANDLE *out) {
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return Fail("SetupDiGetClassDevsW", GetLastError());

    SP_DEVICE_INTERFACE_DATA ifd{ sizeof(ifd) };
    DWORD idx = 0;
    while (SetupDiEnumDeviceInterfaces(set, nullptr, &GUID_DEVINTERFACE_RKMPP,
                                       idx++, &ifd))
    {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &need, nullptr);
        std::vector<uint8_t> buf(need);
        auto *det = reinterpret_cast<PSP_DEVICE_INTERFACE_DETAIL_DATA_W>(buf.data());
        det->cbSize = sizeof(*det);
        if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, det, need, nullptr, nullptr))
            continue;

        HANDLE h = CreateFileW(det->DevicePath, GENERIC_READ | GENERIC_WRITE,
                               FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                               OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) continue;

        /* Caps probe — only accept RKCP3550 (rkv-decoder-v2 core 0). */
        RKMPP_CAPS caps{};  caps.StructSize = sizeof(caps);
        DWORD got = 0;
        if (DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                            &caps, sizeof(caps), &got, nullptr) &&
            caps.Hid == 0x3550 && (caps.SupportedCodecs & RKMPP_CODEC_H264)) {
            *out = h;
            SetupDiDestroyDeviceInfoList(set);
            return 0;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    return Fail("no RKCP3550 H264 device found");
}

static int AllocBuf(HANDLE dev, uint32_t size, RKMPP_BUFFER_USAGE usage,
                    DecodeEngine::Buf *out) {
    RKMPP_ALLOC_BUFFER_IN  in{};
    RKMPP_ALLOC_BUFFER_OUT o{};
    in.StructSize = sizeof(in);
    in.Size       = size;
    in.Usage      = (UINT32)usage;
    DWORD got = 0;
    if (!DeviceIoControl(dev, IOCTL_RKMPP_ALLOC_BUFFER, &in, sizeof(in),
                         &o, sizeof(o), &got, nullptr))
        return Fail("ALLOC_BUFFER", GetLastError());
    out->handle  = o.BufferHandle;
    out->iova    = o.Iova;
    out->user_va = o.UserVa;
    out->size    = o.SizeRoundedUp;
    return 0;
}

static void FreeBuf(HANDLE dev, DecodeEngine::Buf *b) {
    if (!b->handle) return;
    RKMPP_FREE_BUFFER_IN in{ b->handle };
    DWORD got = 0;
    DeviceIoControl(dev, IOCTL_RKMPP_FREE_BUFFER, &in, sizeof(in),
                    nullptr, 0, &got, nullptr);
    *b = {};
}

int DecodeEngine_Init(DecodeEngine *e, uint32_t width, uint32_t height)
{
    e->frame_width  = width;
    e->frame_height = height;

    if (OpenDevice(&e->device) != 0) return 1;

    /* RCB sizing — one consolidated buffer with N sub-regions. */
    uint32_t rcb_total = H264GetRcbBufferSizes(e->rcb_info, width, height);

    /* NV12 + small slack for stride alignment. */
    uint32_t frame_bytes = width * height * 3u / 2u;

    /* Colmv buffer: BSP `vdpu34x_get_colmv_size` (compressed path,
     * which our regbuilder enables for frame_mbs_only streams):
     *   ctu_size=16, colmv_bytes=16, colmv_size=4
     *   segment_w = 64*colmv_size*colmv_size / ctu_size = 64
     *   segment_h = ctu_size = 16
     *   seg_cnt_w = align(width,64)/64, seg_cnt_h = align(height,16)/16
     *   total = align(seg_cnt_w,16)*seg_cnt_h + seg_cnt_w*seg_cnt_h*64*16
     *   align(total, 128)
     * For 1280x720 this is ~900KB.  Our previous 230KB allocation was
     * 4× too small — codec wrote past the end mid-frame and stalled. */
    auto align_up = [](uint32_t v, uint32_t a) { return (v + a - 1) & ~(a - 1); };
    uint32_t seg_cnt_w   = align_up(width, 64) / 64;
    uint32_t seg_cnt_h   = align_up(height, 16) / 16;
    uint32_t seg_head    = align_up(seg_cnt_w, 16) * seg_cnt_h;
    uint32_t seg_payload = seg_cnt_w * seg_cnt_h * 64u * 16u;
    uint32_t colmv_bytes = align_up(seg_head + seg_payload, 128);
    if (colmv_bytes < 4096) colmv_bytes = 4096;
    uint32_t frame_alloc = frame_bytes;
    std::printf("alloc sizes: frame=%u rcb=%u colmv=%u\n",
                frame_bytes, rcb_total, colmv_bytes);

#define ALLOC(b, sz, usage) do { if (AllocBuf(e->device, (sz), (usage), &(b))) return 1; } while (0)
    ALLOC(e->bitstream,    1u << 20,                       RkMppBufferUsageBitstreamInput);
    ALLOC(e->cabac_init,   RKH264_CABAC_INIT_SIZE + RKH264_TABLE_TAIL_PAD,
                                                           RkMppBufferUsageScratch);
    ALLOC(e->pps_table,    RKH264_SPSPPS_UNIT_SIZE + RKH264_TABLE_TAIL_PAD,
                                                           RkMppBufferUsageScratch);
    ALLOC(e->rps_table,    RKH264_RPS_SIZE + RKH264_TABLE_TAIL_PAD,
                                                           RkMppBufferUsageScratch);
    ALLOC(e->scaling_list, RKH264_SCALING_LIST_SIZE + RKH264_TABLE_TAIL_PAD,
                                                           RkMppBufferUsageScratch);
    ALLOC(e->rcb,          rcb_total > 4096 ? rcb_total : 4096,
                                                           RkMppBufferUsageScratch);
    ALLOC(e->error_ref,    frame_alloc,                    RkMppBufferUsageReferenceFrame);
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        ALLOC(e->pool_output[i], frame_alloc, RkMppBufferUsageOutputFrame);
        ALLOC(e->pool_colmv[i],  colmv_bytes, RkMppBufferUsageScratch);
    }
#undef ALLOC

    /* Pre-fill CABAC table. */
    {
        size_t nwords = 0;
        const uint32_t *tab = H264GetCabacInitTable(&nwords);
        std::memcpy(e->cabac_init.user_va, tab, nwords * sizeof(uint32_t));
    }

    /* Init DPB pool. */
    DpbPoolEntry pool[DecodeEngine::kPoolSize];
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        pool[i].output_frame = e->pool_output[i].handle;
        pool[i].colmv        = e->pool_colmv[i].handle;
    }
    if (Dpb_Init(&e->dpb, pool, DecodeEngine::kPoolSize) != DPB_OK)
        return Fail("Dpb_Init");

    e->scratch.resize(2u << 20);
    return 0;
}

void DecodeEngine_Shutdown(DecodeEngine *e)
{
    if (e->device == INVALID_HANDLE_VALUE) return;
    FreeBuf(e->device, &e->bitstream);
    FreeBuf(e->device, &e->cabac_init);
    FreeBuf(e->device, &e->pps_table);
    FreeBuf(e->device, &e->rps_table);
    FreeBuf(e->device, &e->scaling_list);
    FreeBuf(e->device, &e->rcb);
    FreeBuf(e->device, &e->error_ref);
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        FreeBuf(e->device, &e->pool_output[i]);
        FreeBuf(e->device, &e->pool_colmv[i]);
    }
    CloseHandle(e->device);
    e->device = INVALID_HANDLE_VALUE;
}

/* Find the first H.264 slice NAL (type 1 or 5) in an Annex-B AU.  Returns
 * the byte offset of the start code (so the slice prefix + header are
 * intact for the hardware) and the byte count from there to AU end. */
static int find_slice_nal(const uint8_t *au, size_t len,
                          size_t *out_off, size_t *out_size)
{
    for (size_t i = 0; i + 4 < len; i++) {
        bool sc3 = (au[i] == 0 && au[i+1] == 0 && au[i+2] == 1);
        bool sc4 = (au[i] == 0 && au[i+1] == 0 && au[i+2] == 0 && au[i+3] == 1);
        if (!sc3 && !sc4) continue;
        size_t hdr_off = i + (sc3 ? 3 : 4);
        if (hdr_off >= len) return 1;
        uint8_t nal_type = au[hdr_off] & 0x1F;
        if (nal_type == 1 || nal_type == 5) {
            *out_off  = i;
            *out_size = len - i;
            return 0;
        }
        i = hdr_off;   /* skip past this header byte */
    }
    return 1;
}

int DecodeEngine_DecodeOne(DecodeEngine *e,
                           const uint8_t *au, size_t au_len,
                           std::vector<uint8_t> *out_yuv)
{
    if (au_len > e->bitstream.size) return Fail("AU larger than bitstream buf");

    /* 1. Locate the slice NAL.  Strip leading 0x00 byte if 4-byte startcode
     * so the staged prefix is "00 00 01 <NAL_header>" (matches BSP). */
    size_t slice_off = 0, slice_size = 0;
    if (find_slice_nal(au, au_len, &slice_off, &slice_size) != 0)
        return Fail("no slice NAL found");
    size_t sc_len   = (au[slice_off + 2] == 1) ? 3u : 4u;
    size_t skip     = sc_len - 3u;
    size_t copy_off = slice_off + skip;
    size_t copy_len = slice_size - skip;

    /* Stage slice directly at offset 0 — the original 4 KiB pre-pad
     * theory (CABAC backward prefetch from RLC_BASE) was wrong; the
     * 0xae0/0xb00 fault was actually CABAC-table reads with masked
     * high bits, fixed by placing CABAC at iova 0 in the iommu. */
    if (copy_len > e->bitstream.size)
        return Fail("slice larger than bitstream buf");
    std::printf("slice NAL at offset 0x%zx (sc=%zu), staged %zu bytes\n",
                slice_off, sc_len, copy_len);
    std::memcpy(e->bitstream.user_va, au + copy_off, copy_len);

    /* 2. Parse. */
    H264ParseResult parsed{};
    H264ParseStatus s = H264ParseAccessUnit(au, au_len,
                                            e->scratch.data(), e->scratch.size(),
                                            &parsed);
    if (s != H264_PARSE_OK) {
        std::fprintf(stderr, "parser status=%d (need_more=1 invalid=2 unsupported=3)\n",
                     (int)s);
        return Fail("parser failed");
    }

    /* Validate dimensions match what we allocated for. */
    uint32_t w_px = ((uint32_t)parsed.sps.pic_width_in_mbs_minus1 + 1) * 16;
    uint32_t h_px = ((uint32_t)parsed.sps.pic_height_in_map_units_minus1 + 1) * 16;
    if (w_px != e->frame_width || h_px != e->frame_height) {
        std::fprintf(stderr, "stream %ux%u, harness inited for %ux%u\n",
                     w_px, h_px, e->frame_width, e->frame_height);
        return Fail("dim mismatch");
    }

    /* 3. DPB selection. */
    DpbSelection sel{};
    if (Dpb_Select(&e->dpb, &parsed, &sel) != DPB_OK)
        return Fail("Dpb_Select failed");

    /* 4. Pack tables. */
    H264PackSpsPpsUnit(static_cast<uint8_t *>(e->pps_table.user_va),
                       &parsed.sps, &parsed.pps,
                       sel.dpb_entries, /*field_pic=*/0);
    H264PackFrameRps(static_cast<uint8_t *>(e->rps_table.user_va),
                     parsed.decode.frame_num,
                     parsed.sps.log2_max_frame_num_minus4,
                     sel.dpb_entries, sel.ref_lists);
    H264PackScalingList(static_cast<uint8_t *>(e->scaling_list.user_va),
                        &parsed.scaling_matrix,
                        parsed.has_scaling_matrix);

    /* 5. Compose buffer-refs for the regbuilder.  Slice was staged at
     * offset 0 (above), so RLC_BASE = bitstream_iova with no byte
     * offset added. */
    H264BufferRefs refs{};
    refs.bitstream        = e->bitstream.handle;
    refs.bitstream_offset = 0;
    refs.bitstream_size   = (uint32_t)copy_len;
    refs.output_frame     = sel.current_output;
    refs.colmv_cur        = sel.current_colmv;
    refs.error_ref        = e->error_ref.handle;
    refs.pps_table        = e->pps_table.handle;
    refs.rps_table        = e->rps_table.handle;
    refs.cabac_init_table = e->cabac_init.handle;
    /* Always provide the scaling-list iova.  Even when the stream has no
     * scaling matrix, the codec hardware fetches from SCANLIST_ADDR and
     * faults if the address is 0 — verified empirically (page-fault at
     * iova ~0xae0/0xe00 = base 0 + internal offset).  H264PackScalingList
     * fills the buffer with the H.264 default flat=16 lists, which the
     * stream-spec semantics treat as a no-op. */
    refs.scaling_list     = e->scaling_list.handle;
    /* All 10 RCB sub-regions share one backing buffer; per-region
     * iova offsets come from H264GetRcbBufferSizes. */
    for (int i = 0; i < RKH264_RCB_COUNT; i++) {
        refs.rcb[i]        = e->rcb.handle;
        refs.rcb_offset[i] = e->rcb_info[i].offset;
    }
    /* Ref frames + ref colmv from DPB selection. */
    for (int i = 0; i < 16; i++) {
        refs.refs[i]      = sel.refs[i];
        refs.ref_colmv[i] = sel.ref_colmv[i];
    }

    /* 6. Build register list. */
    H264RegWriteList list{};
    H264RegBuildStatus rs = H264BuildRegisterList(&parsed, &refs,
                                                  sel.current_slot, &list);
    if (rs != H264_REGBUILD_OK) {
        std::fprintf(stderr, "regbuilder status=%d\n", (int)rs);
        return Fail("regbuilder failed");
    }

    /* 7. Submit. */
    RKMPP_SUBMIT_JOB_IN  sin{};
    RKMPP_SUBMIT_JOB_OUT sout{};
    sin.StructSize    = sizeof(sin);
    sin.RegWriteCount = list.count;
    sin.TimeoutMs     = 1000;
    std::memcpy(sin.Writes, list.entries, list.count * sizeof(RKMPP_REG_WRITE));

    DWORD got = 0;
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_SUBMIT_JOB, &sin, sizeof(sin),
                         &sout, sizeof(sout), &got, nullptr))
        return Fail("SUBMIT_JOB", GetLastError());

    /* 7b. Peek the job — kernel has applied iova substitution; print the
     * resolved (offset, value) pairs so we can verify register values
     * against the BSP layout when triaging. */
    {
        RKMPP_PEEK_JOB_IN  pin{ sout.JobId };
        RKMPP_PEEK_JOB_OUT pout{};
        if (DeviceIoControl(e->device, IOCTL_RKMPP_PEEK_JOB, &pin, sizeof(pin),
                            &pout, sizeof(pout), &got, nullptr)) {
            std::printf("--- post-subst register list (%u entries) ---\n",
                        pout.RegWriteCount);
            for (uint32_t i = 0; i < pout.RegWriteCount; i++) {
                std::printf("  [%2u] off=0x%03x val=0x%08x  (idx %u)%s\n",
                            i,
                            pout.Writes[i].Offset, pout.Writes[i].Value,
                            pout.Writes[i].Offset / 4,
                            pout.Writes[i].BufferHandle ? " <iova>" : "");
            }
        }
    }

    /* 8. Wait. */
    RKMPP_WAIT_JOB_IN  win{ sout.JobId, 1000, 0 };
    RKMPP_WAIT_JOB_OUT wout{};
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                         &wout, sizeof(wout), &got, nullptr))
        return Fail("WAIT_JOB", GetLastError());
    std::printf("decode: jobid=%llu status=0x%08x hwstatus=0x%08x writes=%u\n",
                (unsigned long long)sout.JobId, wout.Status,
                wout.HardwareStatus, list.count);
    /* If hwstatus has DEC_RDY_STA (bit 2), the codec wrote *something*
     * to the output buffer.  Even with error flags also latched (bit 4
     * dec_error_sta, bit 5 timeout, etc.) the YUV is worth dumping to
     * inspect — partial decode tells us where the codec gave up. */
    const uint32_t kRdySta = 1u << 2;
    bool have_output = (wout.HardwareStatus & kRdySta) != 0;
    if (wout.Status != 0 && !have_output) {
        std::fprintf(stderr, "decode reported non-success status (no output)\n");
        return 5;
    }
    if (wout.Status != 0) {
        std::fprintf(stderr, "decode partial: hwstatus=0x%08x has DEC_RDY but other flags too — dumping anyway\n",
                     wout.HardwareStatus);
    }

    /* 9. Copy out.  Also probe error_ref / colmv_cur buffers — if codec
     * wrote output to one of those instead of decout_base, the harness
     * picks it up so we can see the actual decoded data. */
    {
        uint32_t bytes = e->frame_width * e->frame_height * 3u / 2u;
        auto count_nonzero = [](const void *p, size_t n) -> size_t {
            const uint8_t *b = (const uint8_t*)p; size_t k = 0;
            for (size_t i = 0; i < n; i++) if (b[i] != 0) k++;
            return k;
        };
        size_t nz_dec = count_nonzero(e->pool_output[sel.current_slot].user_va, bytes);
        size_t nz_err = count_nonzero(e->error_ref.user_va, bytes);
        size_t nz_cmv = count_nonzero(e->pool_colmv[sel.current_slot].user_va,
                                       e->pool_colmv[sel.current_slot].size);
        std::printf("nonzero bytes: decout=%zu error_ref=%zu colmv_cur=%zu\n",
                    nz_dec, nz_err, nz_cmv);
    }
    if (out_yuv) {
        uint32_t bytes = e->frame_width * e->frame_height * 3u / 2u;
        out_yuv->resize(bytes);
        std::memcpy(out_yuv->data(), e->pool_output[sel.current_slot].user_va,
                    bytes);
    }

    Dpb_OnDecodeComplete(&e->dpb);
    return 0;
}
