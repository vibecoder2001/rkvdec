/* tests/harness/rkmpp_decode/decode_engine.cpp */
#include "decode_engine.h"
#include "../../../shared/rkmpp_ioctl.h"

#include <setupapi.h>

#include <cstdio>
#include <cstring>
#include <utility>

static int Fail(const char *m, DWORD ec = 0) {
    std::fprintf(stderr, "decode_engine: %s (%lu)\n", m, ec);
    return 1;
}

/* Per-frame debug spam (DMA buffer dumps to win_*.bin, register-list
 * dumps, slice-offset prints) was useful during bring-up but each frame
 * triggers ~5 fopen/fwrite/fclose cycles + 100+ printf lines.  On a
 * network-redirected stdout or Z:\ working dir that adds ~50 ms / frame.
 * Gate it behind RKMPP_DECODE_DEBUG=1 (read once on first use). */
static bool DecodeDebugEnabled() {
    static int cached = -1;
    if (cached < 0) {
        char buf[8] = {};
        DWORD n = GetEnvironmentVariableA("RKMPP_DECODE_DEBUG", buf, sizeof(buf));
        cached = (n > 0 && buf[0] != '0') ? 1 : 0;
    }
    return cached != 0;
}

static int OpenDevice(HANDLE *out, Codec codec) {
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return Fail("SetupDiGetClassDevsW", GetLastError());

    const uint32_t want_codec = (codec == Codec::H265)
                                    ? RKMPP_CODEC_HEVC
                                    : RKMPP_CODEC_H264;

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

        /* Caps probe — only accept RKCP3550 (rkv-decoder-v2 core 0/1) with
         * the requested codec capability bit. */
        RKMPP_CAPS caps{};  caps.StructSize = sizeof(caps);
        DWORD got = 0;
        if (DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                            &caps, sizeof(caps), &got, nullptr) &&
            caps.Hid == 0x3550 && (caps.SupportedCodecs & want_codec)) {
            *out = h;
            SetupDiDestroyDeviceInfoList(set);
            return 0;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    return Fail("no RKCP3550 device with requested codec found");
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

int DecodeEngine_Init(DecodeEngine *e, Codec codec,
                      uint32_t width, uint32_t height)
{
    e->codec        = codec;
    e->frame_width  = width;
    e->frame_height = height;

    if (OpenDevice(&e->device, codec) != 0) return 1;

    /* RCB sizing — codec-agnostic geometry on vdpu34x.  Reuse the H.264
     * sizer for both: the 10 sub-regions, alignment, and per-region
     * register indices match on the HEVC path. */
    uint32_t rcb_total = H264GetRcbBufferSizes(e->rcb_info, width, height);

    /* NV12 + small slack for stride alignment. */
    uint32_t frame_bytes = width * height * 3u / 2u;

    /* Colmv buffer geometry — same compressed path as H.264 (vdpu34x is
     * codec-agnostic for colmv on rk3588). */
    auto align_up = [](uint32_t v, uint32_t a) { return (v + a - 1) & ~(a - 1); };
    uint32_t seg_cnt_w   = align_up(width, 64) / 64;
    uint32_t seg_cnt_h   = align_up(height, 16) / 16;
    uint32_t seg_head    = align_up(seg_cnt_w, 16) * seg_cnt_h;
    uint32_t seg_payload = seg_cnt_w * seg_cnt_h * 64u * 16u;
    uint32_t colmv_bytes = align_up(seg_head + seg_payload, 128);
    if (colmv_bytes < 4096) colmv_bytes = 4096;
    uint32_t frame_alloc = frame_bytes;

    /* Codec-specific table sizes. */
    uint32_t cabac_size   = (codec == Codec::H265)
                                ? RKH265_CABAC_INIT_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_CABAC_INIT_SIZE + RKH264_TABLE_TAIL_PAD;
    uint32_t pps_size     = (codec == Codec::H265)
                                ? RKH265_SPSPPS_UNIT_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_SPSPPS_UNIT_SIZE + RKH264_TABLE_TAIL_PAD;
    uint32_t rps_size     = (codec == Codec::H265)
                                ? RKH265_RPS_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_RPS_SIZE + RKH264_TABLE_TAIL_PAD;
    uint32_t scaling_size = (codec == Codec::H265)
                                ? RKH265_SCALING_LIST_SIZE + RKH265_TABLE_TAIL_PAD
                                : RKH264_SCALING_LIST_SIZE + RKH264_TABLE_TAIL_PAD;

    std::printf("alloc sizes: codec=%s frame=%u rcb=%u colmv=%u cabac=%u pps=%u rps=%u scaling=%u\n",
                codec == Codec::H265 ? "H265" : "H264",
                frame_bytes, rcb_total, colmv_bytes,
                cabac_size, pps_size, rps_size, scaling_size);

#define ALLOC(b, sz, usage) do { if (AllocBuf(e->device, (sz), (usage), &(b))) return 1; } while (0)
    ALLOC(e->bitstream,    1u << 20,                       RkMppBufferUsageBitstreamInput);
    ALLOC(e->cabac_init,   cabac_size,                     RkMppBufferUsageScratch);
    ALLOC(e->pps_table,    pps_size,                       RkMppBufferUsageScratch);
    ALLOC(e->rps_table,    rps_size,                       RkMppBufferUsageScratch);
    ALLOC(e->scaling_list, scaling_size,                   RkMppBufferUsageScratch);
    ALLOC(e->rcb,          rcb_total > 4096 ? rcb_total : 4096,
                                                           RkMppBufferUsageScratch);
    ALLOC(e->error_ref,    frame_alloc,                    RkMppBufferUsageReferenceFrame);
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        ALLOC(e->pool_output[i], frame_alloc, RkMppBufferUsageOutputFrame);
        ALLOC(e->pool_colmv[i],  colmv_bytes, RkMppBufferUsageScratch);
    }
#undef ALLOC

    /* Pre-fill CABAC table. */
    if (codec == Codec::H265) {
        size_t nbytes = 0;
        const uint8_t *tab = H265GetCabacInitTable(&nbytes);
        std::memcpy(e->cabac_init.user_va, tab, nbytes);
    } else {
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
    if (codec == Codec::H265) {
        if (H265Dpb_Init(&e->dpb_h265, pool, DecodeEngine::kPoolSize) != DPB_OK)
            return Fail("H265Dpb_Init");
        H265ParseResultInit(&e->parsed_h265);
    } else {
        if (Dpb_Init(&e->dpb_h264, pool, DecodeEngine::kPoolSize) != DPB_OK)
            return Fail("Dpb_Init");
    }

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
static int find_slice_nal_h264(const uint8_t *au, size_t len,
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

/* HEVC slice NAL locator.  Walks the AU's Annex-B framing looking for
 * the first VCL NAL (nal_unit_type < 32 — types 0..9 trail/RASL etc.,
 * 16..21 IRAP slices, 10..15/22..31 reserved-VCL).  Returns the
 * pre-startcode offset so the staged slice prefix matches what the
 * hardware reads in BSP.  Strips leading VPS (32) / SPS (33) / PPS (34)
 * / AUD (35) / SEI (39,40) / FD (38) / EOS / EOB so STR_LEN ends up
 * being just the slice byte count. */
static int find_slice_nal_h265(const uint8_t *au, size_t len,
                               size_t *out_off, size_t *out_size)
{
    for (size_t i = 0; i + 4 < len; i++) {
        bool sc3 = (au[i] == 0 && au[i+1] == 0 && au[i+2] == 1);
        bool sc4 = (au[i] == 0 && au[i+1] == 0 && au[i+2] == 0 && au[i+3] == 1);
        if (!sc3 && !sc4) continue;
        size_t hdr_off = i + (sc3 ? 3 : 4);
        if (hdr_off >= len) return 1;
        /* HEVC NAL header: 2 bytes — forbidden_zero_bit(1) | nal_unit_type(6) | nuh_layer_id(6) | tid(3).
         * type = (byte0 >> 1) & 0x3F. */
        uint8_t nal_type = (au[hdr_off] >> 1) & 0x3F;
        if (nal_type < 32) {
            *out_off  = i;
            *out_size = len - i;
            return 0;
        }
        i = hdr_off;
    }
    return 1;
}

/* Decode the H.264 path. */
static int DecodeOne_H264(DecodeEngine *e,
                          const uint8_t *au, size_t au_len,
                          std::vector<uint8_t> *out_yuv)
{
    /* 1. Locate the slice NAL. */
    size_t slice_off = 0, slice_size = 0;
    if (find_slice_nal_h264(au, au_len, &slice_off, &slice_size) != 0)
        return Fail("no H.264 slice NAL found");
    size_t sc_len   = (au[slice_off + 2] == 1) ? 3u : 4u;
    size_t skip     = sc_len - 3u;
    size_t copy_off = slice_off + skip;
    size_t copy_len = slice_size - skip;

    if (copy_len > e->bitstream.size)
        return Fail("slice larger than bitstream buf");
    if (DecodeDebugEnabled())
        std::printf("slice NAL at offset 0x%zx (sc=%zu), staged %zu bytes\n",
                    slice_off, sc_len, copy_len);
    std::memcpy(e->bitstream.user_va, au + copy_off, copy_len);

    H264ParseResult &parsed = e->parsed_h264;
    H264ParseStatus s = H264ParseAccessUnit(au, au_len,
                                            e->scratch.data(), e->scratch.size(),
                                            &parsed);
    if (s != H264_PARSE_OK) {
        std::fprintf(stderr, "parser status=%d (need_more=1 invalid=2 unsupported=3)\n",
                     (int)s);
        return Fail("parser failed");
    }

    uint32_t w_px = ((uint32_t)parsed.sps.pic_width_in_mbs_minus1 + 1) * 16;
    uint32_t h_px = ((uint32_t)parsed.sps.pic_height_in_map_units_minus1 + 1) * 16;
    if (w_px != e->frame_width || h_px != e->frame_height) {
        std::fprintf(stderr, "stream %ux%u, harness inited for %ux%u\n",
                     w_px, h_px, e->frame_width, e->frame_height);
        return Fail("dim mismatch");
    }

    DpbSelection sel{};
    if (Dpb_Select(&e->dpb_h264, &parsed, &sel) != DPB_OK)
        return Fail("Dpb_Select failed");

    /* The regbuilder reads `parsed.decode.dpb[]` for reg99..102 (per-slot
     * ref-info nibble) and reg67..98 (per-ref top/bottom POC pairs).  Our
     * parser leaves those zeroed (they're a V4L2-control field on Linux,
     * derived by the user-mode DPB on Windows).  Without this copy every
     * non-IDR frame ships reg99..102 = 0 — codec sees "no valid refs",
     * skips inter prediction, and bands of zeros get written into the
     * output frame.  P-only streams limped along because the codec's
     * implicit fallback uses REF_BASE[0]; B-frames fail outright. */
    memcpy(parsed.decode.dpb, sel.dpb_entries, sizeof(sel.dpb_entries));

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

    if (DecodeDebugEnabled()) {
        FILE *f;
        if (fopen_s(&f, "win_pps.bin", "wb") == 0) {
            fwrite(e->pps_table.user_va, 1, RKH264_SPSPPS_UNIT_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_rps.bin", "wb") == 0) {
            fwrite(e->rps_table.user_va, 1, RKH264_RPS_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_cabac.bin", "wb") == 0) {
            fwrite(e->cabac_init.user_va, 1, RKH264_CABAC_INIT_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_bitstream.bin", "wb") == 0) {
            fwrite(e->bitstream.user_va, 1, copy_len, f);
            fclose(f);
        }
        std::printf("dumped win_{pps,rps,cabac,bitstream}.bin\n");
    }

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
    refs.scaling_list     = e->scaling_list.handle;
    for (int i = 0; i < RKH264_RCB_COUNT; i++) {
        refs.rcb[i]        = e->rcb.handle;
        refs.rcb_offset[i] = e->rcb_info[i].offset;
    }
    for (int i = 0; i < 16; i++) {
        refs.refs[i]      = sel.refs[i];
        refs.ref_colmv[i] = sel.ref_colmv[i];
    }

    H264RegWriteList list{};
    H264RegBuildStatus rs = H264BuildRegisterList(&parsed, &refs,
                                                  sel.current_slot, &list);
    if (rs != H264_REGBUILD_OK) {
        std::fprintf(stderr, "regbuilder status=%d\n", (int)rs);
        return Fail("regbuilder failed");
    }

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

    if (DecodeDebugEnabled()) {
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

    RKMPP_WAIT_JOB_IN  win{ sout.JobId, 1000, 0 };
    RKMPP_WAIT_JOB_OUT wout{};
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                         &wout, sizeof(wout), &got, nullptr))
        return Fail("WAIT_JOB", GetLastError());
    if (DecodeDebugEnabled())
        std::printf("decode: jobid=%llu status=0x%08x hwstatus=0x%08x writes=%u\n",
                    (unsigned long long)sout.JobId, wout.Status,
                    wout.HardwareStatus, list.count);
    const uint32_t kRdySta = 1u << 2;
    bool have_output = (wout.HardwareStatus & kRdySta) != 0;
    if (wout.Status != 0 && !have_output) {
        std::fprintf(stderr, "decode reported non-success status (no output)\n");
        return 5;
    }
    if (wout.Status != 0 && DecodeDebugEnabled()) {
        std::fprintf(stderr, "decode partial: hwstatus=0x%08x has DEC_RDY but other flags too — dumping anyway\n",
                     wout.HardwareStatus);
    }

    if (DecodeDebugEnabled()) {
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

    Dpb_OnDecodeComplete(&e->dpb_h264);
    return 0;
}

/* Decode the HEVC path. */
static int DecodeOne_H265(DecodeEngine *e,
                          const uint8_t *au, size_t au_len,
                          std::vector<uint8_t> *out_yuv)
{
    /* 1. Parse the AU first — VPS/SPS/PPS state needs to land before we
     * dimension anything off it.  H265ParseAccessUnit also resolves
     * slice_data + slice_data_size for the *RBSP* slice; we still want
     * the original Annex-B-framed slice NAL bytes in the bitstream
     * buffer (rkvdec2 reads RLC-stream with the NAL header intact, like
     * the H.264 path) so we run a separate slice-NAL locator on the AU. */
    H265ParseResult &parsed = e->parsed_h265;
    H265ParseStatus s = H265ParseAccessUnit(au, au_len,
                                            e->scratch.data(), e->scratch.size(),
                                            &parsed);
    if (s != H265_PARSE_OK) {
        std::fprintf(stderr, "h265 parser status=%d\n", (int)s);
        return Fail("parser failed");
    }
    if (!parsed.has_slice || parsed.active_sps_id < 0 ||
        parsed.active_pps_id < 0) {
        return Fail("no slice / active SPS+PPS after parse");
    }
    const H265Sps *sps = &parsed.sps[parsed.active_sps_id];
    const H265Pps *pps = &parsed.pps[parsed.active_pps_id];
    const H265Vps *vps = (parsed.active_vps_id >= 0)
                            ? &parsed.vps[parsed.active_vps_id]
                            : nullptr;

    uint32_t w_px = sps->pic_width_in_luma_samples;
    uint32_t h_px = sps->pic_height_in_luma_samples;
    if (w_px != e->frame_width || h_px != e->frame_height) {
        std::fprintf(stderr, "stream %ux%u, harness inited for %ux%u\n",
                     w_px, h_px, e->frame_width, e->frame_height);
        return Fail("dim mismatch");
    }

    /* 2. Locate first VCL NAL — strip leading VPS/SPS/PPS/SEI so STR_LEN
     * (reg016) is *just* the slice byte count, mirroring what the BSP
     * writes.  Earlier H.264 mistake (passing full AU length) sent the
     * codec into runaway parsing past slice end. */
    size_t slice_off = 0, slice_size = 0;
    if (find_slice_nal_h265(au, au_len, &slice_off, &slice_size) != 0)
        return Fail("no HEVC slice NAL found");
    size_t sc_len   = (au[slice_off + 2] == 1) ? 3u : 4u;
    size_t skip     = sc_len - 3u;
    size_t copy_off = slice_off + skip;
    size_t copy_len = slice_size - skip;
    if (copy_len > e->bitstream.size)
        return Fail("slice larger than bitstream buf");
    if (DecodeDebugEnabled())
        std::printf("h265 slice NAL at offset 0x%zx (sc=%zu), staged %zu bytes\n",
                    slice_off, sc_len, copy_len);
    std::memcpy(e->bitstream.user_va, au + copy_off, copy_len);

    /* 3. DPB selection. */
    H265DpbSelection sel{};
    if (H265Dpb_Select(&e->dpb_h265, &parsed, &sel) != DPB_OK)
        return Fail("H265Dpb_Select failed");

    /* 4. Pack tables. */
    if (H265PackPPS(vps, sps, pps,
                    static_cast<uint8_t *>(e->pps_table.user_va),
                    e->pps_table.size) < 0)
        return Fail("H265PackPPS");
    if (H265PackRPS(&parsed,
                    static_cast<uint8_t *>(e->rps_table.user_va),
                    e->rps_table.size) < 0)
        return Fail("H265PackRPS");
    if (H265PackScalingList(sps, pps,
                            static_cast<uint8_t *>(e->scaling_list.user_va),
                            e->scaling_list.size) < 0)
        return Fail("H265PackScalingList");

    if (DecodeDebugEnabled()) {
        FILE *f;
        if (fopen_s(&f, "win_h265_pps.bin", "wb") == 0) {
            fwrite(e->pps_table.user_va, 1, RKH265_SPSPPS_UNIT_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_rps.bin", "wb") == 0) {
            fwrite(e->rps_table.user_va, 1, RKH265_RPS_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_scaling.bin", "wb") == 0) {
            fwrite(e->scaling_list.user_va, 1, RKH265_SCALING_LIST_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_cabac.bin", "wb") == 0) {
            fwrite(e->cabac_init.user_va, 1, RKH265_CABAC_INIT_SIZE, f);
            fclose(f);
        }
        if (fopen_s(&f, "win_h265_bitstream.bin", "wb") == 0) {
            fwrite(e->bitstream.user_va, 1, copy_len, f);
            fclose(f);
        }
        std::printf("dumped win_h265_{pps,rps,scaling,cabac,bitstream}.bin\n");
    }

    /* 5. Compose buffer-refs for the regbuilder. */
    H265BufferRefs refs{};
    refs.bitstream        = e->bitstream.handle;
    refs.bitstream_offset = 0;
    refs.bitstream_size   = (uint32_t)copy_len;
    refs.output_frame     = sel.current_output_iova;
    refs.colmv_cur        = sel.current_colmv_iova;
    refs.error_ref        = e->error_ref.handle;
    refs.pps_table        = e->pps_table.handle;
    refs.rps_table        = e->rps_table.handle;
    refs.scanlist         = e->scaling_list.handle;
    refs.cabac_init_table = e->cabac_init.handle;
    for (int i = 0; i < 10; i++) {
        refs.rcb[i]        = e->rcb.handle;
        refs.rcb_offset[i] = e->rcb_info[i].offset;
    }
    for (int i = 0; i < 16; i++) {
        refs.refs[i]      = sel.refs[i];
        refs.ref_colmv[i] = sel.ref_colmv[i];
        refs.ref_poc[i]   = sel.ref_pocs[i];
        refs.ref_poc_high[i]     = (uint8_t)((sel.ref_pocs[i] >> 28) & 0xF);
        refs.ref_is_long_term[i] = 0;
    }

    /* 6. Build register list. */
    H265RegWriteList list{};
    H265RegBuildStatus rs = H265BuildRegisterList(&parsed, &refs,
                                                  sel.current_slot, &list);
    if (rs != H265_REGBUILD_OK) {
        std::fprintf(stderr, "h265 regbuilder status=%d\n", (int)rs);
        return Fail("h265 regbuilder failed");
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

    if (DecodeDebugEnabled()) {
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
    if (DecodeDebugEnabled())
        std::printf("h265 decode: jobid=%llu status=0x%08x hwstatus=0x%08x writes=%u\n",
                    (unsigned long long)sout.JobId, wout.Status,
                    wout.HardwareStatus, list.count);
    const uint32_t kRdySta = 1u << 2;
    bool have_output = (wout.HardwareStatus & kRdySta) != 0;
    if (wout.Status != 0 && !have_output) {
        std::fprintf(stderr, "h265 decode reported non-success status (no output)\n");
        return 5;
    }
    if (wout.Status != 0 && DecodeDebugEnabled()) {
        std::fprintf(stderr, "h265 decode partial: hwstatus=0x%08x — dumping anyway\n",
                     wout.HardwareStatus);
    }

    if (DecodeDebugEnabled()) {
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

    H265Dpb_OnDecodeComplete(&e->dpb_h265);
    return 0;
}

int DecodeEngine_DecodeOne(DecodeEngine *e,
                           const uint8_t *au, size_t au_len,
                           std::vector<uint8_t> *out_yuv)
{
    if (au_len > e->bitstream.size) return Fail("AU larger than bitstream buf");
    if (e->codec == Codec::H265)
        return DecodeOne_H265(e, au, au_len, out_yuv);
    return DecodeOne_H264(e, au, au_len, out_yuv);
}

int DecodeEngine_DecodeOneFramed(DecodeEngine *e,
                                 NalFraming framing,
                                 AvccLenSize len_size,
                                 const uint8_t *au, size_t au_len,
                                 std::vector<uint8_t> *out_yuv)
{
    if (framing == FRAMING_ANNEXB)
        return DecodeEngine_DecodeOne(e, au, au_len, out_yuv);

    /* AVCC4: convert into a scratch buffer.  Annex-B 4-byte start code
     * matches AVCC4 length-field width, so output size == input size. */
    std::vector<uint8_t> tmp(au_len + 64);
    int n = AvccToAnnexB(au, au_len, len_size, tmp.data(), tmp.size());
    if (n < 0) return Fail("AvccToAnnexB: malformed input");
    return DecodeEngine_DecodeOne(e, tmp.data(), (size_t)n, out_yuv);
}

int DecodeEngine_FeedExtradata(DecodeEngine *e,
                               NalFraming framing,
                               AvccLenSize len_size,
                               const uint8_t *au, size_t au_len)
{
    if (au_len == 0) return 0;

    const uint8_t *p = au;
    size_t         n = au_len;
    std::vector<uint8_t> tmp;
    if (framing == FRAMING_AVCC4) {
        tmp.resize(au_len + 64);
        int got = AvccToAnnexB(au, au_len, len_size, tmp.data(), tmp.size());
        if (got < 0) return Fail("AvccToAnnexB(extradata): malformed");
        p = tmp.data();
        n = (size_t)got;
    }

    if (e->scratch.size() < n * 2) e->scratch.resize(n * 2 + 64);
    if (e->codec == Codec::H265) {
        H265ParseStatus s = H265ParseAccessUnit(p, n,
                                                e->scratch.data(),
                                                e->scratch.size(),
                                                &e->parsed_h265);
        /* NEED_MORE = no slice found, but VPS/SPS/PPS landed → fine. */
        if (s != H265_PARSE_OK && s != H265_PARSE_NEED_MORE) {
            std::fprintf(stderr,
                         "FeedExtradata: H265 parser status=%d\n", (int)s);
            return 1;
        }
    } else {
        H264ParseStatus s = H264ParseAccessUnit(p, n,
                                                e->scratch.data(),
                                                e->scratch.size(),
                                                &e->parsed_h264);
        if (s != H264_PARSE_OK && s != H264_PARSE_NEED_MORE) {
            std::fprintf(stderr,
                         "FeedExtradata: H264 parser status=%d\n", (int)s);
            return 1;
        }
    }
    return 0;
}

/* ---- Submit / Poll / Drain (display-order reorder) ---------------- *
 *
 * Implementation pattern: Submit runs the existing per-AU decode flow,
 * captures POC + YUV from the result, and appends to reorder_q.  After
 * each Submit, while reorder_q exceeds max_num_reorder_pics, the
 * lowest-POC entry is moved to ready_q.  Drain spills everything left
 * in reorder_q into ready_q in POC ascending order.  PollFrame pops the
 * front of ready_q.
 *
 * We compute max_num_reorder_pics from the active SPS on the first
 * Submit that produces a frame; subsequent SPS updates (e.g. a mid-
 * stream resolution change) refresh it.  Fallback to 4 (a generous
 * lower bound that fits every B-pyramid we ship) when SPS is absent. */

static uint32_t resolve_max_reorder(const DecodeEngine *e) {
    if (e->codec == Codec::H265) {
        if (e->parsed_h265.active_sps_id >= 0) {
            const H265Sps &sps = e->parsed_h265.sps[e->parsed_h265.active_sps_id];
            int idx = sps.sps_max_sub_layers_minus1;
            if (idx < 0) idx = 0;
            if (idx > H265_MAX_SUB_LAYERS - 1) idx = H265_MAX_SUB_LAYERS - 1;
            uint32_t v = sps.sps_max_num_reorder_pics[idx];
            return v;
        }
    } else {
        if (e->parsed_h264.has_sps) {
            /* H.264 VUI's bitstream_restriction max_num_reorder_frames
             * isn't parsed yet; max_num_ref_frames is a safe upper bound
             * for B-pyramid streams (reorder ≤ ref_frames in practice). */
            return e->parsed_h264.sps.max_num_ref_frames;
        }
    }
    return 4;
}

static int32_t current_poc(const DecodeEngine *e) {
    if (e->codec == Codec::H265) return e->parsed_h265.poc;
    return (int32_t)e->parsed_h264.decode.top_field_order_cnt;
}

static void bump_lowest(DecodeEngine *e) {
    /* Find lowest-POC entry in reorder_q and move it to ready_q. */
    if (e->reorder_q.empty()) return;
    size_t best = 0;
    for (size_t i = 1; i < e->reorder_q.size(); i++) {
        if (e->reorder_q[i].poc < e->reorder_q[best].poc) best = i;
    }
    e->ready_q.push_back(std::move(e->reorder_q[best]));
    e->reorder_q.erase(e->reorder_q.begin() + best);
}

/* C++ linkage */
int DecodeEngine_Submit(DecodeEngine *e,
                        const uint8_t *au, size_t au_len,
                        int64_t pts_hns)
{
    DecodeEngine::ReorderEntry entry;
    int rc = DecodeEngine_DecodeOne(e, au, au_len, &entry.yuv);
    if (rc != 0) return rc;

    entry.poc = current_poc(e);
    if (pts_hns < 0) {
        /* Synthetic monotonic timeline if caller doesn't supply pts. */
        entry.pts_hns = (int64_t)(e->submit_count *
                                  (uint64_t)10'000'000ULL / 30ULL);
    } else {
        entry.pts_hns = pts_hns;
    }
    entry.dur_hns = (int64_t)(10'000'000ULL / 30ULL);
    e->submit_count++;

    e->reorder_q.push_back(std::move(entry));
    e->max_num_reorder_pics = resolve_max_reorder(e);

    /* Spec C.5.2 / C.4 bump: while DPB output queue has more pending
     * pics than the SPS's max_num_reorder_pics, emit the lowest-POC. */
    while (e->reorder_q.size() > e->max_num_reorder_pics) {
        bump_lowest(e);
    }
    return 0;
}

/* C++ linkage */
int DecodeEngine_SubmitFramed(DecodeEngine *e,
                              NalFraming framing, AvccLenSize len_size,
                              const uint8_t *au, size_t au_len,
                              int64_t pts_hns)
{
    if (framing == FRAMING_ANNEXB)
        return DecodeEngine_Submit(e, au, au_len, pts_hns);
    std::vector<uint8_t> tmp(au_len + 64);
    int n = AvccToAnnexB(au, au_len, len_size, tmp.data(), tmp.size());
    if (n < 0) return Fail("AvccToAnnexB(submit): malformed");
    return DecodeEngine_Submit(e, tmp.data(), (size_t)n, pts_hns);
}

/* C++ linkage */
int DecodeEngine_PollFrame(DecodeEngine *e, DecodedFrame *out)
{
    if (e->ready_q.empty()) return 0;
    DecodeEngine::ReorderEntry entry = std::move(e->ready_q.front());
    e->ready_q.erase(e->ready_q.begin());
    out->poc     = entry.poc;
    out->pts_hns = entry.pts_hns;
    out->dur_hns = entry.dur_hns;
    out->yuv     = std::move(entry.yuv);
    return 1;
}

/* C++ linkage */
void DecodeEngine_Drain(DecodeEngine *e)
{
    /* Move every remaining reorder_q entry to ready_q in POC ascending
     * order.  Stable for ties (shouldn't happen with our streams). */
    while (!e->reorder_q.empty()) bump_lowest(e);
}

int DecodeEngine_Flush(DecodeEngine *e)
{
    /* Drop any pending reorder window — flush in MFT terms means "the
     * caller is dropping output and starting fresh from the next IDR". */
    e->reorder_q.clear();
    e->ready_q.clear();
    e->submit_count = 0;
    e->max_num_reorder_pics = 0;

    /* Re-init the DPB pool (same buffers as DecodeEngine_Init). */
    DpbPoolEntry pool[DecodeEngine::kPoolSize];
    for (int i = 0; i < DecodeEngine::kPoolSize; i++) {
        pool[i].output_frame = e->pool_output[i].handle;
        pool[i].colmv        = e->pool_colmv[i].handle;
    }
    if (e->codec == Codec::H265) {
        if (H265Dpb_Init(&e->dpb_h265, pool, DecodeEngine::kPoolSize) != DPB_OK)
            return Fail("H265Dpb_Init (flush)");
        /* Persistent VPS/SPS/PPS in parsed_h265 stays — next IDR re-validates. */
    } else {
        if (Dpb_Init(&e->dpb_h264, pool, DecodeEngine::kPoolSize) != DPB_OK)
            return Fail("Dpb_Init (flush)");
    }
    return 0;
}
