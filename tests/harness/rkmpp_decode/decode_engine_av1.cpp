/* tests/harness/rkmpp_decode/decode_engine_av1.cpp
 *
 * AV1 decode engine implementation.  See decode_engine_av1.h for the
 * design rationale and operating modes.
 *
 * Status: Software mode is functional end-to-end (dav1d-decoded NV12).
 * Hardware mode allocates buffers and runs the regbuilder, but the
 * actual SUBMIT_JOB kick path is gated until the rkmpp.sys AV1 personality
 * lands (see profile.c — RKCP3560 SupportedCodecs is currently 0).
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "decode_engine_av1.h"
#include "../../../shared/rkmpp_ioctl.h"

#include <setupapi.h>

#include <cstdio>
#include <cstring>
#include <vector>

extern "C" {
#include <dav1d/headers.h>
}

static int Fail(const char *m, DWORD ec = 0) {
    std::fprintf(stderr, "av1_engine: %s (%lu)\n", m, ec);
    return 1;
}

/* ----- Hardware device open -------------------------------------- */

static int OpenAv1Device(HANDLE *out) {
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE)
        return Fail("SetupDiGetClassDevsW", GetLastError());

    SP_DEVICE_INTERFACE_DATA ifd{ sizeof(ifd) };
    DWORD idx = 0;
    int rc = -1;
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

        RKMPP_CAPS caps{};  caps.StructSize = sizeof(caps);
        DWORD got = 0;
        if (DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                            &caps, sizeof(caps), &got, nullptr) &&
            caps.Hid == 0x3560 && (caps.SupportedCodecs & RKMPP_CODEC_AV1)) {
            *out = h;
            rc = 0;
            break;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    return rc;
}

/* ----- Hardware buffer alloc helper ------------------------------ */

static int AllocHwBuf(HANDLE dev, uint32_t size, RKMPP_BUFFER_USAGE usage,
                      Av1DecodeEngine::HwBuf *out)
{
    RKMPP_ALLOC_BUFFER_IN  in{};
    RKMPP_ALLOC_BUFFER_OUT outb{};
    in.StructSize = sizeof(in);
    in.Size       = size;
    in.Usage      = usage;
    DWORD got = 0;
    if (!DeviceIoControl(dev, IOCTL_RKMPP_ALLOC_BUFFER,
                         &in, sizeof(in), &outb, sizeof(outb),
                         &got, nullptr)) {
        return Fail("ALLOC_BUFFER", GetLastError());
    }
    out->handle  = outb.BufferHandle;
    out->iova    = outb.Iova;
    out->user_va = outb.UserVa;
    out->size    = outb.SizeRoundedUp;
    return 0;
}

static void FreeHwBuf(HANDLE dev, Av1DecodeEngine::HwBuf *b) {
    if (!b->handle) return;
    RKMPP_FREE_BUFFER_IN in{ b->handle };
    DWORD got = 0;
    DeviceIoControl(dev, IOCTL_RKMPP_FREE_BUFFER, &in, sizeof(in),
                    nullptr, 0, &got, nullptr);
    *b = {};
}

/* ----- dav1d glue ------------------------------------------------- */

static int InitDav1d(Av1DecodeEngine *e) {
    Dav1dSettings s;
    dav1d_default_settings(&s);
    s.n_threads       = 1;
    s.max_frame_delay = 1;
    if (dav1d_open(&e->dav1d, &s) != 0)
        return Fail("dav1d_open");
    return 0;
}

/* Convert dav1d's planar YUV output to packed NV12 in `dst`.  dav1d
 * exposes Y/U/V as separate planes with arbitrary strides; NV12 is Y
 * followed by interleaved CbCr, both at the picture's native width.   */
static void Dav1dToNv12(const Dav1dPicture *p, std::vector<uint8_t> &dst) {
    const uint32_t W = (uint32_t)p->p.w;
    const uint32_t H = (uint32_t)p->p.h;
    dst.resize(size_t(W) * H * 3 / 2);

    const uint8_t *src_y = (const uint8_t *)p->data[0];
    const uint8_t *src_u = (const uint8_t *)p->data[1];
    const uint8_t *src_v = (const uint8_t *)p->data[2];
    const ptrdiff_t sy   = p->stride[0];
    const ptrdiff_t suv  = p->stride[1];

    /* Y plane: row-by-row stride copy. */
    uint8_t *dy = dst.data();
    for (uint32_t r = 0; r < H; r++) {
        std::memcpy(dy + size_t(r) * W, src_y + r * sy, W);
    }
    /* UV plane: interleave U + V at half resolution. */
    uint8_t *duv = dst.data() + size_t(W) * H;
    const uint32_t cW = W / 2;
    const uint32_t cH = H / 2;
    for (uint32_t r = 0; r < cH; r++) {
        const uint8_t *u = src_u + size_t(r) * suv;
        const uint8_t *v = src_v + size_t(r) * suv;
        uint8_t *o = duv + size_t(r) * W;
        for (uint32_t c = 0; c < cW; c++) {
            o[c * 2 + 0] = u[c];
            o[c * 2 + 1] = v[c];
        }
    }
}

/* ----- Hardware kick path ---------------------------------------- */

/* Globals plumbing the most-recent OBU bytes from Submit through dav1d's
 * pull-style API into DrainPictures → Av1HwKickPicture.  dav1d's
 * Dav1dPicture doesn't carry the input bytes back; we need them again on
 * the hardware path so we keep a thread-local pointer + length captured
 * at Submit time.  Valid only between Submit's send_data and the matching
 * DrainPictures call — no cross-thread access. */
static thread_local const uint8_t *g_av1_last_obu_ptr = nullptr;
static thread_local size_t         g_av1_last_obu_len = 0;


/* AV1 swreg indices that hold *_lsb DMA addresses.  Sourced from BSP
 * trans_tbl_av1_vcd[] in mpp_av1dec.c — the kernel-side IOMMU patcher
 * recognises these positions and substitutes the iova low 32 bits.
 * For each lsb idx, the corresponding *_msb sits at idx-1 (kernel
 * patches the high byte there).  We use the same set to pick which
 * regs need BufferHandle substitution in our REG_WRITE list. */
static const uint32_t kAv1DmaLsbIdx[] = {
    65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
    97, 99, 101, 103, 105, 107, 109, 111, 113, 133, 135, 137, 139,
    141, 143, 145, 147, 167, 169, 171, 173, 175, 177, 179, 183, 190,
    192, 194, 196, 198, 200, 202, 204, 224, 226, 228, 230, 232, 234,
    236, 238, 326, 328, 339, 341, 348, 350, 505, 507,
};
static bool IsAv1DmaLsb(uint32_t idx) {
    for (uint32_t v : kAv1DmaLsbIdx) if (v == idx) return true;
    return false;
}

/* For each essential DMA position we explicitly want substituted, the
 * (lsb_idx, buffer, byte offset within buffer) tuple.  The regbuilder's
 * own DMA emits (currently only output_y) get overwritten by these — the
 * regbuilder is incomplete on AV1 DMA layout (per its TODO list); we
 * authoritatively set the buffers we know the codec must reach.  Other
 * DMA positions stay zero — keyframes can decode without prob_tab /
 * cdef colbufs / refer*; inter-frame coverage is a follow-up. */
struct Av1DmaWrite {
    uint32_t lsb_idx;
    Av1DecodeEngine::HwBuf *buf;
    uint32_t offset;
};

static int Av1HwKickPicture(Av1DecodeEngine *e, const Dav1dPicture *p,
                            int slot_idx, Av1DecodedFrame *f)
{
    /* 1. Run the regbuilder to fill VdpuAv1dRegSet. */
    VdpuAv1dRegSet regs{};
    RkmppAv1Buffers bufs{};
    /* The regbuilder takes int FDs — those values land in *_lsb fields
     * and we'd then look them up to substitute handles.  Since we
     * authoritatively rewrite all DMA writes anyway (essential[] below),
     * we can pass dummies here.  Once the regbuilder learns the full
     * DMA emission, switch to a real FD->handle table. */
    bufs.output_y_fd  = 1;
    bufs.output_uv_fd = 1;
    bufs.bitstream_fd = 2;
    bufs.bitstream_length = (uint32_t)p->p.w;  /* placeholder; overwritten below */
    bufs.tile_info_fd = 3;
    bufs.film_grain_fd = 4;
    bufs.error_ref_fd = 5;
    for (int i = 0; i < 7; i++) {
        bufs.ref_y_fd[i]  = 8 + i;
        bufs.ref_uv_fd[i] = 8 + i;
    }
    if (!p->seq_hdr || !p->frame_hdr) return Fail("av1 missing headers");
    auto rc = rkmpp_av1_build_regs(p->seq_hdr, p->frame_hdr, &e->dpb,
                                   &bufs, &regs);
    if (rc != RKMPP_AV1_OK) return Fail("rkmpp_av1_build_regs", (DWORD)rc);

    /* 2. Patch in fields the regbuilder hasn't covered yet. */
    uint32_t *r32 = reinterpret_cast<uint32_t *>(&regs);
    /* swreg6 = stream_len (bytes). */
    r32[6] = (uint32_t)g_av1_last_obu_len;

    /* swreg1 — control register, kick bit set last by the driver write
     * order (we still emit it here at idx 1 so the kernel writes it). */
    /* Programmed bits per BSP `vdpu_av1_dec_setup`:
     *   sw_dec_e=1, sw_dec_clk_gate_e=1 (idx 2 bit 10 actually... but the
     *   kick path in driver writes only idx 1 with kick bit set and
     *   uses idx 1 as the kick reg).  Clock-gate-enable is at swreg2 bit 10.
     *   For first kick, just set sw_dec_e — other bits stay default (0). */
    r32[1] = 1u;

    /* 3. Memcpy OBU bytes into bitstream HwBuf. */
    if (g_av1_last_obu_len > e->bitstream.size) {
        return Fail("OBU exceeds bitstream HwBuf size");
    }
    std::memcpy(e->bitstream.user_va, g_av1_last_obu_ptr, g_av1_last_obu_len);

    /* 4. Build RKMPP_REG_WRITE list. */
    RKMPP_SUBMIT_JOB_IN in{};
    in.StructSize = sizeof(in);
    in.TimeoutMs  = 1000;
    in.BufRefCount = 0;

    /* Frame size — packed Y bytes, used for the Cb base offset. */
    const uint32_t frame_w = e->frame_width;
    const uint32_t frame_h = e->frame_height;
    const uint32_t y_size  = frame_w * frame_h;

    Av1DmaWrite essential[] = {
        /* 65: dec_out_ybase_lsb  → output Y plane                       */
        { 65,  &e->pool_output[slot_idx], 0 },
        /* 99: dec_out_cbase_lsb  → output CbCr plane (Y plane offset).  */
        { 99,  &e->pool_output[slot_idx], y_size },
        /* 167: tile_base_lsb     → tile-info / uncompressed-header blob */
        { 167, &e->tile_info,             0 },
        /* 169: stream_base_lsb   → compressed bitstream (OBU)           */
        { 169, &e->bitstream,             0 },
    };

    auto pushReg = [&](uint32_t off, uint32_t val,
                       uint64_t handle, uint32_t hoff) -> int {
        if (in.RegWriteCount >= RKMPP_MAX_REG_WRITES) {
            return Fail("REG_WRITE list overflow");
        }
        RKMPP_REG_WRITE *w = &in.Writes[in.RegWriteCount++];
        w->Offset       = off;
        w->Value        = val;
        w->BufferHandle = handle;
        w->IovaOffset   = hoff;
        w->Reserved     = 0;
        return 0;
    };

    /* Emit non-DMA / non-zero swregs in ascending order.  Skip kick
     * (idx 1) — appended last per BSP write-order convention.  Skip
     * DMA lsb positions (handled by essential[] below); skip DMA msb
     * positions (idx-1 of any lsb) — kernel sets those during iova
     * substitution. */
    auto isDmaMsb = [](uint32_t idx) -> bool {
        for (uint32_t v : kAv1DmaLsbIdx) if (v == idx + 1) return true;
        return false;
    };
    for (uint32_t idx = 2; idx < 320; idx++) {
        if (IsAv1DmaLsb(idx) || isDmaMsb(idx)) continue;
        if (r32[idx] == 0) continue;
        if (pushReg(idx * 4, r32[idx], 0, 0) != 0) return 1;
    }
    /* Authoritative DMA writes (substitute via BufferHandle). */
    for (auto &dw : essential) {
        if (!dw.buf || !dw.buf->handle) continue;
        /* msb (idx-1) — value 0; kernel ORs the high byte during iova
         * substitution.  Empty plain write so the slot is touched. */
        if (pushReg((dw.lsb_idx - 1) * 4, 0, 0, 0) != 0) return 1;
        /* lsb (idx) — substituted to (iova + offset) at submit time. */
        if (pushReg(dw.lsb_idx * 4, 0, dw.buf->handle, dw.offset) != 0)
            return 1;
    }
    /* Kick last (driver also serialises but we keep BSP write-order
     * convention here too). */
    if (pushReg(1 * 4, r32[1], 0, 0) != 0) return 1;

    /* 5. SUBMIT_JOB. */
    RKMPP_SUBMIT_JOB_OUT out_sub{};
    DWORD got = 0;
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_SUBMIT_JOB,
                         &in, sizeof(in), &out_sub, sizeof(out_sub),
                         &got, nullptr)) {
        return Fail("SUBMIT_JOB", GetLastError());
    }

    /* 6. WAIT_JOB. */
    RKMPP_WAIT_JOB_IN  win{ out_sub.JobId, 1000, 0 };
    RKMPP_WAIT_JOB_OUT wout{};
    if (!DeviceIoControl(e->device, IOCTL_RKMPP_WAIT_JOB,
                         &win, sizeof(win), &wout, sizeof(wout),
                         &got, nullptr)) {
        return Fail("WAIT_JOB", GetLastError());
    }
    if (wout.Status < 0) {  /* NT_SUCCESS in user mode */
        std::fprintf(stderr,
            "av1_engine: WAIT_JOB status=0x%08x hwstatus=0x%08x\n",
            (unsigned)wout.Status, wout.HardwareStatus);
        return Fail("WAIT_JOB hw error");
    }

    /* 7. Copy output buffer into Av1DecodedFrame YUV. */
    f->yuv.resize(size_t(frame_w) * frame_h * 3 / 2);
    std::memcpy(f->yuv.data(), e->pool_output[slot_idx].user_va, f->yuv.size());
    f->slot_idx = slot_idx;
    return 0;
}

/* Drain whatever pictures dav1d has ready right now.  Each one becomes
 * an Av1DecodedFrame in ready_q.  Software mode populates the YUV
 * directly from dav1d's output; hardware mode runs regbuilder + kicks
 * the codec. */
static int DrainPictures(Av1DecodeEngine *e, int64_t pts_hns) {
    for (;;) {
        Dav1dPicture p{};
        int r = dav1d_get_picture(e->dav1d, &p);
        if (r != 0) {
            /* DAV1D_ERR(EAGAIN) → not enough data yet, return cleanly. */
            return 0;
        }
        Av1DecodedFrame f;
        f.width  = (uint32_t)p.p.w;
        f.height = (uint32_t)p.p.h;
        f.has_film_grain = (p.frame_hdr && p.frame_hdr->film_grain.data.num_y_points > 0);
        f.pts_hns = (pts_hns >= 0) ? pts_hns
                                   : (int64_t)(e->submit_count++ *
                                               (10'000'000ULL / 30));
        f.dur_hns = 10'000'000LL / 30;

        if (e->mode == Av1EngineMode::Software) {
            Dav1dToNv12(&p, f.yuv);
            f.slot_idx = -1;
        } else {
            /* Hardware mode.  Pick a free pool slot (round-robin for
             * first bring-up — refcounted slot management is a follow-up
             * once we have inter-frame coverage).  Run the kick; on
             * any failure, fall back to dav1d output so the engine
             * API stays usable for diagnosis. */
            int slot = (int)(e->submit_count % Av1DecodeEngine::kPoolSize);
            int hw_rc = Av1HwKickPicture(e, &p, slot, &f);
            if (hw_rc != 0) {
                std::fprintf(stderr,
                    "av1_engine: HW kick failed, falling back to dav1d "
                    "output for this picture\n");
                Dav1dToNv12(&p, f.yuv);
                f.slot_idx = -1;
            }
        }

        /* DPB tracking runs in both modes — its state is needed by
         * regbuilder_av1's reference resolution.  Software mode keeps
         * it up to date so a future Hardware-mode mid-stream switch
         * (not yet supported) would have valid state. */
        if (p.seq_hdr && p.frame_hdr) {
            rkmpp_av1_dpb_post_decode(&e->dpb, p.seq_hdr, p.frame_hdr);
        }

        e->ready_q.push_back(std::move(f));
        dav1d_picture_unref(&p);
    }
}

/* ----- Public API ------------------------------------------------- */

int Av1DecodeEngine_Init(Av1DecodeEngine *e, Av1EngineMode mode,
                         uint32_t width, uint32_t height)
{
    *e = {};
    e->mode         = mode;
    e->frame_width  = width;
    e->frame_height = height;
    rkmpp_av1_dpb_init(&e->dpb);

    if (InitDav1d(e) != 0) return 1;

    if (mode == Av1EngineMode::Hardware) {
        if (OpenAv1Device(&e->device) != 0) {
            std::fprintf(stderr,
                "av1_engine: no AV1-capable rkmpp device (RKCP3560 with "
                "RKMPP_CODEC_AV1).  Run with mode=Software for dev-machine "
                "validation, or wait for driver-side AV1 personality.\n");
            dav1d_close(&e->dav1d);
            return 1;
        }
        /* Bitstream input — sized for the largest plausible AV1 OBU TU.
         * 1080p svtav1 P-frames typically run 50..500 KB; reserve 4 MB
         * to cover keyframes / 4K worst case. */
        if (AllocHwBuf(e->device, 4 * 1024 * 1024,
                       RkMppBufferUsageBitstreamInput, &e->bitstream) != 0)
            return 1;
        /* Tile-info / packed uncompressed-header blob.  vdpu383 sizes this
         * at MPP_ALIGN(5160, 128) / 8 + 16 = 661 bytes.  Round to 4 KB. */
        if (AllocHwBuf(e->device, 4096, RkMppBufferUsageScratch,
                       &e->tile_info) != 0)
            return 1;
        /* Film-grain LUT scratch — fixed-size block consumed by the codec
         * when reg9.av1_fgs_en is set.  64 KB is comfortably above MPP's
         * usage. */
        if (AllocHwBuf(e->device, 64 * 1024, RkMppBufferUsageScratch,
                       &e->film_grain) != 0)
            return 1;
        /* Fallback reference frame; same size as a regular ref. */
        const uint32_t frame_bytes = width * height * 3 / 2;
        if (AllocHwBuf(e->device, frame_bytes, RkMppBufferUsageReferenceFrame,
                       &e->error_ref) != 0)
            return 1;
        /* Reference / output pool. */
        for (int i = 0; i < Av1DecodeEngine::kPoolSize; i++) {
            if (AllocHwBuf(e->device, frame_bytes, RkMppBufferUsageOutputFrame,
                           &e->pool_output[i]) != 0)
                return 1;
        }
    }
    return 0;
}

void Av1DecodeEngine_Shutdown(Av1DecodeEngine *e) {
    if (e->dav1d) {
        dav1d_close(&e->dav1d);
        e->dav1d = nullptr;
    }
    if (e->mode == Av1EngineMode::Hardware && e->device != INVALID_HANDLE_VALUE) {
        for (auto &b : e->pool_output) FreeHwBuf(e->device, &b);
        FreeHwBuf(e->device, &e->error_ref);
        FreeHwBuf(e->device, &e->film_grain);
        FreeHwBuf(e->device, &e->tile_info);
        FreeHwBuf(e->device, &e->bitstream);
        CloseHandle(e->device);
        e->device = INVALID_HANDLE_VALUE;
    }
    e->ready_q.clear();
}

int Av1DecodeEngine_Submit(Av1DecodeEngine *e,
                           const uint8_t *obu, size_t len,
                           int64_t pts_hns)
{
    if (!e->dav1d) return -1;

    Dav1dData d{};
    if (dav1d_data_wrap(&d, obu, len,
                        [](const uint8_t *, void *) {},
                        nullptr) != 0) {
        return Fail("dav1d_data_wrap");
    }
    /* Stash OBU bytes for the HW kick to memcpy into bitstream HwBuf.
     * dav1d holds a pointer to `obu` until the matching get_picture
     * returns; same lifetime as our Hardware-mode kick. */
    g_av1_last_obu_ptr = obu;
    g_av1_last_obu_len = len;
    /* dav1d's send/get loop: send returns EAGAIN until pictures are
     * drained, so always drain before retrying send. */
    for (;;) {
        int r = dav1d_send_data(e->dav1d, &d);
        if (r == 0) {
            DrainPictures(e, pts_hns);
            break;
        }
        if (r == DAV1D_ERR(EAGAIN)) {
            DrainPictures(e, pts_hns);
            continue;
        }
        std::fprintf(stderr, "av1_engine: dav1d_send_data err=%d\n", r);
        dav1d_data_unref(&d);
        return -1;
    }
    dav1d_data_unref(&d);
    return 0;
}

int Av1DecodeEngine_PollFrame(Av1DecodeEngine *e, Av1DecodedFrame *out) {
    if (e->ready_q.empty()) return 0;
    *out = std::move(e->ready_q.front());
    e->ready_q.erase(e->ready_q.begin());
    return 1;
}

void Av1DecodeEngine_ReleaseFrame(Av1DecodeEngine *e, Av1DecodedFrame *f) {
    /* Software mode: nothing to release; the YUV vector is owned by the
     * caller's frame.  Hardware mode (future): drop the engine's hold on
     * pool_output[f->slot_idx] so the codec can reuse the slot. */
    (void)e;
    f->slot_idx = -1;
}

void Av1DecodeEngine_Drain(Av1DecodeEngine *e) {
    /* Repeatedly drain until dav1d says EAGAIN with no progress. */
    DrainPictures(e, /*pts_hns=*/-1);
}

int Av1DecodeEngine_Flush(Av1DecodeEngine *e) {
    e->ready_q.clear();
    rkmpp_av1_dpb_init(&e->dpb);
    if (e->dav1d) {
        dav1d_flush(e->dav1d);
    }
    return 0;
}

size_t Av1DecodeEngine_QueueDepth(const Av1DecodeEngine *e) {
    return e->ready_q.size();
}
