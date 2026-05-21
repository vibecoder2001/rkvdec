/* mft/engine/decode_engine_av1.cpp
 *
 * AV1 decode engine implementation.  See decode_engine_av1.h for the
 * design rationale and operating modes.
 *
 * Status: Hardware mode is functional end-to-end using the clean-room
 * av1_parser (no dav1d library link required).  Software mode is a no-op
 * now that dav1d has been removed from the engine.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "decode_engine_av1.h"
#include "../../shared/rkmpp_ioctl.h"
#include "av1_default_cdfs.h"
#include "repack_yuv.h"

#include <setupapi.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

/* Buffer-dump path: when env var RKMPP_AV1_DUMP_DIR is set, the
 * harness writes per-kick text dumps of the input buffers in the same
 * format as BSP MPP captures (one 32-bit word per line, %08x).  Lets
 * us byte-diff our buffers against tests/data/av1/av1capture/ to find
 * where chroma config diverges. */
/* Cache RKMPP_AV1_DUMP_DIR once on first call.  Previously every
 * Av1DumpBuffer/Av1DumpRaw/Av1DumpTileOut call hit getenv() per kick;
 * getenv on Windows can walk the registry-derived environment block
 * which dominates per-kick driver overhead at 4K.  Review MFT #30. */
static const char *Av1DumpDir() {
    static const char *cached = (const char *)(intptr_t)-1;
    if (cached == (const char *)(intptr_t)-1) {
        cached = std::getenv("RKMPP_AV1_DUMP_DIR");
    }
    return cached;
}

static void Av1DumpBuffer(const char *kind, uint32_t kick,
                          const void *data, size_t size_bytes) {
    const char *dir = Av1DumpDir();
    if (!dir || !data || !size_bytes) return;
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s_%u.txt", dir, kind, kick);
    FILE *fp = std::fopen(path, "wb");
    if (!fp) return;
    const uint32_t *p = (const uint32_t *)data;
    size_t n = size_bytes / 4;
    for (size_t i = 0; i < n; i++) std::fprintf(fp, "%08x\n", p[i]);
    std::fclose(fp);
}

static void Av1DumpRaw(const char *kind, uint32_t kick,
                       const void *data, size_t size_bytes) {
    const char *dir = Av1DumpDir();
    if (!dir || !data || !size_bytes) return;
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s_%u.txt", dir, kind, kick);
    FILE *fp = std::fopen(path, "wb");
    if (!fp) return;
    std::fwrite(data, 1, size_bytes, fp);
    std::fclose(fp);
}

/* After each kick: dump tile_out_internal's Y+UV region as a raw NV12 file
 * (viewable with ffplay -f rawvideo -pixel_format nv12 -video_size WxH)
 * and print per-region non-zero byte statistics to stderr so we can verify
 * the codec actually wrote to the full Y and UV regions. */
static void Av1DumpTileOut(uint32_t kick, uint32_t w, uint32_t h,
                           const Av1DecodeEngine::HwBuf *tile_out) {
    if (!tile_out->user_va) return;
    const uint8_t *base = (const uint8_t *)tile_out->user_va;
    const uint32_t y_size  = w * h;
    const uint32_t uv_size = y_size / 2;

    /* Always print region stats to stderr (gated on dump-dir presence so
     * normal runs are quiet; remove the gate if you want stats always). */
    const char *dir = Av1DumpDir();
    if (dir) {
        /* Count non-zero bytes in Y and UV regions. */
        size_t y_nz = 0, uv_nz = 0;
        const uint8_t *yp  = base;
        const uint8_t *uvp = base + y_size;
        for (uint32_t i = 0; i < y_size;  i++) if (yp[i])  y_nz++;
        for (uint32_t i = 0; i < uv_size; i++) if (uvp[i]) uv_nz++;
        std::fprintf(stderr,
            "av1_engine: kick %u tile_out_internal: "
            "Y nz=%zu/%u (%.1f%%)  UV nz=%zu/%u (%.1f%%)\n",
            kick, y_nz, y_size,  100.0 * y_nz  / y_size,
                  uv_nz, uv_size, 100.0 * uv_nz / uv_size);

        /* Dump the raw NV12 (Y+UV) so it can be opened as an image. */
        char path[512];
        std::snprintf(path, sizeof(path),
            "%s/tile_out_%u_%ux%u.nv12", dir, kick, w, h);
        FILE *fp = std::fopen(path, "wb");
        if (fp) {
            std::fwrite(base, 1, y_size + uv_size, fp);
            std::fclose(fp);
        }
    }
}

/* <dav1d/headers.h> is included transitively via av1_parser.h (for
 * Dav1dSequenceHeader / Dav1dFrameHeader struct definitions).  No dav1d
 * library functions are used here. */

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
            caps.Hid == RKMPP_HID_RKCP3560 &&
            (caps.SupportedCodecs & RKMPP_CODEC_AV1)) {
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

/* ----- Hardware kick path ---------------------------------------- */

/* AV1ObuRecord moved to decode_engine_av1.h so Av1DecodeEngine can
 * hold a per-instance std::vector — previously these four pieces of
 * per-Submit state were static thread_locals, which served fine on a
 * single thread but raced once MF cross-thread-dispatches ProcessInput
 * and ProcessOutput.  See [[mft_av1_tu_state_per_instance]]. */


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

/* Walk OBUs in a TU and emit one AV1ObuRecord per picture-producing OBU
 * (OBU_FRAME = 6, OBU_FRAME_HEADER = 3). Each record carries the byte
 * window the codec needs for that OBU's bitstream and a precomputed
 * show_existing_frame flag (MSB of the first uncompressed_header byte;
 * AV1 §5.9.1 — valid when seq_hdr.reduced_still_picture_header=0).
 *
 * DrainPictures consumes records in order and calls the clean-room parser
 * per record. Returns the count of records appended; on parse failure
 * leaves the vector empty. */
static size_t Av1WalkObus(const uint8_t *obu, size_t len,
                          std::vector<AV1ObuRecord> &out) {
    out.clear();
    size_t pos = 0;
    while (pos < len) {
        if (pos + 1 > len) { out.clear(); return 0; }
        uint8_t hdr = obu[pos];
        uint8_t obu_type = (hdr >> 3) & 0xf;
        uint8_t ext_flag = (hdr >> 2) & 0x1;
        uint8_t has_size = (hdr >> 1) & 0x1;
        size_t hdr_len = 1 + (ext_flag ? 1 : 0);
        if (pos + hdr_len > len) { out.clear(); return 0; }
        uint64_t obu_size = 0; size_t size_len = 0;
        if (has_size) {
            for (int i = 0; i < 8; i++) {
                if (pos + hdr_len + i >= len) { out.clear(); return 0; }
                uint8_t b = obu[pos + hdr_len + i];
                obu_size |= ((uint64_t)(b & 0x7f)) << (i * 7);
                size_len++;
                if (!(b & 0x80)) break;
            }
        } else {
            obu_size = len - pos - hdr_len;
        }
        size_t payload_off = pos + hdr_len + size_len;
        size_t next        = payload_off + obu_size;
        if (next <= pos || next > len) { out.clear(); return 0; }

        if (obu_type == 3 || obu_type == 6) {
            AV1ObuRecord r{};
            /* First picture-producing OBU absorbs any preceding non-FRAME
             * OBUs (TD / sequence header / metadata) into its slice — BSP
             * does the same so the codec gets the framing context. */
            const bool first = out.empty();
            r.slice_start    = first ? 0u : (uint32_t)pos;
            r.slice_size     = (uint32_t)(next - r.slice_start);
            r.frame_tag_off  = (uint32_t)(pos - r.slice_start +
                                          hdr_len + size_len);
            r.obu_type       = obu_type;
            r.show_existing  = obu_size > 0 &&
                               ((obu[payload_off] >> 7) & 1) != 0;
            out.push_back(r);
        }
        pos = next;
    }
    return out.size();
}

/* Build the per-tile dim/offset table the codec reads via reg167.
 *
 * Format (clean-room, derived from observation of BSP captures and
 * av1_bringup_table memory; same packing rule as MPP HAL but no code
 * lifted): for each tile in raster scan order (or column-major when
 * tile_transpose=1), 16 bytes:
 *   [0]    tile width  in superblock units (64-px or 128-px depending
 *          on use_128x128_superblock)
 *   [1..3] zero
 *   [4]    tile height in SB units
 *   [5..7] zero
 *   [8..11]  LE u32 — byte offset of tile's first byte from stream_base
 *   [12..15] LE u32 — byte offset of tile's last byte (inclusive)
 *
 * tile_offsets[i] = {start_byte, end_byte} relative to the first tile's
 * first byte (i.e. the tile group payload origin, not the bitstream base).
 * For single-tile frames pass nullptr for tile_offsets; start=0, end=len. */
static void Av1BuildTileInfo(const Dav1dSequenceHeader *seq,
                             const Dav1dFrameHeader *hdr,
                             uint32_t stream_len_bytes,
                             const uint32_t (*tile_offsets)[2],
                             uint8_t *dst, size_t dst_size)
{
    if (!seq || !hdr || !dst || dst_size < 16) return;
    const int tile_cols = hdr->tiling.cols;
    const int tile_rows = hdr->tiling.rows;
    const int n_tiles   = tile_cols * tile_rows;
    if ((size_t)n_tiles * 16 > dst_size) return;
    /* Codec only consumes the first n_tiles*16 bytes via reg167; bytes
     * past that are unread.  Bounding the zero saves ~64 KiB per kick at
     * single-tile resolutions where dst_size is the full HwBuf. */
    std::memset(dst, 0, (size_t)n_tiles * 16);

    /* swreg10.sw_tile_transpose=1 is hardcoded in regbuilder_av1.cpp.
     * BSP vdpu_av1d_set_tile_info_mem iterates outer=cols, inner=rows
     * with this transpose, so entry e in the buffer corresponds to
     * (col = e / rows, row = e % rows).  Tile data offsets are still
     * keyed by raster tile_id = row * cols + col.  Writing in row-major
     * order (the obvious "for t in n_tiles" iteration) silently works
     * for 1×1 tile but scrambles the codec's view as soon as cols>1
     * AND rows>1 — codec stalls with hw_status TIMEOUT bit set. */
    for (int col = 0; col < tile_cols; col++) {
        for (int row = 0; row < tile_rows; row++) {
            const int e        = col * tile_rows + row;
            const int tile_id  = row * tile_cols + col;

        /* Tile width in SBs: col_start_sb[col+1] - col_start_sb[col].
         * row_start_sb is in units of superblocks. */
        uint32_t w_sb = (uint32_t)(hdr->tiling.col_start_sb[col + 1] -
                                   hdr->tiling.col_start_sb[col]);
        uint32_t h_sb = (uint32_t)(hdr->tiling.row_start_sb[row + 1] -
                                   hdr->tiling.row_start_sb[row]);
        uint32_t t_start, t_end;
        if (tile_offsets) {
            t_start = tile_offsets[tile_id][0];
            t_end   = tile_offsets[tile_id][1];
        } else {
            t_start = 0;
            t_end   = stream_len_bytes;
        }

        uint8_t *p = dst + e * 16;
        p[0] = (uint8_t)w_sb;
        p[4] = (uint8_t)h_sb;
        p[8]  = (uint8_t)( t_start        & 0xff);
        p[9]  = (uint8_t)((t_start >>  8) & 0xff);
        p[10] = (uint8_t)((t_start >> 16) & 0xff);
        p[11] = (uint8_t)((t_start >> 24) & 0xff);
        p[12] = (uint8_t)( t_end          & 0xff);
        p[13] = (uint8_t)((t_end   >>  8) & 0xff);
        p[14] = (uint8_t)((t_end   >> 16) & 0xff);
        p[15] = (uint8_t)((t_end   >> 24) & 0xff);
        }
    }
}

/* Build the global-motion-model buffer the codec reads via reg83.
 *
 * Layout (clean-room, per av1_bringup_table memory):
 *   For each of 7 inter ref slots, 32 bytes:
 *     [0..23]  6 × LE i32 — wmmat[6] in DDR order 0,1,3,2,4,5
 *              (note 2/3 swap relative to AV1 syntax order)
 *     [24..25] LE i16 alpha
 *     [26..27] LE i16 beta
 *     [28..29] LE i16 gamma
 *     [30..31] LE i16 delta
 *
 * For keyframes / intrabc / non-warp refs everything is zero (identity).
 * dav1d exposes warped-motion params per slot via hdr->gmv[ref] when
 * type > 0 (translation/rotzoom/affine).  Translation-only frames have
 * non-zero wmmat[0..1] but zero alpha/beta/gamma/delta. */
static void Av1BuildGlobalModel(const Dav1dFrameHeader *hdr,
                                uint8_t *dst, size_t dst_size)
{
    if (!hdr || !dst) return;
    constexpr size_t per_ref = 32;
    constexpr int    n_refs  = 7;
    if (dst_size < per_ref * n_refs) return;
    std::memset(dst, 0, per_ref * n_refs);
    bool inter = (hdr->frame_type != DAV1D_FRAME_TYPE_KEY &&
                  hdr->frame_type != DAV1D_FRAME_TYPE_INTRA &&
                  !hdr->allow_intrabc);
    static const int kIdx[6] = { 0, 1, 3, 2, 4, 5 };
    static constexpr int32_t kIdentity[6] = {
        0, 0, 1 << 16, 0, 0, 1 << 16,
    };
    for (int r = 0; r < n_refs; r++) {
        uint8_t *p = dst + r * per_ref;
        const int32_t *mat = kIdentity;
        int16_t a = 0, b = 0, g = 0, d = 0;
        if (inter) {
            const Dav1dWarpedMotionParams *gm = &hdr->gmv[r];
            mat = gm->matrix;
            a = (int16_t)gm->u.p.alpha;
            b = (int16_t)gm->u.p.beta;
            g = (int16_t)gm->u.p.gamma;
            d = (int16_t)gm->u.p.delta;
        }
        for (int i = 0; i < 6; i++) {
            int32_t v = mat[kIdx[i]];
            std::memcpy(p + i * 4, &v, 4);
        }
        std::memcpy(p + 24, &a, 2);
        std::memcpy(p + 26, &b, 2);
        std::memcpy(p + 28, &g, 2);
        std::memcpy(p + 30, &d, 2);
    }
}

static int Av1HwKickPicture(Av1DecodeEngine *e,
                            const Dav1dFrameHeader *p_frame_hdr,
                            const Dav1dSequenceHeader *p_seq_hdr,
                            int slot_idx, uint64_t kick_no,
                            const AV1ObuRecord *rec,
                            Av1DecodedFrame *f)
{
    /* Alias locals to the flat (non-Dav1dPicture) parameters so the body
     * below can still use "p->frame_hdr" and "p->seq_hdr" idiom via a thin
     * shim struct.  We use direct references instead of the old p->* accesses
     * — all p->frame_hdr / p->seq_hdr occurrences below are replaced with
     * p_frame_hdr / p_seq_hdr. */
    /* This kick's bitstream slice within the TU. For the first picture-
     * producing OBU the slice starts at TU byte 0 (carrying TD / sequence
     * header / metadata along); subsequent OBUs slice from their own
     * header. Matches BSP per-kick stream windows. */
    const uint8_t *obu_ptr = e->tu_ptr + rec->slice_start;
    const size_t   obu_len = rec->slice_size;

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
    /* This OBU's byte count.  swreg6 / swreg258 end up as
     * MPP_ALIGN(this, 128) and bound how many bytes of the bitstream
     * buffer the codec is allowed to read. */
    bufs.bitstream_length = (uint32_t)obu_len;
    bufs.tile_info_fd = 3;
    bufs.film_grain_fd = 4;
    bufs.error_ref_fd = 5;
    for (int i = 0; i < 7; i++) {
        bufs.ref_y_fd[i]  = 8 + i;
        bufs.ref_uv_fd[i] = 8 + i;
    }
    if (!p_seq_hdr || !p_frame_hdr) return Fail("av1 missing headers");
    auto rc = rkmpp_av1_build_regs(p_seq_hdr, p_frame_hdr, &e->dpb,
                                   &bufs, &regs);
    if (rc != RKMPP_AV1_OK) return Fail("rkmpp_av1_build_regs", (DWORD)rc);

    /* 2. Patch in fields the regbuilder hasn't covered yet. */
    uint32_t *r32 = reinterpret_cast<uint32_t *>(&regs);
    /* Codec's entropy cursor expects the start of tile_group bits, not
     * the start of the OBU sequence.  RK3588 vdpu_av1d has no hardware
     * OBU parser — the userspace HAL hands it a slice + frame_tag_size:
     *   - reg169 IovaOffset = (frame_tag_size & ~0xf)  (16-byte aligned)
     *   - swreg5.sw_strm_start_bit = (frame_tag_size & 0xf) * 8
     *   - tile_info end = obu_len - frame_tag_size
     *
     * The OBU walk at Submit time gave us pre_payload_off (= hdr_len +
     * size_leb).  For OBU_FRAME (6) we add frame_hdr_obu_size_bytes from
     * the parser's reported Dav1dFrameHeader (local patch field).  For OBU_FRAME_
     * HEADER (3) the payload IS the frame_header — but show_existing_frame
     * OBU_FRAME_HEADERs don't reach here (skipped in DrainPictures). */
    uint32_t frame_tag_size = rec->frame_tag_off;
    if (rec->obu_type == 6 && p_frame_hdr) {
        frame_tag_size += p_frame_hdr->frame_hdr_obu_size_bytes;
    }
    /* IOMMU offset shifts the codec's read cursor past the OBU framing.
     * Codec reads from iova + offset (16-byte-aligned), entropy-decodes
     * starting at strm_start_bit within that.  Provides a few bytes of
     * OBU framing as "look-back" context the entropy hardware may rely
     * on for state init.  Earlier user-mode-strip approach (memcpy from
     * byte 33 + start_bit=0) produced flat-gray output even though the
     * effective starting bit position was the same — codec hardware
     * apparently wants the original framing context bytes available. */
    const uint32_t bs_iova_offset = frame_tag_size & ~0xfu;
    const uint32_t strm_start_bit = (frame_tag_size & 0xfu) * 8u;
    const uint32_t tile_data_len  =
        (uint32_t)obu_len - frame_tag_size;

    /* swreg5.sw_strm_start_bit (top 7 bits of swreg5).  Patch into
     * regbuilder output post-build. */
    {
        uint32_t v = r32[5];
        v &= ~(0x7Fu << 25);
        v |= (strm_start_bit & 0x7Fu) << 25;
        r32[5] = v;
    }

    /* swreg6 stays as-is (full OBU length aligned, set by regbuilder).
     * MPP sets it to MPP_ALIGN(p_hal->strm_len, 128) where strm_len is
     * the full input packet — codec reads up to that bound; the tile
     * cursor knowing where to STOP decoding is conveyed by tile_info's
     * end-offset.  Don't patch r32[6] here. */

    /* swreg1 — control register, kick bit set last by the driver write
     * order (we still emit it here at idx 1 so the kernel writes it). */
    /* Programmed bits per BSP `vdpu_av1_dec_setup`:
     *   sw_dec_e=1, sw_dec_clk_gate_e=1 (idx 2 bit 10 actually... but the
     *   kick path in driver writes only idx 1 with kick bit set and
     *   uses idx 1 as the kick reg).  Clock-gate-enable is at swreg2 bit 10.
     *   For first kick, just set sw_dec_e — other bits stay default (0). */
    r32[1] = 1u;


    /* 3. Memcpy this OBU's slice into bitstream HwBuf.  Codec reads
     * starting at iova + bs_iova_offset; bs_iova_offset is set to
     * frame_tag_size_aligned and strm_start_bit picks up the residual.
     * For multi-OBU TUs each kick gets just its own OBU window — BSP's
     * HAL works the same way (one kick per OBU_FRAME, slice scoped to
     * that OBU's bytes). */
    if (obu_len > e->bitstream.size) {
        return Fail("OBU exceeds bitstream HwBuf size");
    }
    std::memcpy(e->bitstream.user_va, obu_ptr, obu_len);

    /* 3b. Fill scratch buffer contents the codec will read. */
    /* tile_info end-offset is the tile-data length within the bitstream
     * (excluding OBU framing + frame_header), NOT the full OBU TU.
     *
     * Multi-tile: parse per-tile sizes from the tile group OBU payload.
     * AV1 spec §6.10.1: each non-last tile is prefixed with a
     * tile_size_minus_1 field of n_bytes bytes (little-endian), then
     * tile_size bytes of coded tile data.  The last tile runs to end. */
    /* AV1 spec caps NumTiles at 512 (MAX_TILE_COLS * MAX_TILE_ROWS bound). */
    constexpr int kMaxTiles = 512;
    uint32_t tile_offsets_buf[kMaxTiles][2];
    const uint32_t (*tile_offsets_ptr)[2] = nullptr;
    {
        const int tile_cols = p_frame_hdr->tiling.cols;
        const int tile_rows = p_frame_hdr->tiling.rows;
        const int n_tiles   = tile_cols * tile_rows;
        if (n_tiles > 1 && n_tiles <= kMaxTiles) {
            const int n_bytes = p_frame_hdr->tiling.n_bytes; /* TileSizeBytes */
            const uint8_t *cur = (const uint8_t *)e->bitstream.user_va + frame_tag_size;
            /* tile_group_obu() header (AV1 spec §6.10.1): for NumTiles>1 the
             * payload begins with tile_start_and_end_present_flag (1 bit),
             * optionally tg_start/tg_end (Ceil_Log2(NumTiles) bits each),
             * then byte-aligned.  For single-tile there is no such header —
             * frame_tag_size lands directly on the tile data, giving start=0.
             * For multi-tile we must skip the header so cursor lands on the
             * first tile_size field (or first tile data byte for the last tile).
             *
             * Header size in bits: 1 + (flag ? 2*Ceil_Log2(NumTiles) : 0)
             * Ceil_Log2(NumTiles) <= 9 for AV1_MAX_TILES=512, so header is
             * always 1-3 bytes.  Read the flag bit from the stream itself. */
            uint32_t cursor;
            {
                bool flag = (cur[0] >> 7) & 1;
                if (!flag) {
                    cursor = 1; /* 1-bit flag only, byte-aligned */
                } else {
                    int tg_bits = 0;
                    for (int tmp = n_tiles - 1; tmp > 0; tmp >>= 1) tg_bits++;
                    cursor = (1 + 2 * tg_bits + 7) / 8;
                }
            }
            bool parse_ok = true;
            for (int t = 0; t < n_tiles; t++) {
                if (t < n_tiles - 1) {
                    /* read n_bytes LE tile_size_minus_1 */
                    if (cursor + (uint32_t)n_bytes > tile_data_len) { parse_ok = false; break; }
                    uint32_t sz_minus1 = 0;
                    for (int b = 0; b < n_bytes; b++)
                        sz_minus1 |= (uint32_t)cur[cursor + b] << (b * 8);
                    cursor += (uint32_t)n_bytes;
                    uint32_t tile_size = sz_minus1 + 1;
                    tile_offsets_buf[t][0] = cursor;
                    /* end is exclusive (one-past-last), matching the
                     * single-tile convention where end = tile_data_len. */
                    tile_offsets_buf[t][1] = cursor + tile_size;
                    cursor += tile_size;
                } else {
                    /* last tile: no size field, runs to end of tile group payload */
                    tile_offsets_buf[t][0] = cursor;
                    tile_offsets_buf[t][1] = tile_data_len;
                }
            }
            if (parse_ok)
                tile_offsets_ptr = tile_offsets_buf;
        }
    }
    Av1BuildTileInfo(p_seq_hdr, p_frame_hdr, tile_data_len,
                     tile_offsets_ptr,
                     (uint8_t *)e->tile_info.user_va, e->tile_info.size);
    Av1BuildGlobalModel(p_frame_hdr,
                        (uint8_t *)e->global_model.user_va,
                        e->global_model.size);
    /* tile_out_internal is now used ONLY as the segment_read scratch
     * (reg81), which the codec reads as zero for non-segmented streams.
     * The kernel ALLOC_BUFFER path already RtlZeroMemory's the buffer at
     * alloc, and the codec does not write back to tile_out_internal
     * (pool_internal[] is the VCD write target).  So the per-kick memset
     * is redundant — costs ~12 MiB/kick at 4K with no benefit.
     * If segmentation is ever enabled and the codec writes segment-map
     * output here, this needs to come back conditional on
     * `hdr->segmentation.enabled`. */

    /* prob_tbl content: AV1 inter-frame seeding rule (spec §7.4):
     *   if (primary_ref_frame == PRIMARY_REF_NONE)
     *      load_cdfs(default_cdf)
     *   else
     *      load_cdfs(SavedCdfs[refidx[primary_ref_frame]])
     * SavedCdfs[s] is the post-decode CDF state of the frame that was
     * placed into ref slot s — i.e., what the codec wrote to prob_tbl_out
     * during that frame's kick.  We snapshot prob_tbl_out into
     * e->saved_cdf[s] for each refresh slot post-kick. */
    {
        /* AV1 inter-frame seeding rule (spec §7.4):
         *   if (primary_ref_frame == PRIMARY_REF_NONE)
         *      load_cdfs(default_cdf[qcat])  -- where qcat indexes into
         *      the BSP-captured per-quant default tables (kAv1DefaultCdfsByQcat).
         *   else
         *      load_cdfs(SavedCdfs[refidx[primary_ref_frame]])
         *
         * qcat formula matches dav1d's `dav1d_cdf_thread_init_static`:
         *   qcat = (yac>20) + (yac>60) + (yac>120)
         */
        const Dav1dFrameHeader *fh = p_frame_hdr;
        const uint32_t yac  = fh ? (uint32_t)fh->quant.yac : 0;
        const uint32_t qcat = (yac > 20u) + (yac > 60u) + (yac > 120u);
        const size_t   cdf_bytes  = sizeof(kAv1DefaultCdfsByQcat[0]);
        const size_t   copy_bytes = (cdf_bytes < e->prob_tbl.size)
                                    ? cdf_bytes : e->prob_tbl.size;
        const uint8_t *src = (const uint8_t *)&kAv1DefaultCdfsByQcat[qcat][0];
        /* AV1 spec §7.20 / BSP av1d_codec.c:1756: use default CDFs when
         * error_resilient_mode || frame_is_intra (KEY/INTRA_ONLY) ||
         * primary_ref_frame == PRIMARY_REF_NONE.  Only otherwise inherit
         * from the slot named by primary_ref_frame.  SVT-AV1 marks
         * random-access alt-refs with error_resilient_mode=1 — missing
         * that gate caused prob_tbl_N mismatch vs av1capture_v5 BSP from
         * kick 4 onward. */
        const bool use_defaults =
            !fh
         || fh->error_resilient_mode
         || fh->frame_type == DAV1D_FRAME_TYPE_KEY
         || fh->frame_type == DAV1D_FRAME_TYPE_INTRA
         || fh->primary_ref_frame == DAV1D_PRIMARY_REF_NONE;
        if (!use_defaults) {
            int ref_slot = fh->refidx[fh->primary_ref_frame];
            if (ref_slot >= 0 && ref_slot < 8 &&
                e->saved_cdf[ref_slot].size() >= copy_bytes) {
                src = e->saved_cdf[ref_slot].data();
            }
        }
        std::memcpy(e->prob_tbl.user_va, src, copy_bytes);
        if (copy_bytes < e->prob_tbl.size) {
            std::memset((uint8_t *)e->prob_tbl.user_va + copy_bytes, 0,
                        e->prob_tbl.size - copy_bytes);
        }
    }
    /* Do NOT zero prob_tbl_out: the codec writes the post-decode CDF
     * state there each kick, and we snapshot it after a successful kick
     * (see end of Av1HwKickPicture).  BSP also leaves it untouched. */

    /* 4. Build RKMPP_REG_WRITE list. */
    RKMPP_SUBMIT_JOB_IN in{};
    in.StructSize = sizeof(in);
    in.TimeoutMs  = 1000;
    in.BufRefCount = 0;

    /* Frame size.
     * BSP vdpu_av1d_setup_tile_bufs(): internal luma plane height is
     * ALIGN(frame_h, 16), so the UV base within pool_internal is
     * frame_w * ALIGN(frame_h, 16), NOT frame_w * frame_h.
     * pool_output (PP NV12 output) uses the un-padded dimensions. */
    const uint32_t frame_w    = e->frame_width;
    const uint32_t frame_h    = e->frame_height;
    /* Codec internal layout uses the coded (64-aligned) width for the
     * raster stride.  y_size_int is the UV-plane offset relative to
     * the Y base (reg65 → reg99), so it MUST account for the actual
     * codec-side Y plane footprint, not the display crop.  Using
     * frame_w directly when display != coded made the codec's Y
     * writes overrun into the UV region and PP read bogus data,
     * producing a sparse-superblock pattern in pool_output.       */
    const uint32_t coded_w    = (frame_w + 63u) & ~63u;
    const uint32_t y_h_int    = (frame_h + 15u) & ~15u;
    /* Codec INTERNAL layout (TILE_OUT_LU / TILE_OUT_CH / TILE_OUT_MV)
     * is bit-depth-aware: luma byte size = coded_w * coded_h * depth/8.
     * Matches upstream rockchip_vpu981_av1_dec_luma_size().  For 10-bit
     * streams the internal Y plane is 1.25× wider than 8-bit, so the
     * UV-offset register (reg99) and MV-offset register (reg133) MUST
     * scale with bit-depth — otherwise the codec scribbles Y bytes
     * over the start of UV and produces the "tiling corruption"
     * symptom that survives even after the PP output format is
     * correctly set to P010. */
    const uint32_t bit_depth_int = p_seq_hdr->hbd ? 10u : 8u;
    const uint32_t y_size_int    = coded_w * y_h_int * bit_depth_int / 8u;
    /* PP raster output byte stride:
     *   8-bit  → NV12  (sw_pp_out_format=3): bytesperline = width
     *   10-bit → P010  (sw_pp_out_format=1): bytesperline = width * 2
     * Matches upstream kernel rockchip_vpu981_hw_av1_dec.c:2225-2241.
     * 16-aligned for the PP write fabric.  reg328 (UV-plane base
     * offset relative to reg326) is the BYTE size of the Y plane,
     * not a pixel count — chroma_offset = bytesperline * height. */
    /* Codec PP-output bytes/sample: NV12 = 8, NV15 (10-bit) = 10 bits
     * packed (4 samples / 5 bytes).  Matches the sw_pp_out_format
     * choice in regbuilder_av1.cpp. */
    const uint32_t bpp_bits_out  = p_seq_hdr->hbd ? 10u : 8u;
    const uint32_t coded_w_out   = coded_w;
    /* PP-output Y-plane storage height = (frame_h + 15) & ~15.  Matches
     * upstream V4L2 P010/NV12 frmsize.step_height = MB_DIM = 16
     * (rockchip_vpu_hw.c:99-106) — upstream's pp_out_height (reg332) AND
     * chroma_offset (reg328) both use dst_fmt.height, which v4l2_apply_
     * frmsize_constraints rounded up to 16 (1088 for a 1080p stream).
     * The codec writes 17 SB rows × 64 = 1088 luma rows either way;
     * cropping happens on the display side.  regbuilder_av1.cpp uses
     * the same `(h + 15) & ~15` for pp_out_height so codec + buffer
     * agree.  An 8-row alignment here gave bottom-right corner
     * corruption on 10-bit AV1 (last partial SB row's chroma writes
     * spilled into the UV plane the repack thought it was reading). */
    const uint32_t coded_h_out   = (frame_h + 15u) & ~15u;
    const uint32_t pp_stride_out =
        ((coded_w_out * bpp_bits_out + 7u) / 8u + 15u) & ~15u;
    const uint32_t y_size_out    = pp_stride_out * coded_h_out;

    /* Filter column buffer sub-offsets within filter_mem.
     * Matches hal_av1d_vdpu.c:vdpu_av1d_filtermem_alloc() + mpp_dev_set_reg_offset().
     * Layout (packed, each sub-buffer * num_tile_cols):
     *   [0]            DB_DATA_COL  → reg179
     *   [db_ctrl_off]  DB_CTRL_COL  → reg183
     *   [cdef_col_off] CDEF_COL     → reg85
     *   [sr_col_off]   SR_COL       → reg89
     *   [lr_col_off]   LR_COL       → reg91
     */
    {
        /* BSP hal_av1d_vdpu.c vdpu_av1d_filtermem_alloc():
         *   pic_height  = ALIGN(frame_h, 64)        — always 64px alignment
         *   height_in_sb = pic_height / 64           — 64px "stripes", NOT SB-size based
         *   max_bit_depth = 10                       — hardcoded; sizes for 10-bit even on 8-bit streams
         *   stripe_num  = (pic_height + 8 + 63) / 64 — for LR sub-buffer
         * These match regardless of seq->sb128.  Using actual hbd or sb_size here
         * produces wrong offsets (verified against 4K 8-bit stream failure). */
        const uint32_t max_bit_depth = 10u;
        const uint32_t pic_height    = (frame_h + 63u) & ~63u;
        const uint32_t height_in_sb  = pic_height / 64u;
        const uint32_t stripe_num    = (pic_height + 8u + 63u) / 64u;
        const uint32_t ntc           = (uint32_t)p_frame_hdr->tiling.cols;
        auto a128 = [](uint32_t x) { return (x + 127u) & ~127u; };
        uint32_t off = 0;
        /* DB_DATA_COL — reg179 stays at 0 */
        off += a128(pic_height * 12u * max_bit_depth / 8u) * ntc;
        e->filt_db_ctrl_off  = off;
        off += a128(pic_height * 2u * 16u / 4u) * ntc;
        e->filt_cdef_col_off = off;
        off += a128(height_in_sb * 44u * max_bit_depth * 16u / 8u) * ntc;
        e->filt_sr_col_off   = off;
        off += a128(height_in_sb * (3040u + 1280u)) * ntc;
        e->filt_lr_col_off   = off;
        (void)stripe_num;
    }

    /* DMA writes mirroring what BSP MPP HAL programs for a keyframe at
     * 720p (cross-checked against tests/data/av1/av1capture/reg_0_in.txt).
     *
     * Codec writes its internal Y/UV/colmv layout to tile_out_internal
     * (reg65/99/133); PP module reads from there and writes the user-
     * visible NV12 to pool_output[slot] (reg326/328).
     *
     * filter_mem is the shared DB/CDEF/LR/SR column-buffer scratch and
     * is referenced by 5 reg pairs (the codec uses internal offsets
     * within the buffer for each feature).
     *
     * prob_tbl + prob_tbl_out hold the CDF probability tables (in/out).
     * tile_buf is mc_sync per-tile-column scratch.  global_model holds
     * warped-motion params. */
    const uint32_t uv_size  = y_size_int / 2;
    const uint32_t mv_off   = y_size_int + uv_size + 64;  /* per BSP HAL line 2106 */
    /* Per-ref Y/UV/MV DMA lsb register positions (from regbuilder_av1_reg.h):
     *   ref r → swreg{67,69,71,73,75,77,79}[r]   = sw_referN_ybase_lsb
     *           swreg{101,103,105,107,109,111,113}[r] = sw_referN_cbase_lsb
     *           swreg{135,137,139,141,143,145,147}[r] = sw_referN_dbase_lsb
     * Each ref slot points at the pool_internal[] buffer of the DPB slot
     * named by frame_hdr->refidx[r]. */
    static const uint32_t kRefYLsb [7] = {  67,  69,  71,  73,  75,  77,  79 };
    static const uint32_t kRefUvLsb[7] = { 101, 103, 105, 107, 109, 111, 113 };
    static const uint32_t kRefDLsb [7] = { 135, 137, 139, 141, 143, 145, 147 };

    /* Upper bound: 17 fixed entries + 7 refs * 3 = 38; cap at 48. */
    constexpr size_t kMaxEssential = 48;
    Av1DmaWrite essential_buf[kMaxEssential];
    size_t       essential_n = 0;
    auto essential_push = [&](Av1DmaWrite w) {
        if (essential_n < kMaxEssential) essential_buf[essential_n++] = w;
    };
    /* Current frame's internal Y/UV/MV — write directly to
     * pool_internal[slot_idx] so subsequent kicks that reference this
     * DPB slot can read it via DMA without any CPU copy.  This matches
     * the BSP design where each DPB slot has its own tile_out buffer.
     * tile_out_internal is kept alive only for the segment_map read
     * (reg81), which is pre-zeroed and stays zero for non-segmented
     * streams. */
    essential_push({  65, &e->pool_internal[slot_idx], 0 });
    essential_push({  99, &e->pool_internal[slot_idx], y_size_int });
    essential_push({ 133, &e->pool_internal[slot_idx], mv_off });
    /* segment_read scratch — stays on tile_out_internal (pre-zeroed) */
    essential_push({  81, &e->tile_out_internal, 0 });
    /* global motion */
    essential_push({  83, &e->global_model,            0 });
    /* Filter column buffers — each sub-region within filter_mem.
     * Offsets computed above matching hal_av1d_vdpu.c:filtermem_alloc. */
    essential_push({  85, &e->filter_mem, e->filt_cdef_col_off });
    essential_push({  89, &e->filter_mem, e->filt_sr_col_off   });
    essential_push({  91, &e->filter_mem, e->filt_lr_col_off   });
    essential_push({ 179, &e->filter_mem,                    0 });
    essential_push({ 183, &e->filter_mem, e->filt_db_ctrl_off  });
    /* tile-info dim/offset table */
    essential_push({ 167, &e->tile_info,               0 });
    /* compressed bitstream — codec entropy cursor needs to start at
     * tile-group bits, not OBU framing.  bs_iova_offset = byte
     * offset of tile_group within the bitstream HwBuf, 16-byte
     * aligned; sub-byte residual goes into swreg5.sw_strm_start_bit
     * (set in regs above). */
    essential_push({ 169, &e->bitstream,               bs_iova_offset });
    /* prob tables (CDF in/out) */
    essential_push({ 171, &e->prob_tbl_out,            0 });
    essential_push({ 173, &e->prob_tbl,                0 });
    /* mc_sync scratch */
    essential_push({ 175, &e->tile_buf,                0 });
    essential_push({ 177, &e->tile_buf,                0 });
    /* PP output: user-visible NV12 — codec writes here last. */
    essential_push({ 326, &e->pool_output[slot_idx],   0 });
    essential_push({ 328, &e->pool_output[slot_idx],   y_size_out });

    /* Reference frames — for inter decode the codec reads previous frames'
     * tile_out_internal layout from pool_internal[dpb_to_pool[refidx[r]]].
     * Inter frames with show_existing_frame skipped don't reach here.
     * For intrabc/key frames refidx[r] is meaningless and the codec doesn't
     * read refs (sw_ref_frames=0/1 from regbuilder), so refs left blank are
     * fine. For an unmapped slot (no decoded frame ever assigned), fall
     * back to error_ref so the codec reads valid (zero-filled) memory
     * rather than hitting the IOMMU on a null base. */
    if (p_frame_hdr &&
        p_frame_hdr->frame_type != DAV1D_FRAME_TYPE_KEY &&
        p_frame_hdr->frame_type != DAV1D_FRAME_TYPE_INTRA &&
        !p_frame_hdr->allow_intrabc)
    {
        if (std::getenv("RKMPP_AV1_DUMP_DIR") || std::getenv("RKMPP_AV1_TRACE")) {
            std::fprintf(stderr,
                "AV1_TRACE refmap kick=%llu refidx=[", (unsigned long long)kick_no);
            for (int r = 0; r < 7; r++)
                std::fprintf(stderr, "%d%s", p_frame_hdr->refidx[r], r < 6 ? "," : "");
            std::fprintf(stderr, "] resolved_pool=[");
            for (int r = 0; r < 7; r++) {
                int dpb_slot = p_frame_hdr->refidx[r];
                int pool_slot = (dpb_slot >= 0 && dpb_slot < 8) ? e->dpb_to_pool[dpb_slot] : -1;
                std::fprintf(stderr, "%d%s", pool_slot, r < 6 ? "," : "");
            }
            std::fprintf(stderr, "] frame_offset=%d\n", p_frame_hdr->frame_offset);
            std::fflush(stderr);
        }
        for (int r = 0; r < 7; r++) {
            int dpb_slot = p_frame_hdr->refidx[r];
            Av1DecodeEngine::HwBuf *buf = &e->error_ref;
            uint32_t y_off  = 0;
            uint32_t uv_off = 0;
            uint32_t d_off  = 0;
            if (dpb_slot >= 0 && dpb_slot < 8) {
                int pool_slot = e->dpb_to_pool[dpb_slot];
                if (pool_slot >= 0 && pool_slot < Av1DecodeEngine::kPoolSize &&
                    e->pool_internal[pool_slot].handle)
                {
                    buf    = &e->pool_internal[pool_slot];
                    y_off  = 0;
                    uv_off = y_size_int;
                    d_off  = mv_off;
                }
            }
            essential_push({ kRefYLsb [r], buf, y_off  });
            essential_push({ kRefUvLsb[r], buf, uv_off });
            essential_push({ kRefDLsb [r], buf, d_off  });
        }
    }

    /* Inline reg-push: avoids per-call lambda capture indirection.  Returns
     * 1 on overflow.  Compiles to a few stores. */
    #define PUSH_REG(off_, val_, handle_, hoff_) do {                        \
        if (in.RegWriteCount >= RKMPP_MAX_REG_WRITES) {                      \
            return Fail("REG_WRITE list overflow");                          \
        }                                                                    \
        RKMPP_REG_WRITE *w_ = &in.Writes[in.RegWriteCount++];                \
        w_->Offset       = (off_);                                           \
        w_->Value        = (val_);                                           \
        w_->BufferHandle = (handle_);                                        \
        w_->IovaOffset   = (hoff_);                                          \
        w_->Reserved     = 0;                                                \
    } while (0)

    /* Emit non-DMA / non-zero swregs in ascending order.  Skip kick
     * (idx 1) — appended last per BSP write-order convention.  Skip
     * DMA lsb positions (handled by essential[] below); skip DMA msb
     * positions (idx-1 of any lsb) — kernel sets those during iova
     * substitution.  Build a 512-bit DMA-msb mask once instead of doing
     * a linear search per reg. */
    uint32_t dma_msb_mask[16] = {0};
    for (uint32_t v : kAv1DmaLsbIdx) {
        uint32_t m = v - 1;
        dma_msb_mask[m >> 5] |= 1u << (m & 31);
    }
    /* Walk the full 512-u32 reg set: regs 0..319 are VCD control + DMA
     * bases, regs 320..511 are PP cfg (output enable, format, base,
     * dimensions, strides).  PP cfg is what makes the user-visible
     * NV12 buffer get written; without it, codec runs VCD cleanly to
     * tile_out_internal but PP module stays idle and pool_output
     * stays zero. */
    /* Registers that upstream verisilicon writes as explicit 0 and that
     * the regbuilder doesn't touch (so the bank value is 0 there too).
     * The default skip-on-zero optimization would leave these UNWRITTEN,
     * which means the codec block keeps whatever stale value the prior
     * session left in the MMIO register.  For 10-bit AV1 specifically,
     * a stale bit in swreg260 (PP-mode flags: pp_format_p010_e,
     * ppd_blend_exist, pp_crop_exist, pp_up/down_level, pp_exist, etc.)
     * or swreg266 (sw_error_conceal_e) appears to corrupt the right-edge
     * SB column for the bottom 1-2 SB rows.  Force-write 0 to these
     * regs every kick so the codec block enters every AV1 session in a
     * known state. */
    static const uint32_t kAv1ForceWriteIdx[] = { 260, 266 };
    auto force_writes = [&](uint32_t idx) {
        for (uint32_t f : kAv1ForceWriteIdx)
            if (f == idx) return true;
        return false;
    };
    for (uint32_t idx = 2; idx < 512; idx++) {
        if (IsAv1DmaLsb(idx)) continue;
        if (dma_msb_mask[idx >> 5] & (1u << (idx & 31))) continue;
        if (r32[idx] == 0 && !force_writes(idx)) continue;
        PUSH_REG(idx * 4, r32[idx], 0, 0);
    }
    /* Authoritative DMA writes (substitute via BufferHandle). */
    for (size_t ei = 0; ei < essential_n; ei++) {
        auto &dw = essential_buf[ei];
        if (!dw.buf || !dw.buf->handle) continue;
        /* msb (idx-1) — value 0; kernel ORs the high byte during iova
         * substitution.  Empty plain write so the slot is touched. */
        PUSH_REG((dw.lsb_idx - 1) * 4, 0, 0, 0);
        /* lsb (idx) — substituted to (iova + offset) at submit time. */
        PUSH_REG(dw.lsb_idx * 4, 0, dw.buf->handle, dw.offset);
    }
    /* Kick last (driver also serialises but we keep BSP write-order
     * convention here too). */
    PUSH_REG(1 * 4, r32[1], 0, 0);
    #undef PUSH_REG

    /* 4b. Dump input buffers if RKMPP_AV1_DUMP_DIR is set.  Filenames
     * match BSP MPP capture conventions so we can byte-diff against
     * tests/data/av1/av1capture/.  Use the caller-supplied kick_no
     * (already incremented in DrainPictures) so the keyframe lands in
     * `_0.txt` matching BSP, not `_1.txt`. */
    {
        uint32_t k = (uint32_t)kick_no;
        Av1DumpBuffer("prob_tbl", k,
                      e->prob_tbl.user_va, e->prob_tbl.size);
        Av1DumpBuffer("prob_tbl_out", k,
                      e->prob_tbl_out.user_va, e->prob_tbl_out.size);
        Av1DumpBuffer("global_mode", k,
                      e->global_model.user_va, e->global_model.size);
        Av1DumpBuffer("tile_info", k,
                      e->tile_info.user_va, e->tile_info.size);
        Av1DumpRaw("stream", k,
                   e->bitstream.user_va, obu_len);

        /* Register array dump in BSP format ("reg[N] = XXXXXXXX\n").
         * DMA lsb/msb positions are skipped — their values are fd
         * integers on Linux (meaningless here) or zero (msb).  All
         * other non-zero regs are compared against av1capture/reg_N_in.txt
         * to find control-register divergences between our output and BSP. */
        const char *ddir = Av1DumpDir();
        if (ddir) {
            char rpath[512];
            std::snprintf(rpath, sizeof(rpath), "%s/reg_%u_in.txt", ddir, k);
            FILE *rfp = std::fopen(rpath, "w");
            if (rfp) {
                for (uint32_t idx = 0; idx < 512; idx++) {
                    std::fprintf(rfp, "reg[%3u] = %08x\n", idx, r32[idx]);
                }
                std::fclose(rfp);
            }
        }
    }

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

    /* 7. Copy output buffer into Av1DecodedFrame YUV, cropping from PP's
     * coded raster stride down to the display width the caller sees. */
    {
        /* PP-output is NV12 (8-bit) or NV15-packed (10-bit, 4 samples
         * per 5 bytes); engine returns NV12 or P010 to the host.  See
         * regbuilder_av1.cpp:sw_pp_out_format note for why we use NV15
         * (out_fmt=10) rather than P010 (out_fmt=1) for hbd=1. */
        const uint32_t bit_depth = p_seq_hdr->hbd ? 10u : 8u;
        const uint8_t *src = (const uint8_t *)e->pool_output[slot_idx].user_va;
        RepackCodecOutputToNV12orP010(src, pp_stride_out, coded_h_out,
                                      frame_w, frame_h, bit_depth, &f->yuv);
    }
    f->slot_idx = slot_idx;

    /* Dump pool_output raw (full allocated size) to expose codec-side
     * stride.  Filename encodes configured frame_w/h; consumer can
     * ffmpeg-probe at coded vs display widths to find the real stride. */
    if (const char *dir = Av1DumpDir()) {
        const Av1DecodeEngine::HwBuf &po = e->pool_output[slot_idx];
        if (po.user_va && po.size) {
            char path[512];
            std::snprintf(path, sizeof(path),
                "%s/pool_out_%04u_cfg%ux%u_sz%u.bin",
                dir, (unsigned)kick_no, frame_w, frame_h, po.size);
            FILE *fp = std::fopen(path, "wb");
            if (fp) {
                std::fwrite(po.user_va, 1, po.size, fp);
                std::fclose(fp);
            }
        }
    }

    /* pool_internal[slot_idx] now contains the codec-internal output
     * directly (reg65/99/133 pointed there); no CPU copy needed. */

    /* Dump tile_out_internal content stats + raw NV12 for visual inspection.
     * tile_out_internal is no longer the VCD write target (pool_internal is),
     * so dump pool_internal[slot_idx] instead. */
    Av1DumpTileOut((uint32_t)kick_no, frame_w, frame_h, &e->pool_internal[slot_idx]);

    /* 8. Snapshot post-decode CDFs into saved_cdf[] AND map each refreshed
     * DPB slot to the pool slot that just got its codec-internal output.
     * AV1 inter frames with primary_ref_frame use saved_cdf[] to seed
     * prob_tbl on a future kick; ref-frame DMA bases on a future kick
     * use dpb_to_pool[] to find the corresponding pool_internal[] buffer. */
    if (p_frame_hdr) {
        /* DPB ref-pool tracking always runs (used for ref-frame DMA bases
         * on future inter kicks).  CDF snapshot is gated on the
         * AV1 spec equivalent of BSP's `!disable_frame_end_update_cdf`:
         *   snap only when disable_cdf_update == 0 AND refresh_context == 1.
         * When skipped, saved_cdf[s] retains the prior frame's content,
         * matching BSP's behavior of not running parser_update. */
        const size_t cdf_bytes = sizeof(kAv1DefaultCdfsByQcat[0]);
        const size_t snap_bytes = (cdf_bytes < e->prob_tbl_out.size)
                                  ? cdf_bytes : e->prob_tbl_out.size;
        uint32_t refresh = p_frame_hdr->refresh_frame_flags;
        if (p_frame_hdr->frame_type == DAV1D_FRAME_TYPE_KEY)
            refresh = 0xFF;
        if (std::getenv("RKMPP_AV1_TRACE")) {
            std::fprintf(stderr,
                "AV1_TRACE refresh kick=%llu slot=%d refresh_flags=0x%02x show_frame=%d show_existing=%d\n",
                (unsigned long long)kick_no, slot_idx, refresh,
                p_frame_hdr->show_frame, p_frame_hdr->show_existing_frame);
            std::fflush(stderr);
        }
        /* BSP stores the seed CDFs into cdfs_last[s] pre-decode for every
         * refreshed slot (av1d_codec.c:1779 av1d_store_cdfs); then if
         * !disable_frame_end_update_cdf, parser_update overwrites with the
         * post-decode CDFs.  Net effect: cdfs_last[s] for a refreshed slot
         * is always THIS frame's contribution — post-decode if end-update
         * is enabled, else the seed it started with.  Mirror that: when
         * snap_cdfs is false we still update saved_cdf[s] with the seed
         * (= what we wrote to prob_tbl pre-kick).  Skipping the save
         * leaves saved_cdf[s] holding stale content from an earlier
         * refresher, which corrupts CDF inheritance for any future frame
         * whose primary_ref_frame resolves to this slot. */
        const bool snap_cdfs = !p_frame_hdr->disable_cdf_update
                             && p_frame_hdr->refresh_context;
        const uint8_t *src = snap_cdfs
            ? (const uint8_t *)e->prob_tbl_out.user_va
            : (const uint8_t *)e->prob_tbl.user_va;
        for (int s = 0; s < 8; s++) {
            if (!(refresh & (1u << s))) continue;
            e->dpb_to_pool[s] = slot_idx;
            e->saved_cdf[s].assign(src, src + snap_bytes);
        }
    }
    return 0;
}

/* Move the lowest-PTS entry of reorder_q into ready_q.  Mirrors
 * decode_engine.cpp's bump_lowest for H.265.  Called whenever
 * reorder_q exceeds e->max_reorder_pics so PollFrame returns frames in
 * display order even for hierarchical (pred-struct=2) AV1 streams.
 * For low-delay streams max_reorder_pics is 0 so every push immediately
 * bumps to ready_q, giving zero added latency. */
static void Av1BumpLowestPts(Av1DecodeEngine *e) {
    while (e->reorder_q.size() > e->max_reorder_pics) {
        size_t best = 0;
        for (size_t i = 1; i < e->reorder_q.size(); i++) {
            if (e->reorder_q[i].pts_hns < e->reorder_q[best].pts_hns) best = i;
        }
        e->ready_q.push_back(std::move(e->reorder_q[best]));
        e->reorder_q.erase(e->reorder_q.begin() + best);
    }
}

/* Resolve max_reorder_pics from the cached sequence header.  Called
 * once per seq header parse.  enable_order_hint=0 → no reorder needed
 * (low-delay GOP); =1 → buffer up to 16 frames so the lowest-PTS bump
 * survives even hierarchical-levels=5 mini-GOPs (size 32, max reorder
 * distance 16) which SVT-AV1's pred-struct=2 may produce. */
static void Av1ResolveMaxReorder(Av1DecodeEngine *e) {
    e->max_reorder_pics = e->cached_seq_hdr.order_hint ? 16u : 0u;
}

/* Drain pictures from the OBU records produced by the Submit-time walk.
 * Each record is parsed with the clean-room parser; show_existing frames
 * are served from the pool without kicking HW; coded frames kick HW and
 * update the parser saved-state and dpb_to_pool tables. */
static int DrainPictures(Av1DecodeEngine *e, int64_t pts_hns) {
    /* Walk each OBU record the Submit-time Av1WalkObus produced. */
    for (; e->tu_obu_idx < e->tu_obus.size(); e->tu_obu_idx++) {
        const AV1ObuRecord &rec = e->tu_obus[e->tu_obu_idx];

        if (!e->seq_hdr_valid) {
            std::fprintf(stderr, "av1_engine: no sequence header yet, dropping OBU\n");
            continue;
        }

        /* Locate the OBU payload: the OBU record gives us slice_start and
         * frame_tag_off (= OBU header bytes + leb128 size bytes).  The
         * OBU payload begins at slice_start + frame_tag_off for both
         * OBU_FRAME_HEADER (type 3) and OBU_FRAME (type 6).  The payload
         * length is slice_size - frame_tag_off. */
        const uint8_t *obu_payload = e->tu_ptr + rec.slice_start + rec.frame_tag_off;
        const size_t   obu_payload_len = rec.slice_size > rec.frame_tag_off
                                         ? (rec.slice_size - rec.frame_tag_off) : 0;

        /* Parse the frame header. */
        Dav1dFrameHeader hdr{};
        const bool is_frame_type = (rec.obu_type == 6); /* OBU_FRAME */
        if (Av1ParseFrameHeader(obu_payload, obu_payload_len,
                                is_frame_type,
                                &e->cached_seq_hdr,
                                e->saved_states,
                                &hdr) != 0) {
            std::fprintf(stderr, "av1_engine: Av1ParseFrameHeader failed, dropping\n");
            continue;
        }

        if (hdr.show_existing_frame) {
            /* show_existing_frame: no HW kick.  Look up the pool slot from
             * dpb_to_pool[] via existing_frame_idx and memcpy the
             * pool_output pixels directly into the output frame. */
            int dpb_slot  = hdr.existing_frame_idx;
            int pool_slot = (dpb_slot >= 0 && dpb_slot < 8)
                            ? e->dpb_to_pool[dpb_slot] : -1;

            if (std::getenv("RKMPP_AV1_TRACE")) {
                std::fprintf(stderr,
                    "AV1_TRACE drain show_existing pts_hns=%lld qsize=%zu "
                    "dpb_slot=%d pool_slot=%d\n",
                    (long long)pts_hns, e->ready_q.size(),
                    dpb_slot, pool_slot);
                std::fflush(stderr);
            }

            if (pool_slot < 0 || pool_slot >= Av1DecodeEngine::kPoolSize ||
                !e->pool_output[pool_slot].user_va)
            {
                std::fprintf(stderr,
                    "av1_engine: show_existing dpb_slot=%d has no valid pool slot, dropping\n",
                    dpb_slot);
                /* Do NOT update saved_states for a dropped show_existing. */
                continue;
            }

            /* Width/height come from the saved frame state for that slot. */
            const uint32_t w = (uint32_t)e->saved_states[dpb_slot].saved.width[0];
            const uint32_t h = (uint32_t)e->saved_states[dpb_slot].saved.height;
            const uint32_t coded_w_out = (w + 63u) & ~63u;
            /* 16-row alignment — matches DecodeOne_AV1's coded_h_out
             * (the codec wrote pool_output's UV plane at offset
             * pp_stride * ((h + 15) & ~15) on the original kick). */
            const uint32_t coded_h_out = (h + 15u) & ~15u;

            Av1DecodedFrame f;
            f.width  = w;
            f.height = h;
            f.has_film_grain = false;
            f.pts_hns = (pts_hns >= 0) ? pts_hns
                                       : (int64_t)(e->submit_count *
                                                   (10'000'000ULL / 30));
            f.dur_hns = 10'000'000LL / 30;
            f.slot_idx = -1;  /* no pool hold for show_existing copies */

            /* Same repack as the HW-kick path: NV12 passthrough for 8-bit,
             * NV15 → P010 unpack for 10-bit.  pp_stride matches what the
             * regbuilder configured the codec to write on the original
             * kick: width * bit_depth / 8, 16-aligned. */
            const uint32_t bit_depth = e->cached_seq_hdr.hbd ? 10u : 8u;
            const uint32_t pp_stride = ((coded_w_out * bit_depth + 7u) / 8u + 15u) & ~15u;
            const uint8_t *src = (const uint8_t *)e->pool_output[pool_slot].user_va;
            RepackCodecOutputToNV12orP010(src, pp_stride, coded_h_out,
                                          w, h, bit_depth, &f.yuv);
            e->reorder_q.push_back(std::move(f));
            Av1BumpLowestPts(e);

            /* show_existing_frame: update saved_states per AV1 spec §5.9.2.
             * For show_existing KEY frames refresh_frame_flags=0xFF and the
             * shown frame's saved state is inherited.  For inter, flags=0 so
             * Av1UpdateSavedStates is a no-op.  Call unconditionally. */
            Av1UpdateSavedStates(e->saved_states, &hdr);
            /* DPB tracking for regbuilder. */
            rkmpp_av1_dpb_post_decode(&e->dpb, &e->cached_seq_hdr, &hdr);
            continue;
        }

        /* Coded frame (visible or hidden alt-ref): kick HW. */

        /* Always advance submit_count for pool slot rotation, regardless of
         * pts source.  Advancing only in the pts-ternary branch caused
         * every MFT-driven kick to reuse pool_output[0], overwriting the
         * previous output before the caller could memcpy it. */
        const uint64_t kick_no = e->submit_count++;

        /* Pick a pool slot not currently held by an active DPB ref.
         * Mirrors the BSP MppBufferGroup refcount logic.  Round-robin
         * over free slots; fall back to raw modulo if all slots in use. */
        bool slot_in_use[Av1DecodeEngine::kPoolSize] = {};
        for (int s = 0; s < 8; s++) {
            int ps = e->dpb_to_pool[s];
            if (ps >= 0 && ps < Av1DecodeEngine::kPoolSize)
                slot_in_use[ps] = true;
        }
        int slot = -1;
        for (int probe = 0; probe < Av1DecodeEngine::kPoolSize; probe++) {
            int candidate = (int)((kick_no + probe) % Av1DecodeEngine::kPoolSize);
            if (!slot_in_use[candidate]) { slot = candidate; break; }
        }
        if (slot < 0) slot = (int)(kick_no % Av1DecodeEngine::kPoolSize);

        if (std::getenv("RKMPP_AV1_TRACE")) {
            std::fprintf(stderr,
                "AV1_TRACE slot_alloc kick=%llu slot=%d in_use=%d%d%d%d%d%d%d%d dpb_to_pool=[",
                (unsigned long long)kick_no, slot,
                slot_in_use[0], slot_in_use[1], slot_in_use[2], slot_in_use[3],
                slot_in_use[4], slot_in_use[5], slot_in_use[6], slot_in_use[7]);
            for (int s = 0; s < 8; s++)
                std::fprintf(stderr, "%d%s", e->dpb_to_pool[s], s == 7 ? "" : ",");
            std::fprintf(stderr, "]\n");
            std::fflush(stderr);
        }

        Av1DecodedFrame f;
        f.width  = (uint32_t)hdr.width[0];
        f.height = (uint32_t)hdr.height;
        f.has_film_grain = (hdr.film_grain.data.num_y_points > 0);
        f.pts_hns = (pts_hns >= 0) ? pts_hns
                                   : (int64_t)(kick_no * (10'000'000ULL / 30));
        f.dur_hns = 10'000'000LL / 30;

        if (e->mode == Av1EngineMode::Software) {
            /* Software mode has no dav1d fallback; return empty frame. */
            std::fprintf(stderr,
                "av1_engine: Software mode not supported without dav1d; "
                "switch to Hardware mode\n");
            f.slot_idx = -1;
        } else {
            int hw_rc = Av1HwKickPicture(e, &hdr, &e->cached_seq_hdr,
                                         slot, kick_no, &rec, &f);
            if (std::getenv("RKMPP_AV1_TRACE")) {
                uint32_t h32 = 2166136261u;
                const uint8_t *po = (const uint8_t *)e->pool_output[slot].user_va;
                const size_t   pn = e->pool_output[slot].size;
                if (po) {
                    for (size_t i = 0; i < pn; i += 4096) { h32 ^= po[i]; h32 *= 16777619u; }
                }
                std::fprintf(stderr,
                    "AV1_TRACE post_kick kick=%llu slot=%d hw_rc=%d po_first=%02x "
                    "po_fnv=%08x f_yuv_first=%02x\n",
                    (unsigned long long)kick_no, slot, hw_rc,
                    (po && pn) ? po[0] : 0, h32,
                    f.yuv.empty() ? 0 : f.yuv[0]);
                std::fflush(stderr);
            }
            if (hw_rc != 0) {
                /* HW kick failed — discard this frame rather than returning
                 * garbage.  (No dav1d fallback available.) */
                std::fprintf(stderr,
                    "av1_engine: HW kick failed (kick %llu), frame dropped\n",
                    (unsigned long long)kick_no);
                /* Still update state so future kicks use correct refs. */
                Av1UpdateSavedStates(e->saved_states, &hdr);
                rkmpp_av1_dpb_post_decode(&e->dpb, &e->cached_seq_hdr, &hdr);
                continue;
            }
        }

        /* Update parser saved-state AFTER kick so the just-completed
         * frame's hdr is stored into the right slots.  Must be post-kick
         * to avoid clobbering a ref slot this same frame reads. */
        Av1UpdateSavedStates(e->saved_states, &hdr);
        /* DPB tracking for regbuilder reference resolution. */
        rkmpp_av1_dpb_post_decode(&e->dpb, &e->cached_seq_hdr, &hdr);

        /* Hidden alt-refs (show_frame=0) are decoded so future
         * show_existing_frame can reference them, but must NOT be
         * returned to the caller as displayable pictures. */
        if (hdr.show_frame == 0) {
            if (std::getenv("RKMPP_AV1_TRACE")) {
                std::fprintf(stderr,
                    "AV1_TRACE drain hidden_altref pts_hns=%lld kick=%llu (dropped)\n",
                    (long long)pts_hns, (unsigned long long)kick_no);
                std::fflush(stderr);
            }
            continue;
        }

        if (std::getenv("RKMPP_AV1_TRACE")) {
            std::fprintf(stderr,
                "AV1_TRACE drain visible pts_hns=%lld kick=%llu qsize=%zu w=%u h=%u\n",
                (long long)pts_hns, (unsigned long long)kick_no,
                e->ready_q.size(), f.width, f.height);
            std::fflush(stderr);
        }
        e->reorder_q.push_back(std::move(f));
        Av1BumpLowestPts(e);
    }
    return 0;
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

    if (mode == Av1EngineMode::Hardware) {
        if (OpenAv1Device(&e->device) != 0) {
            std::fprintf(stderr,
                "av1_engine: no AV1-capable rkmpp device (RKCP3560 with "
                "RKMPP_CODEC_AV1).  Run with mode=Hardware on the target "
                "device with rkmpp.sys AV1 personality enabled.\n");
            return 1;
        }
        /* Bitstream input — sized for the largest plausible AV1 OBU TU.
         * 1080p svtav1 P-frames typically run 50..500 KB; reserve 4 MB
         * to cover keyframes / 4K worst case. */
        if (AllocHwBuf(e->device, 4 * 1024 * 1024,
                       RkMppBufferUsageBitstreamInput, &e->bitstream) != 0)
            return 1;
        /* Tile-info: 16 bytes per tile, max 128 tiles per AV1_MAX_TILES.
         * Round to 4 KB. */
        if (AllocHwBuf(e->device, 4096, RkMppBufferUsageScratch,
                       &e->tile_info) != 0)
            return 1;
        /* Global motion model: 7 refs × 32 B = 224 B.  Round to 4 KB. */
        if (AllocHwBuf(e->device, 4096, RkMppBufferUsageScratch,
                       &e->global_model) != 0)
            return 1;
        /* AV1CDFs probability tables — sizeof(AV1CDFs) ≈ 12 KB per BSP
         * capture; round to 16 KB (MPP's MPP_ALIGN(_, 2048) padding). */
        if (AllocHwBuf(e->device, 16 * 1024, RkMppBufferUsageScratch,
                       &e->prob_tbl) != 0)
            return 1;
        if (AllocHwBuf(e->device, 16 * 1024, RkMppBufferUsageScratch,
                       &e->prob_tbl_out) != 0)
            return 1;
        /* Film-grain LUT scratch — only consumed when reg7.sw_apply_grain.
         * 64 KB is comfortably above MPP's sizeof(AV1FilmGrainMemory). */
        if (AllocHwBuf(e->device, 64 * 1024, RkMppBufferUsageScratch,
                       &e->film_grain) != 0)
            return 1;
        /* Filter / col-buffer scratch — pic_height-derived sum of CDEF +
         * LR + SR + DB column buffers.  At 720p num_tile_cols=1 it's
         * ~104 KB per the BSP filtermem_alloc formula; allocate 4 MiB
         * for headroom up to 4K.
         *
         * Resolved 2026-05-21: the historical "frame 0 (keyframe)
         * residual chroma artifact at x=1152..1279, y=616..629" was
         * NOT a filter_mem initial-state issue — it was the
         * driver/rkav1d/job.c cache-config block reading bit-depth
         * from sw_pp_in_format instead of sw_pp_out_format.  Fix
         * verified bit-exact against dav1d on test2_av1.mkv frame 0
         * (1080p 10-bit AV1).  The 0x80-vs-0x00 fill experiments on
         * filter_mem changed nothing observable because they weren't
         * the root cause; kernel zero-init stays as the default. */
        if (AllocHwBuf(e->device, 4 * 1024 * 1024, RkMppBufferUsageScratch,
                       &e->filter_mem) != 0)
            return 1;
        /* mc_sync per-tile-column scratch.  Single-tile case is tiny;
         * 256 KB covers up to 64 tile cols with comfortable margin. */
        if (AllocHwBuf(e->device, 256 * 1024, RkMppBufferUsageScratch,
                       &e->tile_buf) != 0)
            return 1;
        /* Single codec-internal write target (proven bit-exact for kick 0).
         * BSP vdpu_av1d_setup_tile_bufs(): internal Y height = ALIGN(h,16),
         * Y stride = ALIGN(w,64), dir_mvs = ALIGN(num_sbs*384, 16)*2
         * where num_sbs counts 64px stripes.  Sizing luma at the display
         * width (instead of the codec-side coded width) underflows by
         * up to 64 bytes per row when display!=coded, which the codec
         * silently overruns into the chroma region. */
        const uint32_t coded_w_alloc   = (width  + 63u) & ~63u;
        const uint32_t y_h_int_alloc   = (height + 15u) & ~15u;
        /* Codec internal Y/UV is bit-depth-aware (see DecodeOne_AV1's
         * y_size_int).  Allocate for the 10-bit upper bound (1.25×
         * the 8-bit cost) so 10-bit kicks don't overrun the buffer. */
        const uint32_t luma_alloc      = coded_w_alloc * y_h_int_alloc * 10u / 8u;
        const uint32_t chroma_alloc    = luma_alloc >> 1;
        const uint32_t num_sbs_alloc   = ((width  + 63u) / 64u + 1u) *
                                         ((height + 63u) / 64u + 1u);
        const uint32_t dir_mvs_alloc   = ((num_sbs_alloc * 384u + 15u) & ~15u) * 2u;
        const uint32_t internal_size   = (luma_alloc + chroma_alloc + dir_mvs_alloc + 512u + 4095u) & ~4095u;
        if (AllocHwBuf(e->device, internal_size, RkMppBufferUsageScratch,
                       &e->tile_out_internal) != 0)
            return 1;
        /* Per-slot snapshot store — populated post-kick by copying from
         * tile_out_internal so future kicks' refs read from the right slot. */
        for (int i = 0; i < Av1DecodeEngine::kPoolSize; i++) {
            if (AllocHwBuf(e->device, internal_size, RkMppBufferUsageScratch,
                           &e->pool_internal[i]) != 0)
                return 1;
        }
        /* Fallback reference frame; same size as a regular ref. */
        const uint32_t frame_bytes = width * height * 3 / 2;
        if (AllocHwBuf(e->device, frame_bytes, RkMppBufferUsageReferenceFrame,
                       &e->error_ref) != 0)
            return 1;
        /* Per-slot user-visible NV12 (PP module's output).  PP raster
         * output uses the coded (64-aligned) width as its row stride
         * regardless of the display width.  Sizing the buffer at the
         * display width truncates the last few bytes of each row pair,
         * causing PP to either silently corrupt content or fault.    */
        const uint32_t coded_w     = (width  + 63u) & ~63u;
        /* Matches DecodeOne_AV1's coded_h_out alignment — see comment
         * there.  Codec writes 17 SB rows × 64 = 1088 luma rows on a
         * 1080p stream; pool_output must fit the full padded plane. */
        const uint32_t coded_h     = (height + 15u) & ~15u;
        /* Size for the worst case — 10-bit P010 (2 bytes / sample;
         * pp_out_format=1 in regbuilder_av1.cpp, matches upstream
         * kernel rockchip_vpu981_hw_av1_dec).  Same buffer serves
         * 8-bit streams unchanged (2× the bytes — ~6 MiB at 1080p).
         * Per-AU regbuilder picks the actual stride / format from
         * seq->hbd. */
        const uint32_t pp_stride_max = ((coded_w * 16u + 7u) / 8u + 15u) & ~15u;
        const uint32_t output_size   = pp_stride_max * coded_h * 3u / 2u;
        for (int i = 0; i < Av1DecodeEngine::kPoolSize; i++) {
            if (AllocHwBuf(e->device, output_size, RkMppBufferUsageOutputFrame,
                           &e->pool_output[i]) != 0)
                return 1;
        }
    }
    return 0;
}

void Av1DecodeEngine_Shutdown(Av1DecodeEngine *e) {
    if (e->mode == Av1EngineMode::Hardware && e->device != INVALID_HANDLE_VALUE) {
        for (auto &b : e->pool_output)   FreeHwBuf(e->device, &b);
        for (auto &b : e->pool_internal) FreeHwBuf(e->device, &b);
        FreeHwBuf(e->device, &e->error_ref);
        FreeHwBuf(e->device, &e->tile_out_internal);
        FreeHwBuf(e->device, &e->tile_buf);
        FreeHwBuf(e->device, &e->filter_mem);
        FreeHwBuf(e->device, &e->film_grain);
        FreeHwBuf(e->device, &e->prob_tbl_out);
        FreeHwBuf(e->device, &e->prob_tbl);
        FreeHwBuf(e->device, &e->global_model);
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
    /* Stash TU bytes for the HW kick memcpy path.  Per-instance state
     * — was static thread_local globals before the MFT-#14 fix. */
    e->tu_ptr = obu;
    e->tu_len = len;

    /* Walk OBUs: populate picture-producing records AND scan for any
     * OBU_SEQUENCE_HEADER (type 1) in this TU so cached_seq_hdr is
     * up-to-date before DrainPictures parses frame headers. */
    Av1WalkObus(obu, len, e->tu_obus);
    e->tu_obu_idx = 0;

    /* Second linear scan for OBU_SEQUENCE_HEADER (type 1).
     * Av1WalkObus only records picture-producing OBUs (types 3 and 6),
     * so we scan independently here for the sequence header. */
    {
        size_t pos = 0;
        while (pos < len) {
            if (pos + 1 > len) break;
            uint8_t hdr_byte = obu[pos];
            uint8_t obu_type = (hdr_byte >> 3) & 0xf;
            uint8_t ext_flag = (hdr_byte >> 2) & 0x1;
            uint8_t has_size = (hdr_byte >> 1) & 0x1;
            size_t  hdr_len  = 1 + (ext_flag ? 1 : 0);
            if (pos + hdr_len > len) break;
            uint64_t obu_size = 0; size_t size_len = 0;
            if (has_size) {
                for (int i = 0; i < 8; i++) {
                    if (pos + hdr_len + (size_t)i >= len) { obu_size = 0; break; }
                    uint8_t b = obu[pos + hdr_len + i];
                    obu_size |= ((uint64_t)(b & 0x7f)) << (i * 7);
                    size_len++;
                    if (!(b & 0x80)) break;
                }
            } else {
                obu_size = len - pos - hdr_len;
            }
            size_t payload_off = pos + hdr_len + size_len;
            size_t next        = payload_off + obu_size;
            if (next <= pos || next > len) break;

            if (obu_type == 1) { /* OBU_SEQUENCE_HEADER */
                Dav1dSequenceHeader seq{};
                if (Av1ParseSeqHeader(obu + payload_off, (size_t)obu_size, &seq) == 0) {
                    e->cached_seq_hdr = seq;
                    e->seq_hdr_valid  = true;
                    Av1ResolveMaxReorder(e);
                }
            }
            pos = next;
        }
    }

    DrainPictures(e, pts_hns);
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
    /* End-of-stream: empty the reorder buffer in PTS-ascending order so
     * the consumer sees every remaining decoded frame in display order.
     * Mirrors decode_engine.cpp's drain semantics for H.264/HEVC. */
    while (!e->reorder_q.empty()) {
        size_t best = 0;
        for (size_t i = 1; i < e->reorder_q.size(); i++) {
            if (e->reorder_q[i].pts_hns < e->reorder_q[best].pts_hns) best = i;
        }
        e->ready_q.push_back(std::move(e->reorder_q[best]));
        e->reorder_q.erase(e->reorder_q.begin() + best);
    }
}

int Av1DecodeEngine_Flush(Av1DecodeEngine *e) {
    e->reorder_q.clear();
    e->ready_q.clear();
    rkmpp_av1_dpb_init(&e->dpb);
    for (auto &v : e->saved_cdf) v.clear();
    for (auto &s : e->dpb_to_pool) s = -1;
    /* Reset parser saved-states and sequence header validity. */
    for (auto &st : e->saved_states) st = {};
    e->seq_hdr_valid = false;
    return 0;
}

size_t Av1DecodeEngine_QueueDepth(const Av1DecodeEngine *e) {
    /* Account for both the reorder buffer (held back to honour display
     * order on hierarchical GOPs) and the ready queue, so the MFT's
     * input-side backpressure cap reflects total in-flight frames. */
    return e->reorder_q.size() + e->ready_q.size();
}
