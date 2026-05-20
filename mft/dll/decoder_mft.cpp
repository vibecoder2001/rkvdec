/* mft/dll/decoder_mft.cpp — IMFTransform body.
 *
 * Phase 2A: scaffolding + type negotiation only.  Decode loop hooks
 * marked with "Phase 2B:" comments throughout.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "decoder_mft.h"

#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mftransform.h>
#include <d3d11.h>
#include <dxgi.h>

#include <algorithm>
#include <cstring>

/* The decode engine lives as a static lib in mft/engine/
 * so harness binaries and this DLL share one user-mode pipeline.  Codec
 * mapping: rkmpp::CodecKind <-> ::Codec (separate enums, same intent). */
#include "decode_engine.h"
#include "decode_engine_av1.h"
#include "decode_engine_vp9.h"
#include "../av1_parser.h"
#include "../vp9_parser.h"

namespace rkmpp {

static ::Codec ToEngineCodec(CodecKind k) {
    /* AV1 is not yet wired through DecodeEngine — BEGIN_STREAMING
     * short-circuits before reaching this for AV1, but keep the H264
     * fallthrough explicit so future engine work fails loudly rather
     * than silently mis-decoding. */
    return (k == CodecKind::H264) ? ::Codec::H264 : ::Codec::H265;
}

/* Per-call ProcessOutput timing — pairs with the engine-side StageTimes
 * CSV.  Gated on RKMPP_TIMING=1.  Reports total wall-clock, plus the
 * sysmem-path memcpy cost (vector→IMFMediaBuffer) so the user-mode
 * frame budget is fully accounted. */
static bool MftTimingEnabled() {
    static int cached = -1;
    if (cached < 0) {
        char buf[8] = {};
        DWORD n = GetEnvironmentVariableA("RKMPP_TIMING", buf, sizeof(buf));
        cached = (n > 0 && buf[0] != '0') ? 1 : 0;
    }
    return cached != 0;
}
static int64_t MftQpcNow() {
    LARGE_INTEGER c; QueryPerformanceCounter(&c); return c.QuadPart;
}
static int64_t MftQpcUs(int64_t a, int64_t b) {
    static int64_t f = 0;
    if (!f) { LARGE_INTEGER q; QueryPerformanceFrequency(&q); f = q.QuadPart; }
    return (b - a) * 1'000'000LL / f;
}

long g_dll_lock_count = 0;

/* ---------- codec-table helpers ---------------------------------- */

const wchar_t *DecoderFriendlyName(CodecKind k) {
    switch (k) {
    case CodecKind::H264: return L"Rockchip RK3588 H.264 Decoder";
    case CodecKind::HEVC: return L"Rockchip RK3588 HEVC Decoder";
    case CodecKind::AV1:  return L"Rockchip RK3588 AV1 Decoder";
    case CodecKind::VP9:  return L"Rockchip RK3588 VP9 Decoder";
    }
    return L"Rockchip RK3588 Decoder";
}
const GUID &DecoderClsid(CodecKind k) {
    switch (k) {
    case CodecKind::H264: return CLSID_RkmppH264Decoder;
    case CodecKind::HEVC: return CLSID_RkmppHevcDecoder;
    case CodecKind::AV1:  return CLSID_RkmppAv1Decoder;
    case CodecKind::VP9:  return CLSID_RkmppVp9Decoder;
    }
    return CLSID_RkmppH264Decoder;
}
const GUID &DecoderInputSubtype(CodecKind k) {
    switch (k) {
    case CodecKind::H264: return MFVideoFormat_H264;
    case CodecKind::HEVC: return MFVideoFormat_HEVC;
    case CodecKind::AV1:  return MFVideoFormat_AV1;
    case CodecKind::VP9:  return MFVideoFormat_VP90;
    }
    return MFVideoFormat_H264;
}

/* ---------- ctor / dtor ------------------------------------------ */

DecoderMFT::DecoderMFT(CodecKind kind)
    : kind_(kind), refs_(1) {
    DllAddRef();
}

DecoderMFT::~DecoderMFT() {
    /* COM Release-to-zero invariant guarantees no other caller is in any
     * method on this object — destruction happens after the final
     * Release() drops refs to zero, and Release() is called by exactly
     * the last holder.  But a buggy host re-entering during shutdown
     * (or a future async path) would race teardown.  Take lock_
     * defensively so engine/output_type/dxgi state is mutated under
     * the same discipline as the rest of the class.  Review MFT #12. */
    {
        std::lock_guard<std::mutex> g(lock_);
        if (input_type_)  { input_type_->Release();  input_type_  = nullptr; }
        if (output_type_) { output_type_->Release(); output_type_ = nullptr; }
        if (engine_) {
            auto *eng = static_cast<DecodeEngine *>(engine_);
            DecodeEngine_Shutdown(eng);
            delete eng;
            engine_ = nullptr;
        }
        if (engine_av1_) {
            auto *eng = static_cast<Av1DecodeEngine *>(engine_av1_);
            Av1DecodeEngine_Shutdown(eng);
            delete eng;
            engine_av1_ = nullptr;
        }
        if (engine_vp9_) {
            auto *eng = static_cast<Vp9DecodeEngine *>(engine_vp9_);
            Vp9DecodeEngine_Shutdown(eng);
            delete eng;
            engine_vp9_ = nullptr;
        }
        ReleaseD3DManager();
        if (dump_file_) { std::fclose(dump_file_); dump_file_ = nullptr; }
        if (attributes_) { attributes_->Release(); attributes_ = nullptr; }
    }
    DllRelease();
}

void DecoderMFT::ReleaseD3DManager() {
    /* Drop the D3D11 device+context handles first so any subsequent
     * GetVideoService cycle re-acquires fresh ones bound to whatever
     * device the manager currently advertises. */
    if (d3d_context_) { d3d_context_->Release(); d3d_context_ = nullptr; }
    if (d3d_device_)  { d3d_device_->Release();  d3d_device_  = nullptr; }
    if (dxgi_manager_ && dxgi_device_h_) {
        /* CloseDeviceHandle can return MF_E_DXGI_NEW_VIDEO_DEVICE when
         * EVR has reset its DXGI device since the handle was opened.
         * The MF spec says the manager has already invalidated our
         * handle in that case — we still want to NULL our copy so we
         * don't pass a stale value to a future Open / GetVideoService.
         * Ignore the return (informational only); the manager's
         * Release below tears down what's left.  Review MFT #6. */
        (void)dxgi_manager_->CloseDeviceHandle(dxgi_device_h_);
        dxgi_device_h_ = nullptr;
    }
    if (dxgi_manager_) { dxgi_manager_->Release(); dxgi_manager_ = nullptr; }
}

HRESULT DecoderMFT::EnsureAttributes() {
    if (attributes_) return S_OK;
    HRESULT hr = MFCreateAttributes(&attributes_, 4);
    if (FAILED(hr)) return hr;
    /* Advertise as a D3D11-aware decoder so MF hosts route us through
     * the DXGI device manager handshake.  Bind flags match what the EVR
     * and the standard MF NV12 sample allocator expect. */
    (void)attributes_->SetUINT32(MF_SA_D3D11_AWARE, TRUE);
    (void)attributes_->SetUINT32(MF_SA_D3D11_BINDFLAGS,
                                 D3D11_BIND_DECODER | D3D11_BIND_SHADER_RESOURCE);
    /* Hint: we are an in-place transform with sync output. */
    (void)attributes_->SetUINT32(MF_TRANSFORM_ASYNC, FALSE);
    return S_OK;
}

/* ---------- IUnknown --------------------------------------------- */

STDMETHODIMP DecoderMFT::QueryInterface(REFIID iid, void **ppv) {
    if (!ppv) return E_POINTER;
    if (iid == IID_IUnknown || iid == IID_IMFTransform) {
        *ppv = static_cast<IMFTransform*>(this);
        AddRef();
        return S_OK;
    }
    if (iid == IID_IMFQualityAdvise) {
        *ppv = static_cast<IMFQualityAdvise*>(this);
        AddRef();
        return S_OK;
    }
    *ppv = nullptr;
    return E_NOINTERFACE;
}
STDMETHODIMP_(ULONG) DecoderMFT::AddRef()  { return InterlockedIncrement(&refs_); }
STDMETHODIMP_(ULONG) DecoderMFT::Release() {
    long r = InterlockedDecrement(&refs_);
    if (r == 0) delete this;
    return r;
}

/* ---------- stream layout ---------------------------------------- */

STDMETHODIMP DecoderMFT::GetStreamLimits(DWORD *pdwMin_in, DWORD *pdwMax_in,
                                         DWORD *pdwMin_out, DWORD *pdwMax_out) {
    if (!pdwMin_in || !pdwMax_in || !pdwMin_out || !pdwMax_out) return E_POINTER;
    *pdwMin_in = *pdwMax_in = *pdwMin_out = *pdwMax_out = 1;
    return S_OK;
}
STDMETHODIMP DecoderMFT::GetStreamCount(DWORD *pcIn, DWORD *pcOut) {
    if (!pcIn || !pcOut) return E_POINTER;
    *pcIn = 1; *pcOut = 1; return S_OK;
}
STDMETHODIMP DecoderMFT::GetStreamIDs(DWORD, DWORD *, DWORD, DWORD *) {
    /* Default 0/0 IDs — MFT framework synthesizes them. */
    return E_NOTIMPL;
}

STDMETHODIMP DecoderMFT::GetInputStreamInfo(DWORD id, MFT_INPUT_STREAM_INFO *p) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!p) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    p->hnsMaxLatency       = 0;
    p->dwFlags             = MFT_INPUT_STREAM_WHOLE_SAMPLES
                           | MFT_INPUT_STREAM_SINGLE_SAMPLE_PER_BUFFER
                           | MFT_INPUT_STREAM_FIXED_SAMPLE_SIZE
                           | MFT_INPUT_STREAM_HOLDS_BUFFERS;
    p->cbSize              = 1; /* nominal; decoders accept variable */
    p->cbMaxLookahead      = 0;
    p->cbAlignment         = 0;
    return S_OK;
}
STDMETHODIMP DecoderMFT::GetOutputStreamInfo(DWORD id, MFT_OUTPUT_STREAM_INFO *p) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!p) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    p->dwFlags = MFT_OUTPUT_STREAM_PROVIDES_SAMPLES
               | MFT_OUTPUT_STREAM_FIXED_SAMPLE_SIZE
               | MFT_OUTPUT_STREAM_WHOLE_SAMPLES
               | MFT_OUTPUT_STREAM_SINGLE_SAMPLE_PER_BUFFER;
    /* NV12 = width*height*3/2; P010 doubles that (2 bytes/sample). */
    if (width_ && height_) {
        UINT32 base = width_ * height_ * 3u / 2u;
        p->cbSize = (bit_depth_ == 10) ? (base * 2u) : base;
    } else {
        p->cbSize = 0;
    }
    p->cbAlignment = 16;
    return S_OK;
}

/* ---------- attributes / unsupported ------------------------------ */

STDMETHODIMP DecoderMFT::GetAttributes(IMFAttributes **pp) {
    if (!pp) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    HRESULT hr = EnsureAttributes();
    if (FAILED(hr)) return hr;
    *pp = attributes_;
    attributes_->AddRef();
    return S_OK;
}
STDMETHODIMP DecoderMFT::GetInputStreamAttributes(DWORD, IMFAttributes **)  { return E_NOTIMPL; }
STDMETHODIMP DecoderMFT::GetOutputStreamAttributes(DWORD, IMFAttributes **) { return E_NOTIMPL; }
STDMETHODIMP DecoderMFT::DeleteInputStream(DWORD)                      { return E_NOTIMPL; }
STDMETHODIMP DecoderMFT::AddInputStreams(DWORD, DWORD *)               { return E_NOTIMPL; }

STDMETHODIMP DecoderMFT::GetInputStatus(DWORD id, DWORD *flags) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!flags) return E_POINTER;
    /* Intentionally lock-free.  MF's quality manager polls
     * GetInputStatus / GetOutputStatus from worker threads under
     * sustained load; holding `lock_` here would stall those polls
     * for the entire duration of any in-flight ProcessInput, which
     * can block up to ~1 s on the kernel WAIT_JOB IOCTL.  EVR
     * (synchronous host) would then deadlock its topology rebuild.
     *
     * Engine pointers are stable for the MFT's lifetime — assigned
     * once in BEGIN_STREAMING, freed in ~DecoderMFT.  Since this
     * method is on the call stack while we read them, COM's
     * Release-to-zero invariant guarantees no concurrent destructor.
     * QueueDepth itself reads `reorder_q.size() + ready_q.size()`
     * without locks — a slightly-stale value here is acceptable for
     * an advisory "can I accept more?" signal. */
    Av1DecodeEngine *av1 = static_cast<Av1DecodeEngine *>(engine_av1_);
    DecodeEngine    *gen = static_cast<DecodeEngine *>(engine_);
    *flags = 0;
    if (av1) {
        if (Av1DecodeEngine_QueueDepth(av1) < 24)
            *flags = MFT_INPUT_STATUS_ACCEPT_DATA;
    } else if (gen) {
        if (DecodeEngine_QueueDepth(gen) < DecodeEngine_InputQueueCapacity(gen))
            *flags = MFT_INPUT_STATUS_ACCEPT_DATA;
    } else {
        *flags = MFT_INPUT_STATUS_ACCEPT_DATA;
    }
    return S_OK;
}
STDMETHODIMP DecoderMFT::GetOutputStatus(DWORD *flags) {
    if (!flags) return E_POINTER;
    /* Lock-free for the same reason as GetInputStatus; see comment
     * there.  Polling these from MF's worker pool must not stall on
     * a ProcessInput holding `lock_` across the kernel WAIT_JOB. */
    Av1DecodeEngine *av1 = static_cast<Av1DecodeEngine *>(engine_av1_);
    DecodeEngine    *gen = static_cast<DecodeEngine *>(engine_);
    *flags = 0;
    if (av1) {
        if (Av1DecodeEngine_QueueDepth(av1) > 0)
            *flags = MFT_OUTPUT_STATUS_SAMPLE_READY;
    } else if (gen) {
        if (DecodeEngine_QueueDepth(gen) > 0)
            *flags = MFT_OUTPUT_STATUS_SAMPLE_READY;
    }
    return S_OK;
}
STDMETHODIMP DecoderMFT::SetOutputBounds(LONGLONG, LONGLONG) { return E_NOTIMPL; }
STDMETHODIMP DecoderMFT::ProcessEvent(DWORD, IMFMediaEvent *) { return E_NOTIMPL; }

/* ---------- type negotiation ------------------------------------- */

STDMETHODIMP DecoderMFT::GetInputAvailableType(DWORD id, DWORD idx, IMFMediaType **pp) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!pp) return E_POINTER;
    if (idx > 0) return MF_E_NO_MORE_TYPES;

    IMFMediaType *t = nullptr;
    HRESULT hr = MFCreateMediaType(&t);
    if (FAILED(hr)) return hr;
    hr = t->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    if (SUCCEEDED(hr)) hr = t->SetGUID(MF_MT_SUBTYPE, DecoderInputSubtype(kind_));
    if (FAILED(hr)) { t->Release(); return hr; }
    *pp = t;
    return S_OK;
}

STDMETHODIMP DecoderMFT::SetInputType(DWORD id, IMFMediaType *type, DWORD flags) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    std::lock_guard<std::mutex> g(lock_);

    /* Allow clearing. */
    if (!type) {
        if (flags & MFT_SET_TYPE_TEST_ONLY) return S_OK;
        if (input_type_) { input_type_->Release(); input_type_ = nullptr; }
        return S_OK;
    }

    GUID major = {}, sub = {};
    if (FAILED(type->GetGUID(MF_MT_MAJOR_TYPE, &major)) || major != MFMediaType_Video)
        return MF_E_INVALIDMEDIATYPE;
    if (FAILED(type->GetGUID(MF_MT_SUBTYPE, &sub)) || sub != DecoderInputSubtype(kind_))
        return MF_E_INVALIDMEDIATYPE;

    UINT32 w = 0, h = 0;
    if (FAILED(MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h)) || !w || !h)
        return MF_E_INVALIDMEDIATYPE;
    /* Bound dimensions before they flow into allocation math.  The
     * rkvdec hardware accepts up to 8192x8192 (well, less in practice
     * but 8K-wide is the silicon ceiling); reject anything larger so
     * `width_ * height_ * 3 / 2 * bytes_per_sample` (decoder_mft.cpp
     * ~line 606) cannot overflow uint32 with attacker-influenced
     * MF_MT_FRAME_SIZE values.  Also reject odd dimensions — D3D11
     * NV12/P010 textures require even width and height; without this
     * check, an odd-height type passes here and the UpdateSubresource
     * upload writes past the texture's UV plane (height/2 rows).
     * See [[critical_mft_dim_overflow]]. */
    constexpr UINT32 kMaxDim = 8192u;
    if (w > kMaxDim || h > kMaxDim) return MF_E_INVALIDMEDIATYPE;
    if ((w | h) & 1u)               return MF_E_INVALIDMEDIATYPE;

    UINT32 fn = 30, fd = 1;
    /* Framerate is informational; don't fail if absent. */
    (void)MFGetAttributeRatio(type, MF_MT_FRAME_RATE, &fn, &fd);

    /* MPEG sequence header (avcC / hvcC). Optional — some pipelines
     * pass it inline in the first IMFSample instead. */
    std::vector<uint8_t> annexb;
    /* Default to Annex-B framing (length_size = 0).  Only flip to AVCC
     * when MF_MT_MPEG_SEQUENCE_HEADER parses successfully and gives us
     * a length-size hint via avcC/hvcC.  MF sources that deliver
     * Annex-B inline (no extradata) must NOT be treated as AVCC, or
     * AvccToAnnexB will reject every sample as malformed. */
    uint8_t length_size = 0;
    /* Probe both MF_MT_MPEG_SEQUENCE_HEADER (MP4/AVI sources, and also
     * MKV — which surprisingly puts raw Annex-B SPS+PPS here, not avcC)
     * and MF_MT_USER_DATA (some sources stash codec data here instead).
     * Heuristic: if the blob starts with an Annex-B start code, treat
     * it as Annex-B verbatim; otherwise try avcC/hvcC parsing.  Either
     * way the result lands in `annexb` for FeedExtradata + bit-depth
     * detection below.  Containers that ship no extradata at all
     * (MPEG-TS, raw Annex-B sources without a header attribute) fall
     * through to inline-SPS detection in ProcessInput. */
    auto looks_annexb = [](const uint8_t *p, size_t n) {
        if (n >= 4 && p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 1) return true;
        if (n >= 3 && p[0] == 0 && p[1] == 0 && p[2] == 1) return true;
        return false;
    };
    UINT32 hdr_len = 0;
    const GUID kHeaderKeys[2] = { MF_MT_MPEG_SEQUENCE_HEADER, MF_MT_USER_DATA };
    for (const GUID &key : kHeaderKeys) {
        if (!annexb.empty()) break;
        if (FAILED(type->GetBlobSize(key, &hdr_len)) || hdr_len == 0) continue;
        std::vector<uint8_t> hdr(hdr_len);
        if (FAILED(type->GetBlob(key, hdr.data(), hdr_len, nullptr))) continue;

        if (looks_annexb(hdr.data(), hdr.size())) {
            /* Raw Annex-B SPS+PPS — already in the format the engine
             * expects.  Use verbatim; length_size stays 0 (Annex-B
             * framing for downstream samples too unless a later
             * SetInputType call corrects it). */
            std::fprintf(stderr,
                "rkmpp MFT: SetInputType extradata = Annex-B "
                "(%zu bytes)\n", hdr.size());
            std::fflush(stderr);
            annexb      = std::move(hdr);
            length_size = 0;
            continue;
        }

        /* avcC / hvcC path.  Parse into a temporary instance so we can
         * roll back on failure.  AV1 extradata is `av1C` (OBU sequence
         * header) — dav1d will pick up the SPS from the OBU stream
         * itself, so just skip the parse and leave annexb empty. */
        DecoderMFT tmp(kind_);
        HRESULT hr = S_OK;
        if (kind_ == CodecKind::H264) {
            hr = tmp.ParseAvcCExtradata(hdr.data(), hdr.size());
        } else if (kind_ == CodecKind::HEVC) {
            hr = tmp.ParseHvcCExtradata(hdr.data(), hdr.size());
        } else {
            continue;
        }
        if (SUCCEEDED(hr)) {
            annexb      = std::move(tmp.extradata_annexb_);
            length_size = tmp.length_size_;
        }
        /* Failure on this key is silently ignored — try the next, or
         * fall through to inline-SPS handling. */
    }

    if (flags & MFT_SET_TYPE_TEST_ONLY) return S_OK;

    if (input_type_) input_type_->Release();
    type->AddRef();
    input_type_       = type;
    width_            = w;
    height_           = h;
    fps_num_          = fn;
    fps_den_          = fd;
    extradata_annexb_ = std::move(annexb);
    length_size_      = length_size;

    /* Resolve colour metadata for the output type.  Order:
     *   1. Upstream demuxer attributes on the input type (highest signal).
     *   2. H.264 VUI in the avcC SPS (now parsed in extradata).
     *   3. Resolution heuristic: HD (>=720) → BT.709, SD → BT.601.
     * Range defaults to limited (16..235); H.264 VUI overrides if it
     * carries video_full_range_flag. */
    yuv_matrix_         = 0;
    video_primaries_    = 0;
    transfer_function_  = 0;
    nominal_range_      = 0;
    bit_depth_          = 8;
    (void)type->GetUINT32(MF_MT_YUV_MATRIX,         &yuv_matrix_);
    (void)type->GetUINT32(MF_MT_VIDEO_PRIMARIES,    &video_primaries_);
    (void)type->GetUINT32(MF_MT_TRANSFER_FUNCTION,  &transfer_function_);
    (void)type->GetUINT32(MF_MT_VIDEO_NOMINAL_RANGE,&nominal_range_);

    if (kind_ == CodecKind::AV1) {
        /* Pick bit-depth before the first sample so mft_play's
         * GetOutputAvailableType probe right after SetInputType lands
         * on P010 (a host that picks NV12 there locks the whole
         * downstream chain to 8-bit even if the in-band seq_hdr later
         * has hbd=1).  The engine itself still consumes seq_hdr from
         * the OBU stream — this is detection only.
         *
         * Two delivery formats observed in the wild for AV1:
         *   (a) av1C AV1CodecConfigurationRecord (ISO 14496-31 §2.2.1):
         *         byte 0: marker(1)=1 | version(7)=1   (≥ 0x80)
         *         byte 1: seq_profile(3) | seq_level_idx_0(5)
         *         byte 2: seq_tier_0(1) | high_bitdepth(1) | twelve_bit(1) | ...
         *   (b) raw OBU sequence_header (some MF MKV / DMF sources):
         *         byte 0: OBU header — obu_type(4)=1 in bits 6:3
         *                 → byte0 & 0x78 == 0x08
         *         then a leb128 size, then the seq_hdr OBU payload.
         * Probe several keys; pick the first plausible match. */
        const GUID kKeys[2] = {
            MF_MT_MPEG_SEQUENCE_HEADER,
            MF_MT_USER_DATA,
        };
        for (const GUID &key : kKeys) {
            UINT32 av1c_len = 0;
            if (FAILED(type->GetBlobSize(key, &av1c_len)) || av1c_len < 3) continue;
            std::vector<uint8_t> blob(av1c_len);
            if (FAILED(type->GetBlob(key, blob.data(), av1c_len, nullptr))) continue;

            std::fprintf(stderr,
                "rkmpp MFT: AV1 extradata len=%u, first 8:", av1c_len);
            for (UINT32 i = 0; i < (av1c_len < 8 ? av1c_len : 8u); i++)
                std::fprintf(stderr, " %02x", blob[i]);
            std::fprintf(stderr, "\n");
            std::fflush(stderr);

            /* Form (a): av1C box body. */
            if ((blob[0] & 0x80) /* marker */ && (blob[0] & 0x7F) == 1 /* version */) {
                bool high_bitdepth = (blob[2] & 0x40) != 0;
                bool twelve_bit    = (blob[2] & 0x20) != 0;
                if (high_bitdepth && !twelve_bit) bit_depth_ = 10;
                std::fprintf(stderr,
                    "rkmpp MFT: av1C parse → high_bitdepth=%d twelve_bit=%d bit_depth=%u\n",
                    high_bitdepth, twelve_bit, bit_depth_);
                std::fflush(stderr);
                break;
            }

            /* Form (b): raw OBU sequence_header.  Skip OBU header byte
             * + optional extension byte + leb128 size, then call the
             * existing parser.  The extension byte (obu_extension_flag,
             * bit 2 of the header) is rare but legal — without
             * accounting for it the leb128 offset is wrong and we
             * misparse bit-depth, locking NV12 on a 10-bit stream.
             * Review issue MFT #13. */
            uint8_t obu_type = (uint8_t)((blob[0] >> 3) & 0xF);
            bool    has_ext  = (blob[0] & 0x04) != 0;
            bool    has_size = (blob[0] & 0x02) != 0;
            if (obu_type == 1 /* OBU_SEQUENCE_HEADER */) {
                size_t off = 1;
                if (has_ext && off < blob.size()) off++;
                if (has_size) {
                    /* leb128 — at most 8 bytes; the seq_hdr is short. */
                    size_t leb = 0;
                    while (off < blob.size() && leb < 8) {
                        bool more = (blob[off] & 0x80) != 0;
                        off++; leb++;
                        if (!more) break;
                    }
                }
                if (off < blob.size()) {
                    Dav1dSequenceHeader seq{};
                    int prc = Av1ParseSeqHeader(blob.data() + off,
                                                blob.size() - off, &seq);
                    if (prc == 0 && seq.hbd) bit_depth_ = 10;
                    std::fprintf(stderr,
                        "rkmpp MFT: raw seq_hdr parse rc=%d hbd=%d bit_depth=%u\n",
                        prc, seq.hbd, bit_depth_);
                    std::fflush(stderr);
                }
                break;
            }
        }
    }

    if (kind_ == CodecKind::HEVC && !extradata_annexb_.empty()) {
        /* Parse the hvcC SPS NAL out of extradata to extract bit-depth.
         * Drives NV12-vs-P010 output choice in BuildOutputType for HEVC
         * Main10 streams.  Use a temporary parse result so we don't
         * disturb engine state; H265ParseResultInit zero-initialises. */
        H265ParseResult tmp{};
        H265ParseResultInit(&tmp);
        std::vector<uint8_t> scratch(extradata_annexb_.size() * 2 + 64);
        H265ParseStatus s = H265ParseAccessUnit(
            extradata_annexb_.data(), extradata_annexb_.size(),
            scratch.data(), scratch.size(), &tmp);
        if ((s == H265_PARSE_OK || s == H265_PARSE_NEED_MORE) &&
            tmp.active_sps_id >= 0 &&
            tmp.sps[tmp.active_sps_id].valid &&
            tmp.sps[tmp.active_sps_id].bit_depth_luma_minus8 == 2) {
            bit_depth_ = 10;
        }
    }

    if (kind_ == CodecKind::H264 && !extradata_annexb_.empty()) {
        H264ParseResult vui{};
        std::vector<uint8_t> scratch(extradata_annexb_.size() * 2 + 64);
        H264ParseStatus s = H264ParseAccessUnit(
            extradata_annexb_.data(), extradata_annexb_.size(),
            scratch.data(), scratch.size(), &vui);
        if ((s == H264_PARSE_OK || s == H264_PARSE_NEED_MORE) && vui.has_sps) {
            /* Bit-depth from the extradata SPS — drives NV12-vs-P010
             * output-type choice in BuildOutputType.  High-profile
             * streams set bit_depth_luma_minus8 (== 2 for 10-bit); for
             * baseline/main streams the parser pins it to 0 (8-bit). */
            if (vui.sps.bit_depth_luma_minus8 == 2)
                bit_depth_ = 10;
            if (vui.has_vui_colour) {
                if (nominal_range_ == 0)
                    nominal_range_ = vui.vui_full_range_flag
                                   ? MFNominalRange_0_255
                                   : MFNominalRange_16_235;
                if (vui.has_vui_colour_desc) {
                    /* H.264 Table E-3 colour_primaries → MFVideoPrimaries */
                    if (video_primaries_ == 0) {
                        switch (vui.vui_colour_primaries) {
                        case 1:  video_primaries_ = MFVideoPrimaries_BT709;       break;
                        case 4:  video_primaries_ = MFVideoPrimaries_BT470_2_SysM;break;
                        case 5:  video_primaries_ = MFVideoPrimaries_BT470_2_SysBG;break;
                        case 6:  video_primaries_ = MFVideoPrimaries_SMPTE170M;   break;
                        case 7:  video_primaries_ = MFVideoPrimaries_SMPTE240M;   break;
                        case 9:  video_primaries_ = MFVideoPrimaries_BT2020;      break;
                        default: break;
                        }
                    }
                    /* Table E-4 transfer_characteristics → MFVideoTransFunc */
                    if (transfer_function_ == 0) {
                        switch (vui.vui_transfer_characteristics) {
                        case 1:  case 6:
                                 transfer_function_ = MFVideoTransFunc_709;   break;
                        case 4:  transfer_function_ = MFVideoTransFunc_22;    break;
                        case 5:  transfer_function_ = MFVideoTransFunc_28;    break;
                        case 7:  transfer_function_ = MFVideoTransFunc_240M;  break;
                        case 8:  transfer_function_ = MFVideoTransFunc_10;    break;
                        case 11: transfer_function_ = MFVideoTransFunc_sRGB;  break;
                        case 13: transfer_function_ = MFVideoTransFunc_sRGB;  break;
                        case 16: transfer_function_ = MFVideoTransFunc_2084;  break;
                        default: break;
                        }
                    }
                    /* Table E-5 matrix_coefficients → MFVideoTransferMatrix */
                    if (yuv_matrix_ == 0) {
                        switch (vui.vui_matrix_coefficients) {
                        case 1:  yuv_matrix_ = MFVideoTransferMatrix_BT709;    break;
                        case 6:  yuv_matrix_ = MFVideoTransferMatrix_BT601;    break;
                        case 7:  yuv_matrix_ = MFVideoTransferMatrix_SMPTE240M;break;
                        case 9:  yuv_matrix_ = MFVideoTransferMatrix_BT2020_10;break;
                        default: break;
                        }
                    }
                }
            }
        }
    }

    /* Resolution heuristic backstop. */
    if (yuv_matrix_ == 0)
        yuv_matrix_ = (height_ >= 720) ? MFVideoTransferMatrix_BT709
                                       : MFVideoTransferMatrix_BT601;
    if (video_primaries_ == 0)
        video_primaries_ = (height_ >= 720) ? MFVideoPrimaries_BT709
                                            : MFVideoPrimaries_SMPTE170M;
    if (transfer_function_ == 0)
        transfer_function_ = (height_ >= 720) ? MFVideoTransFunc_709
                                              : MFVideoTransFunc_22;
    if (nominal_range_ == 0)
        nominal_range_ = MFNominalRange_16_235;

    std::fprintf(stderr,
        "rkmpp MFT: SetInputType end — bit_depth=%u, extradata_annexb=%zu bytes\n",
        bit_depth_, extradata_annexb_.size());
    std::fflush(stderr);

    return S_OK;
}

HRESULT DecoderMFT::BuildOutputType(IMFMediaType **pp) {
    IMFMediaType *t = nullptr;
    HRESULT hr = MFCreateMediaType(&t);
    if (FAILED(hr)) return hr;
    /* 10-bit streams → P010 (2 bytes/sample, 10 valid bits in upper 10
     * of 16); 8-bit → NV12.  Sample size + default stride track the
     * subtype: P010 doubles both vs NV12.  Engine emits one or the
     * other based on the parsed SPS bit-depth on every frame. */
    const bool is_p010 = (bit_depth_ == 10);
    const GUID &subtype = is_p010 ? MFVideoFormat_P010 : MFVideoFormat_NV12;
    const UINT32 bytes_per_sample = is_p010 ? 2u : 1u;
    const UINT32 sample_size = width_ * height_ * 3u / 2u * bytes_per_sample;
    const UINT32 default_stride = width_ * bytes_per_sample;
    hr = t->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    if (SUCCEEDED(hr)) hr = t->SetGUID(MF_MT_SUBTYPE, subtype);
    if (SUCCEEDED(hr)) hr = MFSetAttributeSize(t, MF_MT_FRAME_SIZE, width_, height_);
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(t, MF_MT_FRAME_RATE, fps_num_, fps_den_);
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(t, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    /* Without DEFAULT_STRIDE/SAMPLE_SIZE/etc EVR computes a default for
     * NV12 that doesn't match our packed layout (width-aligned, no
     * padding) and ends up reading UV from the wrong offsets — UV reads
     * as zeros from the trailing portion of the buffer, producing green
     * frames with partial Y bleed. Pin all the layout attributes
     * EVR/sink stack expects.  P010 uses 2-byte samples but is otherwise
     * laid out identically. */
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_DEFAULT_STRIDE, default_stride);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_SAMPLE_SIZE, sample_size);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    if (SUCCEEDED(hr) && nominal_range_)
        hr = t->SetUINT32(MF_MT_VIDEO_NOMINAL_RANGE,    nominal_range_);
    if (SUCCEEDED(hr) && yuv_matrix_)
        hr = t->SetUINT32(MF_MT_YUV_MATRIX,             yuv_matrix_);
    if (SUCCEEDED(hr) && video_primaries_)
        hr = t->SetUINT32(MF_MT_VIDEO_PRIMARIES,        video_primaries_);
    if (SUCCEEDED(hr) && transfer_function_)
        hr = t->SetUINT32(MF_MT_TRANSFER_FUNCTION,      transfer_function_);
    if (FAILED(hr)) { t->Release(); return hr; }
    *pp = t;
    return S_OK;
}

STDMETHODIMP DecoderMFT::GetOutputAvailableType(DWORD id, DWORD idx, IMFMediaType **pp) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!pp) return E_POINTER;
    if (idx > 0) return MF_E_NO_MORE_TYPES;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    return BuildOutputType(pp);
}

STDMETHODIMP DecoderMFT::SetOutputType(DWORD id, IMFMediaType *type, DWORD flags) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    if (!type) {
        if (flags & MFT_SET_TYPE_TEST_ONLY) return S_OK;
        if (output_type_) { output_type_->Release(); output_type_ = nullptr; }
        return S_OK;
    }
    GUID major = {}, sub = {};
    if (FAILED(type->GetGUID(MF_MT_MAJOR_TYPE, &major)) || major != MFMediaType_Video)
        return MF_E_INVALIDMEDIATYPE;
    if (FAILED(type->GetGUID(MF_MT_SUBTYPE, &sub)))
        return MF_E_INVALIDMEDIATYPE;
    /* Accept NV12 for 8-bit streams, P010 for 10-bit.  Reject the
     * cross-product so consumers can't pick NV12 on a 10-bit stream
     * (which would mean reading NV15 stride from a uint8 buffer
     * sized for NV12 — guaranteed corruption). */
    const bool is_p010 = (bit_depth_ == 10);
    const GUID &expected = is_p010 ? MFVideoFormat_P010 : MFVideoFormat_NV12;
    if (sub != expected)
        return MF_E_INVALIDMEDIATYPE;
    UINT32 w = 0, h = 0;
    if (FAILED(MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &w, &h)) ||
        w != width_ || h != height_)
        return MF_E_INVALIDMEDIATYPE;

    if (flags & MFT_SET_TYPE_TEST_ONLY) return S_OK;
    if (output_type_) output_type_->Release();
    type->AddRef();
    output_type_ = type;
    return S_OK;
}

STDMETHODIMP DecoderMFT::GetInputCurrentType(DWORD id, IMFMediaType **pp) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!pp) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    input_type_->AddRef();
    *pp = input_type_;
    return S_OK;
}
STDMETHODIMP DecoderMFT::GetOutputCurrentType(DWORD id, IMFMediaType **pp) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!pp) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    if (!output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    output_type_->AddRef();
    *pp = output_type_;
    return S_OK;
}

/* ---------- avcC / hvcC parsing ---------------------------------- */

static void appendStartCode(std::vector<uint8_t> &dst) {
    static const uint8_t sc[4] = { 0x00, 0x00, 0x00, 0x01 };
    dst.insert(dst.end(), sc, sc + 4);
}

HRESULT DecoderMFT::ParseAvcCExtradata(const uint8_t *blob, size_t len) {
    /* avcC layout (ISO/IEC 14496-15 §5.3.3.1.2):
     *   u8  configurationVersion (=1)
     *   u8  AVCProfileIndication
     *   u8  profile_compatibility
     *   u8  AVCLevelIndication
     *   u8  reserved(6)|lengthSizeMinusOne(2)
     *   u8  reserved(3)|numOfSequenceParameterSets(5)
     *   { u16 BE spsLen; u8 sps[spsLen]; } * count
     *   u8  numOfPictureParameterSets
     *   { u16 BE ppsLen; u8 pps[ppsLen]; } * count
     */
    if (len < 7) return E_INVALIDARG;
    if (blob[0] != 1) return E_INVALIDARG;
    length_size_ = static_cast<uint8_t>((blob[4] & 0x03) + 1);

    size_t off = 5;
    uint8_t sps_count = blob[off++] & 0x1f;
    extradata_annexb_.clear();
    for (uint8_t i = 0; i < sps_count; ++i) {
        if (off + 2 > len) return E_INVALIDARG;
        uint16_t nal_len = (uint16_t)((blob[off] << 8) | blob[off + 1]);
        off += 2;
        if (off + nal_len > len) return E_INVALIDARG;
        appendStartCode(extradata_annexb_);
        extradata_annexb_.insert(extradata_annexb_.end(),
                                 blob + off, blob + off + nal_len);
        off += nal_len;
    }
    if (off >= len) return S_OK; /* no PPS section: tolerate */
    uint8_t pps_count = blob[off++];
    for (uint8_t i = 0; i < pps_count; ++i) {
        if (off + 2 > len) return E_INVALIDARG;
        uint16_t nal_len = (uint16_t)((blob[off] << 8) | blob[off + 1]);
        off += 2;
        if (off + nal_len > len) return E_INVALIDARG;
        appendStartCode(extradata_annexb_);
        extradata_annexb_.insert(extradata_annexb_.end(),
                                 blob + off, blob + off + nal_len);
        off += nal_len;
    }
    return S_OK;
}

HRESULT DecoderMFT::ParseHvcCExtradata(const uint8_t *blob, size_t len) {
    /* hvcC layout (ISO/IEC 14496-15 §8.3.3.1.2): 22-byte header followed
     * by numOfArrays, then arrays of NAL units (typically VPS, SPS, PPS).
     *   u8  configurationVersion (=1)
     *   ... 21 bytes of profile/level/etc ...
     *   u8  numOfArrays
     *   for each array:
     *     u8  array_completeness(1)|reserved(1)|NAL_unit_type(6)
     *     u16 numNalus
     *     { u16 nalUnitLength; u8 nal[nalUnitLength]; } * numNalus
     *
     * lengthSizeMinusOne sits at offset 21, low 2 bits.
     */
    if (len < 23) return E_INVALIDARG;
    if (blob[0] != 1) return E_INVALIDARG;
    length_size_ = static_cast<uint8_t>((blob[21] & 0x03) + 1);
    uint8_t num_arrays = blob[22];

    size_t off = 23;
    extradata_annexb_.clear();
    for (uint8_t a = 0; a < num_arrays; ++a) {
        if (off + 3 > len) return E_INVALIDARG;
        off += 1; /* skip array_completeness/NAL_unit_type byte */
        uint16_t num_nalus = (uint16_t)((blob[off] << 8) | blob[off + 1]);
        off += 2;
        for (uint16_t n = 0; n < num_nalus; ++n) {
            if (off + 2 > len) return E_INVALIDARG;
            uint16_t nal_len = (uint16_t)((blob[off] << 8) | blob[off + 1]);
            off += 2;
            if (off + nal_len > len) return E_INVALIDARG;
            appendStartCode(extradata_annexb_);
            extradata_annexb_.insert(extradata_annexb_.end(),
                                     blob + off, blob + off + nal_len);
            off += nal_len;
        }
    }
    return S_OK;
}

/* ---------- ProcessMessage / Input / Output ---------------------- */

STDMETHODIMP DecoderMFT::ProcessMessage(MFT_MESSAGE_TYPE msg, ULONG_PTR param) {
    std::lock_guard<std::mutex> g(lock_);
    switch (msg) {
    case MFT_MESSAGE_SET_D3D_MANAGER: {
        ReleaseD3DManager();
        if (param == 0) return S_OK;
        IUnknown *unk = reinterpret_cast<IUnknown *>(param);
        IMFDXGIDeviceManager *mgr = nullptr;
        HRESULT hr = unk->QueryInterface(IID_PPV_ARGS(&mgr));
        if (FAILED(hr)) {
            std::fprintf(stderr,
                "rkmpp MFT: SET_D3D_MANAGER QI(IMFDXGIDeviceManager) "
                "failed 0x%08x — staying on sysmem\n", (unsigned)hr);
            std::fflush(stderr);
            return hr;
        }
        HANDLE h = nullptr;
        hr = mgr->OpenDeviceHandle(&h);
        if (FAILED(hr)) { mgr->Release(); return hr; }

        /* Commit the manager state up front — even if D3D11 device
         * acquisition below fails (because EVR's default presenter
         * wraps a D3D9 device), the IMFDXGIDeviceManager is still
         * valid and signals "EVR is the consumer", which we use to
         * pick the IMF2DBuffer output path that matches EVR's
         * Lock2D-based read.  Without this commit, a D3D9-only EVR
         * would leave dxgi_manager_ null and we'd fall through to
         * the SYSMEM_1D_BUFFER path that EVR has to stride-convert
         * and upload at extra cost. */
        dxgi_manager_  = mgr;
        dxgi_device_h_ = h;

        ID3D11Device *dev = nullptr;
        hr = mgr->GetVideoService(h, IID_PPV_ARGS(&dev));
        if (FAILED(hr)) {
            std::fprintf(stderr,
                "rkmpp MFT: SET_D3D_MANAGER GetVideoService(ID3D11Device) "
                "returned 0x%08x — EVR is on D3D9, using 2D media buffer\n",
                (unsigned)hr);
            std::fflush(stderr);
            return S_OK;  /* manager kept, just no D3D11 device */
        }
        ID3D11DeviceContext *ctx = nullptr;
        dev->GetImmediateContext(&ctx);
        d3d_device_  = dev;
        d3d_context_ = ctx;
        std::fprintf(stderr,
            "rkmpp MFT: SET_D3D_MANAGER acquired ID3D11Device — "
            "D3D11 surface output enabled\n");
        std::fflush(stderr);
        return S_OK;
    }
    case MFT_MESSAGE_NOTIFY_BEGIN_STREAMING: {
        if (streaming_) return S_OK;
        if (!input_type_ || !output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
        /* AV1: try Hardware mode first (rkmpp.sys AV1 personality);
         * fall back to Software (dav1d) if the device isn't present
         * or the HW init fails — keeps the MFT working on dev machines
         * and gracefully degrading if the AV1 codec is offline. */
        if (kind_ == CodecKind::AV1) {
            if (!engine_av1_) {
                auto *eng = new Av1DecodeEngine();
                int rc = Av1DecodeEngine_Init(eng, Av1EngineMode::Hardware,
                                              width_, height_);
                if (rc != 0) {
                    /* HW init failed — re-init in Software mode. */
                    Av1DecodeEngine_Shutdown(eng);
                    rc = Av1DecodeEngine_Init(eng, Av1EngineMode::Software,
                                              width_, height_);
                }
                if (rc != 0) {
                    delete eng;
                    engine_av1_       = nullptr;
                    engine_init_failed_ = true;
                } else {
                    engine_av1_       = eng;
                    engine_init_failed_ = false;
                }
            }
            streaming_ = true;
            return S_OK;
        }
        if (kind_ == CodecKind::VP9) {
            if (!engine_vp9_) {
                auto *eng = new Vp9DecodeEngine();
                int rc = Vp9DecodeEngine_Init(eng, width_, height_);
                std::fprintf(stderr,
                    "rkmpp MFT(vp9): Init(%ux%u) rc=%d\n",
                    width_, height_, rc);
                if (rc != 0) {
                    Vp9DecodeEngine_Shutdown(eng);
                    delete eng;
                    engine_vp9_         = nullptr;
                    engine_init_failed_ = true;
                } else {
                    /* Diagnostic: bring-up still needs the BSP-captured
                     * probe blob to produce a correct keyframe.  Plumb
                     * via env var so mft_decode can drive testing. */
                    char pbuf[260] = {};
                    if (GetEnvironmentVariableA("RKMPP_VP9_PROBE_BLOB",
                                                pbuf, sizeof(pbuf)) > 0) {
                        static std::string s_path = pbuf;
                        eng->probe_blob_path = s_path.c_str();
                        std::fprintf(stderr,
                            "rkmpp MFT(vp9): probe blob = %s\n", pbuf);
                    }
                    char dbuf[260] = {};
                    if (GetEnvironmentVariableA("RKMPP_VP9_DUMP_BANK",
                                                dbuf, sizeof(dbuf)) > 0) {
                        static std::string s_dump = dbuf;
                        eng->dump_prefix = s_dump.c_str();
                        std::fprintf(stderr,
                            "rkmpp MFT(vp9): bank dump prefix = %s\n", dbuf);
                    }
                    engine_vp9_         = eng;
                    engine_init_failed_ = false;
                }
                std::fflush(stderr);
            }
            streaming_ = true;
            return S_OK;
        }
        if (!engine_) {
            auto *eng = new DecodeEngine();
            int rc = DecodeEngine_Init(eng, ToEngineCodec(kind_),
                                       width_, height_);
            if (rc != 0) {
                /* No rkmpp.sys / no codec capability — keep streaming_
                 * true so type-negotiation paths still work, but mark
                 * init failed; ProcessInput will reject samples. */
                delete eng;
                engine_ = nullptr;
                engine_init_failed_ = true;
            } else {
                engine_ = eng;
                engine_init_failed_ = false;
                /* Zero-copy readout (eng->populate_yuv = false) was tried
                 * here but reliably wedged the codec at 4K, even with the
                 * external-hold infrastructure protecting slot lifetime.
                 * Best guess: removing the engine-side ~12 ms memcpy
                 * collapses the gap between back-to-back kicks, exposing
                 * a hardware contention issue (AXI write retire vs. ref
                 * read on the same slot) that the slow read used to
                 * mask.  Reverting to populate_yuv=true keeps the engine
                 * memcpy gap and the codec stays stable.  The Phase 1
                 * hold infrastructure (slot_idx + external_hold) is
                 * left in place for future use if/when we sequence
                 * kicks more carefully. */
                /* Prime persistent SPS/PPS state from container extradata
                 * (avcC / hvcC parsed into Annex-B in SetInputType). */
                if (!extradata_annexb_.empty()) {
                    (void)DecodeEngine_FeedExtradata(
                        eng, FRAMING_ANNEXB, (AvccLenSize)length_size_,
                        extradata_annexb_.data(),
                        extradata_annexb_.size());
                }
            }
        }
        streaming_ = true;
        return S_OK;
    }
    case MFT_MESSAGE_NOTIFY_END_STREAMING:
        if (engine_) {
            auto *eng = static_cast<DecodeEngine *>(engine_);
            DecodeEngine_Shutdown(eng);
            delete eng;
            engine_ = nullptr;
        }
        if (engine_av1_) {
            auto *eng = static_cast<Av1DecodeEngine *>(engine_av1_);
            Av1DecodeEngine_Shutdown(eng);
            delete eng;
            engine_av1_ = nullptr;
        }
        if (engine_vp9_) {
            auto *eng = static_cast<Vp9DecodeEngine *>(engine_vp9_);
            Vp9DecodeEngine_Shutdown(eng);
            delete eng;
            engine_vp9_ = nullptr;
        }
        vp9_out_queue_.clear();
        engine_init_failed_ = false;
        streaming_ = false;
        input_queue_.clear();
        input_timestamps_.clear();
        return S_OK;
    case MFT_MESSAGE_COMMAND_DRAIN:
        std::fprintf(stderr, "rkmpp MFT: COMMAND_DRAIN samples_received=%llu epoch=%u\n",
                     (unsigned long long)samples_received_, stream_epoch_);
        std::fflush(stderr);
        draining_ = true;
        return S_OK;
    case MFT_MESSAGE_COMMAND_FLUSH:
        std::fprintf(stderr, "rkmpp MFT: COMMAND_FLUSH samples_received=%llu epoch=%u→%u\n",
                     (unsigned long long)samples_received_,
                     stream_epoch_, stream_epoch_ + 1);
        std::fflush(stderr);
        input_queue_.clear();
        input_timestamps_.clear();
        draining_ = false;
        last_emitted_pts_ = INT64_MIN;
        ++stream_epoch_;
        if (engine_) {
            auto *eng = static_cast<DecodeEngine *>(engine_);
            (void)DecodeEngine_Flush(eng);
        }
        if (engine_av1_) {
            auto *eng = static_cast<Av1DecodeEngine *>(engine_av1_);
            (void)Av1DecodeEngine_Flush(eng);
        }
        if (engine_vp9_) {
            auto *eng = static_cast<Vp9DecodeEngine *>(engine_vp9_);
            Vp9DecodeEngine_Flush(eng);
        }
        vp9_out_queue_.clear();
        return S_OK;
    case MFT_MESSAGE_NOTIFY_START_OF_STREAM:
        std::fprintf(stderr, "rkmpp MFT: NOTIFY_START_OF_STREAM samples_received=%llu\n",
                     (unsigned long long)samples_received_);
        std::fflush(stderr);
        return S_OK;
    case MFT_MESSAGE_NOTIFY_END_OF_STREAM:
        std::fprintf(stderr, "rkmpp MFT: NOTIFY_END_OF_STREAM samples_received=%llu\n",
                     (unsigned long long)samples_received_);
        std::fflush(stderr);
        return S_OK;
    case MFT_MESSAGE_NOTIFY_RELEASE_RESOURCES:
        std::fprintf(stderr, "rkmpp MFT: NOTIFY_RELEASE_RESOURCES\n");
        std::fflush(stderr);
        return S_OK;
    case MFT_MESSAGE_NOTIFY_REACQUIRE_RESOURCES:
        std::fprintf(stderr, "rkmpp MFT: NOTIFY_REACQUIRE_RESOURCES\n");
        std::fflush(stderr);
        return S_OK;
    default:
        return S_OK;
    }
}

/* One-shot input bitstream dumper for offline replay debugging.
 *
 * Activates on the first ProcessInput call if the sentinel file
 * "mft_dump.flag" exists in the working directory.  Writes a small
 * fixed header (codec kind, width/height/framing/extradata) followed by
 * length-prefixed AU records to "mft_input_dump.bin".  Stops appending
 * once kDumpBytesMax (50 MB) is reached so a long playback can't fill
 * the disk.  No-op when the sentinel is absent — zero cost on the
 * normal playback path beyond a single fopen probe at first sample.
 *
 * On-disk format (little-endian; little-endian dev box matches replay
 * targets — Linux/ARM/x86 dev boxes are all LE):
 *
 *   FileHeader (written once before any AU):
 *     u32 magic         = 'P','D','K','R' (0x524B4450)  // ASCII "RKDP" reversed
 *     u32 version       = 1
 *     u32 codec_kind    = CodecKind (0=H264, 1=HEVC, 2=AV1, 3=VP9)
 *     u32 length_size   = 0 for ANNEXB, 1/2/4 for AVCC{1,2,4}
 *     u32 width
 *     u32 height
 *     u32 fps_num
 *     u32 fps_den
 *     u32 bit_depth
 *     u32 extradata_annexb_len
 *     u8  extradata_annexb[extradata_annexb_len]   // SPS+PPS as Annex-B
 *
 *   AuRecord (one per ProcessInput):
 *     u32 magic         = 'U','A','K','R' (0x55414B52)  // ASCII "RKAU" reversed
 *     u64 sample_no     = samples_received_ at this call (1-based)
 *     i64 pts_hns
 *     u32 size
 *     u8  bytes[size]   // raw AU as received from the host (AVCC- or
 *                       // Annex-B-framed; replay must use length_size)
 *
 * Caller is responsible for holding lock_; this is called from
 * ProcessInput which already holds it. */
void DecoderMFT::DumpAuIfActive(const uint8_t *au, size_t au_len, int64_t pts_hns) {
    if (!dump_checked_) {
        dump_checked_ = true;
        /* The dump mechanism writes the raw bitstream — including any
         * DRM-decrypted content delivered to us — to a path the host
         * implicitly trusts.  Require an explicit absolute path via
         * env var RKMPP_DUMP_PATH; ignore the legacy sentinel-in-CWD
         * pattern so a malicious or hostile host can't activate it.
         * See [[critical_mft_dump_unsanitized]]. */
        char dumpPath[1024] = {0};
        DWORD got = GetEnvironmentVariableA("RKMPP_DUMP_PATH",
                                            dumpPath, sizeof(dumpPath));
        if (got == 0 || got >= sizeof(dumpPath)) return;   /* dump disabled */
        /* Require an absolute path (drive-letter prefix or UNC).  Reject
         * relative paths so a process-launch CWD can't redirect us. */
        bool absoluteOk = false;
        if (got >= 3 &&
            ((dumpPath[0] >= 'A' && dumpPath[0] <= 'Z') ||
             (dumpPath[0] >= 'a' && dumpPath[0] <= 'z')) &&
            dumpPath[1] == ':' &&
            (dumpPath[2] == '\\' || dumpPath[2] == '/')) {
            absoluteOk = true;
        } else if (got >= 2 && dumpPath[0] == '\\' && dumpPath[1] == '\\') {
            absoluteOk = true;
        }
        if (!absoluteOk) return;
        {
            errno_t e = fopen_s(&dump_file_, dumpPath, "wb");
            if (e != 0 || !dump_file_) {
                dump_file_ = nullptr;
            } else {
                std::fprintf(stderr,
                    "rkmpp MFT: dump active → %s "
                    "(cap %zu MB)\n", dumpPath, kDumpBytesMax / (1024 * 1024));
                std::fflush(stderr);
                uint32_t magic     = 0x504B4452; /* "RKDP" little-endian */
                uint32_t version   = 1;
                uint32_t kind      = (uint32_t)kind_;
                uint32_t lenSize   = (uint32_t)length_size_;
                uint32_t w         = (uint32_t)width_;
                uint32_t h         = (uint32_t)height_;
                uint32_t fpsN      = (uint32_t)fps_num_;
                uint32_t fpsD      = (uint32_t)fps_den_;
                uint32_t bd        = (uint32_t)bit_depth_;
                uint32_t exLen     = (uint32_t)extradata_annexb_.size();
                std::fwrite(&magic,   4, 1, dump_file_);
                std::fwrite(&version, 4, 1, dump_file_);
                std::fwrite(&kind,    4, 1, dump_file_);
                std::fwrite(&lenSize, 4, 1, dump_file_);
                std::fwrite(&w,       4, 1, dump_file_);
                std::fwrite(&h,       4, 1, dump_file_);
                std::fwrite(&fpsN,    4, 1, dump_file_);
                std::fwrite(&fpsD,    4, 1, dump_file_);
                std::fwrite(&bd,      4, 1, dump_file_);
                std::fwrite(&exLen,   4, 1, dump_file_);
                if (exLen) std::fwrite(extradata_annexb_.data(), 1, exLen, dump_file_);
                dump_bytes_ += 40 + exLen;
                std::fflush(dump_file_);
            }
        }
    }
    if (!dump_file_)                   return;
    if (dump_bytes_ >= kDumpBytesMax)  return;
    if (au_len == 0)                   return;

    uint32_t magic     = 0x55414B52;  /* "RKAU" little-endian */
    uint64_t sample_no = samples_received_;
    uint32_t size      = (uint32_t)au_len;
    std::fwrite(&magic,     4, 1, dump_file_);
    std::fwrite(&sample_no, 8, 1, dump_file_);
    std::fwrite(&pts_hns,   8, 1, dump_file_);
    std::fwrite(&size,      4, 1, dump_file_);
    std::fwrite(au,         1, au_len, dump_file_);
    dump_bytes_ += 24 + au_len;
    /* Flush per-AU.  At 30 fps × a few KB per AU this is trivial cost
     * but guarantees the tail of the dump is on disk if mpv hangs or
     * is force-killed immediately after the wedge — without this, the
     * last second or so of input could be lost in the OS buffer. */
    std::fflush(dump_file_);
}

STDMETHODIMP DecoderMFT::ProcessInput(DWORD id, IMFSample *sample, DWORD /*flags*/) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!sample) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_ || !output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    if (!streaming_)                    return MF_E_TRANSFORM_TYPE_NOT_SET;

    /* AV1 path: feed each MF sample (one OBU temporal unit) directly
     * into Av1DecodeEngine_Submit; dav1d emits zero or more pictures
     * per call which ProcessOutput drains in display order. */
    if (kind_ == CodecKind::AV1) {
        if (!engine_av1_ || engine_init_failed_) return MF_E_NOTACCEPTING;
        /* Cap = engine's max_reorder_pics (8 for hierarchical streams,
         * 0 for low-delay) + a working margin for the in-flight kick.
         * Must exceed max_reorder_pics so the bump in DrainPictures can
         * fire without us first refusing input — otherwise the engine
         * deadlocks: input refused at kQueueCap, ready_q empty, never
         * reaches the threshold needed to bump. */
        const size_t kQueueCap = 24;
        auto *eng = static_cast<Av1DecodeEngine *>(engine_av1_);
        if (Av1DecodeEngine_QueueDepth(eng) >= kQueueCap) {
            return MF_E_NOTACCEPTING;
        }
        LONGLONG pts = 0;
        if (FAILED(sample->GetSampleTime(&pts))) {
            pts = (LONGLONG)((samples_received_ * 10'000'000ULL * fps_den_)
                             / (fps_num_ ? fps_num_ : 30));
        }
        DWORD buf_count = 0;
        HRESULT hr = sample->GetBufferCount(&buf_count);
        if (FAILED(hr)) return hr;
        std::vector<uint8_t> au;
        for (DWORD i = 0; i < buf_count; ++i) {
            IMFMediaBuffer *buf = nullptr;
            hr = sample->GetBufferByIndex(i, &buf);
            if (FAILED(hr)) return hr;
            BYTE *p = nullptr; DWORD cur = 0, max = 0;
            hr = buf->Lock(&p, &max, &cur);
            if (SUCCEEDED(hr)) {
                /* MF buffer contract says cur <= max; a buggy/hostile
                 * source can violate it.  Without this check, the
                 * insert reads `cur` bytes past the locked region.
                 * See [[critical_mft_buf_len_unchecked]]. */
                if (cur > max) {
                    buf->Unlock();
                    buf->Release();
                    return MF_E_INVALID_STREAM_DATA;
                }
                /* AU size cap (16 MiB).  Legitimate AVC/HEVC/AV1 ATU
                 * sizes are well under 1 MB even at 4K; a multi-MB AU
                 * is either pathological or a DoS attempt.  Without
                 * this cap a hostile source can drive `au.size()`
                 * into multi-GB territory, dominating process memory
                 * + vector-reallocation cost.  Review MFT #11. */
                if (au.size() + (size_t)cur > (size_t)(16u << 20)) {
                    buf->Unlock();
                    buf->Release();
                    return MF_E_INVALID_STREAM_DATA;
                }
                au.insert(au.end(), p, p + cur);
                buf->Unlock();
            }
            buf->Release();
            if (FAILED(hr)) return hr;
        }
        samples_received_++;
        DumpAuIfActive(au.data(), au.size(), (int64_t)pts);
        { char _t[2]; bool _on = GetEnvironmentVariableA("RKMPP_AV1_TRACE", _t, 2) > 0; if (_on) {
            std::fprintf(stderr,
                "AV1_TRACE pi#%llu pts=%lld bytes=%zu head=",
                (unsigned long long)samples_received_,
                (long long)pts, au.size());
            for (size_t i = 0; i < au.size() && i < 16; ++i)
                std::fprintf(stderr, "%02x", au[i]);
            std::fprintf(stderr, "\n");
            std::fflush(stderr);
        }}
        int rc = Av1DecodeEngine_Submit(eng, au.data(), au.size(),
                                        (int64_t)pts);
        if (rc != 0) decode_errors_++;
        /* Refresh bit_depth_ from the engine's cached sequence header.
         * AV1 has no separate container extradata — seq_hdr arrives in
         * the bitstream itself, so bit-depth is only knowable after the
         * first OBU temporal unit is parsed.  Drives NV12-vs-P010
         * output-type choice; ProcessOutput's size guard also self-
         * corrects from the actual frame size. */
        if (rc == 0 && eng->seq_hdr_valid &&
            eng->cached_seq_hdr.hbd && bit_depth_ != 10) {
            bit_depth_ = 10;
        }
        return S_OK;
    }

    /* VP9 path: synchronous DecodeOne per superframe-subframe; produced
     * NV12 frames buffered in vp9_out_queue_ for ProcessOutput to emit. */
    if (kind_ == CodecKind::VP9) {
        if (!engine_vp9_ || engine_init_failed_) return MF_E_NOTACCEPTING;
        const size_t kVp9QueueCap = 8;
        if (vp9_out_queue_.size() >= kVp9QueueCap) return MF_E_NOTACCEPTING;
        LONGLONG pts = 0;
        if (FAILED(sample->GetSampleTime(&pts))) {
            pts = (LONGLONG)((samples_received_ * 10'000'000ULL * fps_den_)
                             / (fps_num_ ? fps_num_ : 30));
        }
        DWORD buf_count = 0;
        HRESULT hr = sample->GetBufferCount(&buf_count);
        if (FAILED(hr)) return hr;
        std::vector<uint8_t> au;
        for (DWORD i = 0; i < buf_count; ++i) {
            IMFMediaBuffer *mfb = nullptr;
            hr = sample->GetBufferByIndex(i, &mfb);
            if (FAILED(hr)) return hr;
            BYTE *p = nullptr; DWORD cur = 0, max = 0;
            hr = mfb->Lock(&p, &max, &cur);
            if (SUCCEEDED(hr)) {
                if (cur > max) {
                    mfb->Unlock();
                    mfb->Release();
                    return MF_E_INVALID_STREAM_DATA;
                }
                /* AU size cap — see AV1 path comment.  Review MFT #11. */
                if (au.size() + (size_t)cur > (size_t)(16u << 20)) {
                    mfb->Unlock();
                    mfb->Release();
                    return MF_E_INVALID_STREAM_DATA;
                }
                au.insert(au.end(), p, p + cur);
                mfb->Unlock();
            }
            mfb->Release();
            if (FAILED(hr)) return hr;
        }
        samples_received_++;
        DumpAuIfActive(au.data(), au.size(), (int64_t)pts);
        const uint8_t *frames[8] = {};
        size_t         fsizes[8] = {};
        int nf = vp9::Vp9Parser_SuperframeSplit(au.data(), au.size(),
                                                frames, fsizes, 8);
        std::fprintf(stderr,
            "rkmpp MFT(vp9): PI au_bytes=%zu nf=%d\n", au.size(), nf);
        if (nf <= 0) { decode_errors_++; std::fflush(stderr); return S_OK; }
        /* Defense-in-depth: spec maxes at 8 subframes per superframe,
         * which matches our `max_frames` arg above, but if the parser
         * ever miscounts (or a future spec rev permits more) the for
         * loop below would walk past frames[]/fsizes[].  Clamp here.
         * Review MFT #15. */
        if (nf > 8) nf = 8;
        /* Eager profile peek — drives BuildOutputType's NV12-vs-P010
         * choice before the first ProcessOutput.  WebM/MKV containers
         * don't surface a VPCC-style extradata via MF, so the only way
         * to know we're 10-bit before SetOutputType locks in NV12 is to
         * read profile out of the first frame's uncompressed header.
         * VP9 uncompressed header (spec §6.2, MSB-first):
         *   frame_marker  f(2)   : byte0[7:6] = 0b10
         *   profile_low   f(1)   : byte0[5]
         *   profile_high  f(1)   : byte0[4]
         *   profile = profile_low | (profile_high << 1)
         * If the negotiated output_type_ is already locked to NV12 and
         * we now know we're 10-bit, ProcessOutput emits STREAM_CHANGE so
         * the client (mpv, EVR, ...) re-queries and lands on P010. */
        /* Only profile 2 (4:2:0 10-bit) is wired through repack_yuv;
         * profile 3 (4:4:4) and 12-bit are rejected by the parser, so
         * don't pre-commit to P010 for them — let DecodeOne fail with
         * the actual error instead of pinning the output type. */
        if (fsizes[0] >= 1 && frames[0]) {
            uint8_t b0 = frames[0][0];
            if (((b0 >> 6) & 0x3u) == 2u) {
                uint8_t profile = ((b0 >> 5) & 0x1u)
                                | (((b0 >> 4) & 0x1u) << 1);
                if (profile == 2u && bit_depth_ != 10) {
                    bit_depth_ = 10;
                    std::fprintf(stderr,
                        "rkmpp MFT(vp9): eager profile=2 → bit_depth=10\n");
                }
            }
        }
        auto *eng = static_cast<Vp9DecodeEngine *>(engine_vp9_);
        for (int i = 0; i < nf; ++i) {
            Vp9DecodedFrame df;
            int rc = Vp9DecodeEngine_DecodeOne(eng, frames[i], fsizes[i],
                                               (int64_t)pts, &df);
            std::fprintf(stderr,
                "rkmpp MFT(vp9): DecodeOne[%d] sz=%zu rc=%d show=%d yuv=%zu\n",
                i, fsizes[i], rc, (int)df.show, df.yuv.size());
            if (rc != 0) { decode_errors_++; continue; }
            /* Refresh bit_depth_ from the first 10-bit frame the engine
             * parses — VP9 has no avcC-style extradata, so Profile 2 is
             * only knowable after the first uncompressed header.  Drives
             * BuildOutputType's NV12-vs-P010 choice; ProcessOutput below
             * also self-corrects per-frame from df.bit_depth. */
            if (rc == 0 && df.bit_depth == 10 && bit_depth_ != 10) {
                bit_depth_ = 10;
            }
            if (df.show && !df.yuv.empty()) {
                Vp9OutFrame o;
                o.yuv       = std::move(df.yuv);
                o.pts_hns   = df.pts_hns;
                o.width     = df.width;
                o.height    = df.height;
                o.bit_depth = df.bit_depth;
                vp9_out_queue_.push_back(std::move(o));
            }
            Vp9DecodeEngine_ReleaseFrame(eng, &df);
        }
        std::fflush(stderr);
        return S_OK;
    }

    if (!engine_ || engine_init_failed_) return MF_E_NOTACCEPTING;

    /* Backpressure: each queued frame holds a DPB slot via the external-
     * hold counter (see dpb.h Dpb_AddExternalHold).  The DPB also needs
     * slots for active reference frames (~5-8 typical).  When the engine
     * pumps faster than EVR drains (e.g. zero-copy at 4K — Submit ~1ms,
     * Output @ 24fps), uncapped queue growth exhausts the slot pool and
     * the next decode hits DPB_FULL → bad refs → codec wedge.  Cap the
     * queue at kQueueCap so EVR's source reader retries this AU later. */
    {
        auto *eng = static_cast<DecodeEngine *>(engine_);
        if (DecodeEngine_QueueDepth(eng) >= DecodeEngine_InputQueueCapacity(eng)) {
            return MF_E_NOTACCEPTING;
        }
    }

    /* Pull PTS / duration from the sample (HNS, 100ns units). */
    LONGLONG pts = 0, dur = 0;
    if (FAILED(sample->GetSampleTime(&pts))) {
        pts = (LONGLONG)((samples_received_ * 10'000'000ULL * fps_den_)
                         / (fps_num_ ? fps_num_ : 30));
    }
    if (FAILED(sample->GetSampleDuration(&dur))) {
        dur = (LONGLONG)((10'000'000ULL * fps_den_) / (fps_num_ ? fps_num_ : 30));
    }

    /* Concat all buffers in the sample into one byte vector. */
    DWORD buf_count = 0;
    HRESULT hr = sample->GetBufferCount(&buf_count);
    if (FAILED(hr)) return hr;

    std::vector<uint8_t> au;
    for (DWORD i = 0; i < buf_count; ++i) {
        IMFMediaBuffer *buf = nullptr;
        hr = sample->GetBufferByIndex(i, &buf);
        if (FAILED(hr)) return hr;
        BYTE *p = nullptr; DWORD cur = 0, max = 0;
        hr = buf->Lock(&p, &max, &cur);
        if (SUCCEEDED(hr)) {
            if (cur > max) {
                buf->Unlock();
                buf->Release();
                return MF_E_INVALID_STREAM_DATA;
            }
            /* AU size cap — see AV1 path comment.  Review MFT #11. */
            if (au.size() + (size_t)cur > (size_t)(16u << 20)) {
                buf->Unlock();
                buf->Release();
                return MF_E_INVALID_STREAM_DATA;
            }
            au.insert(au.end(), p, p + cur);
            buf->Unlock();
        }
        buf->Release();
        if (FAILED(hr)) return hr;
    }
    samples_received_++;
    DumpAuIfActive(au.data(), au.size(), (int64_t)pts);

    /* Per-input trace for diagnosing duplicated/out-of-order MS source
     * reader feeds.  Gated by sentinel `mft_trace.flag` (same gate as
     * the per-emit trace).  Logs sample# / pts / first-byte to let us
     * identify when MS submits the same packet twice. */
    {
        static int trace_state_in = -1;
        if (trace_state_in < 0) {
            FILE *probe = nullptr;
            if (fopen_s(&probe, "mft_trace.flag", "rb") == 0 && probe) {
                trace_state_in = 1;
                fclose(probe);
            } else {
                trace_state_in = 0;
            }
        }
        if (trace_state_in == 1) {
            uint8_t hdr0 = au.size() > 4 ? au[4] : 0;
            uint8_t hdr1 = au.size() > 5 ? au[5] : 0;
            std::fprintf(stderr,
                "rkmpp MFT TRACE: input#%llu pts=%lld bytes=%zu "
                "head=%02x%02x epoch=%u\n",
                (unsigned long long)samples_received_,
                (long long)pts, au.size(), hdr0, hdr1, stream_epoch_);
            std::fflush(stderr);
        }
    }

    /* Decode immediately and push into the engine's reorder window.
     * ProcessOutput drains ready_q via PollFrame; the legacy input_queue_
     * was a vestige from Phase 2B before reorder existed.  Decode-failure
     * is non-fatal (matches the legacy ProcessOutput behaviour): we drop
     * the AU and let the next IDR re-sync. */
    auto *eng = static_cast<DecodeEngine *>(engine_);
    NalFraming  framing  = (length_size_ != 0) ? FRAMING_AVCC4 : FRAMING_ANNEXB;
    AvccLenSize len_size = (AvccLenSize)length_size_;
    /* The engine's pts is overwritten with the synthetic stream after
     * Submit returns; we want the caller's PTS to flow through, so
     * pass it directly.  Duration is per-stream framerate; we ignore
     * the per-sample dur the host supplied (engine's frame rate is
     * the truth). */
    int rc = DecodeEngine_SubmitFramed(eng, framing, len_size,
                                       au.data(), au.size(),
                                       (int64_t)pts,
                                       stream_epoch_);
    if (rc != 0) {
        decode_errors_++;
        /* Don't fail the whole pipeline; report success but produce
         * nothing — the host calls ProcessOutput separately. */
    }
    /* Refresh bit_depth_ from the engine's parsed SPS after the first
     * successful submit.  For streams that carry SPS in container
     * extradata (avcC), bit_depth_ was already set in SetInputType.
     * For Annex-B streams with inline SPS (mft_decode harness, MPEG-TS
     * sources, etc.) the SPS only becomes visible here — and without
     * this refresh, ProcessOutput's expected-size check would reject
     * every 10-bit frame as oversized (engine emits P010; MFT thinks
     * NV12).  Output type stays as already-negotiated NV12 for inline
     * streams; sample bytes still flow through correctly because
     * GetOutputStreamInfo / sysmem allocation here uses bit_depth_,
     * not the negotiated type. */
    if (kind_ == CodecKind::H264 && rc == 0 &&
        eng->parsed_h264.has_sps &&
        eng->parsed_h264.sps.bit_depth_luma_minus8 == 2 &&
        bit_depth_ != 10) {
        bit_depth_ = 10;
    }
    if (kind_ == CodecKind::HEVC && rc == 0 &&
        eng->parsed_h265.active_sps_id >= 0) {
        const auto &sps = eng->parsed_h265.sps[eng->parsed_h265.active_sps_id];
        if (sps.valid && sps.bit_depth_luma_minus8 == 2 && bit_depth_ != 10)
            bit_depth_ = 10;
    }
    (void)dur;
    return S_OK;
}

STDMETHODIMP DecoderMFT::ProcessOutput(DWORD /*flags*/, DWORD c,
                                       MFT_OUTPUT_DATA_BUFFER *buf,
                                       DWORD *status) {
    int64_t mft_t0 = MftTimingEnabled() ? MftQpcNow() : 0;
    int64_t mft_copy_us = 0;
    if (c != 1 || !buf) return E_INVALIDARG;
    if (status) *status = 0;
    buf[0].dwStatus = 0;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_ || !output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    if (buf[0].pSample) return E_FAIL;

    /* AV1 path: hardware (RKCP3560) or software (dav1d).  Same D3D11 /
     * 2D-buffer / 1D-buffer selection as the H.264/HEVC path below — the
     * output is identical NV12 packed width*height*3/2. */
    if (kind_ == CodecKind::AV1) {
        if (!engine_av1_ || engine_init_failed_) {
            buf[0].pSample = nullptr;
            return E_FAIL;
        }
        auto *eng = static_cast<Av1DecodeEngine *>(engine_av1_);
        if (draining_) Av1DecodeEngine_Drain(eng);
        Av1DecodedFrame frame;
        int got = Av1DecodeEngine_PollFrame(eng, &frame);
        if (got <= 0) {
            buf[0].pSample = nullptr;
            return MF_E_TRANSFORM_NEED_MORE_INPUT;
        }

        IMFMediaBuffer   *mbuf    = nullptr;
        ID3D11Texture2D  *texture = nullptr;
        HRESULT hr = S_OK;

        ID3D11Device *use_d3d = d3d_device_;
        {
            static bool logged_av1 = false;
            if (!logged_av1) {
                if (use_d3d && width_ && height_)
                    std::fprintf(stderr, "rkmpp MFT(av1): output mode = D3D11_SURFACE_BUFFER\n");
                else if (dxgi_manager_)
                    std::fprintf(stderr, "rkmpp MFT(av1): output mode = 2D_MEDIA_BUFFER\n");
                else
                    std::fprintf(stderr, "rkmpp MFT(av1): output mode = SYSMEM_1D_BUFFER\n");
                std::fflush(stderr);
                logged_av1 = true;
            }
        }

        if (use_d3d && width_ && height_) {
            D3D11_TEXTURE2D_DESC td = {};
            td.Width            = width_;
            td.Height           = height_;
            td.MipLevels        = 1;
            td.ArraySize        = 1;
            td.Format           = DXGI_FORMAT_NV12;
            td.SampleDesc.Count = 1;
            td.Usage            = D3D11_USAGE_DEFAULT;
            td.BindFlags        = D3D11_BIND_DECODER | D3D11_BIND_SHADER_RESOURCE;
            hr = d3d_device_->CreateTexture2D(&td, nullptr, &texture);
            if (FAILED(hr)) {
                td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
                hr = d3d_device_->CreateTexture2D(&td, nullptr, &texture);
                if (FAILED(hr)) { Av1DecodeEngine_ReleaseFrame(eng, &frame); return hr; }
            }
            const UINT row_pitch = width_;
            d3d_context_->UpdateSubresource(texture, 0, nullptr,
                                            frame.yuv.data(),
                                            row_pitch, row_pitch * height_);
            d3d_context_->UpdateSubresource(texture, 1, nullptr,
                                            frame.yuv.data() + (size_t)row_pitch * height_,
                                            row_pitch, row_pitch * (height_ / 2));
            hr = MFCreateDXGISurfaceBuffer(IID_ID3D11Texture2D, texture,
                                           0, FALSE, &mbuf);
            if (FAILED(hr)) {
                texture->Release();
                Av1DecodeEngine_ReleaseFrame(eng, &frame);
                return hr;
            }
            mbuf->SetCurrentLength((DWORD)frame.yuv.size());
        } else {
            // Bit-depth-aware sizing: P010 doubles every byte count and
            // stride vs NV12.  The engine has already repacked into
            // frame.yuv at the target format (NV12 or P010) — use its
            // size as the source of truth rather than recomputing.
            const UINT  bytes_per_sample = (bit_depth_ == 10) ? 2u : 1u;
            const DWORD row_bytes  = (DWORD)width_ * bytes_per_sample;
            const DWORD y_bytes    = row_bytes * height_;
            const DWORD plane_bytes = (DWORD)frame.yuv.size();
            const bool  use_2d     = (dxgi_manager_ != nullptr);
            if (use_2d) {
                const DWORD fourcc = (bit_depth_ == 10)
                                     ? MAKEFOURCC('P','0','1','0')
                                     : MAKEFOURCC('N','V','1','2');
                hr = MFCreate2DMediaBuffer(width_, height_, fourcc,
                                           FALSE, &mbuf);
                if (FAILED(hr)) { Av1DecodeEngine_ReleaseFrame(eng, &frame); return hr; }
                IMF2DBuffer *buf2d = nullptr;
                hr = mbuf->QueryInterface(IID_PPV_ARGS(&buf2d));
                if (FAILED(hr) || !buf2d) {
                    mbuf->Release();
                    Av1DecodeEngine_ReleaseFrame(eng, &frame);
                    return FAILED(hr) ? hr : E_NOINTERFACE;
                }
                BYTE *dst = nullptr; LONG pitch = 0;
                hr = buf2d->Lock2D(&dst, &pitch);
                if (FAILED(hr)) {
                    buf2d->Release(); mbuf->Release();
                    Av1DecodeEngine_ReleaseFrame(eng, &frame);
                    return hr;
                }
                int64_t cp_t0 = MftTimingEnabled() ? MftQpcNow() : 0;
                const uint8_t *src_y  = frame.yuv.data();
                const uint8_t *src_uv = src_y + (size_t)y_bytes;
                for (UINT32 r = 0; r < height_; r++)
                    std::memcpy(dst + (size_t)r * pitch,
                                src_y + (size_t)r * row_bytes, row_bytes);
                BYTE *dst_uv = dst + (size_t)pitch * height_;
                for (UINT32 r = 0; r < height_ / 2; r++)
                    std::memcpy(dst_uv + (size_t)r * pitch,
                                src_uv + (size_t)r * row_bytes, row_bytes);
                if (MftTimingEnabled()) mft_copy_us = MftQpcUs(cp_t0, MftQpcNow());
                buf2d->Unlock2D();
                buf2d->Release();
                mbuf->SetCurrentLength(plane_bytes);
            } else {
                hr = MFCreateMemoryBuffer(plane_bytes, &mbuf);
                if (FAILED(hr)) { Av1DecodeEngine_ReleaseFrame(eng, &frame); return hr; }
                BYTE *dst = nullptr; DWORD cap = 0, cur = 0;
                hr = mbuf->Lock(&dst, &cap, &cur);
                if (FAILED(hr)) {
                    mbuf->Release();
                    Av1DecodeEngine_ReleaseFrame(eng, &frame);
                    return hr;
                }
                int64_t cp_t0 = MftTimingEnabled() ? MftQpcNow() : 0;
                std::memcpy(dst, frame.yuv.data(), plane_bytes);
                if (MftTimingEnabled()) mft_copy_us = MftQpcUs(cp_t0, MftQpcNow());
                mbuf->Unlock();
                mbuf->SetCurrentLength(plane_bytes);
            }
        }

        IMFSample *out_sample = nullptr;
        hr = MFCreateSample(&out_sample);
        if (FAILED(hr)) {
            mbuf->Release();
            if (texture) texture->Release();
            Av1DecodeEngine_ReleaseFrame(eng, &frame);
            return hr;
        }
        out_sample->AddBuffer(mbuf);
        mbuf->Release();
        if (texture) texture->Release();

        Av1DecodeEngine_ReleaseFrame(eng, &frame);

        const uint32_t fpsn = fps_num_ ? fps_num_ : 30;
        const uint32_t fpsd = fps_den_ ? fps_den_ : 1;
        /* AV1: ignore engine's hardcoded 30fps frame.dur_hns and derive
         * duration from the MFT's negotiated MF_MT_FRAME_RATE.  At 24fps
         * the engine's 333333 hns would leave an 83ms gap before the
         * next sample's PTS at 416666, which the renderer fills with a
         * stale buffer — visible as glitches every frame, worst on the
         * alternating HW/dav1d-output show_existing pattern. */
        const LONGLONG dur  = (LONGLONG)((10'000'000ULL * fpsd) / fpsn);
        const LONGLONG pts  = (frame.pts_hns >= 0)
                            ? (LONGLONG)frame.pts_hns
                            : (LONGLONG)((uint64_t)frames_emitted_
                                         * 10'000'000ULL * fpsd / fpsn);
        out_sample->SetSampleTime(pts);
        out_sample->SetSampleDuration(dur);
        out_sample->SetUINT32(MFSampleExtension_CleanPoint, TRUE);

        buf[0].pSample  = out_sample;
        buf[0].dwStatus = 0;
        if (status) *status = 0;
        frames_emitted_++;
        { char _t[2]; bool _on = GetEnvironmentVariableA("RKMPP_AV1_TRACE", _t, 2) > 0; if (_on) {
            const char *mode = (use_d3d && width_ && height_) ? "D3D11"
                             : (dxgi_manager_ ? "2D" : "SYSMEM");
            /* Dense fingerprint over the full frame.yuv.  The earlier
             * every-4096-byte sample only saw 337 of 1.4M bytes per
             * 720p frame and missed block-level corruption.  This loop
             * folds every byte (FNV-1a over the whole buffer). */
            uint32_t h = 2166136261u;
            const size_t N = frame.yuv.size();
            const uint8_t *p = frame.yuv.data();
            for (size_t i = 0; i < N; i++) {
                h ^= p[i];
                h *= 16777619u;
            }
            std::fprintf(stderr,
                "AV1_TRACE po#%llu pts=%lld dur=%lld mode=%s ybytes=%zu y0=%02x ymid=%02x ylast=%02x fnv=%08x\n",
                (unsigned long long)frames_emitted_,
                (long long)pts, (long long)dur, mode,
                N,
                N ? frame.yuv[0] : 0u,
                N ? frame.yuv[N/2] : 0u,
                N ? frame.yuv[N-1] : 0u,
                h);
            std::fflush(stderr);
        }}
        MaybeLogFrameStats();

        if (MftTimingEnabled()) {
            static bool hdr_av1 = false;
            if (!hdr_av1) {
                std::fprintf(stderr, "MFT_TIMING_AV1,frame,po_total_us,po_copy_us\n");
                hdr_av1 = true;
            }
            std::fprintf(stderr, "MFT_TIMING_AV1,%llu,%lld,%lld\n",
                         (unsigned long long)frames_emitted_,
                         (long long)MftQpcUs(mft_t0, MftQpcNow()),
                         (long long)mft_copy_us);
            std::fflush(stderr);
        }
        return S_OK;
    }

    /* VP9 path: pop a decoded frame from vp9_out_queue_ and emit. */
    if (kind_ == CodecKind::VP9) {
        /* Signal STREAM_CHANGE if the head frame's bit-depth or coded
         * dimensions no longer match the negotiated output type.  VP9
         * permits in-band resolution change (spec §8.6.1) and the
         * profile peek in ProcessInput may discover Profile 2 after
         * SetOutputType has already locked NV12.  Drop output_type_ and
         * advertise the new size/format via BuildOutputType — the
         * client (mpv) re-queries and the queued frame emits next pass.
         *
         * Uses the head frame's dims rather than width_/height_ so the
         * upload below doesn't read past frame.yuv when a stream's
         * coded size shrinks mid-decode. */
        bool need_change = false;
        bool need_p010 = (bit_depth_ == 10);
        if (output_type_ && !vp9_out_queue_.empty()) {
            const Vp9OutFrame &head = vp9_out_queue_.front();
            need_p010 = (head.bit_depth == 10);
            GUID sub = {};
            output_type_->GetGUID(MF_MT_SUBTYPE, &sub);
            const bool have_p010 = (sub == MFVideoFormat_P010);
            if (need_p010 != have_p010
                || head.width  != width_
                || head.height != height_) {
                std::fprintf(stderr,
                    "rkmpp MFT(vp9): STREAM_CHANGE %ux%u/%s → %ux%u/%s\n",
                    (unsigned)width_, (unsigned)height_,
                    have_p010 ? "P010" : "NV12",
                    (unsigned)head.width, (unsigned)head.height,
                    need_p010 ? "P010" : "NV12");
                std::fflush(stderr);
                width_      = head.width;
                height_     = head.height;
                bit_depth_  = head.bit_depth;
                need_change = true;
            }
        }
        if (need_change) {
            output_type_->Release();
            output_type_ = nullptr;
            buf[0].pSample  = nullptr;
            buf[0].dwStatus = MFT_OUTPUT_DATA_BUFFER_FORMAT_CHANGE;
            if (status) *status = MFT_OUTPUT_DATA_BUFFER_FORMAT_CHANGE;
            return MF_E_TRANSFORM_STREAM_CHANGE;
        }
        /* Bit-depth-only case (queue empty but eager peek changed
         * bit_depth_): handle outside the head-frame guard above. */
        if (output_type_ && vp9_out_queue_.empty()) {
            GUID sub = {};
            output_type_->GetGUID(MF_MT_SUBTYPE, &sub);
            const bool have_p010 = (sub == MFVideoFormat_P010);
            if (need_p010 != have_p010) {
                std::fprintf(stderr,
                    "rkmpp MFT(vp9): STREAM_CHANGE (depth-only) need_p010=%d have_p010=%d\n",
                    (int)need_p010, (int)have_p010);
                std::fflush(stderr);
                output_type_->Release();
                output_type_ = nullptr;
                buf[0].pSample  = nullptr;
                buf[0].dwStatus = MFT_OUTPUT_DATA_BUFFER_FORMAT_CHANGE;
                if (status) *status = MFT_OUTPUT_DATA_BUFFER_FORMAT_CHANGE;
                return MF_E_TRANSFORM_STREAM_CHANGE;
            }
        }
        if (vp9_out_queue_.empty()) {
            buf[0].pSample = nullptr;
            return MF_E_TRANSFORM_NEED_MORE_INPUT;
        }
        Vp9OutFrame frame = std::move(vp9_out_queue_.front());
        vp9_out_queue_.pop_front();

        /* 8-bit → NV12 (1 byte/sample); 10-bit → P010 (2 bytes/sample,
         * 10 valid bits in upper 10 — repack done in the engine via
         * RepackCodecOutputToNV12orP010).  All offsets keyed off the
         * head frame's dims; the STREAM_CHANGE block above guarantees
         * frame.{width,height} == width_/height_ here, but use the
         * frame fields for clarity so the upload bytes match what was
         * actually written into frame.yuv. */
        const bool   is_p010    = (frame.bit_depth == 10);
        const DWORD  bpp        = is_p010 ? 2u : 1u;
        const UINT32 fw         = frame.width;
        const UINT32 fh         = frame.height;
        const DWORD  row_bytes  = (DWORD)fw * bpp;
        const DWORD  y_bytes    = row_bytes * fh;
        const DWORD  plane_bytes = (DWORD)frame.yuv.size();
        IMFMediaBuffer  *mbuf    = nullptr;
        ID3D11Texture2D *texture = nullptr;
        HRESULT hr = S_OK;
        ID3D11Device *use_d3d = d3d_device_;

        if (use_d3d && fw && fh) {
            D3D11_TEXTURE2D_DESC td = {};
            td.Width = fw; td.Height = fh;
            td.MipLevels = 1; td.ArraySize = 1;
            td.Format = is_p010 ? DXGI_FORMAT_P010 : DXGI_FORMAT_NV12;
            td.SampleDesc.Count = 1;
            td.Usage = D3D11_USAGE_DEFAULT;
            td.BindFlags = D3D11_BIND_DECODER | D3D11_BIND_SHADER_RESOURCE;
            hr = d3d_device_->CreateTexture2D(&td, nullptr, &texture);
            if (FAILED(hr)) {
                td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
                hr = d3d_device_->CreateTexture2D(&td, nullptr, &texture);
                if (FAILED(hr)) return hr;
            }
            const UINT row_pitch = row_bytes;
            d3d_context_->UpdateSubresource(texture, 0, nullptr,
                                            frame.yuv.data(),
                                            row_pitch, row_pitch * fh);
            d3d_context_->UpdateSubresource(texture, 1, nullptr,
                                            frame.yuv.data() + (size_t)row_pitch * fh,
                                            row_pitch, row_pitch * (fh / 2));
            hr = MFCreateDXGISurfaceBuffer(IID_ID3D11Texture2D, texture,
                                           0, FALSE, &mbuf);
            if (FAILED(hr)) { texture->Release(); return hr; }
            mbuf->SetCurrentLength(plane_bytes);
        } else if (dxgi_manager_) {
            const DWORD fourcc = is_p010 ? MAKEFOURCC('P','0','1','0')
                                         : MAKEFOURCC('N','V','1','2');
            hr = MFCreate2DMediaBuffer(fw, fh, fourcc,
                                       FALSE, &mbuf);
            if (FAILED(hr)) return hr;
            IMF2DBuffer *buf2d = nullptr;
            hr = mbuf->QueryInterface(IID_PPV_ARGS(&buf2d));
            if (FAILED(hr) || !buf2d) {
                mbuf->Release();
                return FAILED(hr) ? hr : E_NOINTERFACE;
            }
            BYTE *dst = nullptr; LONG pitch = 0;
            hr = buf2d->Lock2D(&dst, &pitch);
            if (FAILED(hr)) { buf2d->Release(); mbuf->Release(); return hr; }
            const uint8_t *src_y  = frame.yuv.data();
            const uint8_t *src_uv = src_y + (size_t)y_bytes;
            for (UINT32 r = 0; r < fh; r++)
                std::memcpy(dst + (size_t)r * pitch,
                            src_y + (size_t)r * row_bytes, row_bytes);
            BYTE *dst_uv = dst + (size_t)pitch * fh;
            for (UINT32 r = 0; r < fh / 2; r++)
                std::memcpy(dst_uv + (size_t)r * pitch,
                            src_uv + (size_t)r * row_bytes, row_bytes);
            buf2d->Unlock2D();
            buf2d->Release();
            mbuf->SetCurrentLength(plane_bytes);
        } else {
            hr = MFCreateMemoryBuffer(plane_bytes, &mbuf);
            if (FAILED(hr)) return hr;
            BYTE *dst = nullptr; DWORD cap = 0, cur = 0;
            hr = mbuf->Lock(&dst, &cap, &cur);
            if (FAILED(hr)) { mbuf->Release(); return hr; }
            std::memcpy(dst, frame.yuv.data(), plane_bytes);
            mbuf->Unlock();
            mbuf->SetCurrentLength(plane_bytes);
        }

        IMFSample *out_sample = nullptr;
        hr = MFCreateSample(&out_sample);
        if (FAILED(hr)) {
            mbuf->Release();
            if (texture) texture->Release();
            return hr;
        }
        out_sample->AddBuffer(mbuf);
        mbuf->Release();
        if (texture) texture->Release();

        const uint32_t fpsn = fps_num_ ? fps_num_ : 30;
        const uint32_t fpsd = fps_den_ ? fps_den_ : 1;
        const LONGLONG dur = (LONGLONG)((10'000'000ULL * fpsd) / fpsn);
        const LONGLONG pts = (frame.pts_hns >= 0)
                           ? (LONGLONG)frame.pts_hns
                           : (LONGLONG)((uint64_t)frames_emitted_
                                        * 10'000'000ULL * fpsd / fpsn);
        out_sample->SetSampleTime(pts);
        out_sample->SetSampleDuration(dur);
        out_sample->SetUINT32(MFSampleExtension_CleanPoint, TRUE);

        buf[0].pSample  = out_sample;
        buf[0].dwStatus = 0;
        if (status) *status = 0;
        frames_emitted_++;
        MaybeLogFrameStats();
        (void)mft_t0; (void)mft_copy_us;
        return S_OK;
    }

    if (!engine_ || engine_init_failed_) {
        buf[0].pSample = nullptr;
        return E_FAIL;
    }

    auto *eng = static_cast<DecodeEngine *>(engine_);

    /* If we're draining and the engine still has unbumped frames,
     * spill them now so PollFrame can return them. */
    if (draining_) {
        DecodeEngine_Drain(eng);
    }

    DecodedFrame frame;
    int got = DecodeEngine_PollFrame(eng, &frame);
    if (got <= 0) {
        buf[0].pSample = nullptr;
        return MF_E_TRANSFORM_NEED_MORE_INPUT;
    }
    /* Guard against the engine handing us a frame whose yuv vector was
     * never populated (any code path that skips the kernel→vector
     * memcpy).  Memcpy below from an empty/undersized vector would
     * leave the IMFMediaBuffer's tail uninitialised, and Windows' heap
     * allocator would hand back a region that recently held a prior
     * IMFMediaBuffer — visible to EVR as a previous-frame flash.  Skip
     * emission instead so the worst case is a missed frame, not a
     * delivered-but-wrong frame.
     *
     * Derive bit_depth_ from the actual frame size the engine emitted
     * rather than checking against a precomputed expected size: MF's
     * topology resolver can call SetInputType multiple times during
     * negotiation, and a transient call without MF_MT_MPEG_SEQUENCE_HEADER
     * would reset bit_depth_ to 8 — without this self-correction, every
     * P010 frame would be rejected by the size guard.  The engine is
     * the source of truth for bit-depth (it parses the inline SPS on
     * every AU); resync bit_depth_ to match what it actually produced. */
    {
        const size_t nv12_sz = (size_t)width_ * height_ * 3u / 2u;
        const size_t p010_sz = nv12_sz * 2u;
        const size_t actual  = frame.yuv.size();
        if (actual == p010_sz) {
            bit_depth_ = 10;
        } else if (actual == nv12_sz) {
            bit_depth_ = 8;
        } else {
            std::fprintf(stderr,
                "rkmpp MFT: skipping output — yuv size %zu matches neither "
                "NV12 (%zu) nor P010 (%zu) (slot=%d poc=%d pts=%lld)\n",
                actual, nv12_sz, p010_sz, frame.slot_idx, frame.poc,
                (long long)frame.pts_hns);
            std::fflush(stderr);
            DecodeEngine_ReleaseFrame(eng, &frame);
            buf[0].pSample = nullptr;
            return MF_E_TRANSFORM_NEED_MORE_INPUT;
        }
    }

    /* IMFQualityAdvise drop-mode acknowledged but no longer acted on:
     * with cached buffers + AXI-drain stall in the kernel poller, the
     * per-frame budget at 1080p / 4K is comfortably under realtime,
     * so dropping non-refs would just halve our output rate and feed
     * EVR's quality manager a "you're still behind" signal — keeping
     * drop_mode pinned at 1 even when we're keeping up.  Best to emit
     * every decoded frame and let EVR clear drop_mode on its own. */

    IMFMediaBuffer *mbuf      = nullptr;
    ID3D11Texture2D *texture  = nullptr;
    HRESULT hr = S_OK;

    /* D3D11 output path: allocate a fresh NV12 ID3D11Texture2D per
     * sample, populate via UpdateSubresource, hand to EVR via
     * MFCreateDXGISurfaceBuffer.  No texture pool / no KeyedMutex sync
     * is needed because each frame gets its own texture; EVR holds the
     * sole reference via the IMFSample, and releases when done.  We
     * pay one CreateTexture2D per frame (~tens of µs on real GPU,
     * higher on WARP) but skip EVR's sysmem→GPU upload entirely.
     *
     * At 1440p / 4K with the sysmem path, EVR's render-side GPU
     * upload + stride-convert was the dominant remaining bottleneck
     * (CPU only ~15% busy, drop_mode pinned).  Going DXGI-direct
     * gives EVR a GPU-resident texture — its only work is the
     * NV12→RGB shader and present. */
    ID3D11Device *use_d3d = d3d_device_;
    /* One-shot log of which output-buffer mode actually fires.  Emits a
     * reason when on sysmem so the trace is self-explanatory:
     *   - "no SET_D3D_MANAGER" — host (e.g. mft_play) didn't inject a
     *     device manager into EVR's topology, so EVR never forwarded
     *     one to us.  Fix is on the host side, not in the MFT.
     *   - "QI/GVS failed"      — manager was sent but didn't expose
     *     ID3D11Device or even IMFDXGIDeviceManager.
     *   - file consumer        — non-EVR path (mft_decode → file). */
    {
        static bool logged = false;
        if (!logged) {
            if (use_d3d && width_ && height_) {
                std::fprintf(stderr,
                    "rkmpp MFT: output mode = D3D11_SURFACE_BUFFER\n");
            } else if (dxgi_manager_) {
                std::fprintf(stderr,
                    "rkmpp MFT: output mode = 2D_MEDIA_BUFFER (EVR, "
                    "D3D9-backed manager — no D3D11 device available)\n");
            } else {
                std::fprintf(stderr,
                    "rkmpp MFT: output mode = SYSMEM_1D_BUFFER "
                    "(no SET_D3D_MANAGER from host — EVR's GPU upload "
                    "still happens but in EVR-internal sysmem path)\n");
            }
            std::fflush(stderr);
            logged = true;
        }
    }

    /* P010 doubles row pitch + sample size vs NV12; subresource layout
     * (Y at 0, interleaved UV at 1) and pixel topology (4:2:0) are
     * otherwise identical, so the D3D11 / 2D-buffer / 1D-buffer paths
     * below all scale uniformly with bytes_per_sample. */
    const UINT bytes_per_sample = (bit_depth_ == 10) ? 2u : 1u;
    if (use_d3d && width_ && height_) {
        /* D3D11 output path: allocate an NV12/P010 ID3D11Texture2D, copy
         * the engine's frame into it via Map() (works on WARP and
         * on hypothetical real hardware alike since the decoder still
         * lives in CPU-allocated rkmpp DMA buffers), and wrap via
         * MFCreateDXGISurfaceBuffer.
         *
         * NV12 in D3D11 is a single texture with two subresource planes
         * (Y at index 0, UV at index 1) that share a Map() handle when
         * USAGE_STAGING is used; for plain DEFAULT we Map() each plane
         * separately. We use STAGING because the decoder data is in
         * system memory and downstream consumers will CopyResource()
         * onto a DEFAULT/SHADER texture themselves (or render directly
         * from STAGING via a CPU readback path on WARP). */
        D3D11_TEXTURE2D_DESC td = {};
        td.Width              = width_;
        td.Height             = height_;
        td.MipLevels          = 1;
        td.ArraySize          = 1;
        td.Format             = (bit_depth_ == 10) ? DXGI_FORMAT_P010 : DXGI_FORMAT_NV12;
        td.SampleDesc.Count   = 1;
        td.Usage              = D3D11_USAGE_DEFAULT;
        td.BindFlags          = D3D11_BIND_DECODER | D3D11_BIND_SHADER_RESOURCE;
        td.CPUAccessFlags     = 0;
        td.MiscFlags          = 0;
        hr = d3d_device_->CreateTexture2D(&td, nullptr, &texture);
        if (FAILED(hr)) {
            /* Fall back to SHADER_RESOURCE-only — required for WARP and
             * other non-video-service devices where BIND_DECODER is
             * unsupported. */
            td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
            hr = d3d_device_->CreateTexture2D(&td, nullptr, &texture);
            if (FAILED(hr)) return hr;
        }

        /* Stage upload via UpdateSubresource.  NV12/P010 in D3D11 has
         * TWO subresources per array slice: Y at index 0 (width × height,
         * row pitch = width × bytes_per_sample), UV at index 1
         * (width × height/2 interleaved CbCr).  Updating only subresource
         * 0 leaves the UV plane uninitialised → green frames in EVR
         * (chroma defaults to neutral, but the texture's uninit memory
         * shows as green/random on real GPUs).
         *
         * Source buffer layout: contiguous Y followed by UV at offset
         * width * height * bytes_per_sample. */
        const UINT row_pitch = width_ * bytes_per_sample;
        d3d_context_->UpdateSubresource(texture, 0, nullptr,
                                        frame.yuv.data(),
                                        row_pitch,
                                        row_pitch * height_);
        d3d_context_->UpdateSubresource(texture, 1, nullptr,
                                        frame.yuv.data() + (size_t)row_pitch * height_,
                                        row_pitch,
                                        row_pitch * (height_ / 2));

        hr = MFCreateDXGISurfaceBuffer(IID_ID3D11Texture2D, texture,
                                       0 /* subresource */,
                                       FALSE /* not bottom-up */,
                                       &mbuf);
        if (FAILED(hr)) { texture->Release(); return hr; }
        mbuf->SetCurrentLength((DWORD)frame.yuv.size());
    } else {
        /* System-memory output: plain 1D contiguous buffer with
         * stride == width.  We previously switched to MFCreate2DMediaBuffer
         * to give EVR an IMF2DBuffer with explicit pitch — but the 2D
         * buffer's pitch can be wider than width for hardware alignment,
         * which silently corrupts byte-by-byte consumers (mft_decode →
         * file → ffplay/PSNR) that assume packed NV12 at stride=width.
         * EVR honours MF_MT_DEFAULT_STRIDE on plain media buffers, so
         * 1D + correct attributes is the safer default; if EVR shows
         * scanline issues again we can re-introduce 2D for the
         * SET_D3D_MANAGER path only. */
        /* When EVR is the consumer (signalled by SET_D3D_MANAGER having
         * been called), use MFCreate2DMediaBuffer.  EVR then accesses
         * the buffer via IMF2DBuffer::Lock2D with the buffer's natural
         * pitch — saving EVR an internal stride-conversion sysmem copy
         * before its GPU upload.  For non-EVR consumers (mft_decode →
         * file → ffplay/PSNR), stick with MFCreateMemoryBuffer (packed
         * NV12 stride=width) since byte-exact bitstream comparison
         * relies on that layout.
         *
         * IMF2DBuffer pitch may exceed width_ for hardware alignment,
         * so the copy is row-by-row from our packed `frame.yuv` source
         * into the 2D buffer's possibly-padded layout. */
        const DWORD row_bytes = (DWORD)width_ * bytes_per_sample;
        const DWORD y_bytes   = row_bytes * height_;
        const DWORD uv_bytes  = row_bytes * (height_ / 2u);
        const DWORD plane_bytes = y_bytes + uv_bytes;
        const bool  use_2d   = (dxgi_manager_ != nullptr);
        if (use_2d) {
            const DWORD fourcc = (bit_depth_ == 10)
                ? MAKEFOURCC('P','0','1','0')
                : MAKEFOURCC('N','V','1','2');
            hr = MFCreate2DMediaBuffer(width_, height_, fourcc,
                                       FALSE /* top-down */,
                                       &mbuf);
            if (FAILED(hr)) return hr;
            IMF2DBuffer *buf2d = nullptr;
            hr = mbuf->QueryInterface(IID_PPV_ARGS(&buf2d));
            if (FAILED(hr) || !buf2d) { mbuf->Release(); return FAILED(hr)?hr:E_NOINTERFACE; }
            BYTE *dst = nullptr; LONG pitch = 0;
            hr = buf2d->Lock2D(&dst, &pitch);
            if (FAILED(hr)) { buf2d->Release(); mbuf->Release(); return hr; }
            int64_t cp_t0 = MftTimingEnabled() ? MftQpcNow() : 0;
            const uint8_t *src_y  = frame.yuv.data();
            const uint8_t *src_uv = src_y + (size_t)y_bytes;
            for (UINT32 r = 0; r < height_; r++) {
                std::memcpy(dst + (size_t)r * pitch, src_y + (size_t)r * row_bytes, row_bytes);
            }
            BYTE *dst_uv = dst + (size_t)pitch * height_;
            for (UINT32 r = 0; r < height_ / 2; r++) {
                std::memcpy(dst_uv + (size_t)r * pitch, src_uv + (size_t)r * row_bytes, row_bytes);
            }
            if (MftTimingEnabled()) mft_copy_us = MftQpcUs(cp_t0, MftQpcNow());
            buf2d->Unlock2D();
            buf2d->Release();
            mbuf->SetCurrentLength(plane_bytes);
        } else {
            hr = MFCreateMemoryBuffer(plane_bytes, &mbuf);
            if (FAILED(hr)) return hr;
            BYTE *dst = nullptr; DWORD cap = 0, cur = 0;
            hr = mbuf->Lock(&dst, &cap, &cur);
            if (FAILED(hr)) { mbuf->Release(); return hr; }
            int64_t cp_t0 = MftTimingEnabled() ? MftQpcNow() : 0;
            std::memcpy(dst, frame.yuv.data(), plane_bytes);
            if (MftTimingEnabled()) mft_copy_us = MftQpcUs(cp_t0, MftQpcNow());
            mbuf->Unlock();
            mbuf->SetCurrentLength(plane_bytes);
        }
    }

    IMFSample *out_sample = nullptr;
    hr = MFCreateSample(&out_sample);
    if (FAILED(hr)) {
        mbuf->Release();
        if (texture) texture->Release();
        return hr;
    }
    out_sample->AddBuffer(mbuf);
    mbuf->Release();
    if (texture) texture->Release();

    /* Release the DPB external-hold taken when this frame entered the
     * reorder window.  We've finished extracting the data into the
     * IMFMediaBuffer (sysmem) or D3D11 texture (the other branch above)
     * by this point, so the codec is free to reuse the slot. */
    DecodeEngine_ReleaseFrame(eng, &frame);

    /* PTS / duration: forward the container's per-sample PTS through
     * decode → display reorder → output.  The engine's reorder window
     * emits frames in POC ascending order (display order), so the
     * popped entry's pts_hns is the correct display-time PTS for that
     * frame.  Container PTS lets EVR's clock-paced rendering work
     * naturally — non-ref frames are skipped above (drop mode), refs
     * carry true content time, audio + video stay synced.  Falls back
     * to a synthetic monotonic stamp if the engine handed us pts<0. */
    const uint32_t fpsn = fps_num_ ? fps_num_ : 30;
    const uint32_t fpsd = fps_den_ ? fps_den_ : 1;
    const LONGLONG dur  = (LONGLONG)((10'000'000ULL * fpsd) / fpsn);
    LONGLONG pts;
    if (frame.pts_hns >= 0) {
        pts = (LONGLONG)frame.pts_hns;
    } else {
        pts = (LONGLONG)((uint64_t)frames_emitted_
                         * 10'000'000ULL * fpsd / fpsn);
    }

    /* Primary correctness fix: drop frames whose epoch is older than the
     * current stream epoch.  Epoch is bumped on FLUSH and tagged onto
     * every Submit; a stale-epoch frame here means decode work that was
     * in flight when the host called FLUSH completed after the flush
     * fired and ended up in ready_q with the OLD timeline's tag.
     * Forwarding it to EVR causes a "previous frame flash": EVR receives
     * a sample with a pre-flush pts that's now in the past relative to
     * the new timeline's clock, and presents it for a single vsync.
     *
     * Belt-and-suspenders: if epoch matches but pts moves backward by
     * more than half a frame, also drop.  This catches genuine
     * pts-discontinuity bugs that aren't accompanied by a flush. */
    if (frame.epoch != stream_epoch_) {
        std::fprintf(stderr,
            "rkmpp MFT: dropping stale-epoch sample — frame.epoch=%u "
            "stream_epoch=%u (slot=%d poc=%d pts=%lld) "
            "[survived FLUSH, would flash EVR]\n",
            frame.epoch, stream_epoch_, frame.slot_idx, frame.poc,
            (long long)pts);
        std::fflush(stderr);
        out_sample->Release();
        buf[0].pSample = nullptr;
        return MF_E_TRANSFORM_NEED_MORE_INPUT;
    }
    if (last_emitted_pts_ != INT64_MIN) {
        const LONGLONG half_frame = dur / 2;
        if (pts + half_frame < last_emitted_pts_) {
            std::fprintf(stderr,
                "rkmpp MFT: dropping pts-backward sample — pts=%lld "
                "< last=%lld (slot=%d poc=%d) [no flush observed]\n",
                (long long)pts, (long long)last_emitted_pts_,
                frame.slot_idx, frame.poc);
            std::fflush(stderr);
            out_sample->Release();
            buf[0].pSample = nullptr;
            return MF_E_TRANSFORM_NEED_MORE_INPUT;
        }
    }
    last_emitted_pts_ = pts;

    out_sample->SetSampleTime(pts);
    out_sample->SetSampleDuration(dur);

    /* Per-frame trace — gated by sentinel `mft_trace.flag` in CWD so it's
     * off by default but easy to enable for diagnosing visual artefacts
     * like the random previous-frame-flash on non-B-frame H.264.  Logs
     * frame index, slot, poc, pts, and a 32-bit hash of the first 4 KiB
     * of the YUV.  When user reports "flash at second N", correlate
     * against the log around that time: a repeated hash means we
     * delivered the same content twice; a fresh hash means EVR/sink
     * showed a stale buffer despite us delivering a unique one. */
    {
        static int trace_state = -1;  /* 0=off 1=on, lazy-init */
        if (trace_state < 0) {
            FILE *probe = nullptr;
            if (fopen_s(&probe, "mft_trace.flag", "rb") == 0 && probe) {
                trace_state = 1;
                fclose(probe);
            } else {
                trace_state = 0;
            }
        }
        if (trace_state == 1) {
            uint32_t h = 0x811c9dc5u;
            const uint8_t *p = frame.yuv.data();
            const size_t n = std::min<size_t>(frame.yuv.size(), 4096);
            for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
            std::fprintf(stderr,
                "rkmpp MFT TRACE: emit#%llu slot=%d poc=%d pts=%lld hash=%08x\n",
                (unsigned long long)frames_emitted_,
                frame.slot_idx, frame.poc, (long long)pts, h);
            std::fflush(stderr);
        }
    }

    /* Output samples are decoded NV12 — every frame is a valid clean
     * point for downstream renderers, regardless of source GOP
     * structure. EVR uses this attribute to decide where it can begin
     * presenting after a topology change; without it, it can stall
     * waiting for the first "true" clean point. */
    out_sample->SetUINT32(MFSampleExtension_CleanPoint, TRUE);

    buf[0].pSample  = out_sample;
    buf[0].dwStatus = 0;
    if (status) *status = 0;
    frames_emitted_++;

    MaybeLogFrameStats();

    if (MftTimingEnabled()) {
        static bool hdr = false;
        if (!hdr) {
            std::fprintf(stderr,
                "MFT_TIMING,frame,po_total_us,po_copy_us\n");
            hdr = true;
        }
        std::fprintf(stderr, "MFT_TIMING,%llu,%lld,%lld\n",
                     (unsigned long long)frames_emitted_,
                     (long long)MftQpcUs(mft_t0, MftQpcNow()),
                     (long long)mft_copy_us);
        std::fflush(stderr);
    }
    return S_OK;
}

void DecoderMFT::MaybeLogFrameStats() {
    /* Total frames the engine handed us this stream (emitted + dropped). */
    uint64_t total = frames_emitted_ + frames_skipped_dropmode_;
    if (total == 0 || (total % 15) != 0) return;
    std::fprintf(stderr,
                 "rkmpp MFT: total=%llu emitted=%llu skipped(drop)=%llu "
                 "decode_errors=%llu drop_mode=%d\n",
                 (unsigned long long)total,
                 (unsigned long long)frames_emitted_,
                 (unsigned long long)frames_skipped_dropmode_,
                 (unsigned long long)decode_errors_,
                 (int)drop_mode_);
    std::fflush(stderr);
}

/* ---------- IMFQualityAdvise --------------------------------------- */

STDMETHODIMP DecoderMFT::SetDropMode(MF_QUALITY_DROP_MODE eDropMode) {
    if ((int)eDropMode < (int)MF_DROP_MODE_NONE ||
        (int)eDropMode > (int)MF_DROP_MODE_5) {
        return MF_E_NO_MORE_DROP_MODES;
    }
    std::lock_guard<std::mutex> g(lock_);
    drop_mode_ = eDropMode;
    /* Acknowledge the request but don't propagate to the engine.
     * Engine-side skipping non-ref memcpys made the output rate halve
     * and kept EVR locked in drop_mode — see ProcessOutput comment. */
    return S_OK;
}

STDMETHODIMP DecoderMFT::SetQualityLevel(MF_QUALITY_LEVEL eQualityLevel) {
    /* We don't tune internal quality (the codec is fixed-function); accept
     * any level as a no-op so EVR doesn't think we lack the interface. */
    (void)eQualityLevel;
    return S_OK;
}

STDMETHODIMP DecoderMFT::GetDropMode(MF_QUALITY_DROP_MODE *peDropMode) {
    if (!peDropMode) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    *peDropMode = drop_mode_;
    return S_OK;
}

STDMETHODIMP DecoderMFT::GetQualityLevel(MF_QUALITY_LEVEL *peQualityLevel) {
    if (!peQualityLevel) return E_POINTER;
    *peQualityLevel = MF_QUALITY_NORMAL;
    return S_OK;
}

STDMETHODIMP DecoderMFT::DropTime(LONGLONG /*hnsAmountToDrop*/) {
    /* Time-based drop isn't supported — would require coordinating with
     * the engine's reorder window per timestamp, which we don't expose
     * yet.  EVR falls back to drop-mode-based throttling when this
     * returns NOT_SUPPORTED. */
    return MF_E_DROPTIME_NOT_SUPPORTED;
}

} /* namespace rkmpp */
