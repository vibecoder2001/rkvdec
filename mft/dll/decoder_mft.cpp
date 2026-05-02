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

/* The decode engine lives as a static lib in tests/harness/rkmpp_decode/
 * so harness binaries and this DLL share one user-mode pipeline.  Codec
 * mapping: rkmpp::CodecKind <-> ::Codec (separate enums, same intent). */
#include "../../tests/harness/rkmpp_decode/decode_engine.h"

namespace rkmpp {

static ::Codec ToEngineCodec(CodecKind k) {
    return (k == CodecKind::H264) ? ::Codec::H264 : ::Codec::H265;
}

long g_dll_lock_count = 0;

/* ---------- codec-table helpers ---------------------------------- */

const wchar_t *DecoderFriendlyName(CodecKind k) {
    return (k == CodecKind::H264)
        ? L"Rockchip RK3588 H.264 Decoder"
        : L"Rockchip RK3588 HEVC Decoder";
}
const GUID &DecoderClsid(CodecKind k) {
    return (k == CodecKind::H264) ? CLSID_RkmppH264Decoder
                                  : CLSID_RkmppHevcDecoder;
}
const GUID &DecoderInputSubtype(CodecKind k) {
    return (k == CodecKind::H264) ? MFVideoFormat_H264
                                  : MFVideoFormat_HEVC;
}

/* ---------- ctor / dtor ------------------------------------------ */

DecoderMFT::DecoderMFT(CodecKind kind)
    : kind_(kind), refs_(1) {
    DllAddRef();
}

DecoderMFT::~DecoderMFT() {
    if (input_type_)  input_type_->Release();
    if (output_type_) output_type_->Release();
    if (engine_) {
        auto *eng = static_cast<DecodeEngine *>(engine_);
        DecodeEngine_Shutdown(eng);
        delete eng;
        engine_ = nullptr;
    }
    ReleaseD3DManager();
    if (attributes_) { attributes_->Release(); attributes_ = nullptr; }
    DllRelease();
}

void DecoderMFT::ReleaseD3DManager() {
    if (d3d_context_) { d3d_context_->Release(); d3d_context_ = nullptr; }
    if (d3d_device_)  { d3d_device_->Release();  d3d_device_  = nullptr; }
    if (dxgi_manager_ && dxgi_device_h_) {
        dxgi_manager_->CloseDeviceHandle(dxgi_device_h_);
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
    /* NV12 = width*height*3/2 once dimensions known, else 0. */
    p->cbSize      = (width_ && height_) ? (width_ * height_ * 3 / 2) : 0;
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
    *flags = MFT_INPUT_STATUS_ACCEPT_DATA;
    return S_OK;
}
STDMETHODIMP DecoderMFT::GetOutputStatus(DWORD *flags) {
    if (!flags) return E_POINTER;
    /* Phase 2A: never have a sample ready. */
    *flags = 0;
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
    UINT32 hdr_len = 0;
    if (SUCCEEDED(type->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER, &hdr_len)) && hdr_len > 0) {
        std::vector<uint8_t> hdr(hdr_len);
        if (FAILED(type->GetBlob(MF_MT_MPEG_SEQUENCE_HEADER, hdr.data(), hdr_len, nullptr)))
            return MF_E_INVALIDMEDIATYPE;

        /* Parse into a temporary instance so we can roll back on failure. */
        DecoderMFT tmp(kind_);
        HRESULT hr = (kind_ == CodecKind::H264)
            ? tmp.ParseAvcCExtradata(hdr.data(), hdr.size())
            : tmp.ParseHvcCExtradata(hdr.data(), hdr.size());
        if (FAILED(hr)) {
            /* Don't reject the type just because extradata is malformed —
             * many sources put SPS/PPS inline. Phase 2B will re-validate
             * once it sees the first slice. Log via debug only. */
        } else {
            annexb     = std::move(tmp.extradata_annexb_);
            length_size = tmp.length_size_;
        }
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
    return S_OK;
}

HRESULT DecoderMFT::BuildOutputType(IMFMediaType **pp) {
    IMFMediaType *t = nullptr;
    HRESULT hr = MFCreateMediaType(&t);
    if (FAILED(hr)) return hr;
    hr = t->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    if (SUCCEEDED(hr)) hr = t->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12);
    if (SUCCEEDED(hr)) hr = MFSetAttributeSize(t, MF_MT_FRAME_SIZE, width_, height_);
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(t, MF_MT_FRAME_RATE, fps_num_, fps_den_);
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(t, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    /* Without DEFAULT_STRIDE/SAMPLE_SIZE/etc EVR computes a default for
     * NV12 that doesn't match our packed layout (width-aligned, no
     * padding) and ends up reading UV from the wrong offsets — UV reads
     * as zeros from the trailing portion of the buffer, producing green
     * frames with partial Y bleed. Pin all the layout attributes
     * EVR/sink stack expects. */
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_DEFAULT_STRIDE, (UINT32)width_);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_SAMPLE_SIZE,
                                         (UINT32)(width_ * height_ * 3 / 2));
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    if (SUCCEEDED(hr)) hr = t->SetUINT32(MF_MT_VIDEO_NOMINAL_RANGE,
                                         MFNominalRange_16_235);
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
    if (FAILED(type->GetGUID(MF_MT_SUBTYPE, &sub)) || sub != MFVideoFormat_NV12)
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
        if (FAILED(hr)) return hr;
        HANDLE h = nullptr;
        hr = mgr->OpenDeviceHandle(&h);
        if (FAILED(hr)) { mgr->Release(); return hr; }
        ID3D11Device *dev = nullptr;
        hr = mgr->GetVideoService(h, IID_PPV_ARGS(&dev));
        if (FAILED(hr)) { mgr->CloseDeviceHandle(h); mgr->Release(); return hr; }
        ID3D11DeviceContext *ctx = nullptr;
        dev->GetImmediateContext(&ctx);
        dxgi_manager_  = mgr;
        dxgi_device_h_ = h;
        d3d_device_    = dev;
        d3d_context_   = ctx;
        return S_OK;
    }
    case MFT_MESSAGE_NOTIFY_BEGIN_STREAMING: {
        if (streaming_) return S_OK;
        if (!input_type_ || !output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
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
        engine_init_failed_ = false;
        streaming_ = false;
        input_queue_.clear();
        input_timestamps_.clear();
        return S_OK;
    case MFT_MESSAGE_COMMAND_DRAIN:
        draining_ = true;
        return S_OK;
    case MFT_MESSAGE_COMMAND_FLUSH:
        input_queue_.clear();
        input_timestamps_.clear();
        draining_ = false;
        if (engine_) {
            auto *eng = static_cast<DecodeEngine *>(engine_);
            (void)DecodeEngine_Flush(eng);
        }
        return S_OK;
    case MFT_MESSAGE_NOTIFY_START_OF_STREAM:
    case MFT_MESSAGE_NOTIFY_END_OF_STREAM:
    case MFT_MESSAGE_NOTIFY_RELEASE_RESOURCES:
    case MFT_MESSAGE_NOTIFY_REACQUIRE_RESOURCES:
    default:
        return S_OK;
    }
}

STDMETHODIMP DecoderMFT::ProcessInput(DWORD id, IMFSample *sample, DWORD /*flags*/) {
    if (id != 0) return MF_E_INVALIDSTREAMNUMBER;
    if (!sample) return E_POINTER;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_ || !output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    if (!streaming_)                    return MF_E_TRANSFORM_TYPE_NOT_SET;
    if (!engine_ || engine_init_failed_) return MF_E_NOTACCEPTING;

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
            au.insert(au.end(), p, p + cur);
            buf->Unlock();
        }
        buf->Release();
        if (FAILED(hr)) return hr;
    }
    samples_received_++;

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
                                       (int64_t)pts);
    if (rc != 0) {
        decode_errors_++;
        /* Don't fail the whole pipeline; report success but produce
         * nothing — the host calls ProcessOutput separately. */
    }
    (void)dur;
    return S_OK;
}

STDMETHODIMP DecoderMFT::ProcessOutput(DWORD /*flags*/, DWORD c,
                                       MFT_OUTPUT_DATA_BUFFER *buf,
                                       DWORD *status) {
    if (c != 1 || !buf) return E_INVALIDARG;
    if (status) *status = 0;
    buf[0].dwStatus = 0;
    std::lock_guard<std::mutex> g(lock_);
    if (!input_type_ || !output_type_) return MF_E_TRANSFORM_TYPE_NOT_SET;
    if (buf[0].pSample) return E_FAIL;
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

    IMFMediaBuffer *mbuf      = nullptr;
    ID3D11Texture2D *texture  = nullptr;
    HRESULT hr = S_OK;

    /* D3D11 output path is currently disabled: it requires a texture
     * pool with cross-context sync (KeyedMutex / fences) to avoid GPU/
     * CPU races between our UpdateSubresource and EVR's present.
     * Without that, real-world streams show green/grayscale patches,
     * scanline tears, and judder.  We still accept SET_D3D_MANAGER so
     * EVR doesn't refuse the topology, but always emit a sysmem buffer
     * — EVR uploads it into its own GPU layout with proper sync.
     * Re-enable once we wire a proper sample-allocator with fences. */
    /* D3D11 output path is gated off — see comment above.  Force the
     * sysmem branch by clearing this on entry; we still hold d3d_device_
     * elsewhere if we ever re-enable. */
    ID3D11Device *use_d3d = nullptr;
    if (use_d3d && width_ && height_) {
        /* D3D11 output path: allocate an NV12 ID3D11Texture2D, copy
         * the engine's NV12 frame into it via Map() (works on WARP and
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
        td.Format             = DXGI_FORMAT_NV12;
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

        /* Stage upload via UpdateSubresource.  NV12 in D3D11 has TWO
         * subresources per array slice: Y at index 0 (width × height,
         * row pitch = width), UV at index 1 (width × height/2 with
         * interleaved CbCr, row pitch = width).  Updating only
         * subresource 0 leaves the UV plane uninitialised → green
         * frames in EVR (chroma defaults to neutral, but the texture's
         * uninit memory shows as green/random on real GPUs).
         *
         * Source buffer layout: contiguous Y followed by UV at offset
         * width * height. */
        const UINT row_pitch = width_;
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
        hr = MFCreateMemoryBuffer((DWORD)frame.yuv.size(), &mbuf);
        if (FAILED(hr)) return hr;
        BYTE *dst = nullptr; DWORD cap = 0, cur = 0;
        hr = mbuf->Lock(&dst, &cap, &cur);
        if (FAILED(hr)) { mbuf->Release(); return hr; }
        std::memcpy(dst, frame.yuv.data(), frame.yuv.size());
        mbuf->Unlock();
        mbuf->SetCurrentLength((DWORD)frame.yuv.size());
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

    /* PTS / duration: the engine's reorder window emits frames in
     * display order, but the per-frame pts forwarded from ProcessInput
     * is the source's *decode-order* PTS — handing those to EVR causes
     * out-of-order timestamps on B-frame streams and the renderer
     * silently drops samples. Use a monotonic synthetic PTS based on
     * the negotiated frame rate, which is what every other software
     * MFT does for decode-then-render scenarios. */
    const uint32_t fpsn = fps_num_ ? fps_num_ : 30;
    const uint32_t fpsd = fps_den_ ? fps_den_ : 1;
    const LONGLONG dur  = (LONGLONG)((10'000'000ULL * fpsd) / fpsn);
    const LONGLONG pts  = (LONGLONG)((uint64_t)frames_emitted_
                                     * 10'000'000ULL * fpsd / fpsn);
    out_sample->SetSampleTime(pts);
    out_sample->SetSampleDuration(dur);
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
    return S_OK;
}

} /* namespace rkmpp */
