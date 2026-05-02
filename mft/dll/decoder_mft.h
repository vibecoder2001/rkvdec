/* mft/dll/decoder_mft.h — IMFTransform implementation for rkvdec.
 *
 * Single-input, single-output sync MFT.  Two codec personalities
 * (H.264, HEVC) selected at construction time by the class factory.
 *
 * Phase 2A scope: COM scaffolding + type negotiation + extradata
 * (avcC / hvcC) parsing.  ProcessInput just queues; ProcessOutput
 * always returns NEED_MORE_INPUT.  Phase 2B will wire the actual
 * decode loop into the pre-existing parser_glue → dpb → packed_tables
 * → regbuilder → decode_engine pipeline.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once

#include <windows.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mftransform.h>
#include <d3d11.h>

struct IMFDXGIDeviceManager;

#include <cstdint>
#include <mutex>
#include <vector>

#include "guids.h"

namespace rkmpp {

enum class CodecKind {
    H264,
    HEVC,
};

/* DLL-wide lock count for DllCanUnloadNow. */
extern long g_dll_lock_count;
inline void DllAddRef() { InterlockedIncrement(&g_dll_lock_count); }
inline void DllRelease() { InterlockedDecrement(&g_dll_lock_count); }

class DecoderMFT : public IMFTransform {
public:
    explicit DecoderMFT(CodecKind kind);
    ~DecoderMFT();

    /* IUnknown */
    STDMETHODIMP         QueryInterface(REFIID iid, void **ppv) override;
    STDMETHODIMP_(ULONG) AddRef()  override;
    STDMETHODIMP_(ULONG) Release() override;

    /* IMFTransform — stream layout */
    STDMETHODIMP GetStreamLimits(DWORD *pdwMin_in, DWORD *pdwMax_in,
                                 DWORD *pdwMin_out, DWORD *pdwMax_out) override;
    STDMETHODIMP GetStreamCount(DWORD *pcInput, DWORD *pcOutput) override;
    STDMETHODIMP GetStreamIDs(DWORD nIn, DWORD *pdwIn,
                              DWORD nOut, DWORD *pdwOut) override;
    STDMETHODIMP GetInputStreamInfo(DWORD id, MFT_INPUT_STREAM_INFO *pInfo) override;
    STDMETHODIMP GetOutputStreamInfo(DWORD id, MFT_OUTPUT_STREAM_INFO *pInfo) override;

    /* IMFTransform — attributes (all NOTIMPL: we don't expose any) */
    STDMETHODIMP GetAttributes(IMFAttributes **pp) override;
    STDMETHODIMP GetInputStreamAttributes(DWORD id, IMFAttributes **pp) override;
    STDMETHODIMP GetOutputStreamAttributes(DWORD id, IMFAttributes **pp) override;
    STDMETHODIMP DeleteInputStream(DWORD id) override;
    STDMETHODIMP AddInputStreams(DWORD c, DWORD *ids) override;

    /* IMFTransform — type negotiation */
    STDMETHODIMP GetInputAvailableType(DWORD id, DWORD idx, IMFMediaType **pp) override;
    STDMETHODIMP GetOutputAvailableType(DWORD id, DWORD idx, IMFMediaType **pp) override;
    STDMETHODIMP SetInputType(DWORD id, IMFMediaType *type, DWORD flags) override;
    STDMETHODIMP SetOutputType(DWORD id, IMFMediaType *type, DWORD flags) override;
    STDMETHODIMP GetInputCurrentType(DWORD id, IMFMediaType **pp) override;
    STDMETHODIMP GetOutputCurrentType(DWORD id, IMFMediaType **pp) override;

    /* IMFTransform — status / messages */
    STDMETHODIMP GetInputStatus(DWORD id, DWORD *pdwFlags) override;
    STDMETHODIMP GetOutputStatus(DWORD *pdwFlags) override;
    STDMETHODIMP SetOutputBounds(LONGLONG lower, LONGLONG upper) override;
    STDMETHODIMP ProcessEvent(DWORD id, IMFMediaEvent *ev) override;
    STDMETHODIMP ProcessMessage(MFT_MESSAGE_TYPE msg, ULONG_PTR param) override;
    STDMETHODIMP ProcessInput(DWORD id, IMFSample *sample, DWORD flags) override;
    STDMETHODIMP ProcessOutput(DWORD flags, DWORD c,
                               MFT_OUTPUT_DATA_BUFFER *buf,
                               DWORD *status) override;

private:
    /* avcC / hvcC parsing — extracts SPS/PPS/(VPS) and emits a single
     * Annex-B blob suitable for prepending to the first slice. */
    HRESULT ParseAvcCExtradata(const uint8_t *blob, size_t len);
    HRESULT ParseHvcCExtradata(const uint8_t *blob, size_t len);

    /* Construct the canonical NV12 output type bound to the current
     * input dimensions / framerate.  Caller AddRef's the returned type. */
    HRESULT BuildOutputType(IMFMediaType **pp);

    CodecKind   kind_;
    long        refs_;
    std::mutex  lock_;

    /* Stored types (AddRef'd). */
    IMFMediaType *input_type_  = nullptr;
    IMFMediaType *output_type_ = nullptr;

    /* Negotiated dims / framerate from the input type. */
    UINT32      width_       = 0;
    UINT32      height_      = 0;
    UINT32      fps_num_     = 30;
    UINT32      fps_den_     = 1;

    /* Extradata: parsed Annex-B SPS/PPS(/VPS) blob to prepend on the
     * first slice. Phase 2B will hand this to DecodeEngine_Init or
     * push it through parser_glue. */
    std::vector<uint8_t> extradata_annexb_;
    /* From avcC/hvcC header. Engine uses this to frame per-sample
     * AVCC NALs. Default 4 (most common). */
    uint8_t     length_size_ = 4;

    /* Phase 2B queue of input AUs (raw container payload bytes); pop,
     * AVCC→Annex-B convert, and feed to the engine on ProcessOutput. */
    std::vector<std::vector<uint8_t>> input_queue_;
    /* Parallel queue of (pts, duration) in 100ns units (HNS).  Indices
     * line up with input_queue_; popped together. */
    std::vector<std::pair<int64_t, int64_t>> input_timestamps_;
    /* Hard cap on the input queue — sync MFT shouldn't queue forever.
     * 8 is plenty for a single-input/single-output decoder; the caller
     * is expected to drain ProcessOutput between ProcessInput calls. */
    static constexpr size_t kInputQueueCap = 8;
    uint64_t    samples_received_ = 0;
    uint64_t    frames_emitted_   = 0;
    uint64_t    decode_errors_    = 0;

    bool        streaming_     = false;
    bool        draining_      = false;
    /* Set if BEGIN_STREAMING tried to init the engine and failed (e.g.
     * no rkmpp.sys present).  Type-negotiation tests still succeed; the
     * first ProcessInput surfaces MF_E_NOTACCEPTING. */
    bool        engine_init_failed_ = false;

    /* Engine handle — DecodeEngine* allocated in BEGIN_STREAMING and
     * destroyed in END_STREAMING / ~DecoderMFT. */
    void       *engine_ = nullptr;

    /* D3D11 output state.  Populated by MFT_MESSAGE_SET_D3D_MANAGER.
     * When d3d_device_ is non-null, ProcessOutput emits NV12 textures
     * via MFCreateDXGISurfaceBuffer; otherwise it falls back to a
     * system-memory IMFMediaBuffer.  attributes_ is the lazy store
     * backing GetAttributes (advertises MF_SA_D3D11_AWARE). */
    IMFAttributes        *attributes_       = nullptr;
    IMFDXGIDeviceManager *dxgi_manager_     = nullptr;
    HANDLE                dxgi_device_h_    = nullptr;
    ID3D11Device         *d3d_device_      = nullptr;
    ID3D11DeviceContext  *d3d_context_     = nullptr;

    HRESULT EnsureAttributes();
    void    ReleaseD3DManager();
};

/* Codec-specific friendly names + subtypes used by registration.cpp. */
const wchar_t *DecoderFriendlyName(CodecKind k);
const GUID    &DecoderClsid(CodecKind k);
const GUID    &DecoderInputSubtype(CodecKind k);

} /* namespace rkmpp */
