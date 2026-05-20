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
#include <deque>
#include <mutex>
#include <vector>

#include "guids.h"

namespace rkmpp {

enum class CodecKind {
    H264,
    HEVC,
    AV1,
    VP9,
};

/* DLL-wide lock count for DllCanUnloadNow. */
extern long g_dll_lock_count;
inline void DllAddRef() { InterlockedIncrement(&g_dll_lock_count); }
inline void DllRelease() { InterlockedDecrement(&g_dll_lock_count); }

class DecoderMFT : public IMFTransform, public IMFQualityAdvise {
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

    /* IMFQualityAdvise — EVR's quality manager calls SetDropMode when it
     * sees the decoder falling behind audio.  We honor it by draining
     * surplus frames from the engine's reorder window without copying
     * them into output samples — gives EVR a chance to catch up without
     * needing to drop our output samples on the rendering side, which
     * would otherwise look like fast-forward scrubbing. */
    STDMETHODIMP SetDropMode(MF_QUALITY_DROP_MODE eDropMode) override;
    STDMETHODIMP SetQualityLevel(MF_QUALITY_LEVEL eQualityLevel) override;
    STDMETHODIMP GetDropMode(MF_QUALITY_DROP_MODE *peDropMode) override;
    STDMETHODIMP GetQualityLevel(MF_QUALITY_LEVEL *peQualityLevel) override;
    STDMETHODIMP DropTime(LONGLONG hnsAmountToDrop) override;

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

    /* Colour metadata to advertise on the output media type.  Set in
     * SetInputType from (a) input-type attributes if upstream demuxer
     * supplied them, (b) parsed H.264 VUI in the avcC SPS otherwise,
     * (c) resolution-based defaults (HD → BT.709, SD → BT.601, limited
     * range) as a last resort.  Without these, mpv / EVR fall back to
     * a hardcoded BT.601 default that mis-renders BT.709 content as
     * shifted hue / "all colors wrong but image recognizable". */
    UINT32      yuv_matrix_         = 0; /* MFVideoTransferMatrix_* */
    UINT32      video_primaries_    = 0; /* MFVideoPrimaries_*      */
    UINT32      transfer_function_  = 0; /* MFVideoTransFunc_*      */
    UINT32      nominal_range_      = 0; /* MFNominalRange_*        */

    /* Active bit-depth from the parsed SPS (8 or 10), captured in
     * SetInputType from the avcC SPS (or in ProcessInput on first slice
     * when the stream carries inline SPS).  Drives BuildOutputType's
     * NV12-vs-P010 subtype choice + sample-size calc.  Defaults to 8 so
     * unset streams keep the legacy NV12 output path. */
    UINT32      bit_depth_          = 8;

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

    /* One-shot bitstream dumper for offline-replay debugging.  Active
     * only while the sentinel file "mft_dump.flag" exists in the
     * working directory at ProcessInput time of the first sample.
     * Appends a fixed header plus length-prefixed AU records to
     * `mft_input_dump.bin` and caps at kDumpBytesMax to avoid filling
     * the disk on long playbacks.  See implementation in
     * decoder_mft.cpp for the on-disk format. */
    FILE       *dump_file_       = nullptr;
    bool        dump_checked_    = false;     /* sentinel probed once */
    size_t      dump_bytes_      = 0;
    static constexpr size_t kDumpBytesMax = 50ULL * 1024ULL * 1024ULL;
    /* Per-instance "output mode banner already logged" flag — previously
     * file-scope `static bool logged_av1/_h264/_hevc/_vp9`, which was
     * stale across instances (an H.264 ProcessOutput later in the same
     * process wouldn't print its banner because the AV1 path had
     * already flipped the flag).  Review MFT #25. */
    bool        output_mode_logged_ = false;

    /* Write one input AU to the dumper if active.  No-op on first call
     * if sentinel "mft_dump.flag" is absent, or once the cap is hit.
     * The on-disk format is documented in decoder_mft.cpp at the
     * implementation site. */
    void DumpAuIfActive(const uint8_t *au, size_t au_len, int64_t pts_hns);
    uint64_t    frames_emitted_   = 0;
    uint64_t    decode_errors_    = 0;
    /* Frames popped from the engine but skipped at emit time — tracked
     * per drop-mode reason so the periodic stats line can show what's
     * actually getting filtered out. */
    uint64_t    frames_skipped_dropmode_ = 0;

    /* Last pts we delivered to the consumer.  Used in ProcessOutput to
     * detect "rogue" out-of-order samples (pts moves backward without
     * a FLUSH having reset state) — we drop those rather than emit them
     * to EVR, which would otherwise present each backward-pts frame for
     * a single tick (the canonical "previous frame flash" symptom on
     * MP4 looping / mid-stream seek). INT64_MIN until first emit; reset
     * on FLUSH so a legitimate seek/loop doesn't get penalised.  This
     * is the safety-belt; the primary correctness fix is `stream_epoch_`
     * (below). */
    int64_t     last_emitted_pts_ = INT64_MIN;
    /* Stream-epoch counter — incremented on every FLUSH (and any other
     * timeline reset).  Tagged onto every Submit; the engine forwards
     * the tag to ReorderEntry → DecodedFrame.  ProcessOutput drops any
     * frame whose epoch is older than `stream_epoch_`, so decode results
     * that survived a flush (because they were already in flight when
     * FLUSH fired) get rejected rather than presented on the new
     * timeline.  This is what catches the genuine race; the pts-monotonic
     * guard is a secondary belt-and-suspenders. */
    uint32_t    stream_epoch_ = 0;

    bool        streaming_     = false;
    bool        draining_      = false;
    /* IMFQualityAdvise drop mode — set by EVR's quality manager when we
     * fall behind.  Used in ProcessOutput to drain surplus reorder-queue
     * frames without doing the per-frame sysmem copy, keeping us paced
     * with the audio clock. */
    MF_QUALITY_DROP_MODE drop_mode_ = MF_DROP_MODE_NONE;
    /* Set if BEGIN_STREAMING tried to init the engine and failed (e.g.
     * no rkmpp.sys present).  Type-negotiation tests still succeed; the
     * first ProcessInput surfaces MF_E_NOTACCEPTING. */
    bool        engine_init_failed_ = false;

    /* Engine handle — DecodeEngine* allocated in BEGIN_STREAMING and
     * destroyed in END_STREAMING / ~DecoderMFT.  H.264 / HEVC only. */
    void       *engine_ = nullptr;

    /* Av1DecodeEngine* for kind_ == AV1.  Software mode by default
     * (dav1d-decoded NV12 output) until the rkmpp.sys AV1 personality
     * lands; then Hardware mode kicks the codec. */
    void       *engine_av1_ = nullptr;

    /* Vp9DecodeEngine* for kind_ == VP9. */
    void       *engine_vp9_ = nullptr;

    /* Decoded VP9 NV12 frames awaiting emit.  VP9 engine is synchronous
     * (DecodeOne in/out per superframe subframe); ProcessInput pushes,
     * ProcessOutput pops. */
    struct Vp9OutFrame {
        std::vector<uint8_t> yuv;
        int64_t  pts_hns   = 0;
        uint32_t width     = 0;
        uint32_t height    = 0;
        uint8_t  bit_depth = 8;     /* 8 → NV12, 10 → P010 */
    };
    std::deque<Vp9OutFrame> vp9_out_queue_;

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

    /* Log emitted/skipped/error counts to stderr every 15 frames seen
     * (emitted + drop-skipped).  Called from ProcessOutput on both
     * emit and skip paths; throttled by an internal modulo counter. */
    void    MaybeLogFrameStats();
};

/* Codec-specific friendly names + subtypes used by registration.cpp. */
const wchar_t *DecoderFriendlyName(CodecKind k);
const GUID    &DecoderClsid(CodecKind k);
const GUID    &DecoderInputSubtype(CodecKind k);

} /* namespace rkmpp */
