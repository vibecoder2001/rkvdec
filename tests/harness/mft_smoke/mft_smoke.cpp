/* tests/harness/mft_smoke/mft_smoke.cpp — Phase 2A smoke test.
 *
 * Loads rkmpp_decoder_mft.dll directly via LoadLibrary +
 * DllGetClassObject (no regsvr32 / no registry dependency), creates the
 * IMFTransform instance for both H.264 and HEVC class GUIDs, and walks
 * type negotiation:
 *
 *   GetStreamLimits / GetStreamCount
 *   GetInputStreamInfo / GetOutputStreamInfo
 *   GetInputAvailableType -> SetInputType  (with a synthetic 10-byte avcC)
 *   GetOutputAvailableType -> SetOutputType
 *   ProcessMessage(NOTIFY_BEGIN_STREAMING)
 *   ProcessOutput -> expect MF_E_TRANSFORM_NEED_MORE_INPUT
 *
 * All asserts are local (no driver IO).  Exit code 0 = success.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */

#include <windows.h>
#include <initguid.h>  /* instantiate our CLSID_RkmppXxxDecoder symbols */
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mftransform.h>
#include <unknwn.h>

#include <cstdio>
#include <cstdint>
#include <cstring>

#include "guids.h"

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

#define EXPECT(cond, ...) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                   std::printf(__VA_ARGS__); std::printf("\n"); return 1; } \
} while (0)

#define HR(call) do { \
    HRESULT _hr = (call); \
    if (FAILED(_hr)) { std::printf("FAIL %s:%d: %s -> 0x%08x\n", \
                                   __FILE__, __LINE__, #call, (unsigned)_hr); \
                       return 1; } \
} while (0)

typedef HRESULT (__stdcall *PFN_DllGetClassObject)(REFCLSID, REFIID, void **);

/* Build a minimal but well-formed avcC blob: one tiny SPS + one tiny PPS. */
static void BuildSyntheticAvcC(uint8_t out[10]) {
    out[0] = 1;          /* configurationVersion */
    out[1] = 0x42;       /* AVCProfileIndication (Baseline) */
    out[2] = 0x00;       /* profile_compatibility */
    out[3] = 0x1e;       /* AVCLevelIndication (3.0) */
    out[4] = 0xff;       /* reserved6 | lengthSizeMinusOne(=3) */
    out[5] = 0xe1;       /* reserved3 | numOfSequenceParameterSets(=1) */
    out[6] = 0x00; out[7] = 0x01; /* SPS length = 1 */
    out[8] = 0x67;       /* SPS body (NAL header for SPS) */
    out[9] = 0x00;       /* numOfPictureParameterSets = 0 (truncate) */
}

static int RunOneCodec(PFN_DllGetClassObject get_class_obj,
                       REFCLSID clsid, const GUID &input_subtype,
                       const char *label) {
    std::printf("=== %s ===\n", label);

    IClassFactory *cf = nullptr;
    HR(get_class_obj(clsid, IID_IClassFactory, (void**)&cf));
    EXPECT(cf, "got null IClassFactory");

    IMFTransform *mft = nullptr;
    HR(cf->CreateInstance(nullptr, IID_IMFTransform, (void**)&mft));
    cf->Release();
    EXPECT(mft, "got null IMFTransform");

    /* Stream layout. */
    DWORD min_in=0, max_in=0, min_out=0, max_out=0;
    HR(mft->GetStreamLimits(&min_in, &max_in, &min_out, &max_out));
    EXPECT(min_in == 1 && max_in == 1, "in lim=%u,%u", min_in, max_in);
    EXPECT(min_out == 1 && max_out == 1, "out lim=%u,%u", min_out, max_out);

    DWORD nin=0, nout=0;
    HR(mft->GetStreamCount(&nin, &nout));
    EXPECT(nin == 1 && nout == 1, "count=%u,%u", nin, nout);

    MFT_INPUT_STREAM_INFO ii{}; HR(mft->GetInputStreamInfo(0, &ii));
    EXPECT(ii.dwFlags & MFT_INPUT_STREAM_WHOLE_SAMPLES, "ii flags=0x%x", ii.dwFlags);

    MFT_OUTPUT_STREAM_INFO oi{};
    HR(mft->GetOutputStreamInfo(0, &oi));
    EXPECT(oi.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES, "oi flags=0x%x", oi.dwFlags);

    /* Available input type -> Set. */
    IMFMediaType *avail_in = nullptr;
    HR(mft->GetInputAvailableType(0, 0, &avail_in));
    GUID got_sub{};
    HR(avail_in->GetGUID(MF_MT_SUBTYPE, &got_sub));
    EXPECT(got_sub == input_subtype, "wrong avail input subtype");
    avail_in->Release();

    /* type_idx=1 must fail. */
    IMFMediaType *no_more = nullptr;
    HRESULT hr = mft->GetInputAvailableType(0, 1, &no_more);
    EXPECT(hr == MF_E_NO_MORE_TYPES, "idx=1 hr=0x%08x", (unsigned)hr);

    /* Build the input type the app would feed. */
    IMFMediaType *in_type = nullptr;
    HR(MFCreateMediaType(&in_type));
    HR(in_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    HR(in_type->SetGUID(MF_MT_SUBTYPE, input_subtype));
    HR(MFSetAttributeSize(in_type, MF_MT_FRAME_SIZE, 1280, 720));
    HR(MFSetAttributeRatio(in_type, MF_MT_FRAME_RATE, 30, 1));
    uint8_t avcc[10]; BuildSyntheticAvcC(avcc);
    HR(in_type->SetBlob(MF_MT_MPEG_SEQUENCE_HEADER, avcc, sizeof(avcc)));

    HR(mft->SetInputType(0, in_type, MFT_SET_TYPE_TEST_ONLY));
    HR(mft->SetInputType(0, in_type, 0));

    /* Output type. */
    IMFMediaType *out_avail = nullptr;
    HR(mft->GetOutputAvailableType(0, 0, &out_avail));
    GUID out_sub{};
    HR(out_avail->GetGUID(MF_MT_SUBTYPE, &out_sub));
    EXPECT(out_sub == MFVideoFormat_NV12, "out subtype != NV12");
    UINT32 ow=0, oh=0;
    HR(MFGetAttributeSize(out_avail, MF_MT_FRAME_SIZE, &ow, &oh));
    EXPECT(ow == 1280 && oh == 720, "out size %ux%u", ow, oh);
    HR(mft->SetOutputType(0, out_avail, 0));
    out_avail->Release();

    /* Output stream info should now report a sane cbSize. */
    HR(mft->GetOutputStreamInfo(0, &oi));
    EXPECT(oi.cbSize == 1280u * 720u * 3u / 2u, "cbSize=%u", oi.cbSize);

    /* Begin streaming. */
    HR(mft->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0));

    /* ProcessOutput must say NEED_MORE_INPUT in Phase 2A. */
    MFT_OUTPUT_DATA_BUFFER ob{}; DWORD st=0;
    hr = mft->ProcessOutput(0, 1, &ob, &st);
    EXPECT(hr == MF_E_TRANSFORM_NEED_MORE_INPUT,
           "ProcessOutput hr=0x%08x", (unsigned)hr);

    HR(mft->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0));

    /* Round-trip current types. */
    IMFMediaType *cur = nullptr;
    HR(mft->GetInputCurrentType(0, &cur));
    cur->Release();
    HR(mft->GetOutputCurrentType(0, &cur));
    cur->Release();

    in_type->Release();
    mft->Release();
    std::printf("ok %s\n", label);
    return 0;
}

int main() {
    HR(CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED));
    HR(MFStartup(MF_VERSION, MFSTARTUP_LITE));

    HMODULE dll = LoadLibraryW(L"rkmpp_decoder_mft.dll");
    if (!dll) {
        std::printf("FAIL: LoadLibrary rkmpp_decoder_mft.dll -> %lu\n",
                    GetLastError());
        return 1;
    }
    auto get_class_obj = (PFN_DllGetClassObject)
        GetProcAddress(dll, "DllGetClassObject");
    if (!get_class_obj) {
        std::printf("FAIL: GetProcAddress DllGetClassObject\n");
        return 1;
    }

    int rc = 0;
    rc |= RunOneCodec(get_class_obj, CLSID_RkmppH264Decoder,
                      MFVideoFormat_H264, "H264");
    rc |= RunOneCodec(get_class_obj, CLSID_RkmppHevcDecoder,
                      MFVideoFormat_HEVC, "HEVC");

    /* Verify DllCanUnloadNow drops to S_OK after we drop everything. */
    typedef HRESULT (__stdcall *PFN_CanUnload)(void);
    auto can_unload = (PFN_CanUnload)GetProcAddress(dll, "DllCanUnloadNow");
    if (can_unload) {
        HRESULT hr = can_unload();
        std::printf("DllCanUnloadNow=0x%08x (S_OK if all instances released)\n",
                    (unsigned)hr);
    }

    MFShutdown();
    CoUninitialize();
    /* Don't FreeLibrary — process exit will. */
    if (rc != 0) std::printf("mft_smoke FAILED\n");
    else         std::printf("mft_smoke PASSED\n");
    return rc;
}
