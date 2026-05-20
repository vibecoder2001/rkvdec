/* mft/engine/backend_windows.cpp — DecodeEngineBackend implementation
 * over the rkmpp.sys IOCTL surface (SetupDi enumeration + CreateFile +
 * IOCTL_RKMPP_{ALLOC,FREE,SUBMIT,WAIT,PEEK}_JOB).  Default backend
 * wired up by DecodeEngine_Init when the caller doesn't supply one of
 * their own.
 *
 * Split out of decode_engine.cpp at decode-engine-backend-split.md
 * Task 3.  Pure code move — runtime behaviour unchanged.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "decode_engine_backend.h"
#include "decode_engine.h"            /* Codec enum (mirrored by DE_CODEC_*) */
#include "../../shared/rkmpp_ioctl.h"

#include <windows.h>
#include <setupapi.h>

#include <cstdio>
#include <cstring>
#include <vector>

static int Fail(const char *m, DWORD ec = 0) {
    std::fprintf(stderr, "backend_windows: %s (%lu)\n", m, ec);
    return 1;
}

/* Mirror of decode_engine.cpp::DecodeDebugEnabled() — duplicated rather
 * than exported because the gate is per-process and trivial. */
static bool DecodeDebugEnabled() {
    static int cached = -1;
    if (cached < 0) {
        char buf[8] = {};
        DWORD n = GetEnvironmentVariableA("RKMPP_DECODE_DEBUG", buf, sizeof(buf));
        cached = (n > 0 && buf[0] == '1') ? 1 : 0;
    }
    return cached != 0;
}

static int OpenDevice(HANDLE *out, DecodeEngineCodec codec) {
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return Fail("SetupDiGetClassDevsW", GetLastError());

    const uint32_t want_codec = (codec == DE_CODEC_H265) ? RKMPP_CODEC_HEVC
                              : (codec == DE_CODEC_VP9)  ? RKMPP_CODEC_VP9
                              :                            RKMPP_CODEC_H264;

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

        RKMPP_CAPS caps{};  caps.StructSize = sizeof(caps);
        DWORD got = 0;
        if (DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                            &caps, sizeof(caps), &got, nullptr) &&
            caps.Hid == RKMPP_HID_RKCP3550 &&
            (caps.SupportedCodecs & want_codec)) {
            *out = h;
            SetupDiDestroyDeviceInfoList(set);
            return 0;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    return Fail("no RKCP3550 device with requested codec found");
}

static int AllocBufRaw(HANDLE dev, uint32_t size, RKMPP_BUFFER_USAGE usage,
                       DecodeEngineBuf *out) {
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
    out->dma_fd  = -1;             /* not meaningful on Windows */
    return 0;
}

static void FreeBufRaw(HANDLE dev, DecodeEngineBuf *b) {
    if (!b->handle) return;
    RKMPP_FREE_BUFFER_IN in{ b->handle };
    DWORD got = 0;
    DeviceIoControl(dev, IOCTL_RKMPP_FREE_BUFFER, &in, sizeof(in),
                    nullptr, 0, &got, nullptr);
    *b = {};
}

/* ---- vtable functions -------------------------------------------------- */

static int WinBe_Open(void *ctx, DecodeEngineCodec codec) {
    HANDLE *slot = static_cast<HANDLE *>(ctx);
    return OpenDevice(slot, codec);
}

static void WinBe_Close(void *ctx) {
    HANDLE *slot = static_cast<HANDLE *>(ctx);
    if (*slot != INVALID_HANDLE_VALUE) {
        CloseHandle(*slot);
        *slot = INVALID_HANDLE_VALUE;
    }
}

static int WinBe_AllocBuf(void *ctx, uint32_t size, DecodeEngineBufUsage usage,
                          DecodeEngineBuf *out) {
    HANDLE *slot = static_cast<HANDLE *>(ctx);
    return AllocBufRaw(*slot, size, (RKMPP_BUFFER_USAGE)usage, out);
}

static void WinBe_FreeBuf(void *ctx, DecodeEngineBuf *buf) {
    HANDLE *slot = static_cast<HANDLE *>(ctx);
    FreeBufRaw(*slot, buf);
}

static int WinBe_SubmitDense(void *ctx, const H26xDenseOutput *in,
                              uint32_t timeout_ms, uint32_t *hw_status) {
    HANDLE *slot = static_cast<HANDLE *>(ctx);
    HANDLE dev = *slot;

    RKMPP_SUBMIT_DENSE_JOB_IN  sin{};
    RKMPP_SUBMIT_DENSE_JOB_OUT sout{};
    sin.StructSize    = sizeof(sin);
    sin.IovaSlotCount = in->IovaSlotCount;
    sin.BufRefCount   = 0;
    sin.TimeoutMs     = timeout_ms;
    sin.KickValue     = in->KickValue;
    sin.Bank          = in->Bank;
    if (in->IovaSlotCount > 0) {
        std::memcpy(sin.IovaSlots, in->IovaSlots,
                    in->IovaSlotCount * sizeof(RKMPP_DENSE_IOVA_SLOT));
    }

    DWORD got = 0;
    if (!DeviceIoControl(dev, IOCTL_RKMPP_SUBMIT_DENSE_JOB, &sin, sizeof(sin),
                         &sout, sizeof(sout), &got, nullptr)) {
        DWORD ec = GetLastError();
        std::fprintf(stderr,
            "backend_windows: SUBMIT_DENSE rejected (ec=%lu) kick=0x%08x slots=%u\n",
            ec, in->KickValue, in->IovaSlotCount);
        for (uint32_t i = 0; i < in->IovaSlotCount; i++) {
            const RKMPP_DENSE_IOVA_SLOT *s = &in->IovaSlots[i];
            std::fprintf(stderr,
                "  slot[%2u] reg=%3u  handle=0x%016llx  off=0x%08x\n",
                i, s->RegIdx,
                (unsigned long long)s->BufferHandle, s->IovaOffset);
        }
        std::fflush(stderr);
        return Fail("SUBMIT_DENSE_JOB", ec);
    }

    if (DecodeDebugEnabled()) {
        RKMPP_PEEK_JOB_IN        pin{ sout.JobId };
        RKMPP_PEEK_DENSE_JOB_OUT pout{};
        if (DeviceIoControl(dev, IOCTL_RKMPP_PEEK_DENSE_JOB, &pin, sizeof(pin),
                            &pout, sizeof(pout), &got, nullptr)) {
            std::printf("--- post-subst dense bank (kick=0x%08x) ---\n",
                        pout.KickValue);
            auto dump = [](const char *label, uint32_t first,
                            const uint32_t *src, uint32_t n) {
                for (uint32_t i = 0; i < n; i++) {
                    if (src[i] == 0) continue;
                    std::printf("  [%s idx %3u] = 0x%08x\n",
                                label, first + i, src[i]);
                }
            };
            dump("com ", RKMPP_DENSE_COMMON_FIRST,  pout.Bank.Common,
                 RKMPP_DENSE_COMMON_WORDS);
            dump("cpar", RKMPP_DENSE_CPARAM_FIRST,  pout.Bank.CodecParams,
                 RKMPP_DENSE_CPARAM_WORDS);
            dump("cadr", RKMPP_DENSE_CADDR_FIRST,   pout.Bank.CommonAddr,
                 RKMPP_DENSE_CADDR_WORDS);
            dump("codr", RKMPP_DENSE_CODADDR_FIRST, pout.Bank.CodecAddr,
                 RKMPP_DENSE_CODADDR_WORDS);
            dump("hpoc", RKMPP_DENSE_HIPOC_FIRST,   pout.Bank.HighPoc,
                 RKMPP_DENSE_HIPOC_WORDS);
            dump("stat", RKMPP_DENSE_STAT_FIRST,    pout.Bank.Stat,
                 RKMPP_DENSE_STAT_WORDS);
        }
    }

    RKMPP_WAIT_JOB_IN  win{ sout.JobId, timeout_ms, 0 };
    RKMPP_WAIT_JOB_OUT wout{};
    if (!DeviceIoControl(dev, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                         &wout, sizeof(wout), &got, nullptr))
        return Fail("WAIT_JOB", GetLastError());

    if (hw_status) *hw_status = wout.HardwareStatus;
    return wout.Status == 0 ? 0 : (int)wout.Status;
}

extern "C" void WindowsBackend_Init(DecodeEngineBackend *out, void *device_storage)
{
    out->ctx         = device_storage;
    out->Open        = WinBe_Open;
    out->Close       = WinBe_Close;
    out->AllocBuf    = WinBe_AllocBuf;
    out->FreeBuf     = WinBe_FreeBuf;
    out->SubmitDense = WinBe_SubmitDense;
}
