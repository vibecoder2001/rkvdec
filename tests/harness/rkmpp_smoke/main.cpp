/* tests/harness/rkmpp_smoke/main.cpp
 *
 * Exercises ALLOC → write → SUBMIT → WAIT → reread → FREE on the first
 * RKCP3550 (RVD0) instance.  When run on this x64 dev box, the rkmpp.sys
 * driver is not loaded so the SetupDi enumerate finds nothing — exit 2.
 * The TDD value is in `BuildNoopJob`, exercised by rkmpp_smoke_test.
 */
#define UMDF_USING_NTSTATUS
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include "../../../shared/rkmpp_ioctl.h"

void BuildNoopJob(UINT64 scratchHandle, RKMPP_SUBMIT_JOB_IN *out)
{
    std::memset(out, 0, sizeof(*out));
    out->StructSize    = sizeof(*out);
    out->TimeoutMs     = 1000;

    /* Phase 2 software-completion: a single harmless write to a known-unused
     * scratch offset.  Phase 3 replaces this with the real H.264 register list. */
    out->RegWriteCount = 1;
    out->Writes[0].Offset = 0xfff8;
    out->Writes[0].Value  = 0;

    out->BufRefCount = 1;
    out->BufRefs[0].BufferHandle = scratchHandle;
    out->BufRefs[0].Role         = RkMppBufferUsageScratch;
}

#ifndef RKMPP_SMOKE_TEST

static int OpenRvd0(HANDLE *outHandle)
{
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return 1;

    SP_DEVICE_INTERFACE_DATA ifd{ sizeof(ifd) };
    int idx = 0;
    while (SetupDiEnumDeviceInterfaces(set, nullptr, &GUID_DEVINTERFACE_RKMPP,
                                       idx++, &ifd))
    {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &need, nullptr);
        std::vector<uint8_t> buf(need);
        auto *det = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buf.data());
        det->cbSize = sizeof(*det);
        if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, det, need, nullptr, nullptr))
            continue;

        HANDLE h = CreateFileW(det->DevicePath, GENERIC_READ | GENERIC_WRITE,
                               0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) continue;

        RKMPP_CAPS caps{};
        DWORD got = 0;
        if (DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                            &caps, sizeof(caps), &got, nullptr) &&
            caps.Hid == 0x3550 && caps.Uid == 0)
        {
            std::printf("caps OK: RKCP%04x UID=%u rev=0x%08x codecs=0x%08x\n",
                        caps.Hid, caps.Uid, caps.RevisionWord, caps.SupportedCodecs);
            *outHandle = h;
            SetupDiDestroyDeviceInfoList(set);
            return 0;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    return 2;  /* not found */
}

int wmain()
{
    HANDLE h = nullptr;
    int rc = OpenRvd0(&h);
    if (rc) {
        std::fprintf(stderr,
            rc == 2 ? "rkmpp_smoke: no RVD0 instance found (driver loaded?)\n"
                    : "rkmpp_smoke: SetupDi failure\n");
        return rc;
    }

    /* 1. ALLOC scratch (4 KiB). */
    RKMPP_ALLOC_BUFFER_IN  ain{};
    RKMPP_ALLOC_BUFFER_OUT aout{};
    ain.StructSize = sizeof(ain);
    ain.Size       = 4096;
    ain.Usage      = RkMppBufferUsageScratch;
    DWORD got = 0;
    if (!DeviceIoControl(h, IOCTL_RKMPP_ALLOC_BUFFER, &ain, sizeof(ain),
                         &aout, sizeof(aout), &got, nullptr))
    {
        std::fprintf(stderr, "ALLOC failed: %lu\n", GetLastError());
        CloseHandle(h);
        return 3;
    }
    std::printf("allocated scratch handle=0x%llx iova=0x%llx userVa=%p\n",
                (unsigned long long)aout.BufferHandle,
                (unsigned long long)aout.Iova, aout.UserVa);

    /* 2. Write magic pattern. */
    constexpr uint32_t kPattern = 0xC0DECAFEu;
    *static_cast<volatile uint32_t*>(aout.UserVa) = kPattern;

    /* 3. SUBMIT no-op job. */
    RKMPP_SUBMIT_JOB_IN  sin{};
    RKMPP_SUBMIT_JOB_OUT sout{};
    BuildNoopJob(aout.BufferHandle, &sin);
    if (!DeviceIoControl(h, IOCTL_RKMPP_SUBMIT_JOB, &sin, sizeof(sin),
                         &sout, sizeof(sout), &got, nullptr))
    {
        std::fprintf(stderr, "SUBMIT failed: %lu\n", GetLastError());
        CloseHandle(h);
        return 4;
    }

    /* 4. WAIT. */
    RKMPP_WAIT_JOB_IN  win{};
    RKMPP_WAIT_JOB_OUT wout{};
    win.JobId = sout.JobId;
    win.TimeoutMs = 1000;
    if (!DeviceIoControl(h, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                         &wout, sizeof(wout), &got, nullptr))
    {
        std::fprintf(stderr, "WAIT failed: %lu\n", GetLastError());
        CloseHandle(h);
        return 5;
    }
    std::printf("job %llu completed status=0x%08lx\n",
                (unsigned long long)sout.JobId, (unsigned long)wout.Status);

    /* 5. Verify pattern. */
    uint32_t reread = *static_cast<volatile uint32_t*>(aout.UserVa);
    if (reread != kPattern) {
        std::fprintf(stderr, "pattern corrupted: 0x%08x != 0x%08x\n", reread, kPattern);
        CloseHandle(h);
        return 6;
    }
    std::printf("scratch pattern preserved\n");

    /* 6. FREE. */
    RKMPP_FREE_BUFFER_IN fin{};
    fin.BufferHandle = aout.BufferHandle;
    if (!DeviceIoControl(h, IOCTL_RKMPP_FREE_BUFFER, &fin, sizeof(fin),
                         nullptr, 0, &got, nullptr))
    {
        std::fprintf(stderr, "FREE failed: %lu\n", GetLastError());
    }

    CloseHandle(h);
    return 0;
}
#endif
