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

    /* No register writes — exercises the alloc/submit/wait/free
     * plumbing without touching codec MMIO.  The kernel completes
     * empty / no-kick jobs immediately (no dec_e=1 in the list). */
    out->RegWriteCount = 0;

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

    /* Phase 3a-debug: REVISION read is intentionally skipped in the driver
     * because reading codec MMIO during PrepareHardware was SErroring on
     * this firmware.  Just print whatever caps reports. */
    {
        RKMPP_CAPS caps{};
        DWORD got = 0;
        if (DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                            &caps, sizeof(caps), &got, nullptr))
        {
            std::printf("caps: HID=RKCP%04x UID=%u rev=0x%08x codecs=0x%08x\n",
                        caps.Hid, caps.Uid, caps.RevisionWord, caps.SupportedCodecs);
        }
    }

    /* Phase 3b-4: verify iova-handle substitution.  Allocate a buffer,
     * submit a job with one plain write and one substituted write, peek
     * the queued job, confirm the substituted value resolves to the
     * buffer's iova plus the requested offset. */
    {
        RKMPP_ALLOC_BUFFER_IN  ain{};
        RKMPP_ALLOC_BUFFER_OUT aout{};
        ain.StructSize = sizeof(ain);
        ain.Size       = 4096;
        ain.Usage      = RkMppBufferUsageScratch;
        DWORD got = 0;
        if (!DeviceIoControl(h, IOCTL_RKMPP_ALLOC_BUFFER, &ain, sizeof(ain),
                             &aout, sizeof(aout), &got, nullptr))
        {
            std::fprintf(stderr, "subst-test ALLOC failed: %lu\n",
                         GetLastError());
            CloseHandle(h);
            return 30;
        }

        const UINT32 plainValue   = 0xDEADBEEFu;
        const UINT32 substOffset  = 0x40u;
        const UINT32 expectedSubst = (UINT32)(aout.Iova + substOffset);

        RKMPP_SUBMIT_JOB_IN  sin{};
        RKMPP_SUBMIT_JOB_OUT sout{};
        sin.StructSize    = sizeof(sin);
        sin.RegWriteCount = 2;
        sin.TimeoutMs     = 1000;
        sin.Writes[0].Offset = 0xfff8;
        sin.Writes[0].Value  = plainValue;
        sin.Writes[1].Offset = 0xfffc;
        sin.Writes[1].Value  = 0;            /* must be overwritten */
        sin.Writes[1].BufferHandle = aout.BufferHandle;
        sin.Writes[1].IovaOffset   = substOffset;

        if (!DeviceIoControl(h, IOCTL_RKMPP_SUBMIT_JOB, &sin, sizeof(sin),
                             &sout, sizeof(sout), &got, nullptr))
        {
            std::fprintf(stderr, "subst-test SUBMIT failed: %lu\n",
                         GetLastError());
            CloseHandle(h);
            return 31;
        }

        RKMPP_PEEK_JOB_IN  pin{};
        RKMPP_PEEK_JOB_OUT pout{};
        pin.JobId = sout.JobId;
        if (!DeviceIoControl(h, IOCTL_RKMPP_PEEK_JOB, &pin, sizeof(pin),
                             &pout, sizeof(pout), &got, nullptr))
        {
            std::fprintf(stderr, "subst-test PEEK failed: %lu\n",
                         GetLastError());
            CloseHandle(h);
            return 32;
        }
        if (pout.RegWriteCount != 2 ||
            pout.Writes[0].Value != plainValue ||
            pout.Writes[1].Value != expectedSubst)
        {
            std::fprintf(stderr,
                "subst-test mismatch: plain=0x%08x (want 0x%08x)  "
                "subst=0x%08x (want 0x%08x  iova=0x%llx + 0x%x)\n",
                pout.Writes[0].Value, plainValue,
                pout.Writes[1].Value, expectedSubst,
                aout.Iova, substOffset);
            CloseHandle(h);
            return 33;
        }
        std::printf("subst-test OK: plain=0x%08x  iova-subst=0x%08x\n",
                    pout.Writes[0].Value, pout.Writes[1].Value);

        /* Drain the job and free the buffer so the iteration loop below
         * starts from a clean state. */
        RKMPP_WAIT_JOB_IN  win{};
        RKMPP_WAIT_JOB_OUT wout{};
        win.JobId = sout.JobId;
        win.TimeoutMs = 1000;
        DeviceIoControl(h, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                        &wout, sizeof(wout), &got, nullptr);
        RKMPP_FREE_BUFFER_IN fin{};
        fin.BufferHandle = aout.BufferHandle;
        DeviceIoControl(h, IOCTL_RKMPP_FREE_BUFFER, &fin, sizeof(fin),
                        nullptr, 0, &got, nullptr);
    }

    /* Phase 3a: 100-iteration alloc/submit/wait/reread/free loop. Each
     * iteration exercises the buffer pool, IOMMU MapMdl (with real paging
     * enabled now), the job queue, and software-completion DPC.  Phase 3b
     * replaces software completion with hardware kick. */
    constexpr int kIterations = 100;
    constexpr uint32_t kPattern = 0xC0DECAFEu;
    DWORD got = 0;

    for (int iter = 0; iter < kIterations; iter++) {
        RKMPP_ALLOC_BUFFER_IN  ain{};
        RKMPP_ALLOC_BUFFER_OUT aout{};
        ain.StructSize = sizeof(ain);
        ain.Size       = 4096;
        ain.Usage      = RkMppBufferUsageScratch;
        if (!DeviceIoControl(h, IOCTL_RKMPP_ALLOC_BUFFER, &ain, sizeof(ain),
                             &aout, sizeof(aout), &got, nullptr))
        {
            std::fprintf(stderr, "iter %d: ALLOC failed: %lu\n",
                         iter, GetLastError());
            CloseHandle(h);
            return 3;
        }

        *static_cast<volatile uint32_t*>(aout.UserVa) = kPattern ^ (uint32_t)iter;

        RKMPP_SUBMIT_JOB_IN  sin{};
        RKMPP_SUBMIT_JOB_OUT sout{};
        BuildNoopJob(aout.BufferHandle, &sin);
        if (!DeviceIoControl(h, IOCTL_RKMPP_SUBMIT_JOB, &sin, sizeof(sin),
                             &sout, sizeof(sout), &got, nullptr))
        {
            std::fprintf(stderr, "iter %d: SUBMIT failed: %lu\n",
                         iter, GetLastError());
            CloseHandle(h);
            return 4;
        }

        RKMPP_WAIT_JOB_IN  win{};
        RKMPP_WAIT_JOB_OUT wout{};
        win.JobId = sout.JobId;
        win.TimeoutMs = 1000;
        if (!DeviceIoControl(h, IOCTL_RKMPP_WAIT_JOB, &win, sizeof(win),
                             &wout, sizeof(wout), &got, nullptr))
        {
            std::fprintf(stderr, "iter %d: WAIT failed: %lu\n",
                         iter, GetLastError());
            CloseHandle(h);
            return 5;
        }

        uint32_t reread = *static_cast<volatile uint32_t*>(aout.UserVa);
        if (reread != (kPattern ^ (uint32_t)iter)) {
            std::fprintf(stderr, "iter %d: pattern corrupted: 0x%08x\n",
                         iter, reread);
            CloseHandle(h);
            return 6;
        }

        RKMPP_FREE_BUFFER_IN fin{};
        fin.BufferHandle = aout.BufferHandle;
        if (!DeviceIoControl(h, IOCTL_RKMPP_FREE_BUFFER, &fin, sizeof(fin),
                             nullptr, 0, &got, nullptr))
        {
            std::fprintf(stderr, "iter %d: FREE failed: %lu\n",
                         iter, GetLastError());
            CloseHandle(h);
            return 8;
        }
    }

    std::printf("rkmpp_smoke: %d iterations OK\n", kIterations);
    CloseHandle(h);
    return 0;
}
#endif
