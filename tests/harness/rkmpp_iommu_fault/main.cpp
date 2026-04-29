/* tests/harness/rkmpp_iommu_fault/main.cpp
 *
 * Phase 3a scaffold: opens RVD0 and queries the fault-handler state via
 * IOCTL_RKMPP_INJECT_IOMMU_FAULT.  Phase 3b will replace this with actual
 * fault injection (submit a job with a deliberately-bad iova).
 *
 * Exit code 0 if the IOCTL succeeds (regardless of whether a fault was
 * triggered — Phase 3a doesn't trigger any).  Non-zero on IOCTL failure.
 */
#define UMDF_USING_NTSTATUS
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <cstdio>
#include <cstdint>
#include <vector>

#include "../../../shared/rkmpp_ioctl.h"

static HANDLE OpenRvd0()
{
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return INVALID_HANDLE_VALUE;

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
            SetupDiDestroyDeviceInfoList(set);
            return h;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    return INVALID_HANDLE_VALUE;
}

int wmain()
{
    HANDLE h = OpenRvd0();
    if (h == INVALID_HANDLE_VALUE) {
        std::fprintf(stderr, "rkmpp_iommu_fault: no RVD0 instance found\n");
        return 1;
    }

    RKMPP_FAULT_RESULT res{};
    DWORD got = 0;
    if (!DeviceIoControl(h, IOCTL_RKMPP_INJECT_IOMMU_FAULT, nullptr, 0,
                         &res, sizeof(res), &got, nullptr))
    {
        std::fprintf(stderr, "IOCTL failed: %lu\n", GetLastError());
        CloseHandle(h);
        return 2;
    }

    std::printf("fault state: triggered=%u status=0x%08x iova=0x%llx\n",
                res.Triggered, res.StatusReg, (unsigned long long)res.FaultIova);
    std::printf("(Phase 3a scaffold -- actual fault injection lands in Phase 3b)\n");
    CloseHandle(h);
    return 0;
}
