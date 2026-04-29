/* tests/harness/rkmpp_caps_dump/main.cpp
 * Enumerates GUID_DEVINTERFACE_RKMPP, opens each instance, calls
 * IOCTL_RKMPP_GET_CAPS, and prints a one-line summary per device.
 */
#define UMDF_USING_NTSTATUS
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <cstdio>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "../../../shared/rkmpp_ioctl.h"

std::string FormatCaps(const RKMPP_CAPS &c)
{
    std::ostringstream os;
    os << "RKCP" << std::hex << c.Hid << std::dec
       << " UID=" << c.Uid
       << " rev=0x" << std::hex << c.RevisionWord << std::dec
       << " codecs=";
    bool first = true;
    auto bit = [&](uint32_t mask, const char *name) {
        if (c.SupportedCodecs & mask) { if (!first) os << "+"; os << name; first = false; }
    };
    bit(RKMPP_CODEC_H264,   "H264");
    bit(RKMPP_CODEC_HEVC,   "HEVC");
    bit(RKMPP_CODEC_VP9,    "VP9");
    bit(RKMPP_CODEC_AV1,    "AV1");
    bit(RKMPP_CODEC_JPEG_D, "JPEG_D");
    bit(RKMPP_CODEC_AVS,    "AVS");
    if (first) os << "(none)";
    return os.str();
}

#ifndef RKMPP_CAPS_DUMP_TEST  /* test_main.cpp re-includes this without main() */
int wmain()
{
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) {
        std::fprintf(stderr, "SetupDiGetClassDevsW failed: %lu\n", GetLastError());
        return 1;
    }

    SP_DEVICE_INTERFACE_DATA ifd{ sizeof(ifd) };
    int idx = 0, found = 0;
    while (SetupDiEnumDeviceInterfaces(set, nullptr, &GUID_DEVINTERFACE_RKMPP,
                                       idx++, &ifd))
    {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &need, nullptr);
        std::vector<uint8_t> buf(need);
        auto *det = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buf.data());
        det->cbSize = sizeof(*det);
        if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, det, need, nullptr, nullptr)) {
            std::fprintf(stderr, "GetDetail failed: %lu\n", GetLastError());
            continue;
        }

        HANDLE h = CreateFileW(det->DevicePath, GENERIC_READ, 0, nullptr,
                               OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            std::fwprintf(stderr, L"open %s failed: %lu\n",
                          det->DevicePath, GetLastError());
            continue;
        }
        RKMPP_CAPS caps{};
        DWORD got = 0;
        if (!DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                             &caps, sizeof(caps), &got, nullptr)) {
            std::fwprintf(stderr, L"IOCTL on %s failed: %lu\n",
                          det->DevicePath, GetLastError());
        } else {
            std::printf("%s\n", FormatCaps(caps).c_str());
            found++;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    if (!found) { std::fprintf(stderr, "no rkmpp instances found\n"); return 2; }
    return 0;
}
#endif
