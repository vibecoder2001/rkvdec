/* Unit tests for the pretty-printer in rkmpp_caps_dump. */
/* <windows.h> must precede rkmpp_ioctl.h: devioctl.h (in WDK shared/) uses
 * ULONG/UINT32 which come from windows.h, and DEFINE_GUID needs guiddef.h.
 * Including <windows.h> first satisfies both. */
#include <windows.h>
#include <initguid.h>
#include <cstdio>
#include <sstream>
#include <string>

#include "../../../shared/rkmpp_ioctl.h"

std::string FormatCaps(const RKMPP_CAPS &c);  /* defined in main.cpp */

static int Fail(const char *msg) { std::fprintf(stderr, "FAIL: %s\n", msg); return 1; }

int main() {
    RKMPP_CAPS c{};
    c.StructSize      = sizeof(c);
    c.Hid             = 0x3550;
    c.Uid             = 0;
    c.RevisionWord    = 0xdeadbeef;
    c.SupportedCodecs = RKMPP_CODEC_H264;

    std::string s = FormatCaps(c);
    if (s.find("RKCP3550") == std::string::npos) return Fail("missing HID");
    if (s.find("UID=0")    == std::string::npos) return Fail("missing UID");
    if (s.find("0xdeadbeef") == std::string::npos) return Fail("missing rev");
    if (s.find("H264")     == std::string::npos) return Fail("missing H264 codec");
    return 0;
}
