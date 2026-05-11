/* devioctl.h — Linux stub: prevents shared/rkmpp_ioctl.h from failing */
#pragma once
#include <stdint.h>
#define CTL_CODE(dev,func,method,access) 0u
#define METHOD_BUFFERED     0
#define METHOD_IN_DIRECT    1
#define METHOD_OUT_DIRECT   2
#define METHOD_NEITHER      3
#define FILE_READ_ACCESS    1
#define FILE_WRITE_ACCESS   2
#define FILE_ANY_ACCESS     0
