/* driver/rkmpp/driver.c — DriverEntry + EvtDeviceAdd for rkmpp.sys */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD RkMppEvtDeviceAdd;

NTSTATUS RkMppDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit);  /* device.c */

NTSTATUS
DriverEntry(_In_ PDRIVER_OBJECT DriverObject, _In_ PUNICODE_STRING RegistryPath)
{
    WDF_DRIVER_CONFIG cfg;
    WDF_DRIVER_CONFIG_INIT(&cfg, RkMppEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}

NTSTATUS
RkMppEvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
{
    UNREFERENCED_PARAMETER(Driver);
    return RkMppDeviceCreate(DeviceInit);
}
