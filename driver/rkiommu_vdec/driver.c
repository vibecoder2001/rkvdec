/* driver/rkiommu/driver.c — KMDF entry point for rkiommu.sys.
 *
 * Phase 2: DriverEntry + EvtDeviceAdd delegate all device work to device.c.
 */
#include <ntddk.h>
#include <wdf.h>

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD RkIommuEvtDeviceAdd;

extern NTSTATUS RkIommuDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit);

NTSTATUS
DriverEntry(_In_ PDRIVER_OBJECT DriverObject, _In_ PUNICODE_STRING RegistryPath)
{
    WDF_DRIVER_CONFIG cfg;
    WDF_DRIVER_CONFIG_INIT(&cfg, RkIommuEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}

NTSTATUS
RkIommuEvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
{
    UNREFERENCED_PARAMETER(Driver);
    return RkIommuDeviceCreate(DeviceInit);
}
