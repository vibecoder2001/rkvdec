/* driver/rkmpp_ccu/ifc.c — in-kernel query-interface registration.
 *
 * Phase 2: wires all five RKMPP_CCU_INTERFACE function pointers.
 * Implementation lives in ccu.c; forward declarations are here.
 */
#include <initguid.h>  /* must precede ntddk.h to force GUID instantiation */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ccu_ifc.h"

/* Implemented in ccu.c */
NTSTATUS RkMppCcuQueryVersion        (_Out_ PUINT32 Version);
NTSTATUS RkMppCcuRaiseCluster        (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDropCluster         (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuAssertCoreReset     (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDeassertCoreReset   (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuFullCoreReset       (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuGateCoreLeafClocks  (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuUngateCoreLeafClocks(_In_  PVOID   Ctx);
NTSTATUS RkMppCcuRaiseAv1Cluster     (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDropAv1Cluster      (_In_  PVOID   Ctx);

NTSTATUS RkMppCcuRegisterIfc(_In_ WDFDEVICE Device)
{
    /* Publish the PnP device-interface symlink so client drivers can find us
     * via IoGetDeviceInterfaces.  WdfDeviceAddQueryInterface alone is not
     * enough — it only registers the IRP_MN_QUERY_INTERFACE handler. */
    NTSTATUS s = WdfDeviceCreateDeviceInterface(Device,
                                                &GUID_DEVINTERFACE_RKMPP_CCU,
                                                NULL);
    if (!NT_SUCCESS(s)) return s;

    RKMPP_CCU_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    ifc.Header.Size              = sizeof(ifc);
    ifc.Header.Version           = RKMPP_CCU_IFC_VERSION;
    ifc.Header.Context           = WdfDeviceWdmGetDeviceObject(Device);
    ifc.Header.InterfaceReference    = WdfDeviceInterfaceReferenceNoOp;
    ifc.Header.InterfaceDereference  = WdfDeviceInterfaceDereferenceNoOp;
    ifc.QueryVersion             = RkMppCcuQueryVersion;
    ifc.RaiseCluster             = RkMppCcuRaiseCluster;
    ifc.DropCluster              = RkMppCcuDropCluster;
    ifc.AssertCoreReset          = RkMppCcuAssertCoreReset;
    ifc.DeassertCoreReset        = RkMppCcuDeassertCoreReset;
    ifc.FullCoreReset            = RkMppCcuFullCoreReset;
    ifc.GateCoreLeafClocks       = RkMppCcuGateCoreLeafClocks;
    ifc.UngateCoreLeafClocks     = RkMppCcuUngateCoreLeafClocks;
    ifc.RaiseAv1Cluster          = RkMppCcuRaiseAv1Cluster;
    ifc.DropAv1Cluster           = RkMppCcuDropAv1Cluster;

    WDF_QUERY_INTERFACE_CONFIG cfg;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&cfg,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKMPP_CCU,
                                    NULL);
    return WdfDeviceAddQueryInterface(Device, &cfg);
}
