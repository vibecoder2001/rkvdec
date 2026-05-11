/* driver/rkmpp_ccu/ifc.c — in-kernel query-interface registration.
 *
 * Wires every RKMPP_CCU_INTERFACE function pointer.  Implementation
 * lives in ccu.c; forward declarations are here.  As of v7 the
 * leaf-clock + core-reset methods are per-instance (Rvdec0 / Rvdec1).
 */
#include <initguid.h>  /* must precede ntddk.h to force GUID instantiation */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ccu_ifc.h"

/* Implemented in ccu.c */
NTSTATUS RkMppCcuQueryVersion           (_Out_ PUINT32 Version);
NTSTATUS RkMppCcuRaiseCluster           (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDropCluster            (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuAssertRvdec0CoreReset  (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDeassertRvdec0CoreReset(_In_  PVOID   Ctx);
NTSTATUS RkMppCcuGateRvdec0LeafClocks   (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuUngateRvdec0LeafClocks (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuAssertRvdec1CoreReset  (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDeassertRvdec1CoreReset(_In_  PVOID   Ctx);
NTSTATUS RkMppCcuGateRvdec1LeafClocks   (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuUngateRvdec1LeafClocks (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuFullCoreReset0         (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuFullCoreReset1         (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuFullAv1Reset           (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuRaiseAv1Cluster        (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuDropAv1Cluster         (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuGateAv1LeafClocks      (_In_  PVOID   Ctx);
NTSTATUS RkMppCcuUngateAv1LeafClocks    (_In_  PVOID   Ctx);

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
    ifc.AssertRvdec0CoreReset    = RkMppCcuAssertRvdec0CoreReset;
    ifc.DeassertRvdec0CoreReset  = RkMppCcuDeassertRvdec0CoreReset;
    ifc.GateRvdec0LeafClocks     = RkMppCcuGateRvdec0LeafClocks;
    ifc.UngateRvdec0LeafClocks   = RkMppCcuUngateRvdec0LeafClocks;
    ifc.AssertRvdec1CoreReset    = RkMppCcuAssertRvdec1CoreReset;
    ifc.DeassertRvdec1CoreReset  = RkMppCcuDeassertRvdec1CoreReset;
    ifc.GateRvdec1LeafClocks     = RkMppCcuGateRvdec1LeafClocks;
    ifc.UngateRvdec1LeafClocks   = RkMppCcuUngateRvdec1LeafClocks;
    ifc.FullCoreReset0           = RkMppCcuFullCoreReset0;
    ifc.FullCoreReset1           = RkMppCcuFullCoreReset1;
    ifc.RaiseAv1Cluster          = RkMppCcuRaiseAv1Cluster;
    ifc.DropAv1Cluster           = RkMppCcuDropAv1Cluster;
    ifc.GateAv1LeafClocks        = RkMppCcuGateAv1LeafClocks;
    ifc.UngateAv1LeafClocks      = RkMppCcuUngateAv1LeafClocks;
    ifc.FullAv1Reset             = RkMppCcuFullAv1Reset;

    WDF_QUERY_INTERFACE_CONFIG cfg;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&cfg,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKMPP_CCU,
                                    NULL);
    return WdfDeviceAddQueryInterface(Device, &cfg);
}
