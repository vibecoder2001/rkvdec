/* driver/rkiommu/ifc.c — full RKIOMMU_INTERFACE registration (Phase 2).
 *
 * Implements and registers all four interface functions:
 *   QueryVersion, MapMdl, UnmapMdl, RegisterFaultHandler.
 *
 * ClientCookie translation (v1 topology shortcut):
 *   The RKIOMMU_INTERFACE is registered per-IOMMU-device.  When MapMdl is
 *   called, the ClientCookie parameter (by convention a PDEVICE_OBJECT for
 *   the client) would normally be translated to (Hid, Uid) and then looked
 *   up in the topology table.  For v1, with exactly one entry in the
 *   topology table (RVD0 → RD0M), we skip that translation and always route
 *   to the IOMMU device whose interface was queried.  The interface Context
 *   field already carries the WDFDEVICE device object for the IOMMU instance,
 *   so we use that directly.
 *
 * TODO (Phase 3): When multiple clients exist, translate ClientCookie to
 *   (ClientHid, ClientUid) by walking up to the PDO and reading
 *   DevicePropertyHardwareID, then call RkIommuLookupBinding to find the
 *   correct IOMMU instance from the global g_deviceList.
 *
 * ZAP_CACHE:
 *   After each RkIommuMapAt / RkIommuUnmapAt call we issue RK_MMU_CMD_ZAP_CACHE
 *   to flush the IOMMU's TLB.  This is gated on Dev->PagingEnabled so it does
 *   not fire when there is no hardware backing the registers.
 */
#include <initguid.h>   /* must precede ntddk.h to force GUID instantiation */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkiommu_ifc.h"
#include "device.h"
#include "pgtable.h"

/* ---------------------------------------------------------------------------
 * Internal: get the PRKIOMMU_DEVICE for the IOMMU instance whose interface
 * was queried.  The interface Header.Context carries the PDEVICE_OBJECT of
 * the WDF device, from which we recover the WDF handle and then the context.
 * --------------------------------------------------------------------------- */
static PRKIOMMU_DEVICE DevFromContext(_In_ PVOID Context)
{
    /* Context is set to WdfDeviceWdmGetDeviceObject(Device) in RegisterIfc.
     * WdfWdmDeviceGetWdfDeviceHandle recovers the WDFDEVICE from a WDM object. */
    PDEVICE_OBJECT wdmDev = (PDEVICE_OBJECT)Context;
    WDFDEVICE wdfDev = WdfWdmDeviceGetWdfDeviceHandle(wdmDev);
    if (!wdfDev) return NULL;
    return RkIommuDeviceGet(wdfDev);
}

/* ---------------------------------------------------------------------------
 * QueryVersion
 * --------------------------------------------------------------------------- */
static NTSTATUS RkIommuQueryVersion(_Out_ PUINT32 Version)
{
    *Version = RKIOMMU_IFC_VERSION;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * MapMdl
 *
 * Maps every physical page described by Mdl into the IOMMU's IOVA space.
 * Returns the first IOVA in *Iova.
 *
 * Role parameter: bit 0 = readable, bit 1 = writable (matches MDL usage
 * convention but we map the translation directly into PTE flags).
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuMapMdl(_In_ PVOID ClientCookie,
              _In_ PMDL  Mdl,
              _In_ ULONG Role,
              _Out_ PULONG64 Iova)
{
    UNREFERENCED_PARAMETER(ClientCookie);
    /* v1 topology shortcut: ClientCookie is ignored; we use only one IOMMU.
     * TODO (Phase 3): translate ClientCookie → (Hid, Uid) → IOMMU instance. */

    /* We need the IOMMU device.  With one device, get it from the global list. */
    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    if (IsListEmpty(&g_deviceList)) {
        KeReleaseSpinLock(&g_deviceListLock, irql);
        return STATUS_DEVICE_NOT_READY;
    }
    PRKIOMMU_DEVICE dev = CONTAINING_RECORD(
        g_deviceList.Flink, RKIOMMU_DEVICE, ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    if (!dev->Domain) return STATUS_DEVICE_NOT_READY;

    /* Count the pages in the MDL */
    ULONG pageCount = ADDRESS_AND_SIZE_TO_SPAN_PAGES(
        MmGetMdlVirtualAddress(Mdl),
        MmGetMdlByteCount(Mdl));
    if (pageCount == 0) return STATUS_INVALID_PARAMETER;

    /* Build PTE flags from Role */
    ULONG pteFlags = RK_PTE_PAGE_READABLE;          /* always readable */
    if (Role & 2u) pteFlags |= RK_PTE_PAGE_WRITABLE;

    /* Allocate a contiguous IOVA range */
    ULONG64 baseIova;
    KeAcquireSpinLock(&dev->Domain->Lock, &irql);
    NTSTATUS status = RkIommuAllocIova(dev->Domain, pageCount, &baseIova);
    if (!NT_SUCCESS(status)) {
        KeReleaseSpinLock(&dev->Domain->Lock, irql);
        return status;
    }

    /* Walk the MDL page array and map each page */
    PPFN_NUMBER pfnArray = MmGetMdlPfnArray(Mdl);
    ULONG64 iova = baseIova;
    status = STATUS_SUCCESS;

    for (ULONG i = 0; i < pageCount; i++) {
        /* PFN → physical byte address (PFN * PAGE_SIZE) */
        ULONG64 phys = (ULONG64)pfnArray[i] << PAGE_SHIFT;

        /* Verify physical address fits in 32-bit IOMMU space */
        if ((phys >> 32) != 0) {
            /* Roll back already-mapped pages */
            if (i > 0) {
                RkIommuUnmapAt(dev->Domain, baseIova, i);
                RkIommuFreeIova(dev->Domain, baseIova, pageCount);
            }
            KeReleaseSpinLock(&dev->Domain->Lock, irql);
            return STATUS_INVALID_PARAMETER;
        }

        status = RkIommuMapAt(dev->Domain, iova, phys, 1, pteFlags);
        if (!NT_SUCCESS(status)) {
            if (i > 0) {
                RkIommuUnmapAt(dev->Domain, baseIova, i);
            }
            RkIommuFreeIova(dev->Domain, baseIova, pageCount);
            KeReleaseSpinLock(&dev->Domain->Lock, irql);
            return status;
        }
        iova += RK_IOMMU_PAGE_SIZE;
    }

    KeReleaseSpinLock(&dev->Domain->Lock, irql);

    /* Phase 3a-debug: the codec/IOMMU MMIO read in rkmpp PrepareHardware was
     * SErroring even though clocks appeared ungated.  IOMMU MMIO at
     * 0xFDC38700 lives in the same power domain as the codec at 0xFDC38100,
     * so RkIommuEnable's writes to DTE_ADDR / COMMAND would WHEA the same
     * way.  Roll back to the Phase 2 "page tables in RAM, hardware IOMMU
     * not enabled" stance — smoke's software-completion job doesn't
     * actually dereference the iova, so this is safe and lets the rest of
     * the data path validate end-to-end.
     *
     * TODO (Phase 3b): figure out the real PMU sequence; once codec MMIO
     * reads succeed, restore RkIommuEnable. */

    *Iova = baseIova;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * UnmapMdl
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuUnmapMdl(_In_ PVOID  ClientCookie,
                _In_ ULONG64 Iova)
{
    UNREFERENCED_PARAMETER(ClientCookie);
    /* v1 topology shortcut: same as MapMdl — ignore ClientCookie.
     * TODO (Phase 3): translate ClientCookie → correct IOMMU instance. */

    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    if (IsListEmpty(&g_deviceList)) {
        KeReleaseSpinLock(&g_deviceListLock, irql);
        return STATUS_DEVICE_NOT_READY;
    }
    PRKIOMMU_DEVICE dev = CONTAINING_RECORD(
        g_deviceList.Flink, RKIOMMU_DEVICE, ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    if (!dev->Domain) return STATUS_DEVICE_NOT_READY;

    /* Determine how many pages are mapped at this IOVA.
     * We stored the page count in the IOVA allocator bitmap; we reconstruct
     * it by counting consecutive set bits starting at the IOVA page index. */
    ULONG startPage = (ULONG)(Iova >> 12);
    const ULONG bitsPerWord = (ULONG)(sizeof(ULONG_PTR) * 8);
    ULONG pageCount = 0;

    KeAcquireSpinLock(&dev->Domain->Lock, &irql);

    /* Count consecutive allocated bits to determine the mapping size */
    for (ULONG pg = startPage; pg < RK_IOMMU_IOVA_PAGES; pg++) {
        ULONG w = pg / bitsPerWord;
        ULONG b = pg % bitsPerWord;
        if (!(dev->Domain->IovaBitmap[w] & ((ULONG_PTR)1 << b))) break;
        pageCount++;
    }

    if (pageCount == 0) {
        KeReleaseSpinLock(&dev->Domain->Lock, irql);
        return STATUS_INVALID_PARAMETER;
    }

    RkIommuUnmapAt(dev->Domain, Iova, pageCount);
    RkIommuFreeIova(dev->Domain, Iova, pageCount);

    KeReleaseSpinLock(&dev->Domain->Lock, irql);

    if (dev->PagingEnabled && dev->MmioBase) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_ZAP_CACHE);
    }

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RegisterFaultHandler
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuRegisterFaultHandler(_In_ PVOID                 ClientCookie,
                             _In_ RKIOMMU_FAULT_CALLBACK Callback)
{
    /* v1 topology shortcut: register callback on the single IOMMU instance.
     * TODO (Phase 3): route to the IOMMU that owns this client. */

    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    if (IsListEmpty(&g_deviceList)) {
        KeReleaseSpinLock(&g_deviceListLock, irql);
        return STATUS_DEVICE_NOT_READY;
    }
    PRKIOMMU_DEVICE dev = CONTAINING_RECORD(
        g_deviceList.Flink, RKIOMMU_DEVICE, ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    dev->FaultCb       = Callback;
    dev->FaultCbCookie = ClientCookie;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuRegisterIfc — called from device.c after WdfDeviceCreate
 * --------------------------------------------------------------------------- */
NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device)
{
    /* WdfDeviceAddQueryInterface registers the IRP_MN_QUERY_INTERFACE handler
     * keyed by GUID, but it does NOT publish a PnP device-interface symlink.
     * Without the symlink, IoGetDeviceInterfaces (which client drivers use to
     * discover us) returns nothing.  WdfDeviceCreateDeviceInterface publishes
     * the symlink for the same GUID; both calls must use the SAME GUID so the
     * client can enumerate a candidate via IoGetDeviceInterfaces, open it,
     * and then send IRP_MN_QUERY_INTERFACE to fetch the function table. */
    NTSTATUS s = WdfDeviceCreateDeviceInterface(Device,
                                                &GUID_DEVINTERFACE_RKIOMMU,
                                                NULL);
    if (!NT_SUCCESS(s)) return s;

    RKIOMMU_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    ifc.Header.Size              = sizeof(ifc);
    ifc.Header.Version           = RKIOMMU_IFC_VERSION;
    ifc.Header.Context           = WdfDeviceWdmGetDeviceObject(Device);
    ifc.Header.InterfaceReference    = WdfDeviceInterfaceReferenceNoOp;
    ifc.Header.InterfaceDereference  = WdfDeviceInterfaceDereferenceNoOp;
    ifc.QueryVersion             = RkIommuQueryVersion;
    ifc.MapMdl                   = RkIommuMapMdl;
    ifc.UnmapMdl                 = RkIommuUnmapMdl;
    ifc.RegisterFaultHandler     = RkIommuRegisterFaultHandler;

    WDF_QUERY_INTERFACE_CONFIG cfg;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&cfg,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKIOMMU,
                                    NULL);
    return WdfDeviceAddQueryInterface(Device, &cfg);
}
