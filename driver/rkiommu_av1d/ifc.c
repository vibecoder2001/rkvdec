/* driver/rkiommu_av1d/ifc.c — full RKIOMMU_INTERFACE registration for rkiommu_av1d.sys.
 *
 * Implements and registers all RKIOMMU_INTERFACE methods.
 *
 * AV1D-specific differences from rkiommu_vdec/ifc.c:
 *   - All mappings are RW (AV1 IP writes to its bitstream region past tile data)
 *   - TLB flush uses AV1_MMU_FLUSH pulse (not ZAP_CACHE command)
 *   - Snapshot reads AV1_MMU_STATUS_AV1 / AV1_MMU_PAGE_FAULT_ADDR_AV1
 *   - ForceReset emulated via Disable+Enable (no FORCE_RESET command on AV1D)
 */
#include <initguid.h>   /* must precede ntddk.h to force GUID instantiation */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkiommu_ifc.h"
#include "../../shared/rkmpp_ioctl.h"  /* RkMppBufferUsage* enum */
#include "device.h"
#include "../shared/rkmpp_log.h"
#include "../shared/iommu/pgtable.h"

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
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuMapMdl(_In_ PVOID ProviderContext,
              _In_ PMDL  Mdl,
              _In_ ULONG Role,
              _Out_ PULONG64 Iova)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (!dev->Domain) return STATUS_DEVICE_NOT_READY;
    KIRQL irql;

    /* Count the pages in the MDL */
    ULONG pageCount = ADDRESS_AND_SIZE_TO_SPAN_PAGES(
        MmGetMdlVirtualAddress(Mdl),
        MmGetMdlByteCount(Mdl));
    if (pageCount == 0) return STATUS_INVALID_PARAMETER;

    /* AV1D IOMMU: all mappings are read+write.  BSP maps everything RW;
     * the AV1 IP issues AXI writes to its bitstream region past the tile data. */
    ULONG pteFlags = RK_PTE_PAGE_READABLE | RK_PTE_PAGE_WRITABLE;
    (void)Role;

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

    /* Lazy-enable IOMMU paging on the first MapMdl that places real
     * data into the address space. */
    NTSTATUS enableStatus = RkIommuEnableHw(dev);
    if (!NT_SUCCESS(enableStatus)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: enable failed 0x%08x for first MapMdl\n",
                   enableStatus);
    }

    *Iova = baseIova;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * UnmapMdl
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuUnmapMdl(_In_ PVOID  ProviderContext,
                _In_ ULONG64 Iova)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (!dev->Domain) return STATUS_DEVICE_NOT_READY;
    KIRQL irql;

    /* See rkiommu_vdec/ifc.c for the bounded-walk rationale.  AV1D
     * domain has the same shared-bitmap-over-Domain shape and the same
     * peer-over-unmap hazard if two AV1 sessions Shutdown adjacent. */
    ULONG startPage = (ULONG)(Iova >> 12);
    const ULONG bitsPerWord = (ULONG)(sizeof(ULONG_PTR) * 8);
    ULONG pageCount = 0;

    KeAcquireSpinLock(&dev->Domain->Lock, &irql);

    if (!(dev->Domain->IovaStartBitmap[startPage / bitsPerWord] &
          ((ULONG_PTR)1 << (startPage % bitsPerWord)))) {
        KeReleaseSpinLock(&dev->Domain->Lock, irql);
        return STATUS_INVALID_PARAMETER;
    }

    for (ULONG pg = startPage; pg < RK_IOMMU_IOVA_PAGES; pg++) {
        ULONG w = pg / bitsPerWord;
        ULONG b = pg % bitsPerWord;
        if (!(dev->Domain->IovaBitmap[w] & ((ULONG_PTR)1 << b))) break;
        if (pg != startPage &&
            (dev->Domain->IovaStartBitmap[w] & ((ULONG_PTR)1 << b)))
            break;
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
            (volatile ULONG*)(dev->MmioBase + AV1_MMU_FLUSH), AV1_MMU_FLUSH_BIT);
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + AV1_MMU_FLUSH), 0u);
    }

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RegisterFaultHandler
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuRegisterFaultHandler(_In_ PVOID                 ProviderContext,
                             _In_ PVOID                 ConsumerContext,
                             _In_ RKIOMMU_FAULT_CALLBACK Callback)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;

    dev->FaultCb       = Callback;
    dev->FaultCbCookie = ConsumerContext;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * Snapshot — read-only sample of fault-related MMIO for post-kick diag.
 * AV1D: STATUS at 0x384, fault addr at 0x380, PTA base at 0x38C.
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuSnapshot(_In_ PVOID ProviderContext,
                _Out_ PRKIOMMU_FAULT_SNAPSHOT Out)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;

    Out->Status        = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + AV1_MMU_STATUS_AV1));
    Out->IntRawStat    = Out->Status;
    Out->IntStatus     = Out->Status;
    Out->PageFaultAddr = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + AV1_MMU_PAGE_FAULT_ADDR_AV1));
    Out->DteAddr       = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + AV1_MMU_AHB_TBL_ARRAY_BASE_L));
    return STATUS_SUCCESS;
}

/* FlushTlb — pulse AV1_MMU_FLUSH to drain stale TLB entries. */
static NTSTATUS
RkIommuFlushTlb(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    if (!dev->PagingEnabled)    return STATUS_SUCCESS;

    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + AV1_MMU_FLUSH), AV1_MMU_FLUSH_BIT);
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + AV1_MMU_FLUSH), 0u);
    return STATUS_SUCCESS;
}

/* Reattach — Disable + Enable the IOMMU so the hardware drops walk-cache
 * state and re-fetches the page directory. */
static NTSTATUS
RkIommuReattach(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    if (!dev->Domain)           return STATUS_DEVICE_NOT_READY;

    NTSTATUS s = RkIommuDisableHw(dev);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: Reattach disable phase failed 0x%08x\n", s);
        return s;
    }
    s = RkIommuEnableHw(dev);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_av1d: Reattach enable phase failed 0x%08x\n", s);
        return s;
    }
    RKMPP_LOG_INFO(
               "rkiommu_av1d: reattached (HID=RKCP%04x UID=%u)\n",
               dev->Hid, dev->Uid);
    return STATUS_SUCCESS;
}

/* ForceReset — AV1D has no FORCE_RESET command.
 * Disable+Enable clears walk-cache state. */
static NTSTATUS
RkIommuForceReset(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;

    /* AV1D has no FORCE_RESET command.  Disable+Enable clears walk-cache state. */
    NTSTATUS s = RkIommuDisableHw(dev);
    if (!NT_SUCCESS(s)) return s;
    return RkIommuEnableHw(dev);
}

/* Enable — bridge ifc ProviderContext to the internal RkIommuEnableHw helper.
 * The internal handles Domain==NULL, idempotent re-enable and sets
 * PagingEnabled=TRUE.  Called by rkav1d during D0Entry, after the CCU has
 * ungated the AV1 leaf clocks. */
static NTSTATUS
RkIommuEnable(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    return RkIommuEnableHw(dev);
}

/* Disable — bridge to the internal RkIommuDisableHw helper which writes
 * AHB_CONTROL=0 and clears PagingEnabled.  Called by rkav1d during
 * D0Exit, before the CCU gates the AV1 leaf clocks. */
static NTSTATUS
RkIommuDisable(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    return RkIommuDisableHw(dev);
}

/* MaskIrq — disconnect our ISR from the kernel interrupt dispatcher via
 * WdfInterruptDisable.  This is a software-only operation; it does not
 * touch MMIO and is therefore safe across the clock-gating window. */
static NTSTATUS
RkIommuMaskIrq(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (dev->Interrupt != NULL)
        WdfInterruptDisable(dev->Interrupt);
    return STATUS_SUCCESS;
}

/* UnmaskIrq — reconnect our ISR via WdfInterruptEnable after the CCU has
 * restored AV1 leaf clocks.  Pure software hookup, no MMIO. */
static NTSTATUS
RkIommuUnmaskIrq(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (dev->Interrupt != NULL)
        WdfInterruptEnable(dev->Interrupt);
    return STATUS_SUCCESS;
}

/* IsPtAttached — AV1D is always the sole master instance (single-instance
 * topology); its page tables are allocated at PrepareHardware time.
 * Returns TRUE once Domain is allocated (i.e. hardware is ready).
 * AV1D has no slave/master split so Domain!=NULL is the definitive
 * readiness signal. */
static BOOLEAN
RkIommuAv1dIfcIsPtAttached(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return FALSE;
    return (dev->Domain != NULL) ? TRUE : FALSE;
}

/* ---------------------------------------------------------------------------
 * RkIommuRegisterIfc — called from device.c after WdfDeviceCreate
 * --------------------------------------------------------------------------- */
NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device)
{
    NTSTATUS s = WdfDeviceCreateDeviceInterface(Device,
                                                &GUID_DEVINTERFACE_RKIOMMU,
                                                NULL);
    if (!NT_SUCCESS(s)) return s;

    PRKIOMMU_DEVICE devCtx = RkIommuDeviceGet(Device);

    RKIOMMU_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    ifc.Header.Size              = sizeof(ifc);
    ifc.Header.Version           = RKIOMMU_IFC_VERSION;
    ifc.Header.Context           = WdfDeviceWdmGetDeviceObject(Device);
    ifc.Header.InterfaceReference    = WdfDeviceInterfaceReferenceNoOp;
    ifc.Header.InterfaceDereference  = WdfDeviceInterfaceDereferenceNoOp;
    ifc.Hid                      = devCtx ? devCtx->Hid : 0;
    ifc.Uid                      = devCtx ? devCtx->Uid : 0;
    ifc.QueryVersion             = RkIommuQueryVersion;
    ifc.MapMdl                   = RkIommuMapMdl;
    ifc.UnmapMdl                 = RkIommuUnmapMdl;
    ifc.RegisterFaultHandler     = RkIommuRegisterFaultHandler;
    ifc.Snapshot                 = RkIommuSnapshot;
    ifc.FlushTlb                 = RkIommuFlushTlb;
    ifc.Reattach                 = RkIommuReattach;
    ifc.ForceReset               = RkIommuForceReset;
    ifc.Enable                   = RkIommuEnable;
    ifc.Disable                  = RkIommuDisable;
    ifc.MaskIrq                  = RkIommuMaskIrq;
    ifc.UnmaskIrq                = RkIommuUnmaskIrq;
    ifc.IsPtAttached             = RkIommuAv1dIfcIsPtAttached;

    WDF_QUERY_INTERFACE_CONFIG cfg;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&cfg,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKIOMMU,
                                    NULL);
    return WdfDeviceAddQueryInterface(Device, &cfg);
}
