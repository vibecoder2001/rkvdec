/* driver/rkiommu_vdec/ifc.c — full RKIOMMU_INTERFACE registration (Phase 2).
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
#include "../../shared/rkmpp_ioctl.h"  /* RkMppBufferUsage* enum */
#include "device.h"
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
 *
 * Role parameter: bit 0 = readable, bit 1 = writable (matches MDL usage
 * convention but we map the translation directly into PTE flags).
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuMapMdl(_In_ PVOID ProviderContext,
              _In_ PMDL  Mdl,
              _In_ ULONG Role,
              _Out_ PULONG64 Iova)
{
    /* The first parameter is the per-instance Header.Context the consumer
     * received in its queried RKIOMMU_INTERFACE.  Consumers MUST pass
     * `iommu->Header.Context` (not their own device object) so we resolve
     * back to the specific iommu instance the interface was queried on.
     * Topology routing depends on this — picking g_deviceList.Flink would
     * always land in VPMU (UID=0). */
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (!dev->Domain) return STATUS_DEVICE_NOT_READY;
    KIRQL irql;

    /* Count the pages in the MDL */
    ULONG pageCount = ADDRESS_AND_SIZE_TO_SPAN_PAGES(
        MmGetMdlVirtualAddress(Mdl),
        MmGetMdlByteCount(Mdl));
    if (pageCount == 0) return STATUS_INVALID_PARAMETER;

    /* Build PTE flags from Role.
     *
     * Role is an enum value (RkMppBufferUsage*), NOT a bitmask:
     *   1 = BitstreamInput   — read-only (codec reads stream)
     *   2 = ReferenceFrame   — read+write (codec may use as decode target)
     *   3 = OutputFrame      — read+write (codec writes decoded pixels)
     *   4 = Scratch          — read+write (RCB row buffers, colmv, etc)
     *
     * Old code did `if (Role & 2u)` which incorrectly treats 4 as read-only:
     *   4 & 2 == 0 → no WRITABLE flag → RCB write attempts fault on
     *   the write-port MMU.  Mapping all non-bitstream buffers as
     *   read+write is functionally equivalent to BSP (which uses
     *   IOMMU_READ | IOMMU_WRITE for all dma_buf attachments). */
    ULONG pteFlags = RK_PTE_PAGE_READABLE;
    if (Role != (ULONG)RkMppBufferUsageBitstreamInput)
        pteFlags |= RK_PTE_PAGE_WRITABLE;

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
     * data into the address space.  Phase 3a temporarily skipped this
     * because IOMMU MMIO would WHEA on read before the codec PD was
     * brought up; Phase 3b's cluster bring-up (0e9ec1e and friends)
     * fixed that, so the IOMMU at 0xFDC38700 is now reachable.  Without
     * this call, page tables exist in RAM but the IOMMU hardware never
     * gets DTE_ADDR / ENABLE_PAGING — rkvdec2 issues AXI reads at our
     * iovas that no one translates, decode silently never starts. */
    NTSTATUS enableStatus = RkIommuEnableHw(dev);
    if (!NT_SUCCESS(enableStatus)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: enable failed 0x%08x for first MapMdl\n",
                   enableStatus);
        /* Don't roll back the page tables — the consumer can still try
         * to use the iova if it has its own translation, and the next
         * MapMdl will retry RkIommuEnableHw. */
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
    /* See RkIommuMapMdl: the consumer passes its iommu->Header.Context. */
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (!dev->Domain) return STATUS_DEVICE_NOT_READY;
    KIRQL irql;

    /* Determine how many pages are mapped at this IOVA.  We reconstruct
     * the original allocation size by counting consecutive set bits in
     * IovaBitmap, bounded by the next set bit in IovaStartBitmap.  The
     * latter is critical for multi-File concurrency: without it the walk
     * extends past our range into a peer File's adjacent allocation,
     * RkIommuUnmapAt zeroes the peer's PTEs, and the peer's next DMA
     * faults.  See [[iommu_unmap_overunmaps_peer]] for the failure mode. */
    ULONG startPage = (ULONG)(Iova >> 12);
    const ULONG bitsPerWord = (ULONG)(sizeof(ULONG_PTR) * 8);
    ULONG pageCount = 0;

    KeAcquireSpinLock(&dev->Domain->Lock, &irql);

    /* Verify the caller's iova actually starts an allocation we issued.
     * Catches double-unmap and stray pointers before any state mutation. */
    if (!(dev->Domain->IovaStartBitmap[startPage / bitsPerWord] &
          ((ULONG_PTR)1 << (startPage % bitsPerWord)))) {
        KeReleaseSpinLock(&dev->Domain->Lock, irql);
        return STATUS_INVALID_PARAMETER;
    }

    for (ULONG pg = startPage; pg < RK_IOMMU_IOVA_PAGES; pg++) {
        ULONG w = pg / bitsPerWord;
        ULONG b = pg % bitsPerWord;
        if (!(dev->Domain->IovaBitmap[w] & ((ULONG_PTR)1 << b))) break;
        /* Stop at the next allocation's start — don't extend into a peer. */
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
        int n_mmu = (dev->MmioLength >= 0x80) ? 2 : 1;
        for (int mi = 0; mi < n_mmu; mi++) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG*)(dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
                RK_MMU_CMD_ZAP_CACHE);
        }
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
    /* ProviderContext = iommu->Header.Context (routes us to the right
     * iommu instance).  ConsumerContext is opaque — stored verbatim and
     * handed back to Callback when the IOMMU faults. */
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;

    dev->FaultCb       = Callback;
    dev->FaultCbCookie = ConsumerContext;
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * Snapshot — read-only sample of fault-related MMIO for post-kick diag.
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuSnapshot(_In_ PVOID ProviderContext,
                _Out_ PRKIOMMU_FAULT_SNAPSHOT Out)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    Out->Status        = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + RK_MMU_STATUS));
    Out->IntRawStat    = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + RK_MMU_INT_RAWSTAT));
    Out->IntStatus     = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + RK_MMU_INT_STATUS));
    Out->PageFaultAddr = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + RK_MMU_PAGE_FAULT_ADDR));
    Out->DteAddr       = READ_REGISTER_ULONG(
        (volatile ULONG*)(dev->MmioBase + RK_MMU_DTE_ADDR));
    if (dev->MmioLength >= 0x80) {
        volatile UCHAR *base1 = dev->MmioBase + 0x40;
        Out->Status1        = READ_REGISTER_ULONG((volatile ULONG*)(base1 + RK_MMU_STATUS));
        Out->IntRawStat1    = READ_REGISTER_ULONG((volatile ULONG*)(base1 + RK_MMU_INT_RAWSTAT));
        Out->IntStatus1     = READ_REGISTER_ULONG((volatile ULONG*)(base1 + RK_MMU_INT_STATUS));
        Out->PageFaultAddr1 = READ_REGISTER_ULONG((volatile ULONG*)(base1 + RK_MMU_PAGE_FAULT_ADDR));
        Out->DteAddr1       = READ_REGISTER_ULONG((volatile ULONG*)(base1 + RK_MMU_DTE_ADDR));
    }
    return STATUS_SUCCESS;
}

/* FlushTlb — issue ZAP_CACHE to drain stale TLB entries.  Called by
 * codec drivers right before each kick (matches BSP rkvdec2_run path).
 */
static NTSTATUS
RkIommuFlushTlb(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    if (!dev->PagingEnabled)    return STATUS_SUCCESS;
    int n_mmu = (dev->MmioLength >= 0x80) ? 2 : 1;
    for (int mi = 0; mi < n_mmu; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_ZAP_CACHE);
    }
    return STATUS_SUCCESS;
}

/* Reattach — Disable + Enable the IOMMU so the hardware drops walk-cache
 * state and re-fetches the page directory.  Mirrors Linux's
 * `mpp_iommu_refresh` (rockchip_iommu_disable + rockchip_iommu_enable).
 * Called by codec drivers from EvtFileCleanup / hang-recovery paths
 * after the codec is known quiescent. */
static NTSTATUS
RkIommuReattach(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    if (!dev->Domain)           return STATUS_DEVICE_NOT_READY;

    NTSTATUS s = RkIommuDisableHw(dev);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: Reattach disable phase failed 0x%08x\n", s);
        return s;
    }
    s = RkIommuEnableHw(dev);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: Reattach enable phase failed 0x%08x\n", s);
        return s;
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu_vdec: reattached (HID=RKCP%04x UID=%u)\n",
               dev->Hid, dev->Uid);
    return STATUS_SUCCESS;
}

/* ForceReset — issue RK_MMU_CMD_FORCE_RESET to all MMU instances under
 * STALL, then poll DTE_ADDR == 0 for completion.  Mirrors BSP
 * `rk_iommu_force_reset` (rockchip-iommu.c:564).  Resets the MMU's
 * internal state machine — walk caches, prefetcher, fault state — but
 * does NOT touch CRU/AXI/AHB/NIU bus blocks the wide
 * `Ccu.FullCoreReset0/1` would.  Caller MUST follow with `Reattach` to
 * reprogram DTE_ADDR (FORCE_RESET zeroes it out).
 *
 * Used as the soft tier of the two-tier session-end recovery: gentle
 * enough to leave HEVC's bus state intact when we just want to clear
 * H.264 codec FSM state. */
static NTSTATUS
RkIommuForceReset(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev || !dev->MmioBase) return STATUS_DEVICE_NOT_READY;

    int n_mmu = (dev->MmioLength >= 0x80) ? 2 : 1;

    /* STALL all instances first.  FORCE_RESET while a master has
     * in-flight AXI traffic produces undefined behaviour. */
    for (int mi = 0; mi < n_mmu; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_ENABLE_STALL);
        KeStallExecutionProcessor(20);
    }

    /* Issue FORCE_RESET to all instances. */
    for (int mi = 0; mi < n_mmu; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_FORCE_RESET);
    }

    /* Poll for DTE_ADDR == 0 on all instances — BSP's
     * rk_iommu_is_reset_done.  Cap at ~100 ms (matches BSP
     * RK_MMU_FORCE_RESET_TIMEOUT_US = 100000). */
    NTSTATUS rs = STATUS_SUCCESS;
    for (int mi = 0; mi < n_mmu; mi++) {
        ULONG poll = 0;
        for (poll = 0; poll < 100000; poll++) {
            ULONG dte = READ_REGISTER_ULONG(
                (volatile ULONG*)(dev->MmioBase + (mi * 0x40) + RK_MMU_DTE_ADDR));
            if (dte == 0) break;
            KeStallExecutionProcessor(1);
        }
        if (poll >= 100000) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                       "rkiommu_vdec: ForceReset MMU#%d DTE_ADDR didn't clear\n", mi);
            rs = STATUS_TIMEOUT;
        }
    }

    /* UN-STALL — even on timeout, leave the MMU in a known state. */
    for (int mi = 0; mi < n_mmu; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_DISABLE_STALL);
    }

    /* FORCE_RESET zeroed DTE_ADDR — paging is effectively off now even
     * though the PagingEnabled flag still says on.  Clear the flag so
     * the next RkIommuEnableHw doesn't short-circuit on the "already
     * enabled" check. */
    if (NT_SUCCESS(rs)) dev->PagingEnabled = FALSE;

    return rs;
}

/* Enable — bridge ifc ProviderContext to the internal RkIommuEnableHw
 * helper.  The internal handles Domain==NULL, idempotent re-enable and
 * sets PagingEnabled=TRUE. */
static NTSTATUS
RkIommuEnable(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    return RkIommuEnableHw(dev);
}

/* Disable — bridge to the internal RkIommuDisableHw helper which STALLs
 * all instances, writes RK_MMU_INT_MASK=0 (masks fault IRQ at the
 * device level, independent of the WDF interrupt vector), issues
 * DISABLE_PAGING, zeroes DTE_ADDR, and clears PagingEnabled.  Called
 * by rkvdec's ReleaseHardware BEFORE Ccu.DropCluster, so the IOMMU
 * MMIO is quiesced while clocks are still on. */
static NTSTATUS
RkIommuDisable(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    return RkIommuDisableHw(dev);
}

/* MaskIrq — disconnect our ISR from the kernel interrupt dispatcher via
 * WdfInterruptDisable.  Software-only operation, no MMIO.  Exposed for
 * ifc parity with rkiommu_av1d; rkvdec's ReleaseHardware does NOT call
 * this (cross-device WdfInterruptDisable from a D-state-adjacent path
 * races KMDF auto-disconnect; see commit 9a71579). */
static NTSTATUS
RkIommuMaskIrq(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (dev->Interrupt != NULL)
        WdfInterruptDisable(dev->Interrupt);
    return STATUS_SUCCESS;
}

/* UnmaskIrq — reconnect our ISR via WdfInterruptEnable.  Pair to
 * MaskIrq.  Exposed for ifc parity with rkiommu_av1d. */
static NTSTATUS
RkIommuUnmaskIrq(_In_ PVOID ProviderContext)
{
    PRKIOMMU_DEVICE dev = DevFromContext(ProviderContext);
    if (!dev) return STATUS_DEVICE_NOT_READY;
    if (dev->Interrupt != NULL)
        WdfInterruptEnable(dev->Interrupt);
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

    WDF_QUERY_INTERFACE_CONFIG cfg;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&cfg,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKIOMMU,
                                    NULL);
    return WdfDeviceAddQueryInterface(Device, &cfg);
}
