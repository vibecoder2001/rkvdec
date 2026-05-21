/* driver/rkiommu_vdec/device.h — per-instance RKIOMMU_DEVICE and global list API. */
#pragma once
#include <ntddk.h>
#include <wdf.h>
#include "../shared/iommu/pgtable.h"
#include "../shared/iommu/fault.h"
#include "../../shared/rkiommu_ifc.h"
#include "../shared/rkmpp/peer_attach.h"

/* Per-instance device context, embedded in the WDF device object. */
typedef struct _RKIOMMU_DEVICE {
    UINT32              Hid;            /* RKCP3570 */
    UINT32              Uid;            /* ACPI _UID */

    volatile UCHAR     *MmioBase;       /* PAGE_NOCACHE kernel VA */
    SIZE_T              MmioLength;

    PRKIOMMU_DOMAIN     Domain;         /* page table domain */

    BOOLEAN             PagingEnabled;  /* TRUE after RkIommuEnable() */

    /* BSP _DSD flags — set in EvtPrepareHardware based on HID/UID. */
    BOOLEAN             FlagDisableMmuReset;   /* rockchip,disable-mmu-reset */
    BOOLEAN             FlagEnableCmdRetry;    /* rockchip,enable-cmd-retry  */
    BOOLEAN             FlagShootdownEntire;   /* rockchip,shootdown-entire  */

    /* Registered fault callback (set by client via RegisterFaultHandler).
     * Protected by FaultLock: the (Cb, Cookie) PAIR must be read or
     * written atomically — without a lock the DPC could read a freshly
     * installed Cb against a stale (NULL or other consumer's) Cookie and
     * pass garbage to the wrong destructor.  Review #9. */
    KSPIN_LOCK             FaultLock;
    RKIOMMU_FAULT_CALLBACK FaultCb;
    PVOID                  FaultCbCookie;

    /* Fault context populated in ISR, consumed in DPC */
    RKIOMMU_FAULT_CTX   FaultCtx;

    WDFINTERRUPT        Interrupt;

    /* Global instance list linkage */
    LIST_ENTRY          ListEntry;

    /* ---- Master/slave coordination (multi-core dispatch) ----
     *
     * IsMaster: TRUE for the rkiommu instance that OWNS the page
     * tables.  RK3588 rkvdec: Hid=0x3570 Uid=9 is master, Uid=10 is
     * slave.  AV1 (Hid=0x3571, single instance) is always master in
     * its own topology.  Decided in EvtPrepareHardware via
     * RkMppIsMasterIommu().
     *
     * PtAttached: TRUE once this device's MMU registers are programmed
     * with a valid DTE_ADDR.  Master sets it after Domain alloc +
     * DTE programming.  Slave sets it after OnMasterArrival has
     * programmed its DTE with master's PdPhys.  Consumers (codec
     * drivers) wait on PtAttachedEvent before issuing IOCTLs.
     *
     * Consumers[]: master-only registry of slave + codec consumers
     * that hold the master interface.  Populated by
     * MasterRegisterQueryRemove.  Walked by master's
     * EvtDeviceQueryRemove (Phase 4) to cascade-detach. */
    BOOLEAN              IsMaster;
    /* Three roles total: IsMaster (UID 9, owns PT and publishes ifc),
     * IsCodecSlave (UID 10, attaches to master's PT), or "standalone"
     * (neither flag set — non-codec IOMMUs like VPMU/ENC that own
     * their own PT and do not participate in the master/slave system). */
    BOOLEAN              IsCodecSlave;
    BOOLEAN              PtAttached;
    KEVENT               PtAttachedEvent;
    /* StateLock + Tearing — guard transitions of {MasterOpen, Domain,
     * ShadowDomain, MasterFileObj, MasterIfcCtx, PtAttached} on the
     * slave path against the master-PnP arrival callback racing
     * EvtReleaseHardware.  Without this, arrival running on a worker
     * thread can be midway through RkIommuSlaveAttach while
     * ReleaseHardware tears the context down → UAF on ShadowDomain
     * or the slave context itself.  Tearing is set TRUE under
     * StateLock by ReleaseHardware before any teardown; arrival checks
     * it under the lock and bails. */
    KSPIN_LOCK           StateLock;
    BOOLEAN              Tearing;
    KSPIN_LOCK           ConsumersLock;
    struct {
        PVOID                                ConsumerCtx;
        PVOID                                Cb;   /* RKIOMMU_MASTER_QUERY_REMOVE_CB */
    } Consumers[4];
    ULONG                ConsumerCount;

    /* ---- Slave-side master client (multi-core dispatch) ----
     * Used only when !IsMaster.  Opened in OnMasterArrival,
     * closed in OnMasterQueryRemove (Phase 4) or ReleaseHardware. */
    RKMPP_PEER_WATCH         MasterWatch;
    BOOLEAN                  MasterOpen;
    PVOID                    MasterIfcCtx;      /* master's RKIOMMU_DEVICE* via ifc */
    PVOID                    MasterUnregisterFn;/* RKIOMMU_MASTER_UNREGISTER_QUERY_REMOVE — stored as PVOID to avoid pulling the master ifc header into device.h */
    PFILE_OBJECT             MasterFileObj;
    /* Shadow Domain populated when slave attaches to master.  Only
     * PdPhys is meaningful — all other fields NULL.  RkIommuEnableHw
     * reads PdPhys via Dev->Domain->PdPhys.  Allocated from non-paged
     * pool; freed when slave detaches.  Distinct from a real Domain so
     * that lifecycle is clearly per-slave-attach, not per-rkiommu-load. */
    PRKIOMMU_DOMAIN          ShadowDomain;
} RKIOMMU_DEVICE, *PRKIOMMU_DEVICE;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKIOMMU_DEVICE, RkIommuDeviceGet)

/* Called from DriverEntry / EvtDeviceAdd */
NTSTATUS RkIommuDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit);

/* Global instance list — searched by MapMdl to find the right IOMMU */
extern LIST_ENTRY  g_deviceList;
extern KSPIN_LOCK  g_deviceListLock;

/* Enable IOMMU paging on the hardware (lazy, called on first map).
 * Returns STATUS_SUCCESS on success, STATUS_DEVICE_HARDWARE_ERROR on timeout.
 *
 * Internal "Hw" helper.  The bare-named RkIommuEnable in ifc.c is the
 * public RKIOMMU_INTERFACE.Enable wrapper that resolves ProviderContext
 * to PRKIOMMU_DEVICE and then calls this. */
/* PASSIVE_LEVEL — issues KeStallExecutionProcessor 20µs stalls inside the
 * STALL/UN-STALL bracket; called from EvtPrepareHardware / EvtFileCleanup
 * / EvtReleaseHardware (all PASSIVE).  Review #7. */
_IRQL_requires_(PASSIVE_LEVEL)
NTSTATUS RkIommuEnableHw(_In_ PRKIOMMU_DEVICE Dev);

/* Disable IOMMU paging on the hardware.  STALLs all instances, masks
 * IRQs, sends DISABLE_PAGING, zeroes DTE_ADDR, and clears PagingEnabled
 * so the next MapMdl/Reattach lazily re-enables.  Idempotent. */
_IRQL_requires_(PASSIVE_LEVEL)
NTSTATUS RkIommuDisableHw(_In_ PRKIOMMU_DEVICE Dev);

/* Phase 2: master-only.  Publishes GUID_DEVINTERFACE_RKIOMMU_MASTER so
 * slave instances can query GetPageTableBase().  No-op for slaves. */
NTSTATUS RkIommuPublishMasterInterface(_In_ WDFDEVICE Device,
                                       _In_ PRKIOMMU_DEVICE Ctx);
