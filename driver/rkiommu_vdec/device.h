/* driver/rkiommu_vdec/device.h — per-instance RKIOMMU_DEVICE and global list API. */
#pragma once
#include <ntddk.h>
#include <wdf.h>
#include "../shared/iommu/pgtable.h"
#include "../shared/iommu/fault.h"
#include "../../shared/rkiommu_ifc.h"

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

    /* Registered fault callback (set by client via RegisterFaultHandler) */
    RKIOMMU_FAULT_CALLBACK FaultCb;
    PVOID                  FaultCbCookie;

    /* Fault context populated in ISR, consumed in DPC */
    RKIOMMU_FAULT_CTX   FaultCtx;

    WDFINTERRUPT        Interrupt;

    /* Global instance list linkage */
    LIST_ENTRY          ListEntry;
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
_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS RkIommuEnableHw(_In_ PRKIOMMU_DEVICE Dev);

/* Disable IOMMU paging on the hardware.  STALLs all instances, masks
 * IRQs, sends DISABLE_PAGING, zeroes DTE_ADDR, and clears PagingEnabled
 * so the next MapMdl/Reattach lazily re-enables.  Idempotent.
 *
 * Internal "Hw" helper.  See RkIommuEnableHw for naming rationale. */
_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS RkIommuDisableHw(_In_ PRKIOMMU_DEVICE Dev);
