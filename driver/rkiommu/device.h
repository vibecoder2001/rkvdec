/* driver/rkiommu/device.h — per-instance RKIOMMU_DEVICE and global list API. */
#pragma once
#include <ntddk.h>
#include <wdf.h>
#include "pgtable.h"
#include "fault.h"
#include "../../shared/rkiommu_ifc.h"

/* Per-instance device context, embedded in the WDF device object. */
typedef struct _RKIOMMU_DEVICE {
    UINT32              Hid;            /* RKCP3570 or RKCP3571 */
    UINT32              Uid;            /* ACPI _UID */

    volatile UCHAR     *MmioBase;       /* PAGE_NOCACHE kernel VA */
    SIZE_T              MmioLength;

    PRKIOMMU_DOMAIN     Domain;         /* page table domain */

    BOOLEAN             PagingEnabled;  /* TRUE after RkIommuEnable() */

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

/* Enable IOMMU paging on the hardware (lazy, called on first map) */
_IRQL_requires_max_(DISPATCH_LEVEL)
VOID RkIommuEnable(_In_ PRKIOMMU_DEVICE Dev);
