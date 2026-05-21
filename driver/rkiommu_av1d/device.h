/* driver/rkiommu_av1d/device.h — per-instance RKIOMMU_DEVICE for rkiommu_av1d.sys. */
#pragma once
#include <ntddk.h>
#include <wdf.h>
#include "../shared/iommu/pgtable.h"
#include "../shared/iommu/fault.h"
#include "../../shared/rkiommu_ifc.h"

typedef struct _RKIOMMU_DEVICE {
    UINT32              Hid;
    UINT32              Uid;
    volatile UCHAR     *MmioBase;
    SIZE_T              MmioLength;
    PRKIOMMU_DOMAIN     Domain;
    BOOLEAN             PagingEnabled;
    BOOLEAN             FlagDisableMmuReset;
    BOOLEAN             FlagEnableCmdRetry;
    BOOLEAN             FlagShootdownEntire;
    /* See rkiommu_vdec/device.h: (FaultCb, FaultCbCookie) is an
     * atomic pair protected by FaultLock.  Review #9. */
    KSPIN_LOCK             FaultLock;
    RKIOMMU_FAULT_CALLBACK FaultCb;
    PVOID                  FaultCbCookie;
    RKIOMMU_FAULT_CTX   FaultCtx;
    WDFINTERRUPT        Interrupt;
    LIST_ENTRY          ListEntry;
} RKIOMMU_DEVICE, *PRKIOMMU_DEVICE;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKIOMMU_DEVICE, RkIommuDeviceGet)

NTSTATUS RkIommuDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit);

extern LIST_ENTRY  g_deviceList;
extern KSPIN_LOCK  g_deviceListLock;

_IRQL_requires_(PASSIVE_LEVEL)
NTSTATUS RkIommuEnableHw(_In_ PRKIOMMU_DEVICE Dev);

_IRQL_requires_(PASSIVE_LEVEL)
NTSTATUS RkIommuDisableHw(_In_ PRKIOMMU_DEVICE Dev);
