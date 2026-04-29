#pragma once

#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkiommu_ifc.h"
#include "../../shared/rkmpp_ccu_ifc.h"

typedef struct _RKMPP_IFC_CLIENT {
    RKIOMMU_INTERFACE   Iommu;
    RKMPP_CCU_INTERFACE Ccu;
    BOOLEAN             IommuOpen;
    BOOLEAN             CcuOpen;
} RKMPP_IFC_CLIENT;

NTSTATUS RkMppOpenIfcs(_In_ WDFDEVICE Device, _Out_ RKMPP_IFC_CLIENT *Out);
VOID     RkMppCloseIfcs(_Inout_ RKMPP_IFC_CLIENT *Out);
