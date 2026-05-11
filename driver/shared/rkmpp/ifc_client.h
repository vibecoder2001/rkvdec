#pragma once

#include <ntddk.h>
#include <wdf.h>
#include "../../../shared/rkiommu_ifc.h"
#include "../../../shared/rkmpp_ccu_ifc.h"

typedef struct _RKMPP_IFC_CLIENT {
    RKIOMMU_INTERFACE   Iommu;
    RKMPP_CCU_INTERFACE Ccu;
    /* Provider file-object references — keep these alive for as long as
     * we hold the cached function pointers, otherwise the provider's
     * binary can unload and our pointers dangle (BSOD 0xCE). */
    PFILE_OBJECT        IommuFileObj;
    PFILE_OBJECT        CcuFileObj;
    BOOLEAN             IommuOpen;
    BOOLEAN             CcuOpen;
} RKMPP_IFC_CLIENT;

/* OpenIfcs needs the client's (Hid, Uid) so it can pick the rkiommu
 * instance that owns its translation domain (per topology). */
NTSTATUS RkMppOpenIfcs(_In_ WDFDEVICE Device,
                       _In_ UINT32 ClientHid,
                       _In_ UINT32 ClientUid,
                       _Out_ RKMPP_IFC_CLIENT *Out);
VOID     RkMppCloseIfcs(_Inout_ RKMPP_IFC_CLIENT *Out);
