/* shared/rkiommu_ifc.h — In-kernel device interface exported by
 * rkiommu.sys for codec cores to consume.
 *
 * Phase 2: full MapMdl/UnmapMdl + fault registration.
 */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKIOMMU,
    0x4f9b1c23, 0x82a9, 0x4cd8, 0xb3, 0x14, 0x57, 0xa1, 0x0e, 0x44, 0x9d, 0x12);

#define RKIOMMU_IFC_VERSION 2u

typedef NTSTATUS (*RKIOMMU_QUERY_VERSION)(_Out_ PUINT32);

typedef NTSTATUS (*RKIOMMU_MAP_MDL)(
    _In_  PVOID ClientCookie,
    _In_  PMDL  Mdl,
    _In_  ULONG Role,
    _Out_ PULONG64 Iova);

typedef NTSTATUS (*RKIOMMU_UNMAP_MDL)(
    _In_ PVOID ClientCookie,
    _In_ ULONG64 Iova);

typedef VOID (*RKIOMMU_FAULT_CALLBACK)(
    _In_ PVOID ClientCookie,
    _In_ ULONG64 FaultIova,
    _In_ ULONG StatusReg);

typedef NTSTATUS (*RKIOMMU_REGISTER_FAULT)(
    _In_ PVOID ClientCookie,
    _In_ RKIOMMU_FAULT_CALLBACK Callback);

typedef struct _RKIOMMU_INTERFACE {
    INTERFACE                Header;
    RKIOMMU_QUERY_VERSION    QueryVersion;
    RKIOMMU_MAP_MDL          MapMdl;
    RKIOMMU_UNMAP_MDL        UnmapMdl;
    RKIOMMU_REGISTER_FAULT   RegisterFaultHandler;
} RKIOMMU_INTERFACE, *PRKIOMMU_INTERFACE;
