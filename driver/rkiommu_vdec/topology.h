/* driver/rkiommu/topology.h — client→IOMMU binding lookup. */
#pragma once
#include <ntddk.h>

_IRQL_requires_max_(DISPATCH_LEVEL)
NTSTATUS RkIommuLookupBinding(
    _In_  UINT32  ClientHid,
    _In_  UINT32  ClientUid,
    _Out_ UINT32 *IommuHid,
    _Out_ UINT32 *IommuUid);
