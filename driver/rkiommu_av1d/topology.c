/* driver/rkiommu_av1d/topology.c — client→IOMMU binding for AV1D cluster. */
#include <ntddk.h>
#include "topology.h"

typedef struct _RKIOMMU_BINDING {
    UINT32 ClientHid;
    UINT32 ClientUid;
    UINT32 IommuHid;
    UINT32 IommuUid;
} RKIOMMU_BINDING;

static const RKIOMMU_BINDING g_topology[] = {
    { 0x3560, 0,   /* AV1D (RKCP3560, UID 0) */
      0x3571, 0 }, /* A1MU (RKCP3571, UID 0) */
};

static const ULONG g_topologyCount =
    sizeof(g_topology) / sizeof(g_topology[0]);

_Use_decl_annotations_
NTSTATUS RkIommuLookupBinding(UINT32 ClientHid, UINT32 ClientUid,
                               UINT32 *IommuHid, UINT32 *IommuUid)
{
    for (ULONG i = 0; i < g_topologyCount; i++) {
        if (g_topology[i].ClientHid == ClientHid &&
            g_topology[i].ClientUid == ClientUid)
        {
            *IommuHid = g_topology[i].IommuHid;
            *IommuUid = g_topology[i].IommuUid;
            return STATUS_SUCCESS;
        }
    }
    return STATUS_NOT_FOUND;
}
