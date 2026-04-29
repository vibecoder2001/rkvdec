/* driver/rkiommu/topology.c — static client→IOMMU binding table.
 *
 * v1 wires a single entry: RVD0 (RKCP3550, _UID 0) → RD0M (RKCP3570, _UID 9).
 *
 * TODO (Phase 3): Confirm RD0M's exact _UID against the platform firmware.
 *   The value 9 is a placeholder derived from the ACPI DSDT topology for
 *   rkvdec0 on RK3588; the real value must be read from the firmware _UID
 *   for the RKCP3570 node that owns rkvdec0's IOMMU.
 *
 * TODO (Phase 3): When more client→IOMMU pairs are needed, expand the table
 *   and add real HID/UID translation in RkIommuLookupBinding.
 */
#include <ntddk.h>
#include "topology.h"

typedef struct _RKIOMMU_BINDING {
    UINT32 ClientHid;
    UINT32 ClientUid;
    UINT32 IommuHid;
    UINT32 IommuUid;
} RKIOMMU_BINDING;

/* ---------------------------------------------------------------------------
 * Topology table — one entry per client→IOMMU pair.
 *
 * Entry 0: RVD0 (rkvdec0 decoder core, RKCP3550 _UID 0)
 *          → RD0M (RKCP3570 IOMMU, _UID 9 — PLACEHOLDER, see TODO above)
 * --------------------------------------------------------------------------- */
static const RKIOMMU_BINDING g_topology[] = {
    { 0x3550, 0,   /* RVD0 — rkvdec0 decoder core (RKCP3550, UID 0) */
      0x3570, 9 }, /* RD0M — rkvdec0 IOMMU      (RKCP3570, UID 9 — PLACEHOLDER) */
};

static const ULONG g_topologyCount =
    sizeof(g_topology) / sizeof(g_topology[0]);

/* ---------------------------------------------------------------------------
 * RkIommuLookupBinding
 *
 * Returns the IOMMU HID and UID for the given client (ClientHid, ClientUid).
 *
 * STATUS_SUCCESS     — found; *IommuHid and *IommuUid set.
 * STATUS_NOT_FOUND   — no entry for this client.
 * --------------------------------------------------------------------------- */
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
