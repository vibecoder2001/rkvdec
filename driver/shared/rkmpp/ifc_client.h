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

/* Topology lookups for multi-core rkvdec2 dispatch.  See spec
 * docs/superpowers/specs/2026-05-18-rkvdec-multicore-dispatch-design.md,
 * section "Identification — codec ↔ IOMMU and master/slave". */

/* Returns TRUE if the given rkiommu (Hid, Uid) is the master instance
 * (the one that owns the page tables).  On RK3588: rkvdec uses
 * rkiommu Hid=0x3570 Uid=9 as master, Uid=10 as slave.  AV1 has only
 * one rkiommu instance — always master in its own topology. */
BOOLEAN RkMppIsMasterIommu(_In_ UINT32 IommuHid, _In_ UINT32 IommuUid);

/* TRUE only for codec slave IOMMU instance(s) that share a master's
 * page-table base.  On RK3588 rkvdec: UID 10 only.  Non-codec IOMMUs
 * (UIDs 0-8 on Hid=0x3570 — VPMU, ENC, etc.) own their own page tables
 * and are NEITHER master NOR slave; they take the original Domain-alloc
 * path.  AV1 has no slave (single codec instance). */
BOOLEAN RkMppIsCodecSlaveIommu(_In_ UINT32 IommuHid, _In_ UINT32 IommuUid);

/* For an rkvdec codec at (CodecHid, CodecUid), tell the caller whether
 * a peer rkvdec exists in the topology and, if so, what its (Hid, Uid)
 * is.  Used by RVD0 (Uid==0) to find RVD1 (Uid==1).  Returns FALSE for
 * cores that have no peer (AV1, or RVD1 looking for "its" peer — we
 * only dispatch master→slave). */
BOOLEAN RkMppLookupPeerCodec(_In_  UINT32 CodecHid,
                             _In_  UINT32 CodecUid,
                             _Out_ UINT32 *PeerHid,
                             _Out_ UINT32 *PeerUid);

/* Issue IRP_MN_QUERY_INTERFACE on `devObj` for `Guid` at `IfcVersion`,
 * filling `Buf` (BufLen bytes).  Exposed so peer-arrival callbacks in
 * codec drivers can query a newly-arrived peer device interface
 * directly by symbolic link → device object. */
NTSTATUS RkMppQueryOne(_In_ PDEVICE_OBJECT devObj,
                       _In_ const GUID *Guid,
                       _In_ USHORT IfcVersion,
                       _Out_writes_bytes_(BufLen) PVOID Buf,
                       _In_ USHORT BufLen);

struct _RKMPP_PEER_WORKER_INTERFACE;  /* fwd-decl to avoid include in client header */
NTSTATUS RkMppQueryPeerWorkerBySymlink(_In_ PUNICODE_STRING SymbolicLink,
                                       _Out_ struct _RKMPP_PEER_WORKER_INTERFACE *Out,
                                       _Out_ PFILE_OBJECT *OutFileObj);
