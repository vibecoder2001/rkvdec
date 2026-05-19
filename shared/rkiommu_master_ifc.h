/* shared/rkiommu_master_ifc.h — In-kernel interface published by the
 * MASTER rkiommu_vdec instance (UID 9) so the SLAVE instance (UID 10)
 * can attach its MMU to the master's page-table root.  Mirrors BSP
 * Linux's mpp_rkvdec2_link.c:1342 (rkvdec2_attach_ccu): non-core-0
 * rkvdec2 cores reuse core 0's IOMMU domain by aliasing the domain
 * pointer.  We achieve the same by programming both MMU instances'
 * RK_MMU_DTE_ADDR with the same physical address.
 *
 * Spec: docs/superpowers/specs/2026-05-18-rkvdec-multicore-dispatch-
 * design.md, section "IOMMU shared page tables". */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKIOMMU_MASTER,
    0x8c19d4b2, 0x7f30, 0x4c89, 0xa9, 0x55, 0x21, 0x06, 0x44, 0x9e, 0x37, 0xb1);

/* v2: add UnregisterQueryRemove so consumers can scrub their callback
 * from the master's Consumers[] registry on their own teardown path.
 * Without this, a consumer that unloads first (e.g. RVD0 via ACPI _DEP
 * when master rkiommu is being disabled) leaves a stale callback ptr
 * pointing at a freed Device context — master's later cascade-detach
 * then invokes the stale callback → bugcheck → reboot to recover. */
#define RKIOMMU_MASTER_IFC_VERSION 2u

/* Returns the PHYSICAL address of the page-directory root that the
 * master programs into its own RK_MMU_DTE_ADDR.  Slave programs the
 * same value into its DTE_ADDR.  Stable for the lifetime of the
 * master device's PrepareHardware → ReleaseHardware cycle. */
typedef NTSTATUS (*RKIOMMU_MASTER_GET_PT_BASE)(
    _In_  PVOID    ProviderContext,
    _Out_ PULONG32 PdPhys);

/* Inverse query-remove notification — symmetric to peer worker. */
typedef VOID (*RKIOMMU_MASTER_QUERY_REMOVE_CB)(_In_ PVOID ConsumerContext);

typedef NTSTATUS (*RKIOMMU_MASTER_REGISTER_QUERY_REMOVE)(
    _In_ PVOID                          ProviderContext,
    _In_ PVOID                          ConsumerContext,
    _In_ RKIOMMU_MASTER_QUERY_REMOVE_CB Callback);

/* Match key is ConsumerContext (same value passed to RegisterQueryRemove).
 * Idempotent — unregistering a context that's already gone is a no-op. */
typedef VOID (*RKIOMMU_MASTER_UNREGISTER_QUERY_REMOVE)(
    _In_ PVOID ProviderContext,
    _In_ PVOID ConsumerContext);

typedef struct _RKIOMMU_MASTER_INTERFACE {
    INTERFACE                              Header;
    UINT32                                 Hid;   /* 0x3570 */
    UINT32                                 Uid;   /* 9 */
    RKIOMMU_MASTER_GET_PT_BASE             GetPageTableBase;
    RKIOMMU_MASTER_REGISTER_QUERY_REMOVE   RegisterQueryRemove;
    RKIOMMU_MASTER_UNREGISTER_QUERY_REMOVE UnregisterQueryRemove;   /* v2 */
} RKIOMMU_MASTER_INTERFACE, *PRKIOMMU_MASTER_INTERFACE;
