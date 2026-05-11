/* driver/rkiommu/fault.h — IRQ ISR/DPC declarations for rkiommu.sys. */
#pragma once
#include <ntddk.h>
#include <wdf.h>
#include "../../../shared/rkiommu_ifc.h"

/* Fault context saved by the ISR and consumed by the DPC. */
typedef struct _RKIOMMU_FAULT_CTX {
    ULONG  IntStatus;       /* RK_MMU_INT_STATUS snapshot */
    ULONG  FaultIova;       /* RK_MMU_PAGE_FAULT_ADDR snapshot */
} RKIOMMU_FAULT_CTX;

EVT_WDF_INTERRUPT_ISR  RkIommuEvtIsr;
EVT_WDF_INTERRUPT_DPC  RkIommuEvtDpc;
