# Phase 2 — In-kernel ifcs, Buffer Pool, No-op Job Round-trip

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring the three Phase 1 driver skeletons to life: rkmpp_ccu.sys raises and drops the rkv-decoder-v2 cluster on demand; rkiommu.sys maps and unmaps user buffers into the RVD0 IOMMU; rkmpp.sys exposes `IOCTL_RKMPP_ALLOC_BUFFER` / `FREE_BUFFER` / `SUBMIT_JOB` / `WAIT_JOB` and can round-trip a no-op job (a tiny register-list that just writes a magic value to a scratch register and signals completion). User mode validates the whole stack with a `rkmpp_smoke` tool that allocates buffers, writes a pattern, submits a no-op job, and re-reads the buffer. **Still no decode** — that lives in Phase 3+.

**Architecture:** This phase makes the three drivers cooperate. rkmpp.sys becomes the orchestrator: at `EvtPrepareHardware` it queries `GUID_DEVINTERFACE_RKIOMMU` and `GUID_DEVINTERFACE_RKMPP_CCU` to obtain function pointers into the other two drivers. Around any job batch, it calls `RaiseCluster` / `DropCluster`. For each allocated buffer it calls `MapMdl` / `UnmapMdl`. The buffer pool itself (DMA-coherent common buffers) lives in rkmpp.sys.

**Tech Stack:** Same as Phase 1. Adds: `WdfCommonBufferCreate` for DMA-coherent buffers, IO target / WDF interface query for cross-driver calls, `MmProbeAndLockPages` for the user-pages-bitstream input path.

**Reference:**
- Spec: `docs/superpowers/specs/2026-04-28-rkvdec-mft-h264-design.md` §3
- Phase 1 plan: `docs/superpowers/plans/2026-04-28-phase1-driver-skeleton.md`
- Linux IOMMU page-table format: `drivers/iommu/rockchip-iommu.c` (mainline) — single-level page directory + single-level page tables, 4KiB pages, 32-bit iova space.

---

## File structure additions

```
shared/
  rkmpp_ioctl.h          # MODIFY — add ALLOC/FREE/SUBMIT/WAIT codes + structs
  rkmpp_ccu_ifc.h        # MODIFY — add RaiseCluster / DropCluster
  rkiommu_ifc.h          # MODIFY — add MapMdl / UnmapMdl / RegisterFaultHandler
driver/rkiommu/
  pgtable.c              # NEW — Rockchip IOMMU page-table programming
  fault.c                # NEW — IRQ + DPC fault dispatch
  topology.c             # NEW — RVD0→RD0M static map for v1
  ifc.c                  # MODIFY — implement MapMdl/UnmapMdl/RegisterFaultHandler
  device.c               # NEW — splits MMIO + IRQ from driver.c
driver/rkmpp_ccu/
  ccu.c                  # NEW — clock/reset/power register writes
  ifc.c                  # MODIFY — implement Raise/DropCluster, AssertCoreReset
driver/rkmpp/
  ifc_client.c           # NEW — opens IOMMU and CCU ifcs at PrepareHardware
  bufpool.c              # NEW — buffer pool, ALLOC/FREE handlers
  job.c                  # NEW — register-list submit, ISR/DPC, WAIT
  ioctl.c                # MODIFY — wire new IOCTLs
tests/harness/
  rkmpp_smoke/           # NEW
    CMakeLists.txt
    main.cpp
    test_main.cpp
```

---

### Task 1: Extend `shared/` headers for the new ifc + IOCTL surface

**Files:**
- Modify: `shared/rkmpp_ioctl.h`
- Modify: `shared/rkmpp_ccu_ifc.h`
- Modify: `shared/rkiommu_ifc.h`

- [ ] **Step 1: Append to `shared/rkmpp_ioctl.h`**

```c
/* ---- Phase 2: buffer + job IOCTLs ---- */

#define IOCTL_RKMPP_ALLOC_BUFFER \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x801, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_FREE_BUFFER \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x802, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_SUBMIT_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x803, METHOD_BUFFERED, FILE_WRITE_ACCESS)
#define IOCTL_RKMPP_WAIT_JOB \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x804, METHOD_BUFFERED, FILE_WRITE_ACCESS)

typedef enum _RKMPP_BUFFER_USAGE {
    RkMppBufferUsageBitstreamInput = 1,
    RkMppBufferUsageReferenceFrame = 2,
    RkMppBufferUsageOutputFrame    = 3,
    RkMppBufferUsageScratch        = 4,  /* Phase 2 smoke test only */
} RKMPP_BUFFER_USAGE;

typedef struct _RKMPP_ALLOC_BUFFER_IN {
    UINT32 StructSize;
    UINT32 Size;            /* bytes */
    UINT32 Usage;           /* RKMPP_BUFFER_USAGE */
    UINT32 Reserved;
} RKMPP_ALLOC_BUFFER_IN;

typedef struct _RKMPP_ALLOC_BUFFER_OUT {
    UINT32 StructSize;
    UINT64 BufferHandle;    /* opaque to user; round-trips back via FREE/SUBMIT */
    UINT64 Iova;             /* device-visible iova; useful for telemetry */
    HANDLE SectionHandle;    /* user maps with NtMapViewOfSection / MapViewOfFile */
    UINT32 SizeRoundedUp;
    UINT32 Reserved;
} RKMPP_ALLOC_BUFFER_OUT;

typedef struct _RKMPP_FREE_BUFFER_IN {
    UINT64 BufferHandle;
} RKMPP_FREE_BUFFER_IN;

/* A register-list entry: write Value to register at Offset (relative to the core's MMIO base).
 * Phase 2 only supports plain writes.  Phase 3 will add buffer-handle substitution
 * (so the user-mode register builder can name a buffer by handle and the driver
 * patches its iova in).
 */
typedef struct _RKMPP_REG_WRITE {
    UINT32 Offset;
    UINT32 Value;
} RKMPP_REG_WRITE;

#define RKMPP_MAX_REG_WRITES 256
#define RKMPP_MAX_BUF_REFS    16

typedef struct _RKMPP_BUFFER_REF {
    UINT64 BufferHandle;
    UINT32 Role;            /* opaque tag, e.g. role index in register list */
    UINT32 Reserved;
} RKMPP_BUFFER_REF;

typedef struct _RKMPP_SUBMIT_JOB_IN {
    UINT32 StructSize;
    UINT32 RegWriteCount;
    UINT32 BufRefCount;
    UINT32 TimeoutMs;
    RKMPP_REG_WRITE Writes[RKMPP_MAX_REG_WRITES];
    RKMPP_BUFFER_REF BufRefs[RKMPP_MAX_BUF_REFS];
} RKMPP_SUBMIT_JOB_IN;

typedef struct _RKMPP_SUBMIT_JOB_OUT {
    UINT64 JobId;
} RKMPP_SUBMIT_JOB_OUT;

typedef struct _RKMPP_WAIT_JOB_IN {
    UINT64 JobId;
    UINT32 TimeoutMs;
    UINT32 Reserved;
} RKMPP_WAIT_JOB_IN;

typedef struct _RKMPP_WAIT_JOB_OUT {
    NTSTATUS Status;        /* STATUS_SUCCESS, STATUS_DEVICE_HUNG, ... */
    UINT32   HardwareStatus;
    UINT64   ElapsedQpc;
} RKMPP_WAIT_JOB_OUT;
```

- [ ] **Step 2: Extend `shared/rkmpp_ccu_ifc.h`**

Replace the existing `RKMPP_CCU_INTERFACE` definition with:

```c
#define RKMPP_CCU_IFC_VERSION 2u

typedef NTSTATUS (*RKMPP_CCU_QUERY_VERSION)(_Out_ PUINT32);
typedef NTSTATUS (*RKMPP_CCU_RAISE_CLUSTER) (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_DROP_CLUSTER)  (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_ASSERT_RESET)  (_In_ PVOID ClientCookie);
typedef NTSTATUS (*RKMPP_CCU_DEASSERT_RESET)(_In_ PVOID ClientCookie);

typedef struct _RKMPP_CCU_INTERFACE {
    INTERFACE                 Header;
    RKMPP_CCU_QUERY_VERSION   QueryVersion;
    RKMPP_CCU_RAISE_CLUSTER   RaiseCluster;
    RKMPP_CCU_DROP_CLUSTER    DropCluster;
    RKMPP_CCU_ASSERT_RESET    AssertCoreReset;
    RKMPP_CCU_DEASSERT_RESET  DeassertCoreReset;
} RKMPP_CCU_INTERFACE, *PRKMPP_CCU_INTERFACE;
```

`ClientCookie` is the calling driver's `PDEVICE_OBJECT` so the CCU can refcount per-client.

- [ ] **Step 3: Extend `shared/rkiommu_ifc.h`**

Replace the existing `RKIOMMU_INTERFACE` with:

```c
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
```

- [ ] **Step 4: Compile-check**

Run: `cl /nologo /W4 /WX /Zs /I shared shared/rkmpp_ioctl.h shared/rkmpp_ccu_ifc.h shared/rkiommu_ifc.h`
Expected: 0 errors, 0 warnings.

- [ ] **Step 5: Commit**

```bash
git add shared/
git commit -m "phase2: extend shared headers — buffer/job IOCTLs, full CCU/IOMMU ifcs"
```

---

### Task 2: rkmpp_ccu.sys — implement RaiseCluster/DropCluster (RDCC only)

**Files:**
- Create: `driver/rkmpp_ccu/ccu.c`
- Modify: `driver/rkmpp_ccu/driver.c` — split MMIO map into a PrepareHardware
- Modify: `driver/rkmpp_ccu/ifc.c` — wire the new function pointers
- Modify: `driver/rkmpp_ccu/rkmpp_ccu.vcxproj` — add ccu.c

The implementation here is **RKCP3503-specific** (the rkv-decoder-v2 cluster); RKCP3501 / RKCP3502 keep their no-op ifc until those clusters are actually used.

- [ ] **Step 1: Write `driver/rkmpp_ccu/ccu.c`**

```c
/* driver/rkmpp_ccu/ccu.c — RKCP3503 (RDCC) clock/reset/power.
 *
 * The RDCC MMIO layout is taken from the Linux mainline driver:
 *   drivers/clk/rockchip/clk-rk3588.c  (cluster CRU registers)
 *   drivers/soc/rockchip/pm_domains.c  (rkv-decoder-v2 power domain)
 *
 * For Phase 2 we only need three operations against the RDCC block:
 *   - cluster ungate / gate (single CLK_GATE register, two bits)
 *   - core reset assert/deassert (single SOFTRST register, one bit per core)
 *   - power-domain raise/drop is handled by the platform's PEP -- we model
 *     it here as a refcounted call into ZwPowerInformation if needed; for
 *     the very first bring-up, ungating the clocks is sufficient because the
 *     domain is left on by UEFI.
 *
 * Concrete register offsets are filled in from the TRM / mainline driver in
 * Step 2.  In Step 1 we set up the structure so the offsets are the only
 * thing left to confirm.
 */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ccu_ifc.h"

typedef struct _RDCC_REGS {
    /* Offsets within the CCU MMIO range. */
    ULONG ClkGate;        /* RDCC clock gate register */
    ULONG ClkGateMask;    /* Bits we own (one per core) */
    ULONG SoftReset;      /* RDCC soft-reset register */
    ULONG SoftResetMask;  /* Bits we own (one per core) */
} RDCC_REGS;

/* TODO(phase2-step2): confirm these against rk3588 TRM / Linux source. */
static const RDCC_REGS g_rdcc = {
    .ClkGate       = 0x0000,
    .ClkGateMask   = 0x00000003,  /* RVD0 + RVD1 */
    .SoftReset     = 0x0010,
    .SoftResetMask = 0x00000003,
};

NTSTATUS RkMppCcuRaiseCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);  /* refcount lives in ifc.c */
    return STATUS_SUCCESS;        /* programmed in Step 2 */
}

NTSTATUS RkMppCcuDropCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuAssertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuQueryVersion(_Out_ PUINT32 v) { *v = RKMPP_CCU_IFC_VERSION; return STATUS_SUCCESS; }
```

- [ ] **Step 2: Confirm offsets and program them**

Look up the RKCP3503 (rkv-decoder-v2 CCU) clock-gate and soft-reset register offsets in `linux:drivers/clk/rockchip/clk-rk3588.c` (search for `rkv-decoder-v2` or `rkvdec_ccu`). Update `g_rdcc` in `ccu.c` with the real offsets, then implement the bodies:

```c
/* Replacement bodies — assumes a global g_mmio pointer set by the
 * EvtPrepareHardware in driver.c (added in Step 4 below). */
extern volatile UCHAR *g_rdcc_mmio;
extern LONG            g_raise_refcount;

NTSTATUS RkMppCcuRaiseCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (InterlockedIncrement(&g_raise_refcount) == 1) {
        ULONG v = READ_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate));
        WRITE_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate),
                             v & ~g_rdcc.ClkGateMask);  /* 0 = ungated */
    }
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDropCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (InterlockedDecrement(&g_raise_refcount) == 0) {
        ULONG v = READ_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate));
        WRITE_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.ClkGate),
                             v | g_rdcc.ClkGateMask);   /* 1 = gated */
    }
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuAssertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    ULONG v = READ_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset));
    WRITE_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset),
                         v | g_rdcc.SoftResetMask);
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    ULONG v = READ_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset));
    WRITE_REGISTER_ULONG((volatile ULONG*)(g_rdcc_mmio + g_rdcc.SoftReset),
                         v & ~g_rdcc.SoftResetMask);
    return STATUS_SUCCESS;
}
```

- [ ] **Step 3: Modify `driver/rkmpp_ccu/driver.c` to map MMIO**

Add `EVT_WDF_DEVICE_PREPARE_HARDWARE` and `EVT_WDF_DEVICE_RELEASE_HARDWARE` callbacks that map the first memory descriptor and store the base in a single global `g_rdcc_mmio` (we only have one RDCC in v1; multiple-CCU-instance refactor lands when adding RECC / JPCC). Define and zero-initialise `g_raise_refcount`.

(Concrete code: copy the `EvtPrepareHardware` from `rkmpp/device.c` Phase 1 Task 6 Step 4, drop the ACPI ID parse, and write to `g_rdcc_mmio` instead of a per-context field.)

- [ ] **Step 4: Wire `ifc.c`**

Replace the placeholder `RKMPP_CCU_INTERFACE` setup with all the new function pointers:

```c
ifc.QueryVersion       = RkMppCcuQueryVersion;
ifc.RaiseCluster       = RkMppCcuRaiseCluster;
ifc.DropCluster        = RkMppCcuDropCluster;
ifc.AssertCoreReset    = RkMppCcuAssertCoreReset;
ifc.DeassertCoreReset  = RkMppCcuDeassertCoreReset;
```

- [ ] **Step 5: Build with code analysis**

Run: `msbuild driver\rkmpp_ccu\rkmpp_ccu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings.

- [ ] **Step 6: Commit**

```bash
git add driver/rkmpp_ccu/ shared/rkmpp_ccu_ifc.h
git commit -m "phase2: rkmpp_ccu — RDCC raise/drop/reset implementation"
```

---

### Task 3: rkiommu.sys — page table + MapMdl/UnmapMdl

**Files:**
- Create: `driver/rkiommu/pgtable.c`
- Create: `driver/rkiommu/topology.c`
- Create: `driver/rkiommu/fault.c`
- Create: `driver/rkiommu/device.c`
- Modify: `driver/rkiommu/ifc.c` — implement MapMdl/UnmapMdl/RegisterFaultHandler
- Modify: `driver/rkiommu/rkiommu.vcxproj` — add new files

The Rockchip IOMMU uses a two-level page table: 1024-entry directory of 4KiB physical addresses, each pointing to a 1024-entry page table of 4KiB pages. iova space is 32-bit. Reference: `linux:drivers/iommu/rockchip-iommu.c`.

- [ ] **Step 1: Write `driver/rkiommu/pgtable.c`**

```c
/* driver/rkiommu/pgtable.c — Rockchip IOMMU two-level page table. */
#include <ntddk.h>
#include <wdf.h>

#define RK_IOMMU_PG_SHIFT       12
#define RK_IOMMU_PG_SIZE        (1u << RK_IOMMU_PG_SHIFT)
#define RK_IOMMU_PT_ENTRIES     1024u
#define RK_IOMMU_PD_ENTRIES     1024u

#define RK_PTE_VALID            (1u << 0)
#define RK_PTE_WRITE            (1u << 2)
#define RK_PDE_VALID            (1u << 0)

typedef struct _RKIOMMU_DOMAIN {
    PMDL    PdMdl;          /* page directory backing */
    PVOID   PdVa;           /* kernel-VA of the PD */
    ULONG_PTR PdPa;         /* physical address of the PD (used to program DTE_ADDR) */
    PMDL    PtMdls[RK_IOMMU_PD_ENTRIES];   /* page-table MDLs */
    PVOID   PtVas [RK_IOMMU_PD_ENTRIES];
    KSPIN_LOCK Lock;
} RKIOMMU_DOMAIN;

NTSTATUS RkIommuDomainCreate(_Out_ RKIOMMU_DOMAIN **OutDomain)
{
    RKIOMMU_DOMAIN *d = ExAllocatePool2(POOL_FLAG_NON_PAGED,
                                        sizeof(*d), 'mido');
    if (!d) return STATUS_INSUFFICIENT_RESOURCES;
    KeInitializeSpinLock(&d->Lock);

    PHYSICAL_ADDRESS lo = {0}, hi; hi.QuadPart = MAXULONGLONG;
    PHYSICAL_ADDRESS boundary = {0};
    d->PdVa = MmAllocateContiguousNodeMemory(RK_IOMMU_PD_ENTRIES * sizeof(ULONG),
                                             lo, hi, boundary,
                                             PAGE_READWRITE | PAGE_NOCACHE,
                                             MM_ANY_NODE_OK);
    if (!d->PdVa) { ExFreePool(d); return STATUS_INSUFFICIENT_RESOURCES; }
    RtlZeroMemory(d->PdVa, RK_IOMMU_PD_ENTRIES * sizeof(ULONG));
    d->PdPa = MmGetPhysicalAddress(d->PdVa).LowPart;
    *OutDomain = d;
    return STATUS_SUCCESS;
}

VOID RkIommuDomainDestroy(_In_ RKIOMMU_DOMAIN *d)
{
    for (ULONG i = 0; i < RK_IOMMU_PD_ENTRIES; i++) {
        if (d->PtVas[i]) MmFreeContiguousMemory(d->PtVas[i]);
    }
    if (d->PdVa) MmFreeContiguousMemory(d->PdVa);
    ExFreePool(d);
}

/* Allocate a page table for PD index, if not already present. */
static NTSTATUS RkIommuEnsurePt(_In_ RKIOMMU_DOMAIN *d, _In_ ULONG pdi)
{
    if (d->PtVas[pdi]) return STATUS_SUCCESS;
    PHYSICAL_ADDRESS lo = {0}, hi; hi.QuadPart = MAXULONGLONG;
    PHYSICAL_ADDRESS boundary = {0};
    PVOID va = MmAllocateContiguousNodeMemory(RK_IOMMU_PT_ENTRIES * sizeof(ULONG),
                                              lo, hi, boundary,
                                              PAGE_READWRITE | PAGE_NOCACHE,
                                              MM_ANY_NODE_OK);
    if (!va) return STATUS_INSUFFICIENT_RESOURCES;
    RtlZeroMemory(va, RK_IOMMU_PT_ENTRIES * sizeof(ULONG));
    d->PtVas[pdi] = va;

    ULONG pa = MmGetPhysicalAddress(va).LowPart;
    ((ULONG*)d->PdVa)[pdi] = pa | RK_PDE_VALID;
    return STATUS_SUCCESS;
}

/* Map an MDL into the IOMMU starting at the given iova.  Caller picks iova. */
NTSTATUS RkIommuMapAt(_In_ RKIOMMU_DOMAIN *d,
                     _In_ ULONG64 Iova,
                     _In_ PMDL Mdl)
{
    if (Iova & (RK_IOMMU_PG_SIZE - 1)) return STATUS_INVALID_PARAMETER;
    ULONG pageCount = ADDRESS_AND_SIZE_TO_SPAN_PAGES(MmGetMdlVirtualAddress(Mdl),
                                                     MmGetMdlByteCount(Mdl));
    PPFN_NUMBER pfns = MmGetMdlPfnArray(Mdl);

    KIRQL irql; KeAcquireSpinLock(&d->Lock, &irql);
    NTSTATUS status = STATUS_SUCCESS;
    for (ULONG i = 0; i < pageCount; i++) {
        ULONG pdi = (ULONG)((Iova >> 22) & 0x3ff) + (i / RK_IOMMU_PT_ENTRIES);
        ULONG pti = (ULONG)((Iova >> 12) & 0x3ff) + (i % RK_IOMMU_PT_ENTRIES);
        status = RkIommuEnsurePt(d, pdi);
        if (!NT_SUCCESS(status)) break;
        ULONG pa = (ULONG)(pfns[i] << RK_IOMMU_PG_SHIFT);
        ((ULONG*)d->PtVas[pdi])[pti] = pa | RK_PTE_VALID | RK_PTE_WRITE;
    }
    KeReleaseSpinLock(&d->Lock, irql);
    return status;
}

NTSTATUS RkIommuUnmapAt(_In_ RKIOMMU_DOMAIN *d, _In_ ULONG64 Iova, _In_ ULONG PageCount)
{
    KIRQL irql; KeAcquireSpinLock(&d->Lock, &irql);
    for (ULONG i = 0; i < PageCount; i++) {
        ULONG pdi = (ULONG)((Iova >> 22) & 0x3ff) + (i / RK_IOMMU_PT_ENTRIES);
        ULONG pti = (ULONG)((Iova >> 12) & 0x3ff) + (i % RK_IOMMU_PT_ENTRIES);
        if (d->PtVas[pdi]) ((ULONG*)d->PtVas[pdi])[pti] = 0;
    }
    KeReleaseSpinLock(&d->Lock, irql);
    return STATUS_SUCCESS;
}
```

> **Important caveat to fix in Step 4:** the loop above picks the iova page-by-page but does not account for the case where the contiguous iova range crosses a 4 MiB PD-entry boundary. The arithmetic `(Iova >> 22) & 0x3ff) + (i / 1024)` handles that, but the `(Iova >> 12) & 0x3ff` term needs to wrap modulo 1024 — which the `i % 1024` already implies because we re-derive `pdi` from `i / 1024`. However the **initial** `pti` is only correct for `i = 0`; for `i > 0` we want `pti = (i % 1024)` once we've incremented past the first PD entry. Fix in Step 4.

- [ ] **Step 2: Write `driver/rkiommu/topology.c`**

```c
/* Static "client ACPI handle → IOMMU _UID" table for RK3588.
 * Phase 2 only wires RVD0 → RD0M.  Other entries can be filled in as
 * additional codecs come online; the IOMMU driver consults this table at
 * MapMdl time to pick the right per-_UID domain.
 */
#include <ntddk.h>

typedef struct _RKIOMMU_CLIENT_BINDING {
    UINT32 ClientHid; UINT32 ClientUid;
    UINT32 IommuHid;  UINT32 IommuUid;  /* the IOMMU instance that serves the client */
} RKIOMMU_CLIENT_BINDING;

static const RKIOMMU_CLIENT_BINDING g_bindings[] = {
    /* RVD0 (RKCP3550 UID 0) → RD0M (RKCP3570 UID for RD0M). */
    { 0x3550, 0,  0x3570, 9 },   /* TODO(phase2-step5): confirm RD0M _UID */
    { 0x3550, 1,  0x3570, 10 },  /* TODO: confirm RD1M _UID */
};

NTSTATUS RkIommuLookupBinding(UINT32 ClientHid, UINT32 ClientUid,
                              UINT32 *IommuHid, UINT32 *IommuUid)
{
    for (ULONG i = 0; i < ARRAYSIZE(g_bindings); i++) {
        if (g_bindings[i].ClientHid == ClientHid &&
            g_bindings[i].ClientUid == ClientUid) {
            *IommuHid = g_bindings[i].IommuHid;
            *IommuUid = g_bindings[i].IommuUid;
            return STATUS_SUCCESS;
        }
    }
    return STATUS_NOT_FOUND;
}
```

- [ ] **Step 3: Write `driver/rkiommu/device.c` and `fault.c`**

`device.c` mirrors rkmpp's per-instance device logic from Phase 1 (parse HID + _UID, map MMIO, register `GUID_DEVINTERFACE_RKIOMMU`) plus instantiates one `RKIOMMU_DOMAIN` per device and connects the IOMMU IRQ to a DPC handler in `fault.c`.

`fault.c`:

```c
/* driver/rkiommu/fault.c — IRQ + DPC, dispatches to client callback. */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkiommu_ifc.h"

typedef struct _RKIOMMU_FAULT_CTX {
    RKIOMMU_FAULT_CALLBACK Callback;
    PVOID                  ClientCookie;
    volatile ULONG64       LastFaultIova;
    volatile ULONG         LastStatus;
} RKIOMMU_FAULT_CTX;

BOOLEAN RkIommuEvtIsr(_In_ WDFINTERRUPT Interrupt, _In_ ULONG MessageId)
{
    UNREFERENCED_PARAMETER(MessageId);
    /* TODO(phase2-step5): read STATUS reg, capture FAULT_ADDR, ack the IRQ.
     * Push the values into the queued DPC ctx and request the DPC. */
    WdfInterruptQueueDpcForIsr(Interrupt);
    return TRUE;
}

VOID RkIommuEvtDpc(_In_ WDFINTERRUPT Interrupt, _In_ WDFOBJECT AssociatedObject)
{
    UNREFERENCED_PARAMETER(AssociatedObject);
    RKIOMMU_FAULT_CTX *ctx = /* WdfObjectGetTypedContext(... per-interrupt ctx ...) */ NULL;
    if (ctx && ctx->Callback) {
        ctx->Callback(ctx->ClientCookie, ctx->LastFaultIova, ctx->LastStatus);
    }
}
```

- [ ] **Step 4: Implement the ifc bodies in `driver/rkiommu/ifc.c`**

```c
NTSTATUS RkIommuMapMdl(_In_ PVOID ClientCookie, _In_ PMDL Mdl,
                      _In_ ULONG Role, _Out_ PULONG64 OutIova)
{
    /* 1. ClientCookie is the client driver's PDEVICE_OBJECT.  Translate it
     *    to a (ClientHid, ClientUid) by walking up to its physical device
     *    object and reading its hardware-id property (same parse as Phase 1).
     * 2. RkIommuLookupBinding() to find the IOMMU instance that serves it.
     * 3. Find the RKIOMMU_DEVICE for that (Hid, Uid) — kept in a global
     *    list registered at EvtDeviceAdd time.
     * 4. Allocate an iova range from a per-device bitmap allocator
     *    (32-bit space, 4KiB pages, simple first-fit is fine for v1).
     * 5. RkIommuMapAt(domain, iova, mdl).  Make sure to fix the (Iova >> 12)
     *    starting-pti arithmetic flagged in Task 3 Step 1: compute
     *      first_pti = (Iova >> 12) & 0x3ff;
     *    and after the first-PT cross-over set pti = i % 1024 directly.
     * 6. Return iova in *OutIova.
     */
    UNREFERENCED_PARAMETER(Role);
    /* ... implement per the comment ... */
    return STATUS_NOT_IMPLEMENTED;  /* fill in */
}

NTSTATUS RkIommuUnmapMdl(_In_ PVOID ClientCookie, _In_ ULONG64 Iova) { /* ... */ return STATUS_NOT_IMPLEMENTED; }
NTSTATUS RkIommuRegisterFault(_In_ PVOID ClientCookie, _In_ RKIOMMU_FAULT_CALLBACK cb)
{
    /* Find the RKIOMMU_FAULT_CTX for the IOMMU serving this client and
     * stash (cb, ClientCookie). */
    UNREFERENCED_PARAMETER(ClientCookie);
    UNREFERENCED_PARAMETER(cb);
    return STATUS_NOT_IMPLEMENTED;  /* fill in */
}
```

The `STATUS_NOT_IMPLEMENTED` placeholders are not allowed in a final commit — Step 4's deliverable is to replace each with real code following the numbered comment. Use the iova-bitmap allocator pattern from `linux:drivers/iommu/rockchip-iommu.c::rk_iommu_map_iova` for reference; a 1MiB iova bitmap is plenty for v1 (2^20 pages = 4 GiB iova space).

- [ ] **Step 5: Confirm RD0M / RD1M `_UID`s on real hardware**

On the RK3588 target, after Phase 1's rkiommu deployment:

```
Get-PnpDevice -Class System | Where-Object { $_.HardwareID -match "RKCP3570" } | ForEach-Object {
    $u = (Get-PnpDeviceProperty $_ -KeyName 'DEVPKEY_Device_UINumber').Data
    [pscustomobject]@{ Name = $_.FriendlyName; UID = $u }
}
```

Cross-reference against the firmware's ACPI listing for RD0M/RD1M and update the TODOs in `topology.c` Step 2 with the real UIDs.

- [ ] **Step 6: Build with code analysis**

Run: `msbuild driver\rkiommu\rkiommu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings.

- [ ] **Step 7: Commit**

```bash
git add driver/rkiommu/ shared/rkiommu_ifc.h
git commit -m "phase2: rkiommu — page table + MapMdl/UnmapMdl + fault dispatch"
```

---

### Task 4: rkmpp.sys — open the IOMMU and CCU ifcs at PrepareHardware

**Files:**
- Create: `driver/rkmpp/ifc_client.c`
- Modify: `driver/rkmpp/device.c` — call into ifc_client at PrepareHardware
- Modify: `driver/rkmpp/rkmpp.vcxproj` — add ifc_client.c

- [ ] **Step 1: Write `driver/rkmpp/ifc_client.c`**

```c
/* driver/rkmpp/ifc_client.c — opens the rkiommu and rkmpp_ccu ifcs. */
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

static NTSTATUS QueryByGuid(WDFDEVICE Device,
                            const GUID *Guid, PVOID Buf, ULONG BufLen)
{
    PIRP irp = IoAllocateIrp(WdfDeviceWdmGetAttachedDevice(Device)->StackSize, FALSE);
    if (!irp) return STATUS_INSUFFICIENT_RESOURCES;
    PIO_STACK_LOCATION sl = IoGetNextIrpStackLocation(irp);
    sl->MajorFunction = IRP_MJ_PNP;
    sl->MinorFunction = IRP_MN_QUERY_INTERFACE;
    sl->Parameters.QueryInterface.InterfaceType = (LPGUID)Guid;
    sl->Parameters.QueryInterface.Size          = (USHORT)BufLen;
    sl->Parameters.QueryInterface.Version       = 2;  /* Phase 2 ifcs */
    sl->Parameters.QueryInterface.Interface     = (PINTERFACE)Buf;
    sl->Parameters.QueryInterface.InterfaceSpecificData = NULL;
    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;

    /* Walk the PnP namespace: query each rkiommu / rkmpp_ccu instance until
     * one accepts the query.  In Phase 2 the rkiommu driver picks the right
     * instance internally based on the caller's PDEVICE_OBJECT, so any
     * instance will do — we send the query to the first device matching
     * GUID_DEVINTERFACE_RK{IOMMU,MPP_CCU}.
     *
     * Implementation sketch: use IoGetDeviceInterfaces() to enumerate the
     * device interface, IoGetDeviceObjectPointer() to get its FILE_OBJECT
     * and DEVICE_OBJECT, then IoCallDriver() the IRP above.
     *
     * Real code lives below — this comment is the contract.
     */
    /* ... ~40 lines of IoGetDeviceInterfaces + IoCallDriver ... */
    IoFreeIrp(irp);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppOpenIfcs(_In_ WDFDEVICE Device, _Out_ RKMPP_IFC_CLIENT *Out)
{
    RtlZeroMemory(Out, sizeof(*Out));
    NTSTATUS s;
    s = QueryByGuid(Device, &GUID_DEVINTERFACE_RKIOMMU, &Out->Iommu, sizeof(Out->Iommu));
    if (!NT_SUCCESS(s)) return s;
    Out->IommuOpen = TRUE;

    s = QueryByGuid(Device, &GUID_DEVINTERFACE_RKMPP_CCU, &Out->Ccu, sizeof(Out->Ccu));
    if (!NT_SUCCESS(s)) { Out->Iommu.Header.InterfaceDereference(Out->Iommu.Header.Context); return s; }
    Out->CcuOpen = TRUE;
    return STATUS_SUCCESS;
}

VOID RkMppCloseIfcs(_Inout_ RKMPP_IFC_CLIENT *c)
{
    if (c->IommuOpen) c->Iommu.Header.InterfaceDereference(c->Iommu.Header.Context);
    if (c->CcuOpen)   c->Ccu.Header.InterfaceDereference(c->Ccu.Header.Context);
    RtlZeroMemory(c, sizeof(*c));
}
```

- [ ] **Step 2: Call `RkMppOpenIfcs` from `RkMppEvtPrepareHardware`**

Add an `RKMPP_IFC_CLIENT Ifcs;` field to `RKMPP_DEVICE` in `device.c`, call `RkMppOpenIfcs(Device, &ctx->Ifcs)` after MMIO mapping, and `RkMppCloseIfcs(&ctx->Ifcs)` from `RkMppEvtReleaseHardware`. Verify version: `ctx->Ifcs.Iommu.Header.Version` and `ctx->Ifcs.Ccu.Header.Version` must both be `2`; otherwise return `STATUS_REVISION_MISMATCH`.

- [ ] **Step 3: Verify on hardware**

Build all three drivers, deploy, then check the kernel log: there should be no `STATUS_NOT_FOUND` / `STATUS_INVALID_DEVICE_REQUEST` from the IRP_MN_QUERY_INTERFACE path. Add a debug print after `RkMppOpenIfcs` succeeds:

```c
DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
           "rkmpp: ifcs opened (iommu v%u, ccu v%u)\n",
           ctx->Ifcs.Iommu.Header.Version, ctx->Ifcs.Ccu.Header.Version);
```

Expected on the target: `rkmpp: ifcs opened (iommu v2, ccu v2)` for RVD0.

- [ ] **Step 4: Commit**

```bash
git add driver/rkmpp/
git commit -m "phase2: rkmpp — open iommu and ccu in-kernel ifcs at PrepareHardware"
```

---

### Task 5: rkmpp.sys — buffer pool, ALLOC_BUFFER, FREE_BUFFER

**Files:**
- Create: `driver/rkmpp/bufpool.c`
- Modify: `driver/rkmpp/ioctl.c` — wire IOCTL_RKMPP_ALLOC_BUFFER / IOCTL_RKMPP_FREE_BUFFER
- Modify: `driver/rkmpp/rkmpp.vcxproj` — add bufpool.c

- [ ] **Step 1: Write `driver/rkmpp/bufpool.c`**

```c
/* driver/rkmpp/bufpool.c — DMA-coherent buffer pool.
 *
 * Each allocation:
 *   - allocates a contiguous physical region (MmAllocateContiguousNodeMemory)
 *   - builds an MDL over it
 *   - creates a section object and maps a view into the calling process so
 *     user mode can read/write it
 *   - asks rkiommu to map the MDL into the device's IOMMU, returning iova
 *
 * The kernel side keeps a per-file-object list of buffers so EvtFileCleanup
 * can free leaks.
 */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ioctl.h"
#include "../../shared/rkiommu_ifc.h"

typedef struct _RKMPP_BUFFER {
    LIST_ENTRY      Link;
    PVOID           KernelVa;
    PMDL            Mdl;
    SIZE_T          Size;
    ULONG64         Iova;
    HANDLE          SectionHandle;       /* user-mode receives a duplicated handle */
    UINT32          Usage;
    UINT64          Cookie;              /* the BufferHandle returned to user */
} RKMPP_BUFFER;

NTSTATUS RkMppBufAlloc(_In_ WDFDEVICE Device, _In_ WDFFILEOBJECT File,
                       _In_ const RKMPP_ALLOC_BUFFER_IN *In,
                       _Out_ RKMPP_ALLOC_BUFFER_OUT *Out)
{
    /* 1. Round Size up to PAGE_SIZE.
     * 2. MmAllocateContiguousNodeMemory of that size, NOCACHE.
     * 3. Build an MDL with IoAllocateMdl + MmBuildMdlForNonPagedPool.
     * 4. Call ctx->Ifcs.Iommu.MapMdl(client_devobj, mdl, In->Usage, &iova).
     * 5. ZwCreateSection backed by a physical-memory section pointing at
     *    the MDL pages, ZwMapViewOfSection into the calling process.
     * 6. Allocate a unique Cookie (atomic counter), wrap into RKMPP_BUFFER,
     *    insert into the file-object's list under a spinlock.
     * 7. Fill *Out with handle, iova, section handle, rounded size.
     *
     * Real implementation ~120 lines.  Each numbered step needs explicit
     * NTSTATUS handling and unwind.
     */
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(File);
    UNREFERENCED_PARAMETER(In);
    UNREFERENCED_PARAMETER(Out);
    return STATUS_NOT_IMPLEMENTED;  /* TODO: implement per the comment */
}

NTSTATUS RkMppBufFree(_In_ WDFDEVICE Device, _In_ WDFFILEOBJECT File, _In_ UINT64 Cookie)
{
    /* Find by Cookie, remove from list, UnmapMdl, free MDL, free contig mem,
     * ZwClose the section, free RKMPP_BUFFER. */
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(File);
    UNREFERENCED_PARAMETER(Cookie);
    return STATUS_NOT_IMPLEMENTED;
}

VOID RkMppBufFreeAll(_In_ WDFFILEOBJECT File)
{
    /* Called from EvtFileCleanup. */
    UNREFERENCED_PARAMETER(File);
}
```

- [ ] **Step 2: Implement the bodies fully**

Replace the `STATUS_NOT_IMPLEMENTED` returns with real code, per the numbered comments. The unwind discipline: each successful step adds a label before it; each failure path goto's the label that frees prior state. No partial state allowed to escape on error.

- [ ] **Step 3: Wire `IOCTL_RKMPP_ALLOC_BUFFER` and `IOCTL_RKMPP_FREE_BUFFER` in `ioctl.c`**

Inside `RkMppEvtIoDeviceControl`'s switch on `IoControlCode`, add:

```c
case IOCTL_RKMPP_ALLOC_BUFFER: {
    if (InputBufferLength < sizeof(RKMPP_ALLOC_BUFFER_IN) ||
        OutputBufferLength < sizeof(RKMPP_ALLOC_BUFFER_OUT)) {
        status = STATUS_BUFFER_TOO_SMALL; break;
    }
    RKMPP_ALLOC_BUFFER_IN *in;  RKMPP_ALLOC_BUFFER_OUT *out;
    status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in),  (PVOID*)&in,  NULL);
    if (!NT_SUCCESS(status)) break;
    status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out), (PVOID*)&out, NULL);
    if (!NT_SUCCESS(status)) break;
    status = RkMppBufAlloc(WdfIoQueueGetDevice(Queue),
                           WdfRequestGetFileObject(Request), in, out);
    if (NT_SUCCESS(status)) info = sizeof(*out);
    break;
}
case IOCTL_RKMPP_FREE_BUFFER: {
    if (InputBufferLength < sizeof(RKMPP_FREE_BUFFER_IN)) {
        status = STATUS_BUFFER_TOO_SMALL; break;
    }
    RKMPP_FREE_BUFFER_IN *in;
    status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in), (PVOID*)&in, NULL);
    if (!NT_SUCCESS(status)) break;
    status = RkMppBufFree(WdfIoQueueGetDevice(Queue),
                          WdfRequestGetFileObject(Request),
                          in->BufferHandle);
    break;
}
```

Also register `EvtFileCleanup` on the WDFDEVICE (in `device.c`) to call `RkMppBufFreeAll` for the closing file object.

- [ ] **Step 4: Build with code analysis**

Run: `msbuild driver\rkmpp\rkmpp.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings.

- [ ] **Step 5: Commit**

```bash
git add driver/rkmpp/
git commit -m "phase2: rkmpp — buffer pool with ALLOC_BUFFER/FREE_BUFFER"
```

---

### Task 6: rkmpp.sys — SUBMIT_JOB / WAIT_JOB with no-op execution

**Files:**
- Create: `driver/rkmpp/job.c`
- Modify: `driver/rkmpp/device.c` — connect IRQ
- Modify: `driver/rkmpp/ioctl.c` — wire SUBMIT/WAIT
- Modify: `driver/rkmpp/profile.c` — add `ScratchRegOffset` to RVD0 profile

For Phase 2 we don't try to make rkvdec actually decode anything. Instead the "job" is a tiny register write list that targets a known scratch / config register, kicks the core (or, if that's nontrivial without a real workload, deliberately doesn't kick — see Step 1), and is completed by either an IRQ or by a timer-driven completion (whichever lands sooner on real hardware). The point of Phase 2 is to validate the **kernel/user data path**, not the codec itself.

- [ ] **Step 1: Decide the no-op kick mechanism**

Choose ONE of these and document the choice as a comment at the top of `job.c`:

- **(a) Real IRQ via a benign register sequence.** Find a single rkv-decoder-v2 register write that completes a "decode 0 bytes" job and raises the completion IRQ. The Linux driver's idle/quiesce sequence is the reference. Preferred — exercises the full ISR/DPC path.
- **(b) Software-only completion.** No MMIO writes; SUBMIT_JOB queues a job that is immediately marked complete from a DPC. Still exercises the file-object → queue → completion path. Use this only if (a) is blocked by missing TRM detail.

Pick (a) if you can confirm the sequence in <1 hour; otherwise pick (b) and add a TODO to revisit in Phase 3 when the register-list builder lands.

- [ ] **Step 2: Write `driver/rkmpp/job.c`**

```c
/* driver/rkmpp/job.c — register-list submission and completion.
 *
 * Phase 2: a "job" is a list of (offset, value) writes plus a list of
 * buffer-handle references (Phase 2 ignores the latter, since the no-op
 * register list doesn't dereference any iova).  Phase 3 extends this with
 * buffer-handle substitution.
 */
#include <ntddk.h>
#include <wdf.h>
#include "../../shared/rkmpp_ioctl.h"

typedef struct _RKMPP_JOB {
    LIST_ENTRY Link;
    UINT64     Id;
    KEVENT     Done;
    NTSTATUS   Result;
    UINT32     HardwareStatus;
    LARGE_INTEGER StartQpc, EndQpc;
} RKMPP_JOB;

/* Per-device queue, single in-flight, FIFO. */
typedef struct _RKMPP_JOB_QUEUE {
    KSPIN_LOCK Lock;
    LIST_ENTRY Pending;
    RKMPP_JOB *InFlight;
    UINT64     NextId;
} RKMPP_JOB_QUEUE;

NTSTATUS RkMppSubmitJob(_In_ WDFDEVICE Device,
                        _In_ const RKMPP_SUBMIT_JOB_IN *In,
                        _Out_ RKMPP_SUBMIT_JOB_OUT *Out)
{
    /* 1. Validate counts (RegWriteCount <= RKMPP_MAX_REG_WRITES, etc.).
     * 2. Allocate RKMPP_JOB, KeInitializeEvent.
     * 3. Acquire ctx->Ifcs.Ccu.RaiseCluster(devobj).  Refcounted.
     * 4. Under queue lock, push to Pending; if !InFlight, pop and start.
     * 5. To "start": apply each (offset, value) with WRITE_REGISTER_ULONG;
     *    if mode (a), the last write is the kick.  If mode (b), schedule
     *    a DPC immediately to mark Done.
     * 6. Return JobId in *Out.
     */
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(In);
    UNREFERENCED_PARAMETER(Out);
    return STATUS_NOT_IMPLEMENTED;  /* implement per comment */
}

NTSTATUS RkMppWaitJob(_In_ WDFDEVICE Device,
                     _In_ UINT64 JobId, _In_ ULONG TimeoutMs,
                     _Out_ RKMPP_WAIT_JOB_OUT *Out)
{
    /* 1. Find job by Id; if completed and freed, return STATUS_NOT_FOUND.
     * 2. KeWaitForSingleObject(&job->Done, ..., TimeoutMs).
     * 3. On signal: copy job->Result, HardwareStatus, ElapsedQpc to *Out;
     *    free job; ctx->Ifcs.Ccu.DropCluster(devobj).
     * 4. On timeout: mark job as failed, ask CCU to AssertCoreReset +
     *    DeassertCoreReset, set Out->Status = STATUS_DEVICE_HUNG.
     */
    UNREFERENCED_PARAMETER(Device);
    UNREFERENCED_PARAMETER(JobId);
    UNREFERENCED_PARAMETER(TimeoutMs);
    UNREFERENCED_PARAMETER(Out);
    return STATUS_NOT_IMPLEMENTED;  /* implement per comment */
}

BOOLEAN RkMppEvtIsr(_In_ WDFINTERRUPT Interrupt, _In_ ULONG MessageId)
{
    UNREFERENCED_PARAMETER(MessageId);
    /* Read+ack the core's IRQ status; if the bit corresponding to job-done
     * is set, queue DPC. */
    WdfInterruptQueueDpcForIsr(Interrupt);
    return TRUE;
}

VOID RkMppEvtDpc(_In_ WDFINTERRUPT Interrupt, _In_ WDFOBJECT AssociatedObject)
{
    UNREFERENCED_PARAMETER(Interrupt);
    UNREFERENCED_PARAMETER(AssociatedObject);
    /* Mark queue->InFlight done, signal its Done event, pop next pending. */
}
```

- [ ] **Step 3: Implement the bodies fully**

Replace each `STATUS_NOT_IMPLEMENTED`. Use exclusive WDFQUEUE-internal serialization on the in-flight pointer. For the IRQ connection, add a `WdfInterruptCreate` call in `device.c::RkMppEvtPrepareHardware` after MMIO mapping, with `EvtInterruptIsr = RkMppEvtIsr` and `EvtInterruptDpc = RkMppEvtDpc`.

- [ ] **Step 4: Wire `IOCTL_RKMPP_SUBMIT_JOB` / `IOCTL_RKMPP_WAIT_JOB` in `ioctl.c`**

Add cases analogous to ALLOC_BUFFER above, calling `RkMppSubmitJob` and `RkMppWaitJob`.

- [ ] **Step 5: Build**

Run: `msbuild driver\rkmpp\rkmpp.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings.

- [ ] **Step 6: Commit**

```bash
git add driver/rkmpp/
git commit -m "phase2: rkmpp — SUBMIT_JOB/WAIT_JOB with single-in-flight queue and no-op kick"
```

---

### Task 7: User-mode `rkmpp_smoke` — TDD'd round-trip test

**Files:**
- Create: `tests/harness/rkmpp_smoke/CMakeLists.txt`
- Create: `tests/harness/rkmpp_smoke/main.cpp`
- Create: `tests/harness/rkmpp_smoke/test_main.cpp`
- Modify: `tests/harness/CMakeLists.txt` — add subdir

The smoke test, in user mode, does:
1. Open `\\.\rkmpp` instance for RVD0.
2. `IOCTL_RKMPP_GET_CAPS` — sanity check.
3. `IOCTL_RKMPP_ALLOC_BUFFER` × 2 (one bitstream-input shape, one scratch).
4. `MapViewOfFile` on each section handle, write a magic pattern into scratch.
5. `IOCTL_RKMPP_SUBMIT_JOB` with a no-op register list and the two buffers as references.
6. `IOCTL_RKMPP_WAIT_JOB`.
7. Re-read the scratch buffer's mapped view, verify the pattern still matches (driver/IOMMU didn't corrupt it).
8. `IOCTL_RKMPP_FREE_BUFFER` × 2.

The TDD-able part is the **no-op register-list builder** — a pure function that turns a struct describing the test scenario into a `RKMPP_SUBMIT_JOB_IN`. We test that with golden values, and only the SetupAPI/IOCTL code is exercised on real hardware.

- [ ] **Step 1: `tests/harness/CMakeLists.txt`**

Append:

```cmake
add_subdirectory(rkmpp_smoke)
```

- [ ] **Step 2: `tests/harness/rkmpp_smoke/CMakeLists.txt`**

```cmake
add_executable(rkmpp_smoke main.cpp)
target_include_directories(rkmpp_smoke PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
target_link_libraries(rkmpp_smoke PRIVATE setupapi cfgmgr32)

add_executable(rkmpp_smoke_test test_main.cpp main.cpp)
target_compile_definitions(rkmpp_smoke_test PRIVATE RKMPP_SMOKE_TEST)
target_include_directories(rkmpp_smoke_test PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
add_test(NAME rkmpp_smoke_test COMMAND rkmpp_smoke_test)
```

- [ ] **Step 3: Write the failing test**

```cpp
/* tests/harness/rkmpp_smoke/test_main.cpp — TDD the no-op job builder. */
#include <cstdio>
#include "../../../shared/rkmpp_ioctl.h"

void BuildNoopJob(UINT64 scratchHandle, RKMPP_SUBMIT_JOB_IN *out);

static int Fail(const char *m) { std::fprintf(stderr, "FAIL: %s\n", m); return 1; }

int main()
{
    RKMPP_SUBMIT_JOB_IN job{};
    BuildNoopJob(0xCAFEBABEDEADBEEFull, &job);

    if (job.StructSize != sizeof(job)) return Fail("StructSize wrong");
    if (job.RegWriteCount == 0)        return Fail("expected >= 1 reg write");
    if (job.BufRefCount   != 1)        return Fail("expected 1 buf ref");
    if (job.BufRefs[0].BufferHandle != 0xCAFEBABEDEADBEEFull)
        return Fail("buf ref handle wrong");
    if (job.TimeoutMs == 0)            return Fail("timeout zero");
    return 0;
}
```

Run: build, expect link failure (`BuildNoopJob` undefined).

- [ ] **Step 4: Implement `main.cpp` with `BuildNoopJob` + the round-trip**

```cpp
/* tests/harness/rkmpp_smoke/main.cpp */
#define UMDF_USING_NTSTATUS
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../../../shared/rkmpp_ioctl.h"

void BuildNoopJob(UINT64 scratchHandle, RKMPP_SUBMIT_JOB_IN *out)
{
    std::memset(out, 0, sizeof(*out));
    out->StructSize    = sizeof(*out);
    out->TimeoutMs     = 1000;

    /* Phase 2 mode (b) "software completion": one harmless register read-back
     * by writing offset 0xfff8 with value 0 (a known-unused scratch).  Phase 3
     * will replace this with the real H.264 register list. */
    out->RegWriteCount = 1;
    out->Writes[0].Offset = 0xfff8;
    out->Writes[0].Value  = 0;

    out->BufRefCount = 1;
    out->BufRefs[0].BufferHandle = scratchHandle;
    out->BufRefs[0].Role         = RkMppBufferUsageScratch;
}

#ifndef RKMPP_SMOKE_TEST
/* Open RVD0, do the round-trip, return 0 on success. ~150 lines:
 *  - SetupDi enumerate GUID_DEVINTERFACE_RKMPP, pick the first whose
 *    GET_CAPS reports HID=0x3550 UID=0.
 *  - ALLOC scratch (4 KiB), map view, write 0xC0DECAFE pattern.
 *  - BuildNoopJob(scratchHandle, &job); SUBMIT_JOB; WAIT_JOB (timeout 1s).
 *  - Re-read first 8 bytes of scratch view, verify pattern intact.
 *  - FREE_BUFFER, return 0 / nonzero with a printable reason on failure.
 */
int wmain() {
    /* Real implementation here */
    std::fprintf(stderr, "rkmpp_smoke: not yet wired\n");
    return 0;  /* placeholder until hardware-backed run */
}
#endif
```

(The `wmain` body needs to be filled in; the comment lists every step. Treat the comment as a checklist when editing.)

- [ ] **Step 5: Run unit test, verify green**

```
cmake --build build/harness --config Debug
ctest --test-dir build/harness -C Debug -V
```

Expected: `rkmpp_smoke_test ............ Passed`. (`rkmpp_caps_dump_test` from Phase 1 still passes too.)

- [ ] **Step 6: Run the smoke test on the target**

```
Copy-Item build\harness\rkmpp_smoke\Debug\rkmpp_smoke.exe \\<board>\C`$\drvtest\
Invoke-Command -ComputerName <board> { C:\drvtest\rkmpp_smoke.exe; $LASTEXITCODE }
```

Expected: exit code 0, stdout includes lines like:

```
caps OK: RKCP3550 UID=0 rev=0x... codecs=H264
allocated scratch handle=0x... iova=0x...
job 1 completed in <N> us
scratch pattern preserved
```

- [ ] **Step 7: Commit**

```bash
git add tests/harness/rkmpp_smoke/ tests/harness/CMakeLists.txt
git commit -m "phase2: rkmpp_smoke tool — end-to-end alloc/submit/wait round-trip"
```

---

### Task 8: Phase 2 verification and tag

- [ ] **Step 1: Re-run all unit tests**

```
ctest --test-dir build/harness -C Debug -V
```

Expected: every test green.

- [ ] **Step 2: Re-run smoke on target**

Same as Task 7 Step 6. Expected: exit 0.

- [ ] **Step 3: Stress run (1000 iterations)**

```
Invoke-Command -ComputerName <board> {
    1..1000 | ForEach-Object { C:\drvtest\rkmpp_smoke.exe } | Group-Object | Format-Table
}
```

Expected: 1000 iterations, all exit 0, no kernel-log warnings between runs.

- [ ] **Step 4: Update README with Phase 2 baseline**

Append a `## Phase 2 verification` section showing one captured smoke-run output.

- [ ] **Step 5: Commit and tag**

```bash
git add README.md
git commit -m "phase2: end-to-end no-op job round-trip verified on RVD0"
git tag phase2-done
```

---

## Spec coverage check (self-review)

| Spec section | Phase 2 task |
|---|---|
| §3.1 — rkmpp.sys IOCTL surface (ALLOC/FREE/SUBMIT/WAIT) | Tasks 5, 6 |
| §3.1 — buffer model (hybrid, pooled out/ref) | Task 5 (pooled scratch path); user-pages bitstream input is deferred to Phase 3 |
| §3.1 — concurrency (FIFO, single in-flight) | Task 6 |
| §3.1 — failure modes (timeout, IOMMU fault) | Task 6 (timeout via WAIT), Task 3 (fault dispatch infrastructure) |
| §3.1 — process exit cleanup | Task 5 (EvtFileCleanup → BufFreeAll) |
| §3.2 — RaiseCluster/DropCluster, AssertCoreReset | Task 2 |
| §3.3 — MapMdl/UnmapMdl, RegisterFaultHandler | Task 3 |
| §6 — data flow (steps 4–5: SUBMIT → wait → complete) | Task 6 |

Out of scope for Phase 2 (deferred):

- H.264 register-list payload (Phase 4).
- FFmpeg parser vendoring (Phase 3).
- MFT shell (Phase 6).
- Real decode through the harness (Phase 5).
- User-pages-bitstream-input transient-IOMMU-mapping path (rolled into Phase 3 along with parser).
