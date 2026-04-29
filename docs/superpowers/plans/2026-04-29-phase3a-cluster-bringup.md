# Phase 3a — Real Cluster Bring-Up, Interrupt Wiring, IOMMU Enable

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Phase 2 software-only stance with verified hardware bring-up: full clock ungating, power-domain control, reset handling, working interrupts, real IOMMU paging, and confirmed REVISION read on RVD0/RVD1. Acceptance gate: `rkmpp_smoke` returns a non-zero `RevisionWord` for RKCP3550, the IOMMU's INT_STATUS shows zero faults across 100 ALLOC/FREE iterations, and the IOMMU fault handler fires correctly when a deliberately-bad mapping is issued.

**Architecture:** rkmpp_ccu gains a power-domain controller (PMU mapping at `0xFD8D0000`, similar to the system-CRU mapping it already does), expanded clock masks across CON(40)+CON(41), and reset polling. rkmpp's PrepareHardware reintroduces RaiseCluster + REVISION read, but only after each cluster-bring-up step is verified by readback. rkiommu's MapMdl reintroduces RkIommuEnable with the BSP-specified flags. WdfInterruptCreate is fixed by passing explicit raw/translated descriptors.

**Tech Stack:** Same as Phase 2. Adds: PMU MMIO mapping, polling helpers, IOMMU `RKVDEC_MMU_INT_STATUS` / `RKVDEC_MMU_INT_RAW_STAT` validation.

**Reference:**
- Spec: `docs/superpowers/specs/2026-04-28-rkvdec-mft-h264-design.md` §3
- Phase 2 plan: `docs/superpowers/plans/2026-04-28-phase2-bufferpool-noop-job.md`
- Phase 2 hardware-validated tag: `phase2-hw-validated`
- BSP register definitions: `rockchip-linux/kernel` branch `develop-5.10`
  - `drivers/clk/rockchip/clk-rk3588.c` — CRU layout
  - `drivers/pmdomain/rockchip/pm-domains.c` — PMU PD layout (verify against rk3588 entries)
  - `drivers/iommu/rockchip-iommu.c` — IOMMU register layout, command codes
  - `drivers/video/rockchip/mpp/mpp_rkvdec2.c` — rkv-decoder-v2 codec MMIO

---

## File structure additions

```
shared/
  rkmpp_ccu_ifc.h         # MODIFY — add VerifyClusterUp() entry
driver/rkmpp_ccu/
  ccu.c                   # MODIFY — expand mask, add CON(41), add readback
  pmu.c                   # NEW    — PMU power-domain control (rkvdec PD)
  pmu.h                   # NEW
  driver.c                # MODIFY — map PMU MMIO alongside CRU
driver/rkmpp/
  device.c                # MODIFY — restore RaiseCluster + REVISION read,
                          #          fix WdfInterruptCreate
  job.c                   # MODIFY — restore raise/drop pair around jobs
driver/rkiommu/
  ifc.c                   # MODIFY — restore RkIommuEnable + ZAP_CACHE
  device.c                # MODIFY — apply BSP flags (disable-mmu-reset,
                          #          enable-cmd-retry, shootdown-entire)
tests/harness/rkmpp_smoke/
  main.cpp                # MODIFY — assert rev != 0, run 100 iterations
tests/harness/rkmpp_iommu_fault/
  CMakeLists.txt          # NEW
  main.cpp                # NEW    — deliberately bad mapping, observe fault
```

---

### Task 1: rkmpp_ccu — expand clock-gate mask to all rkvdec clocks

**Files:**
- Modify: `driver/rkmpp_ccu/ccu.c`

The current mask is `0x00000007` (bits 0,1,2 of CON(40) — bus-roots + CCU bus). The codec **core** clocks live at bits 7,8,9 of CON(40) for RVD0 and bits 0,1,6,7,8 of CON(41) for RVD1. Without the core clocks ungated, MMIO reads to the codec's regs window SError.

- [ ] **Step 1: Expand RDCC_REGS for two CON registers**

In `driver/rkmpp_ccu/ccu.c`, replace the single-CON `g_rdcc` table with one that covers CON(40) + CON(41):

```c
typedef struct _RDCC_REGS {
    ULONG ClkGateCon40;     /* CRU offset for CLKGATE_CON(40) */
    ULONG ClkGateCon40Mask; /* bits we own in CON(40) */
    ULONG ClkGateCon41;     /* CRU offset for CLKGATE_CON(41) */
    ULONG ClkGateCon41Mask; /* bits we own in CON(41) */
    ULONG SoftRstCon40;
    ULONG SoftRstCon40Mask;
    ULONG SoftRstCon41;
    ULONG SoftRstCon41Mask;
} RDCC_REGS;

static const RDCC_REGS g_rdcc = {
    /* CLKGATE_CON(40) bits 0,1,2,7,8,9:
     *   0  hclk_rkvdec0_root        7  clk_rkvdec0_ca
     *   1  aclk_rkvdec0_root        8  clk_rkvdec0_hevc_ca
     *   2  aclk_rkvdec_ccu          9  clk_rkvdec0_core
     */
    .ClkGateCon40     = 0x8A0,
    .ClkGateCon40Mask = 0x00000387u,

    /* CLKGATE_CON(41) bits 0,1,6,7,8:
     *   0  hclk_rkvdec1_root        7  clk_rkvdec1_hevc_ca
     *   1  aclk_rkvdec1_root        8  clk_rkvdec1_core
     *   6  clk_rkvdec1_ca
     */
    .ClkGateCon41     = 0x8A4,
    .ClkGateCon41Mask = 0x000001C3u,

    /* SOFTRST_CON(40) bit 9 = SRST_RKVDEC0_CORE.
     *   bit 3  SRST_H_RKVDEC0  bit 4  SRST_A_RKVDEC0  bit 9 SRST_RKVDEC0_CORE
     */
    .SoftRstCon40     = 0xAA0,
    .SoftRstCon40Mask = 0x00000218u,  /* bits 3,4,9 */

    /* SOFTRST_CON(41) bit 8 = SRST_RKVDEC1_CORE.
     *   bit 2  SRST_H_RKVDEC1  bit 3  SRST_A_RKVDEC1  bit 8 SRST_RKVDEC1_CORE
     */
    .SoftRstCon41     = 0xAA4,
    .SoftRstCon41Mask = 0x0000010Cu,  /* bits 2,3,8 */
};
```

- [ ] **Step 2: Update RaiseCluster / DropCluster to write both CON registers**

```c
NTSTATUS RkMppCcuRaiseCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    if (InterlockedIncrement(&g_raise_refcount) == 1) {
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40, g_rdcc.ClkGateCon40Mask, 0);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41, g_rdcc.ClkGateCon41Mask, 0);
        /* Read-back verify within 100µs.  CRU writes are typically latched
         * in <1 cycle but we add a safety margin. */
        for (ULONG i = 0; i < 100; i++) {
            ULONG v40 = READ_REGISTER_ULONG(
                (volatile ULONG*)(g_cru_mmio + g_rdcc.ClkGateCon40));
            ULONG v41 = READ_REGISTER_ULONG(
                (volatile ULONG*)(g_cru_mmio + g_rdcc.ClkGateCon41));
            if (((v40 & g_rdcc.ClkGateCon40Mask) == 0) &&
                ((v41 & g_rdcc.ClkGateCon41Mask) == 0)) {
                return STATUS_SUCCESS;
            }
            KeStallExecutionProcessor(1);
        }
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: RaiseCluster readback failed\n");
        return STATUS_DEVICE_HARDWARE_ERROR;
    }
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDropCluster(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_SUCCESS;
    if (InterlockedDecrement(&g_raise_refcount) == 0) {
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40,
                         g_rdcc.ClkGateCon40Mask, g_rdcc.ClkGateCon40Mask);
        RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41,
                         g_rdcc.ClkGateCon41Mask, g_rdcc.ClkGateCon41Mask);
    }
    return STATUS_SUCCESS;
}
```

- [ ] **Step 3: Update reset functions to optionally select RVD0 vs RVD1**

The current reset functions don't know which core. Refactor to take a core index:

```c
/* The RKMPP_CCU_INTERFACE only exposes AssertCoreReset / DeassertCoreReset
 * without a core argument, because in v1 we only reset RVD0.  When Phase 3b
 * adds RVD1 we'll extend the ifc — for now keep the v1 signature and apply
 * to RVD0 only. */
NTSTATUS RkMppCcuAssertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    /* Assert SRST_RKVDEC0_CORE only.  Bus resets stay deasserted. */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40, 0x200u, 0x200u);
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}

NTSTATUS RkMppCcuDeassertCoreReset(_In_ PVOID Ctx)
{
    UNREFERENCED_PARAMETER(Ctx);
    if (!g_cru_mmio) return STATUS_DEVICE_NOT_READY;
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.SoftRstCon40, 0x200u, 0);
    /* Wait for status to clear.  RK3588 SOFTRST_CON registers are write-only
     * for the lower 16 bits; we infer success from the absence of an SError
     * on the next codec MMIO read. */
    KeStallExecutionProcessor(20);
    return STATUS_SUCCESS;
}
```

- [ ] **Step 4: Build**

Run:
```
cmd /c "`"D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat`" x64_arm64 >nul && msbuild driver\rkmpp_ccu\rkmpp_ccu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:WindowsTargetPlatformVersion=10.0.26100.0 /p:RunCodeAnalysis=true /t:Rebuild /v:m"
```
Expected: 0 errors, 0 warnings.

- [ ] **Step 5: Commit**

```bash
git add driver/rkmpp_ccu/ccu.c
git commit -m "phase3a: rkmpp_ccu — expand clock mask to all rkvdec0+rkvdec1+CCU clocks, add readback verify"
```

---

### Task 2: rkmpp_ccu — map the PMU and add power-domain control

**Files:**
- Create: `driver/rkmpp_ccu/pmu.h`
- Create: `driver/rkmpp_ccu/pmu.c`
- Modify: `driver/rkmpp_ccu/driver.c`
- Modify: `driver/rkmpp_ccu/rkmpp_ccu.vcxproj`

The RK3588 PMU at `0xFD8D0000` controls power-domain gating. The rkvdec power domains (`PD_RKVDEC0`, `PD_RKVDEC1`) need to be powered on before the codec cores can be accessed. UEFI may or may not leave them on; bring them up explicitly.

The PMU register layout is request-then-poll: write the requested state to `PMU_PWR_GATE_SFTCON*`, then poll `PMU_PWR_DWN_ST*` until the status bit matches.

- [ ] **Step 1: Find PMU register offsets in BSP**

Look up in rockchip-linux develop-5.10:
- `drivers/pmdomain/rockchip/pm-domains.c` — find the `rk3588_pm_domains` table for `PD_RKVDEC0` and `PD_RKVDEC1`. Capture the `pwr_offset`, `pwr_mask`, `req_offset`, `req_mask`, `idle_offset`, `idle_mask` for each.

Document the values in `pmu.h`:

```c
/* driver/rkmpp_ccu/pmu.h */
#pragma once

#include <ntddk.h>

/* PMU MMIO base on RK3588 — not exposed via ACPI; mapped directly. */
#define RKMPP_PMU_PHYS_BASE   0xFD8D0000ULL
#define RKMPP_PMU_MAP_LENGTH  0x1000u

/* Per-domain register set sourced from rockchip-linux develop-5.10
 * drivers/pmdomain/rockchip/pm-domains.c rk3588_pm_domains[]. */
typedef struct _RKMPP_PMU_DOMAIN {
    ULONG PwrOffset;     /* power on/off control register */
    ULONG PwrBit;        /* bit position in PwrOffset (1 = off, 0 = on) */
    ULONG StatusOffset;  /* power status register */
    ULONG StatusBit;     /* bit position in StatusOffset (1 = off, 0 = on) */
    ULONG IdleReqOffset; /* idle-request register (request bus quiesce) */
    ULONG IdleReqBit;
    ULONG IdleAckOffset; /* idle-acknowledge register */
    ULONG IdleAckBit;
} RKMPP_PMU_DOMAIN;

/* TODO(P3a-Task2): fill in from BSP source. PD_RKVDEC0 and PD_RKVDEC1 entries. */
extern const RKMPP_PMU_DOMAIN g_pdRkvdec0;
extern const RKMPP_PMU_DOMAIN g_pdRkvdec1;

NTSTATUS RkMppPmuPowerOn (_In_ const RKMPP_PMU_DOMAIN *D);
NTSTATUS RkMppPmuPowerOff(_In_ const RKMPP_PMU_DOMAIN *D);
```

- [ ] **Step 2: Implement pmu.c**

```c
/* driver/rkmpp_ccu/pmu.c */
#include "pmu.h"

extern volatile UCHAR *g_pmu_mmio;

/* Concrete values taken from BSP rk3588_pm_domains[] entries. */
const RKMPP_PMU_DOMAIN g_pdRkvdec0 = {
    /* TODO(P3a-Task2-Step1): replace with BSP-confirmed values. The fields
     * below are placeholders that compile but DO NOT reflect real hardware
     * layout — confirm before booting on hardware. */
    .PwrOffset     = 0x0150,  /* PMU_PWR_GATE_SFTCON1, bit for RKVDEC0 */
    .PwrBit        = 1u << 11,
    .StatusOffset  = 0x0180,  /* PMU_PWR_DWN_ST1 */
    .StatusBit     = 1u << 11,
    .IdleReqOffset = 0x01A0,  /* PMU_BUS_IDLE_REQ */
    .IdleReqBit    = 1u << 14,
    .IdleAckOffset = 0x01A4,  /* PMU_BUS_IDLE_ACK */
    .IdleAckBit    = 1u << 14,
};

const RKMPP_PMU_DOMAIN g_pdRkvdec1 = {
    .PwrOffset     = 0x0150,
    .PwrBit        = 1u << 12,
    .StatusOffset  = 0x0180,
    .StatusBit     = 1u << 12,
    .IdleReqOffset = 0x01A0,
    .IdleReqBit    = 1u << 15,
    .IdleAckOffset = 0x01A4,
    .IdleAckBit    = 1u << 15,
};

/* HIWORD-MASK helper, same convention as ccu.c. */
static FORCEINLINE void
PmuHiwordWrite(ULONG offset, ULONG mask, ULONG value)
{
    ULONG word = (mask << 16) | (value & mask);
    WRITE_REGISTER_ULONG((volatile ULONG*)(g_pmu_mmio + offset), word);
}

NTSTATUS RkMppPmuPowerOn(_In_ const RKMPP_PMU_DOMAIN *D)
{
    if (!g_pmu_mmio) return STATUS_DEVICE_NOT_READY;

    /* 1. Clear bus-idle request (allow bus traffic). */
    PmuHiwordWrite(D->IdleReqOffset, D->IdleReqBit, 0);

    /* 2. Wait for bus-idle ack to clear. */
    for (ULONG i = 0; i < 1000; i++) {
        ULONG ack = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_pmu_mmio + D->IdleAckOffset));
        if ((ack & D->IdleAckBit) == 0) goto bus_unidled;
        KeStallExecutionProcessor(1);
    }
    return STATUS_DEVICE_HARDWARE_ERROR;
bus_unidled:

    /* 3. Clear power-off bit (turn power on). */
    PmuHiwordWrite(D->PwrOffset, D->PwrBit, 0);

    /* 4. Wait for power-status to show "on" (status bit clears). */
    for (ULONG i = 0; i < 10000; i++) {
        ULONG st = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_pmu_mmio + D->StatusOffset));
        if ((st & D->StatusBit) == 0) return STATUS_SUCCESS;
        KeStallExecutionProcessor(1);
    }
    return STATUS_DEVICE_HARDWARE_ERROR;
}

NTSTATUS RkMppPmuPowerOff(_In_ const RKMPP_PMU_DOMAIN *D)
{
    if (!g_pmu_mmio) return STATUS_DEVICE_NOT_READY;

    /* Reverse: assert bus idle, then power off. */
    PmuHiwordWrite(D->IdleReqOffset, D->IdleReqBit, D->IdleReqBit);
    for (ULONG i = 0; i < 1000; i++) {
        ULONG ack = READ_REGISTER_ULONG(
            (volatile ULONG*)(g_pmu_mmio + D->IdleAckOffset));
        if ((ack & D->IdleAckBit) != 0) goto bus_idled;
        KeStallExecutionProcessor(1);
    }
    return STATUS_DEVICE_HARDWARE_ERROR;
bus_idled:
    PmuHiwordWrite(D->PwrOffset, D->PwrBit, D->PwrBit);
    return STATUS_SUCCESS;
}
```

> The placeholder offsets in `g_pdRkvdec0` and `g_pdRkvdec1` MUST be replaced with values cited from the BSP. Step 1 of this task is to do that lookup. Do not boot on hardware until the offsets are confirmed.

- [ ] **Step 3: Map PMU MMIO in driver.c**

Add a `g_pmu_mmio` global and map it alongside the system CRU when HID is RKCP3503:

```c
/* In driver/rkmpp_ccu/driver.c, additions to existing code: */
volatile UCHAR *g_pmu_mmio   = NULL;
static SIZE_T   g_pmu_mmio_len = 0;

#define RKMPP_PMU_PHYS_BASE   0xFD8D0000ULL
#define RKMPP_PMU_MAP_LENGTH  0x1000u

/* Inside the existing if (hid == 0x3503) block, after the CRU map: */
PHYSICAL_ADDRESS pmuPhys;
pmuPhys.QuadPart = RKMPP_PMU_PHYS_BASE;
PVOID pmuVa = MmMapIoSpaceEx(pmuPhys, RKMPP_PMU_MAP_LENGTH,
                             PAGE_READWRITE | PAGE_NOCACHE);
if (!pmuVa) {
    /* Unwind the CRU map we already took. */
    MmUnmapIoSpace((PVOID)g_cru_mmio, g_cru_mmio_len);
    g_cru_mmio = NULL; g_cru_mmio_len = 0;
    return STATUS_INSUFFICIENT_RESOURCES;
}
g_pmu_mmio     = (volatile UCHAR*)pmuVa;
g_pmu_mmio_len = RKMPP_PMU_MAP_LENGTH;
DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
           "rkmpp_ccu: PMU mapped @ %p (phys 0x%llx)\n",
           pmuVa, (unsigned long long)pmuPhys.QuadPart);
```

In `RkMppCcuEvtReleaseHardware`, unmap before unmapping the CRU:

```c
if (g_pmu_mmio) {
    MmUnmapIoSpace((PVOID)g_pmu_mmio, g_pmu_mmio_len);
    g_pmu_mmio = NULL; g_pmu_mmio_len = 0;
}
```

- [ ] **Step 4: Wire PowerOn into RaiseCluster, PowerOff into DropCluster**

In `ccu.c::RkMppCcuRaiseCluster`, **before** ungating clocks:

```c
if (InterlockedIncrement(&g_raise_refcount) == 1) {
    /* 1. Power up the rkvdec0 + rkvdec1 power domains. */
    NTSTATUS s = RkMppPmuPowerOn(&g_pdRkvdec0);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_RKVDEC0 power-on failed 0x%08x\n", s);
        InterlockedDecrement(&g_raise_refcount);
        return s;
    }
    s = RkMppPmuPowerOn(&g_pdRkvdec1);
    if (!NT_SUCCESS(s)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp_ccu: PD_RKVDEC1 power-on failed 0x%08x\n", s);
        RkMppPmuPowerOff(&g_pdRkvdec0);
        InterlockedDecrement(&g_raise_refcount);
        return s;
    }

    /* 2. Now ungate clocks (existing CRU writes). */
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon40, g_rdcc.ClkGateCon40Mask, 0);
    RkCcuHiwordWrite(g_cru_mmio, g_rdcc.ClkGateCon41, g_rdcc.ClkGateCon41Mask, 0);

    /* 3. Readback verify (existing loop). */
    /* ...as before... */
}
```

In `RkMppCcuDropCluster`, mirror in reverse: gate clocks first, then power off both domains.

- [ ] **Step 5: Add ccu.c includes and pmu.c to vcxproj**

In `ccu.c`, add `#include "pmu.h"`. In `rkmpp_ccu.vcxproj`, add `<ClCompile Include="pmu.c" />`.

- [ ] **Step 6: Build and commit**

```
cmd /c "...vcvarsall x64_arm64..." && msbuild driver\rkmpp_ccu\rkmpp_ccu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:WindowsTargetPlatformVersion=10.0.26100.0 /p:RunCodeAnalysis=true /t:Rebuild /v:m
```
Expect 0 errors, 0 warnings.

```bash
git add driver/rkmpp_ccu/
git commit -m "phase3a: rkmpp_ccu — PMU power-domain control for PD_RKVDEC0/1, paired with cluster raise/drop"
```

---

### Task 3: rkmpp — restore RaiseCluster + REVISION read in PrepareHardware

**Files:**
- Modify: `driver/rkmpp/device.c`

- [ ] **Step 1: Reinstate the cluster raise after ifc open**

Replace the Phase-2-software-only block in `RkMppEvtPrepareHardware` with:

```c
/* Step 3: ungate the rkv-decoder-v2 cluster.  PMU + CRU + reset deassert
 * are all handled inside RaiseCluster.  Refcounted; matching DropCluster
 * lives in ReleaseHardware. */
PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
status = ctx->Ifcs.Ccu.RaiseCluster(cookie);
if (!NT_SUCCESS(status)) {
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp: RaiseCluster failed 0x%08x\n", status);
    RkMppCloseIfcs(&ctx->Ifcs);
    return status;
}

/* Step 4: deassert this core's reset (RVD0 only in v1; RVD1 stays in
 * reset because RaiseCluster doesn't deassert it).  TODO(Phase 3b): pick
 * RVD0 vs RVD1 based on Hid+Uid via an extended ifc. */
if (ctx->Hid == 0x3550 && ctx->Uid == 0) {
    ctx->Ifcs.Ccu.DeassertCoreReset(cookie);
}

/* Step 5: now safe to read codec MMIO. */
const RKMPP_PROFILE *p = RkMppFindProfile(ctx->Hid, ctx->Uid);
if (p) {
    ctx->RevisionWord = READ_REGISTER_ULONG(
        (volatile ULONG*)((PUCHAR)ctx->MmioBase + p->RevisionRegOffset));
    ctx->SupportedCodecs = p->SupportedCodecs;
}

DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
           "rkmpp: HID=RKCP%04x UID=%u rev=0x%08x codecs=0x%08x\n",
           ctx->Hid, ctx->Uid, ctx->RevisionWord, ctx->SupportedCodecs);
return STATUS_SUCCESS;
```

- [ ] **Step 2: Reinstate the matching DropCluster in ReleaseHardware**

```c
NTSTATUS
RkMppEvtReleaseHardware(_In_ WDFDEVICE Device, _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);

    if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.DropCluster) {
        ctx->Ifcs.Ccu.DropCluster(WdfDeviceWdmGetDeviceObject(Device));
    }
    RkMppCloseIfcs(&ctx->Ifcs);

    if (ctx->MmioBase) {
        MmUnmapIoSpace(ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase = NULL;
    }
    return STATUS_SUCCESS;
}
```

- [ ] **Step 3: Build, commit**

```bash
git add driver/rkmpp/device.c
git commit -m "phase3a: rkmpp — restore RaiseCluster + REVISION read with reset deassert"
```

---

### Task 4: rkmpp — restore per-job RaiseCluster/DropCluster in job.c

**Files:**
- Modify: `driver/rkmpp/job.c`

- [ ] **Step 1: Reinsert RaiseCluster in RkMppJobSubmit (before queueing)**

Right after `job->Id = ...`:

```c
PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
if (ccu && ccu->RaiseCluster) {
    NTSTATUS s = ccu->RaiseCluster(ccu->Header.Context);
    if (!NT_SUCCESS(s)) {
        ExFreePoolWithTag(job, 'JppM');
        return s;
    }
}
```

- [ ] **Step 2: Reinsert DropCluster in RkMppJobComplete**

After `KeReleaseSpinLock(&q->Lock, oldIrql);` and before "Kick the next job":

```c
PRKMPP_CCU_INTERFACE ccu = RkMppGetCcuIfc(Device);
if (ccu && ccu->DropCluster) {
    ccu->DropCluster(ccu->Header.Context);
}
```

- [ ] **Step 3: Build, commit**

```bash
git add driver/rkmpp/job.c
git commit -m "phase3a: rkmpp — restore raise/drop cluster around each job"
```

---

### Task 5: rkmpp — fix WdfInterruptCreate by passing explicit raw/translated descriptors

**Files:**
- Modify: `driver/rkmpp/device.c`

`STATUS_WDF_INVALID_INTERRUPT_CONFIG` (0xC020020F) at `WdfInterruptCreate` time on ARM64 GIC interrupts is the framework rejecting the auto-resource-binding path. Passing the descriptor pair explicitly works around it.

- [ ] **Step 1: Save raw + translated descriptors during the resource iteration**

In `RkMppEvtPrepareHardware`, drop `UNREFERENCED_PARAMETER(ResourcesRaw);` and capture both descriptors when an interrupt resource is found:

```c
PCM_PARTIAL_RESOURCE_DESCRIPTOR irqRaw = NULL;
PCM_PARTIAL_RESOURCE_DESCRIPTOR irqTrans = NULL;

ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
for (ULONG i = 0; i < count; i++) {
    PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
        WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
    if (d->Type == CmResourceTypeMemory && !ctx->MmioBase) {
        /* unchanged MMIO map */
    } else if (d->Type == CmResourceTypeInterrupt && !irqTrans) {
        irqTrans = d;
        irqRaw   = WdfCmResourceListGetDescriptor(ResourcesRaw, i);
    }
}
```

- [ ] **Step 2: Set InterruptRaw / InterruptTranslated explicitly in the config**

After clusters are raised and before the success return:

```c
if (irqRaw && irqTrans) {
    WDF_INTERRUPT_CONFIG intCfg;
    WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkMppEvtIsr, RkMppEvtDpc);
    intCfg.InterruptRaw        = irqRaw;
    intCfg.InterruptTranslated = irqTrans;

    WDF_OBJECT_ATTRIBUTES intAttr;
    WDF_OBJECT_ATTRIBUTES_INIT(&intAttr);
    intAttr.ParentObject = Device;

    NTSTATUS intStatus = WdfInterruptCreate(
        Device, &intCfg, &intAttr, &ctx->JobQueue.Interrupt);
    if (!NT_SUCCESS(intStatus)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkmpp: WdfInterruptCreate failed 0x%08x\n", intStatus);
        /* Still non-fatal — caps + alloc + soft-completion path works
         * without an ISR.  Phase 3b's hardware kick path makes it fatal. */
    }
}
```

- [ ] **Step 3: Verify on hardware**

After install, DebugView should NOT show `WdfInterruptCreate failed` anymore for any rkmpp instance.

- [ ] **Step 4: Build, commit**

```bash
git add driver/rkmpp/device.c
git commit -m "phase3a: rkmpp — fix WdfInterruptCreate by passing explicit raw/translated IRQ descriptors"
```

---

### Task 6: rkiommu — restore RkIommuEnable with BSP flags

**Files:**
- Modify: `driver/rkiommu/device.c`
- Modify: `driver/rkiommu/ifc.c`

The Phase 2 stance disabled IOMMU programming entirely. Phase 3a restores it, with the three BSP-mandated flags (`disable-mmu-reset`, `enable-cmd-retry`, `shootdown-entire`) applied.

- [ ] **Step 1: Read BSP flags from `_DSD` at PrepareHardware**

In `driver/rkiommu/device.c`, in PrepareHardware, after the HID/UID parse, call `IoGetDevicePropertyData` for the relevant `DEVPKEY_Device_*` values. Easier: hardcode the flags for known HIDs that match the DSD values (RKCP3570 UIDs 7,8,9,10 set all three flags per the firmware):

```c
ctx->FlagDisableMmuReset = (ctx->Hid == 0x3570 && ctx->Uid >= 7 && ctx->Uid <= 10) ||
                           (ctx->Hid == 0x3571);
ctx->FlagEnableCmdRetry  = ctx->FlagDisableMmuReset;
ctx->FlagShootdownEntire = ctx->FlagDisableMmuReset;
```

Add three `BOOLEAN` fields to `RKIOMMU_DEVICE` for these flags.

- [ ] **Step 2: Restore RkIommuEnable in MapMdl**

In `driver/rkiommu/ifc.c::RkIommuMapMdl`, replace the Phase-2 deferred block with:

```c
KeReleaseSpinLock(&dev->Domain->Lock, irql);

if (!dev->PagingEnabled) {
    NTSTATUS s = RkIommuEnable(dev);
    if (!NT_SUCCESS(s)) {
        /* Roll back the just-allocated mapping */
        KeAcquireSpinLock(&dev->Domain->Lock, &irql);
        RkIommuUnmapAt(dev->Domain, baseIova, pageCount);
        RkIommuFreeIova(dev->Domain, baseIova, pageCount);
        KeReleaseSpinLock(&dev->Domain->Lock, irql);
        return s;
    }
}

if (dev->MmioBase) {
    /* ZAP_CACHE: shoot down the entire TLB if the BSP flag says so. */
    ULONG cmd = dev->FlagShootdownEntire ? RK_MMU_CMD_ZAP_CACHE
                                         : RK_MMU_CMD_ZAP_CACHE;
    /* (single command opcode; the flag affects the BSP's strategy of which
     *  iova range to invalidate — for v1 we always shoot the entire TLB) */
    if (dev->FlagEnableCmdRetry) {
        for (ULONG i = 0; i < 3; i++) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG*)(dev->MmioBase + RK_MMU_COMMAND), cmd);
            ULONG st = READ_REGISTER_ULONG(
                (volatile ULONG*)(dev->MmioBase + RK_MMU_STATUS));
            if ((st & RK_MMU_STATUS_PAGING_ENABLED) != 0) break;
        }
    } else {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(dev->MmioBase + RK_MMU_COMMAND), cmd);
    }
}

*Iova = baseIova;
return STATUS_SUCCESS;
```

- [ ] **Step 3: Update RkIommuEnable to skip MMU reset if flag set**

In `driver/rkiommu/device.c::RkIommuEnable`:

```c
NTSTATUS RkIommuEnable(_Inout_ PRKIOMMU_DEVICE Dev)
{
    if (!Dev->MmioBase || !Dev->Domain) return STATUS_DEVICE_NOT_READY;

    /* 1. Optionally reset the MMU. BSP flag says skip on RK3588. */
    if (!Dev->FlagDisableMmuReset) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_RESET);
        KeStallExecutionProcessor(50);
    }

    /* 2. Program the page-directory base. */
    WRITE_REGISTER_ULONG(
        (volatile ULONG*)(Dev->MmioBase + RK_MMU_DTE_ADDR),
        (ULONG)Dev->Domain->PdPa);

    /* 3. Enable paging.  Cmd-retry: write up to 3 times. */
    ULONG attempts = Dev->FlagEnableCmdRetry ? 3 : 1;
    for (ULONG i = 0; i < attempts; i++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_COMMAND),
            RK_MMU_CMD_ENABLE_PAGING);
        ULONG st = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_STATUS));
        if ((st & RK_MMU_STATUS_PAGING_ENABLED) != 0) {
            Dev->PagingEnabled = TRUE;
            return STATUS_SUCCESS;
        }
        KeStallExecutionProcessor(20);
    }
    return STATUS_DEVICE_HARDWARE_ERROR;
}
```

- [ ] **Step 4: Restore the UnmapMdl ZAP_CACHE call**

In `RkIommuUnmapMdl`, after the unmap-and-free, write `RK_MMU_CMD_ZAP_CACHE` to the COMMAND register. Same retry pattern as MapMdl.

- [ ] **Step 5: Build, commit**

```bash
git add driver/rkiommu/
git commit -m "phase3a: rkiommu — restore RkIommuEnable + ZAP_CACHE with BSP flags"
```

---

### Task 7: Smoke test — assert REVISION is non-zero, run 100 iterations

**Files:**
- Modify: `tests/harness/rkmpp_smoke/main.cpp`

- [ ] **Step 1: Strengthen the caps check**

In `wmain` (the non-test path), after the `OpenRvd0` call:

```c
/* Phase 3a: RevisionWord must now be non-zero (real hardware bring-up). */
if (caps.RevisionWord == 0) {
    std::fprintf(stderr, "rkmpp_smoke: rev=0 — cluster bring-up did not "
                         "complete; REVISION read returned 0\n");
    CloseHandle(h);
    return 7;
}
```

- [ ] **Step 2: Wrap the alloc/submit/wait/free in a 100-iteration loop**

```c
constexpr int kIterations = 100;
for (int iter = 0; iter < kIterations; iter++) {
    /* existing alloc → write pattern → submit → wait → reread → free */
}
std::printf("rkmpp_smoke: %d iterations OK\n", kIterations);
```

- [ ] **Step 3: Build (Release ARM64)**

```
cmake --build build/harness-arm64 --config Release
```
Expected: success.

- [ ] **Step 4: Commit**

```bash
git add tests/harness/rkmpp_smoke/main.cpp
git commit -m "phase3a: rkmpp_smoke — assert rev != 0 + 100-iteration alloc/submit/free loop"
```

---

### Task 8: New tool — rkmpp_iommu_fault — verify the IOMMU fault handler fires

**Files:**
- Create: `tests/harness/rkmpp_iommu_fault/CMakeLists.txt`
- Create: `tests/harness/rkmpp_iommu_fault/main.cpp`
- Modify: `tests/harness/CMakeLists.txt` — `add_subdirectory(rkmpp_iommu_fault)`
- Modify: `shared/rkmpp_ioctl.h` — add `IOCTL_RKMPP_INJECT_IOMMU_FAULT`
- Modify: `driver/rkmpp/ioctl.c` — handle the new IOCTL
- Modify: `driver/rkiommu/ifc.c` — expose a debug-only `InjectFault` helper

This tool deliberately programs an iova that doesn't correspond to a valid mapping, then issues a small DMA from the codec to it. The IOMMU faults; the registered fault handler fires; the IOCTL returns the captured fault iova.

- [ ] **Step 1: Define the IOCTL**

In `shared/rkmpp_ioctl.h`:

```c
#define IOCTL_RKMPP_INJECT_IOMMU_FAULT \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x8FE, METHOD_BUFFERED, FILE_WRITE_ACCESS)

typedef struct _RKMPP_FAULT_RESULT {
    ULONG  Triggered;     /* 0/1 */
    ULONG64 FaultIova;
    ULONG  StatusReg;
} RKMPP_FAULT_RESULT;
```

- [ ] **Step 2: Stub IOCTL handler in driver/rkmpp/ioctl.c**

```c
case IOCTL_RKMPP_INJECT_IOMMU_FAULT: {
    if (OutputBufferLength < sizeof(RKMPP_FAULT_RESULT)) {
        status = STATUS_BUFFER_TOO_SMALL; break;
    }
    RKMPP_FAULT_RESULT *out;
    status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                            (PVOID*)&out, NULL);
    if (!NT_SUCCESS(status)) break;
    /* Issue a single 4-byte read from the codec to iova 0xDEADB000.
     * The IOMMU has no mapping at that iova → fault → registered callback
     * captures iova into a per-device field.
     *
     * Phase 3a implementation: write the codec's RKVDEC_DMA_SRC register
     * with iova=0xDEADB000 and kick a tiny dummy job.  The captured iova
     * comes back via a device-context field set by the fault handler.
     */
    /* TODO(P3a-Task8-Step5): implement after RkMppOnIommuFault is wired */
    out->Triggered = 0;
    out->FaultIova = 0;
    out->StatusReg = 0;
    info = sizeof(*out);
    status = STATUS_SUCCESS;
    break;
}
```

- [ ] **Step 3: Register the rkmpp fault handler in PrepareHardware**

In `driver/rkmpp/device.c`, after `RaiseCluster`:

```c
if (ctx->Ifcs.Iommu.RegisterFaultHandler) {
    ctx->Ifcs.Iommu.RegisterFaultHandler(
        WdfDeviceWdmGetDeviceObject(Device),
        RkMppOnIommuFault);
}
```

And implement `RkMppOnIommuFault`:

```c
VOID RkMppOnIommuFault(_In_ PVOID ClientCookie,
                       _In_ ULONG64 FaultIova,
                       _In_ ULONG StatusReg)
{
    UNREFERENCED_PARAMETER(ClientCookie);
    PRKMPP_DEVICE ctx = /* recover via ClientCookie */;
    InterlockedExchange64((LONG64*)&ctx->LastFaultIova, (LONG64)FaultIova);
    InterlockedExchange((LONG*)&ctx->LastFaultStatus, (LONG)StatusReg);
    InterlockedExchange((LONG*)&ctx->FaultTriggered, 1);
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkmpp: IOMMU fault iova=0x%llx status=0x%x\n",
               (unsigned long long)FaultIova, StatusReg);
}
```

- [ ] **Step 4: Implement the fault-injection IOCTL**

Wire the IOCTL handler to:
1. Reset `ctx->FaultTriggered = 0`.
2. Submit a tiny job whose register list writes a known iova-of-no-mapping into `RKVDEC_DMA_SRC` and asserts the kick bit.
3. Wait up to 100ms for the fault.
4. Copy `LastFaultIova`, `LastFaultStatus`, `FaultTriggered` into the output struct.

This task depends on Phase 3b's real hardware kick path being available. **Mark this task as deferred to early Phase 3b** with a clear comment in the IOCTL stub. Phase 3a's deliverable is the wiring (callback registration, IOCTL plumbing) but not the actual injection.

- [ ] **Step 5: Build the user-mode tool**

`tests/harness/rkmpp_iommu_fault/CMakeLists.txt`:

```cmake
add_executable(rkmpp_iommu_fault main.cpp)
target_include_directories(rkmpp_iommu_fault PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
target_link_libraries(rkmpp_iommu_fault PRIVATE setupapi cfgmgr32)
```

`main.cpp`: open `\\.\rkmpp` for RVD0, call `IOCTL_RKMPP_INJECT_IOMMU_FAULT`, print the result. Exit non-zero if `Triggered = 0` after 1s.

- [ ] **Step 6: Commit**

```bash
git add shared/rkmpp_ioctl.h driver/rkmpp/ driver/rkiommu/ tests/harness/
git commit -m "phase3a: scaffold rkmpp_iommu_fault tool + fault-handler wiring; injection deferred to 3b"
```

---

### Task 9: Hardware verification on RK3588

This is the acceptance gate for Phase 3a. Not a TDD step — a manual verification.

- [ ] **Step 1: Copy the latest binaries to the board and reinstall**

Same delete-driver / add-driver sequence as Phase 2.

- [ ] **Step 2: Confirm DebugView shows non-zero REVISION for RVD0 and RVD1**

Expected new lines:
```
rkmpp: HID=RKCP3550 UID=0 rev=0x<nonzero> codecs=0x00000001
rkmpp: HID=RKCP3550 UID=1 rev=0x<nonzero> codecs=0x00000001
```

The `<nonzero>` value is the rkv-decoder-v2 hardware identification word. Capture it; **append the observed value to a new `docs/hw-observations.md`** with one line per observation.

- [ ] **Step 3: Confirm no `WdfInterruptCreate failed` lines anymore**

If they're still there, debug `intCfg.InterruptRaw`/`InterruptTranslated` placement and consult `WDF_INTERRUPT_CONFIG_INIT` documentation for ARM64 specifics.

- [ ] **Step 4: Run rkmpp_smoke 100x — should all pass**

```cmd
C:\drvtest\rkmpp_smoke.exe
echo exit=%ERRORLEVEL%
```
Expected: `100 iterations OK` and `exit=0`.

- [ ] **Step 5: Document hardware observations and tag**

```bash
git tag phase3a-hw-validated
```

---

## Spec coverage check

| Spec section | Phase 3a task |
|---|---|
| §3.1 — REVISION read at PrepareHardware | Task 3 |
| §3.1 — IRQ wiring + ISR/DPC | Task 5 |
| §3.1 — RaiseCluster/DropCluster around jobs | Task 4 |
| §3.2 — full clock + reset control for cluster | Task 1 |
| §3.2 — power-domain control | Task 2 |
| §3.3 — IOMMU paging + ZAP_CACHE + BSP flags | Task 6 |
| §7 — IOMMU fault path → kernel log + recovery | Task 8 (wiring), Phase 3b (injection) |

## What's deferred to Phase 3b

- Real hardware kick (writing register list to MMIO + setting kick bit).
- Buffer-handle → iova substitution in SUBMIT_JOB.
- Fault-injection actually faulting (needs the kick path).
- FFmpeg parser + register-list builder + first decode.
