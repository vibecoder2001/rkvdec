# Phase 1 — Driver Skeleton & Caps Plumbing

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Three KMDF drivers (`rkmpp.sys`, `rkmpp_ccu.sys`, `rkiommu.sys`) that build, install, and PnP-attach to their declared RKCP35xx HIDs on RK3588, plus a user-mode test tool (`rkmpp_caps_dump`) that opens `\\.\rkmpp` device instances and prints HID, _UID, and hardware revision via `IOCTL_RKMPP_GET_CAPS`. No decode, no buffers, no in-kernel cross-driver calls yet — those are Phase 2.

**Architecture:** Per spec §2/§3, the three drivers are split along ACPI device classes. In Phase 1 each driver is a thin KMDF skeleton: rkiommu and rkmpp_ccu register placeholder in-kernel device interfaces but answer no requests yet; rkmpp.sys is the only driver that touches MMIO (just to read the revision word) and the only one with an IOCTL surface in this phase.

**Tech Stack:** WDK 10 / KMDF 1.33 / ARM64, MSVC C/C++17, CMake (for the user-mode test harness only — drivers use the WDK's MSBuild templates), PowerShell for deploy scripts.

**Reference docs:**
- Spec: `docs/superpowers/specs/2026-04-28-rkvdec-mft-h264-design.md`
- HID map: `~/.claude/projects/.../memory/rkcp35xx_hid_map.md`
- WDK KMDF Hello World template: <wdk install>\Samples\KmdfHelloWorld
- Reference for ACPI-attached KMDF on the same SoC: `C:\Users\vibecoder\Rockchip-Windows-Drivers\drivers\` (gpio, i2c, dma all attach by ACPI HID)

---

## File structure (locked in for the phase)

```
rkvdec/
├─ .gitignore
├─ README.md
├─ scripts/
│  └─ deploy.ps1                # build → copy → pnputil on a target RK3588
├─ shared/
│  ├─ rkmpp_ioctl.h             # IOCTL codes + RKMPP_CAPS struct
│  ├─ rkmpp_ccu_ifc.h           # GUID + version-query for the CCU ifc
│  └─ rkiommu_ifc.h             # GUID + version-query for the IOMMU ifc
├─ driver/
│  ├─ rkiommu/
│  │  ├─ rkiommu.vcxproj
│  │  ├─ rkiommu.inx            # matches RKCP3570 + RKCP3571
│  │  ├─ driver.c               # DriverEntry, EvtDeviceAdd
│  │  └─ ifc.c                  # registers GUID_DEVINTERFACE_RKIOMMU
│  ├─ rkmpp_ccu/
│  │  ├─ rkmpp_ccu.vcxproj
│  │  ├─ rkmpp_ccu.inx          # matches RKCP3501..RKCP3503
│  │  ├─ driver.c
│  │  └─ ifc.c
│  └─ rkmpp/
│     ├─ rkmpp.vcxproj
│     ├─ rkmpp.inx              # matches RKCP3510..RKCP3560
│     ├─ driver.c               # DriverEntry, EvtDeviceAdd
│     ├─ device.c               # MMIO map, read revision, register dev iface
│     ├─ ioctl.c                # IOCTL_RKMPP_GET_CAPS handler
│     └─ profile.c              # HID+_UID → profile (rev mask, codec bitmap)
└─ tests/
   └─ harness/
      ├─ CMakeLists.txt
      └─ rkmpp_caps_dump/
         ├─ CMakeLists.txt
         └─ main.cpp
```

---

### Task 1: Repo skeleton and .gitignore

**Files:**
- Create: `.gitignore`
- Create: `README.md`

- [ ] **Step 1: Write `.gitignore`**

```gitignore
# Build outputs
build/
out/
**/Debug/
**/Release/
**/ARM64/
**/*.user
**/*.suo
*.aps
*.tlog
*.obj
*.pdb
*.ilk
*.exp
*.lib
*.exe
*.dll
*.sys
*.cat
# CMake
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
Makefile
# IDE
.vs/
.vscode/
*.code-workspace
```

- [ ] **Step 2: Write `README.md`**

```markdown
# rkvdec — RK3588 hardware video decode for Windows

Hardware-accelerated H.264 decode on RK3588 / Windows ARM64, exposed as a
Media Foundation Transform (MFT). Phase 1 ships only the kernel driver
skeleton (rkmpp.sys, rkmpp_ccu.sys, rkiommu.sys) and a `rkmpp_caps_dump`
tool. See `docs/superpowers/specs/` for the design and
`docs/superpowers/plans/` for phase plans.

## Build (drivers)

Open `driver/<name>/<name>.vcxproj` in Visual Studio with the WDK
extension installed; target Configuration=Debug, Platform=ARM64.

## Build (test harness)

```
cmake -S tests/harness -B build/harness -A ARM64
cmake --build build/harness --config Debug
```

## Deploy to a target RK3588

```
scripts/deploy.ps1 -Target <ip-or-hostname>
```
```

- [ ] **Step 3: Verify and commit**

Run: `git status`
Expected: shows `.gitignore` and `README.md` as new files.

```bash
git add .gitignore README.md
git commit -m "phase1: repo skeleton, README, gitignore"
```

---

### Task 2: Shared headers — IOCTL codes and ifc GUIDs

**Files:**
- Create: `shared/rkmpp_ioctl.h`
- Create: `shared/rkmpp_ccu_ifc.h`
- Create: `shared/rkiommu_ifc.h`

- [ ] **Step 1: Write `shared/rkmpp_ioctl.h`**

```c
/* shared/rkmpp_ioctl.h — IOCTL surface for rkmpp.sys, included by both
 * the driver and user-mode clients. Phase 1: GET_CAPS only.
 */
#pragma once

#include <devioctl.h>

#define FILE_DEVICE_RKMPP 0x8086u  /* arbitrary, vendor-defined range */

#define IOCTL_RKMPP_GET_CAPS \
    CTL_CODE(FILE_DEVICE_RKMPP, 0x800, METHOD_BUFFERED, FILE_READ_ACCESS)

/* Output of IOCTL_RKMPP_GET_CAPS. */
typedef struct _RKMPP_CAPS {
    UINT32 StructSize;          /* sizeof(RKMPP_CAPS) — versioning */
    UINT32 Hid;                 /* e.g. 0x3550 for RKCP3550 */
    UINT32 Uid;                 /* ACPI _UID, 0 for RVD0 */
    UINT32 RevisionWord;        /* raw value of the core's REVISION reg */
    UINT32 SupportedCodecs;     /* bitmap, see RKMPP_CODEC_* below */
    UINT32 Reserved[8];
} RKMPP_CAPS, *PRKMPP_CAPS;

/* SupportedCodecs bits */
#define RKMPP_CODEC_H264   (1u << 0)
#define RKMPP_CODEC_HEVC   (1u << 1)
#define RKMPP_CODEC_VP9    (1u << 2)
#define RKMPP_CODEC_AV1    (1u << 3)
#define RKMPP_CODEC_JPEG_D (1u << 4)
#define RKMPP_CODEC_AVS    (1u << 5)

/* Per-instance device-interface GUID. User mode enumerates instances of
 * this and opens each. Phase 1 v1 should see one entry: RVD0.
 */
DEFINE_GUID(GUID_DEVINTERFACE_RKMPP,
    0x9a1f4d11, 0x7c5e, 0x4ad7, 0xa4, 0x10, 0x1d, 0x21, 0x9e, 0x07, 0x6b, 0x10);
```

- [ ] **Step 2: Write `shared/rkmpp_ccu_ifc.h`**

```c
/* shared/rkmpp_ccu_ifc.h — In-kernel device interface exported by
 * rkmpp_ccu.sys for codec cores in its cluster to consume.
 *
 * Phase 1: only the version probe exists. Cluster raise/drop lands in Phase 2.
 */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKMPP_CCU,
    0x3b2a8e02, 0x6d31, 0x4a08, 0x9a, 0x77, 0x3c, 0x6b, 0xb8, 0x10, 0xe2, 0x55);

#define RKMPP_CCU_IFC_VERSION 1u

typedef NTSTATUS (*RKMPP_CCU_QUERY_VERSION)(_Out_ PUINT32 Version);

typedef struct _RKMPP_CCU_INTERFACE {
    INTERFACE Header;                         /* INTERFACE per WDM */
    RKMPP_CCU_QUERY_VERSION QueryVersion;
    /* Phase 2: RaiseCluster, DropCluster, AssertCoreReset ... */
} RKMPP_CCU_INTERFACE, *PRKMPP_CCU_INTERFACE;
```

- [ ] **Step 3: Write `shared/rkiommu_ifc.h`**

```c
/* shared/rkiommu_ifc.h — In-kernel device interface exported by
 * rkiommu.sys for codec cores to consume.
 *
 * Phase 1: only the version probe exists. MapMdl/UnmapMdl land in Phase 2.
 */
#pragma once

#include <wdm.h>

DEFINE_GUID(GUID_DEVINTERFACE_RKIOMMU,
    0x4f9b1c23, 0x82a9, 0x4cd8, 0xb3, 0x14, 0x57, 0xa1, 0x0e, 0x44, 0x9d, 0x12);

#define RKIOMMU_IFC_VERSION 1u

typedef NTSTATUS (*RKIOMMU_QUERY_VERSION)(_Out_ PUINT32 Version);

typedef struct _RKIOMMU_INTERFACE {
    INTERFACE Header;
    RKIOMMU_QUERY_VERSION QueryVersion;
    /* Phase 2: MapMdl, UnmapMdl, RegisterFaultHandler ... */
} RKIOMMU_INTERFACE, *PRKIOMMU_INTERFACE;
```

- [ ] **Step 4: Compile-check the headers**

Run: from a developer command prompt with WDK set up:

```
cl /nologo /W4 /WX /Zs /I shared shared/rkmpp_ioctl.h shared/rkmpp_ccu_ifc.h shared/rkiommu_ifc.h
```

Expected: no warnings, no errors. (`/Zs` = syntax-only; the headers don't define functions so this just validates parsing.)

- [ ] **Step 5: Commit**

```bash
git add shared/
git commit -m "phase1: shared IOCTL and in-kernel ifc headers"
```

---

### Task 3: rkiommu.sys — INF and KMDF skeleton

**Files:**
- Create: `driver/rkiommu/rkiommu.inx`
- Create: `driver/rkiommu/driver.c`
- Create: `driver/rkiommu/ifc.c`
- Create: `driver/rkiommu/rkiommu.vcxproj`

- [ ] **Step 1: Write `driver/rkiommu/rkiommu.inx`**

```inf
;
; rkiommu.inx — Rockchip RK3588 IOMMU driver
; Matches RKCP3570 (rockchip,iommu-v2, 11 instances by _UID)
;     and RKCP3571 (rockchip,iommu-av1d).
;
[Version]
Signature   = "$WINDOWS NT$"
Class       = System
ClassGuid   = {4d36e97d-e325-11ce-bfc1-08002be10318}
Provider    = %ManufacturerName%
DriverVer   = ; will be stamped by stampinf
CatalogFile = rkiommu.cat
PnpLockdown = 1

[DestinationDirs]
DefaultDestDir = 13   ; driver store

[SourceDisksNames]
1 = %DiskName%,,,""

[SourceDisksFiles]
rkiommu.sys = 1,,

[Manufacturer]
%ManufacturerName% = Standard,NTARM64

[Standard.NTARM64]
%rkiommu.DeviceDesc% = rkiommu_Inst, ACPI\RKCP3570
%rkiommu.DeviceDesc% = rkiommu_Inst, ACPI\RKCP3571

[rkiommu_Inst.NT]
CopyFiles = Drivers_Dir

[Drivers_Dir]
rkiommu.sys

[rkiommu_Inst.NT.Services]
AddService = rkiommu, %SPSVCINST_ASSOCSERVICE%, rkiommu_Service_Inst

[rkiommu_Service_Inst]
DisplayName    = %rkiommu.SVCDESC%
ServiceType    = 1   ; SERVICE_KERNEL_DRIVER
StartType      = 3   ; SERVICE_DEMAND_START
ErrorControl   = 1
ServiceBinary  = %13%\rkiommu.sys

[Strings]
SPSVCINST_ASSOCSERVICE = 0x00000002
ManufacturerName       = "Rockchip Windows Drivers Project"
DiskName               = "rkiommu Installation Disk"
rkiommu.DeviceDesc     = "Rockchip RK3588 IOMMU"
rkiommu.SVCDESC        = "rkiommu Service"
```

- [ ] **Step 2: Write `driver/rkiommu/driver.c`**

```c
/* driver/rkiommu/driver.c — KMDF skeleton for rkiommu.sys */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkiommu_ifc.h"

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD RkIommuEvtDeviceAdd;

NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device);  /* in ifc.c */

NTSTATUS
DriverEntry(_In_ PDRIVER_OBJECT DriverObject, _In_ PUNICODE_STRING RegistryPath)
{
    WDF_DRIVER_CONFIG cfg;
    WDF_DRIVER_CONFIG_INIT(&cfg, RkIommuEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}

NTSTATUS
RkIommuEvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
{
    UNREFERENCED_PARAMETER(Driver);

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, WDF_NO_OBJECT_ATTRIBUTES, &device);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu: device added\n");

    return RkIommuRegisterIfc(device);
}
```

- [ ] **Step 3: Write `driver/rkiommu/ifc.c`**

```c
/* driver/rkiommu/ifc.c — version-only in-kernel ifc for Phase 1 */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkiommu_ifc.h"

static NTSTATUS RkIommuQueryVersion(_Out_ PUINT32 Version)
{
    *Version = RKIOMMU_IFC_VERSION;
    return STATUS_SUCCESS;
}

NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device)
{
    RKIOMMU_INTERFACE ifc;
    RtlZeroMemory(&ifc, sizeof(ifc));
    ifc.Header.Size             = sizeof(ifc);
    ifc.Header.Version          = RKIOMMU_IFC_VERSION;
    ifc.Header.Context          = WdfDeviceWdmGetDeviceObject(Device);
    ifc.Header.InterfaceReference   = WdfDeviceInterfaceReferenceNoOp;
    ifc.Header.InterfaceDereference = WdfDeviceInterfaceDereferenceNoOp;
    ifc.QueryVersion            = RkIommuQueryVersion;

    WDF_QUERY_INTERFACE_CONFIG cfg;
    WDF_QUERY_INTERFACE_CONFIG_INIT(&cfg,
                                    (PINTERFACE)&ifc,
                                    &GUID_DEVINTERFACE_RKIOMMU,
                                    NULL);
    return WdfDeviceAddQueryInterface(Device, &cfg);
}
```

- [ ] **Step 4: Write `driver/rkiommu/rkiommu.vcxproj` (boilerplate)**

```xml
<?xml version="1.0" encoding="utf-8"?>
<Project DefaultTargets="Build" ToolsVersion="Current"
         xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <ItemGroup Label="ProjectConfigurations">
    <ProjectConfiguration Include="Debug|ARM64">
      <Configuration>Debug</Configuration>
      <Platform>ARM64</Platform>
    </ProjectConfiguration>
  </ItemGroup>
  <PropertyGroup Label="Globals">
    <ProjectGuid>{B1C7A6A3-1111-4F00-9000-000000000001}</ProjectGuid>
    <RootNamespace>rkiommu</RootNamespace>
    <TargetVersion>Windows10</TargetVersion>
    <WindowsTargetPlatformVersion>10.0</WindowsTargetPlatformVersion>
    <DriverType>KMDF</DriverType>
    <DriverTargetPlatform>Universal</DriverTargetPlatform>
    <ConfigurationType>Driver</ConfigurationType>
    <PlatformToolset>WindowsKernelModeDriver10.0</PlatformToolset>
  </PropertyGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.Default.props" />
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.props" />
  <ItemGroup>
    <ClCompile Include="driver.c" />
    <ClCompile Include="ifc.c" />
    <Inf Include="rkiommu.inx" />
  </ItemGroup>
  <Import Project="$(VCTargetsPath)\Microsoft.Cpp.targets" />
</Project>
```

- [ ] **Step 5: Build the driver**

Run from a Visual Studio + WDK developer command prompt:

```
msbuild driver\rkiommu\rkiommu.vcxproj /p:Configuration=Debug /p:Platform=ARM64
```

Expected: succeeds; outputs `driver\rkiommu\ARM64\Debug\rkiommu\rkiommu.sys` and `rkiommu.inf`.

- [ ] **Step 6: Static analysis (no new warnings)**

Run:

```
msbuild driver\rkiommu\rkiommu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true
```

Expected: build succeeds with 0 errors and 0 warnings (the project is small enough that we treat warnings as errors here).

- [ ] **Step 7: Commit**

```bash
git add driver/rkiommu/
git commit -m "phase1: rkiommu.sys — KMDF skeleton matching RKCP3570/RKCP3571"
```

---

### Task 4: rkiommu.sys — deploy and verify on RK3588

**Files:**
- Create: `scripts/deploy.ps1`

- [ ] **Step 1: Write `scripts/deploy.ps1`**

```powershell
# scripts/deploy.ps1 — copies a built driver package to a target RK3588
# and installs it via pnputil.
param(
    [Parameter(Mandatory=$true)] [string] $Target,
    [string[]] $Drivers = @("rkiommu", "rkmpp_ccu", "rkmpp")
)

$ErrorActionPreference = "Stop"
$root = Resolve-Path "$PSScriptRoot\.."

foreach ($d in $Drivers) {
    $pkg = Get-ChildItem "$root\driver\$d\ARM64\Debug\$d" -ErrorAction SilentlyContinue
    if (-not $pkg) {
        Write-Warning "skipping $d — not built"
        continue
    }
    Write-Host "Deploying $d to $Target"
    $remote = "\\$Target\C`$\drvtest\$d"
    New-Item -ItemType Directory -Force -Path $remote | Out-Null
    Copy-Item "$($pkg.FullName)\*" $remote -Recurse -Force
    Invoke-Command -ComputerName $Target -ScriptBlock {
        param($d)
        pnputil /add-driver "C:\drvtest\$d\$d.inf" /install
    } -ArgumentList $d
}
```

- [ ] **Step 2: Deploy**

Run: `pwsh scripts/deploy.ps1 -Target <board> -Drivers rkiommu`
Expected: pnputil prints "Driver package added successfully" and a "Drivers installed: 1, 0 errors" summary.

- [ ] **Step 3: Verify enumeration**

Run on the target:
```
Get-PnpDevice -Class System | Where-Object { $_.HardwareID -match "RKCP357[01]" } | Format-Table FriendlyName, Status, ConfigManagerErrorCode
```

Expected: at least the RD0M instance of RKCP3570 enumerates with `Status = OK` and `ConfigManagerErrorCode = 0`. Additional RKCP3570 _UIDs may appear; all should be OK.

- [ ] **Step 4: Verify driver log**

Run on the target with a kernel debugger or WPP/DbgPrint capture (`bcdedit /debug on` and a kd connection, or `tracelog` capture of `DPFLTR_IHVDRIVER_ID`):

Expected: at least one `rkiommu: device added` line per attached _UID instance.

- [ ] **Step 5: Commit**

```bash
git add scripts/deploy.ps1
git commit -m "phase1: deploy.ps1 helper; rkiommu attaches on RK3588"
```

---

### Task 5: rkmpp_ccu.sys — INF, KMDF skeleton, deploy

**Files:**
- Create: `driver/rkmpp_ccu/rkmpp_ccu.inx`
- Create: `driver/rkmpp_ccu/driver.c`
- Create: `driver/rkmpp_ccu/ifc.c`
- Create: `driver/rkmpp_ccu/rkmpp_ccu.vcxproj`

- [ ] **Step 1: Write `driver/rkmpp_ccu/rkmpp_ccu.inx`**

Identical structure to rkiommu.inx, with these substitutions:
- All occurrences of `rkiommu` → `rkmpp_ccu`.
- The two `ACPI\RKCP357X` HWIDs replaced by:
  ```
  %rkmpp_ccu.DeviceDesc% = rkmpp_ccu_Inst, ACPI\RKCP3501
  %rkmpp_ccu.DeviceDesc% = rkmpp_ccu_Inst, ACPI\RKCP3502
  %rkmpp_ccu.DeviceDesc% = rkmpp_ccu_Inst, ACPI\RKCP3503
  ```
- Strings updated: `rkmpp_ccu.DeviceDesc = "Rockchip RK3588 MPP cluster control unit"`.

(Don't paraphrase — copy `rkiommu.inx` line-for-line and apply the substitutions above. The remaining sections — `[Version]`, `[DestinationDirs]`, etc. — are identical.)

- [ ] **Step 2: Write `driver/rkmpp_ccu/driver.c`**

Identical to rkiommu's `driver.c` with these substitutions:
- All `RkIommu` → `RkMppCcu`.
- All `rkiommu:` log strings → `rkmpp_ccu:`.
- `#include "../../shared/rkiommu_ifc.h"` → `#include "../../shared/rkmpp_ccu_ifc.h"`.

- [ ] **Step 3: Write `driver/rkmpp_ccu/ifc.c`**

Identical to rkiommu's `ifc.c` with the same substitutions, plus:
- `RKIOMMU_INTERFACE` / `RKIOMMU_IFC_VERSION` / `GUID_DEVINTERFACE_RKIOMMU` → `RKMPP_CCU_INTERFACE` / `RKMPP_CCU_IFC_VERSION` / `GUID_DEVINTERFACE_RKMPP_CCU`.
- `RkIommuQueryVersion` → `RkMppCcuQueryVersion`.

- [ ] **Step 4: Write `driver/rkmpp_ccu/rkmpp_ccu.vcxproj`**

Copy `rkiommu.vcxproj`, change the ProjectGuid to `{B1C7A6A3-1111-4F00-9000-000000000002}`, change `<RootNamespace>` to `rkmpp_ccu`, and update the `<Inf>` filename.

- [ ] **Step 5: Build**

Run: `msbuild driver\rkmpp_ccu\rkmpp_ccu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings, produces `rkmpp_ccu.sys`.

- [ ] **Step 6: Deploy and verify**

Run: `pwsh scripts/deploy.ps1 -Target <board> -Drivers rkmpp_ccu`

On the target:
```
Get-PnpDevice -Class System | Where-Object { $_.HardwareID -match "RKCP350[123]" } | Format-Table FriendlyName, Status, ConfigManagerErrorCode
```

Expected: the RKCP3503 (RDCC) instance enumerates with `Status = OK` and `ConfigManagerErrorCode = 0`. RKCP3501/RKCP3502 may or may not be present in firmware; whatever shows up should be `OK`.

- [ ] **Step 7: Commit**

```bash
git add driver/rkmpp_ccu/
git commit -m "phase1: rkmpp_ccu.sys — KMDF skeleton matching RKCP3501..RKCP3503"
```

---

### Task 6: rkmpp.sys — INF and skeleton (no IOCTL yet)

**Files:**
- Create: `driver/rkmpp/rkmpp.inx`
- Create: `driver/rkmpp/driver.c`
- Create: `driver/rkmpp/device.c`
- Create: `driver/rkmpp/profile.c`
- Create: `driver/rkmpp/rkmpp.vcxproj`

- [ ] **Step 1: Write `driver/rkmpp/rkmpp.inx`**

Same structure as rkiommu.inx, but matching all eight codec HIDs:

```
[Standard.NTARM64]
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3510
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3511
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3512
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3520
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3521
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3540
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3550
%rkmpp.DeviceDesc% = rkmpp_Inst, ACPI\RKCP3560
```

Class: `Media` (`{4d36e96c-e325-11ce-bfc1-08002be10318}`). Strings: `rkmpp.DeviceDesc = "Rockchip RK3588 MPP codec core"`.

- [ ] **Step 2: Write `driver/rkmpp/profile.c`**

```c
/* driver/rkmpp/profile.c — HID + _UID → per-instance profile */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"

typedef struct _RKMPP_PROFILE {
    UINT32 Hid;
    UINT32 Uid;
    UINT32 SupportedCodecs;     /* bitmap from rkmpp_ioctl.h */
    UINT32 RevisionRegOffset;   /* MMIO offset of the REVISION register */
} RKMPP_PROFILE;

static const RKMPP_PROFILE g_profiles[] = {
    /* RVD0 / RVD1 — rkv-decoder-v2 cores. v1 only enables H.264. */
    { 0x3550, 0, RKMPP_CODEC_H264, 0x0000 /* TODO: confirm vs hardware */ },
    { 0x3550, 1, RKMPP_CODEC_H264, 0x0000 },
    /* All other matching HIDs probe but expose no codec capability in Phase 1. */
    { 0x3510, 0, 0, 0x0000 },
    { 0x3511, 0, 0, 0x0000 },
    { 0x3512, 0, 0, 0x0000 },
    { 0x3520, 0, 0, 0x0000 },
    { 0x3521, 0, 0, 0x0000 },
    { 0x3521, 1, 0, 0x0000 },
    { 0x3521, 2, 0, 0x0000 },
    { 0x3521, 3, 0, 0x0000 },
    { 0x3540, 0, 0, 0x0000 },
    { 0x3540, 1, 0, 0x0000 },
    { 0x3560, 0, 0, 0x0000 },
};

const RKMPP_PROFILE*
RkMppFindProfile(_In_ UINT32 Hid, _In_ UINT32 Uid)
{
    for (ULONG i = 0; i < ARRAYSIZE(g_profiles); i++) {
        if (g_profiles[i].Hid == Hid && g_profiles[i].Uid == Uid) {
            return &g_profiles[i];
        }
    }
    return NULL;
}
```

> The `RevisionRegOffset = 0x0000` value is provisional. Task 7 confirms it on real hardware against the Linux `rkvdec2` driver source and amends the profile.

- [ ] **Step 3: Write `driver/rkmpp/driver.c`**

```c
/* driver/rkmpp/driver.c — DriverEntry + EvtDeviceAdd for rkmpp.sys */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD RkMppEvtDeviceAdd;

NTSTATUS RkMppDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit);  /* device.c */

NTSTATUS
DriverEntry(_In_ PDRIVER_OBJECT DriverObject, _In_ PUNICODE_STRING RegistryPath)
{
    WDF_DRIVER_CONFIG cfg;
    WDF_DRIVER_CONFIG_INIT(&cfg, RkMppEvtDeviceAdd);
    return WdfDriverCreate(DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES,
                           &cfg, WDF_NO_HANDLE);
}

NTSTATUS
RkMppEvtDeviceAdd(_In_ WDFDRIVER Driver, _Inout_ PWDFDEVICE_INIT DeviceInit)
{
    UNREFERENCED_PARAMETER(Driver);
    return RkMppDeviceCreate(DeviceInit);
}
```

- [ ] **Step 4: Write `driver/rkmpp/device.c` (skeleton — MMIO + ifc registration only, no IOCTL queue yet)**

```c
/* driver/rkmpp/device.c — per-instance device for rkmpp.sys.
 *
 * Phase 1 responsibilities:
 *   - parse HID + _UID from the ACPI hardware-ID list
 *   - look up the profile
 *   - map MMIO _CRS resource 0
 *   - read the REVISION register and stash it
 *   - register GUID_DEVINTERFACE_RKMPP so user mode can find this instance
 */
#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>

#include "../../shared/rkmpp_ioctl.h"

typedef struct _RKMPP_DEVICE {
    UINT32                 Hid;
    UINT32                 Uid;
    UINT32                 RevisionWord;
    UINT32                 SupportedCodecs;
    PVOID                  MmioBase;
    SIZE_T                 MmioLength;
} RKMPP_DEVICE, *PRKMPP_DEVICE;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKMPP_DEVICE, RkMppDeviceGet);

extern const struct _RKMPP_PROFILE* RkMppFindProfile(UINT32 Hid, UINT32 Uid);

EVT_WDF_DEVICE_PREPARE_HARDWARE     RkMppEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE     RkMppEvtReleaseHardware;

static NTSTATUS RkMppReadAcpiId(_In_ WDFDEVICE Device,
                                _Out_ PUINT32 Hid, _Out_ PUINT32 Uid);

NTSTATUS
RkMppDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit)
{
    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = RkMppEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = RkMppEvtReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    WDF_OBJECT_ATTRIBUTES attr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr, RKMPP_DEVICE);

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, &attr, &device);
    if (!NT_SUCCESS(status)) return status;

    return WdfDeviceCreateDeviceInterface(device,
                                          &GUID_DEVINTERFACE_RKMPP,
                                          NULL);
}

NTSTATUS
RkMppEvtPrepareHardware(_In_ WDFDEVICE Device,
                        _In_ WDFCMRESLIST ResourcesRaw,
                        _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);

    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    NTSTATUS status = RkMppReadAcpiId(Device, &ctx->Hid, &ctx->Uid);
    if (!NT_SUCCESS(status)) return status;

    /* Find first MMIO descriptor */
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            ctx->MmioBase = MmMapIoSpaceEx(d->u.Memory.Start,
                                           d->u.Memory.Length,
                                           PAGE_READWRITE | PAGE_NOCACHE);
            ctx->MmioLength = d->u.Memory.Length;
            break;
        }
    }
    if (!ctx->MmioBase) return STATUS_INSUFFICIENT_RESOURCES;

    const struct _RKMPP_PROFILE *p = RkMppFindProfile(ctx->Hid, ctx->Uid);
    if (p) {
        ctx->RevisionWord = READ_REGISTER_ULONG(
            (volatile ULONG*)((PUCHAR)ctx->MmioBase + 0));  /* off 0 in Phase 1 */
        ctx->SupportedCodecs = p->SupportedCodecs;
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkmpp: HID=RKCP%04x UID=%u rev=0x%08x codecs=0x%08x\n",
               ctx->Hid, ctx->Uid, ctx->RevisionWord, ctx->SupportedCodecs);
    return STATUS_SUCCESS;
}

NTSTATUS
RkMppEvtReleaseHardware(_In_ WDFDEVICE Device, _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    if (ctx->MmioBase) {
        MmUnmapIoSpace(ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase = NULL;
    }
    return STATUS_SUCCESS;
}

/* Reads ACPI _HID and _UID from the device-instance properties. */
static NTSTATUS RkMppReadAcpiId(_In_ WDFDEVICE Device,
                                _Out_ PUINT32 Hid, _Out_ PUINT32 Uid)
{
    /* The matching hardware ID is of the form ACPI\RKCP35xx; we read it via
     * IoGetDeviceProperty(DevicePropertyHardwareID).
     */
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(Device);
    WCHAR buf[256] = {0};
    ULONG size = 0;

    NTSTATUS status = IoGetDeviceProperty(pdo, DevicePropertyHardwareID,
                                          sizeof(buf), buf, &size);
    if (!NT_SUCCESS(status)) return status;

    /* Expect "ACPI\RKCP35xx" — pull the last 4 hex digits. */
    UNICODE_STRING us;
    RtlInitUnicodeString(&us, buf);
    PCWSTR p = wcsstr(buf, L"RKCP");
    if (!p || wcslen(p) < 8) return STATUS_INVALID_DEVICE_REQUEST;

    UINT32 hid = 0;
    for (int i = 4; i < 8; i++) {
        WCHAR c = p[i];
        UINT32 d;
        if (c >= L'0' && c <= L'9') d = c - L'0';
        else if (c >= L'a' && c <= L'f') d = 10 + (c - L'a');
        else if (c >= L'A' && c <= L'F') d = 10 + (c - L'A');
        else return STATUS_INVALID_DEVICE_REQUEST;
        hid = (hid << 4) | d;
    }
    *Hid = hid;

    /* _UID: read via DevicePropertyAddress is wrong; use IRP_MN_QUERY_ID
     * BusQueryInstanceID, or IoGetDeviceProperty(DevicePropertyUINumber).
     * For Phase 1 a property query is acceptable: */
    ULONG uid = 0;
    status = IoGetDeviceProperty(pdo, DevicePropertyUINumber,
                                 sizeof(uid), &uid, &size);
    if (!NT_SUCCESS(status)) uid = 0;  /* defaults to 0, matches RVD0 */
    *Uid = uid;
    return STATUS_SUCCESS;
}
```

- [ ] **Step 5: Write `driver/rkmpp/rkmpp.vcxproj`**

Copy `rkiommu.vcxproj`, change ProjectGuid to `{B1C7A6A3-1111-4F00-9000-000000000003}`, RootNamespace to `rkmpp`, list the new source files (`driver.c`, `device.c`, `profile.c`), and update the `<Inf>` filename.

- [ ] **Step 6: Build with code analysis**

Run: `msbuild driver\rkmpp\rkmpp.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings, produces `rkmpp.sys`.

- [ ] **Step 7: Commit**

```bash
git add driver/rkmpp/
git commit -m "phase1: rkmpp.sys — skeleton matching RKCP3510..RKCP3560 with MMIO map and ACPI ID parse"
```

---

### Task 7: Confirm RVD0 revision-register offset on real hardware

**Files:**
- Modify: `driver/rkmpp/profile.c:14-15` (RVD0 / RVD1 entries' `RevisionRegOffset`)

- [ ] **Step 1: Cross-reference Linux source**

Look up `RKVDEC2_REG_VERSION` (or the equivalent constant) in the Linux mainline `drivers/staging/media/rkvdec` / `drivers/media/platform/verisilicon/rkvdec` tree. Record the offset.

- [ ] **Step 2: Update `g_profiles` in `driver/rkmpp/profile.c`**

Replace both RVD0 and RVD1 lines' `0x0000` with the value found in Step 1. Add a comment with the source:

```c
/* RKVDEC2_REG_VERSION per Linux drivers/.../rkvdec2.h commit <hash> */
{ 0x3550, 0, RKMPP_CODEC_H264, 0x0XXX },
{ 0x3550, 1, RKMPP_CODEC_H264, 0x0XXX },
```

- [ ] **Step 3: Apply the offset in `driver/rkmpp/device.c`**

In `RkMppEvtPrepareHardware`, replace the hardcoded `0` in the `READ_REGISTER_ULONG` call with `p->RevisionRegOffset`.

- [ ] **Step 4: Rebuild + redeploy + verify**

Build, deploy with `pwsh scripts/deploy.ps1 -Target <board> -Drivers rkmpp`, and check the kernel log for the `rkmpp: HID=RKCP3550 UID=0 rev=0x...` line.

Expected: revision word is non-zero and matches the value documented in the Linux driver / RK3588 TRM for rkv-decoder-v2.

- [ ] **Step 5: Commit**

```bash
git add driver/rkmpp/profile.c driver/rkmpp/device.c
git commit -m "phase1: rkmpp — confirm RVD0 REVISION offset against Linux rkvdec2"
```

---

### Task 8: rkmpp.sys — IOCTL_RKMPP_GET_CAPS

**Files:**
- Create: `driver/rkmpp/ioctl.c`
- Modify: `driver/rkmpp/device.c` — install IO queue
- Modify: `driver/rkmpp/rkmpp.vcxproj` — add `ioctl.c`

- [ ] **Step 1: Write `driver/rkmpp/ioctl.c`**

```c
/* driver/rkmpp/ioctl.c — IOCTL surface for rkmpp.sys.
 * Phase 1 implements only IOCTL_RKMPP_GET_CAPS.
 */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"

typedef struct _RKMPP_DEVICE RKMPP_DEVICE, *PRKMPP_DEVICE;
extern PRKMPP_DEVICE RkMppDeviceGet(_In_ WDFDEVICE);

/* Fields the IOCTL handler reads from the device context. */
typedef struct _RKMPP_DEVICE_PUBLIC {
    UINT32 Hid; UINT32 Uid; UINT32 RevisionWord; UINT32 SupportedCodecs;
} RKMPP_DEVICE_PUBLIC;

/* Provided by device.c — copies the four fields from the real ctx. */
extern void RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out);

EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL RkMppEvtIoDeviceControl;

NTSTATUS RkMppQueueInit(_In_ WDFDEVICE Device)
{
    WDF_IO_QUEUE_CONFIG cfg;
    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&cfg, WdfIoQueueDispatchSequential);
    cfg.EvtIoDeviceControl = RkMppEvtIoDeviceControl;

    WDFQUEUE q;
    return WdfIoQueueCreate(Device, &cfg, WDF_NO_OBJECT_ATTRIBUTES, &q);
}

VOID
RkMppEvtIoDeviceControl(_In_ WDFQUEUE Queue,
                        _In_ WDFREQUEST Request,
                        _In_ size_t OutputBufferLength,
                        _In_ size_t InputBufferLength,
                        _In_ ULONG IoControlCode)
{
    UNREFERENCED_PARAMETER(InputBufferLength);

    NTSTATUS status = STATUS_INVALID_DEVICE_REQUEST;
    ULONG_PTR info = 0;

    if (IoControlCode == IOCTL_RKMPP_GET_CAPS) {
        if (OutputBufferLength < sizeof(RKMPP_CAPS)) {
            status = STATUS_BUFFER_TOO_SMALL;
        } else {
            PRKMPP_CAPS out;
            status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                    (PVOID*)&out, NULL);
            if (NT_SUCCESS(status)) {
                RKMPP_DEVICE_PUBLIC pub;
                RkMppGetPublic(WdfIoQueueGetDevice(Queue), &pub);

                RtlZeroMemory(out, sizeof(*out));
                out->StructSize       = sizeof(*out);
                out->Hid              = pub.Hid;
                out->Uid              = pub.Uid;
                out->RevisionWord     = pub.RevisionWord;
                out->SupportedCodecs  = pub.SupportedCodecs;
                info = sizeof(*out);
            }
        }
    }

    WdfRequestCompleteWithInformation(Request, status, info);
}
```

- [ ] **Step 2: Add `RkMppGetPublic` and queue init to `driver/rkmpp/device.c`**

Append at the bottom of `device.c`:

```c
extern NTSTATUS RkMppQueueInit(WDFDEVICE Device);  /* in ioctl.c */

void RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    Out->Hid             = ctx->Hid;
    Out->Uid             = ctx->Uid;
    Out->RevisionWord    = ctx->RevisionWord;
    Out->SupportedCodecs = ctx->SupportedCodecs;
}
```

And in `RkMppDeviceCreate`, replace `return WdfDeviceCreateDeviceInterface(...)` with:

```c
status = WdfDeviceCreateDeviceInterface(device, &GUID_DEVINTERFACE_RKMPP, NULL);
if (!NT_SUCCESS(status)) return status;
return RkMppQueueInit(device);
```

Add this typedef near the top of `device.c` (matching what `ioctl.c` declares):

```c
typedef struct _RKMPP_DEVICE_PUBLIC {
    UINT32 Hid; UINT32 Uid; UINT32 RevisionWord; UINT32 SupportedCodecs;
} RKMPP_DEVICE_PUBLIC;
```

- [ ] **Step 3: Update `rkmpp.vcxproj` to include `ioctl.c`**

Add `<ClCompile Include="ioctl.c" />` next to the existing entries.

- [ ] **Step 4: Build with code analysis**

Run: `msbuild driver\rkmpp\rkmpp.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:RunCodeAnalysis=true`
Expected: 0 errors, 0 warnings.

- [ ] **Step 5: Deploy and verify enumeration**

Run: `pwsh scripts/deploy.ps1 -Target <board> -Drivers rkmpp`

On the target:
```
Get-PnpDevice -Class Media | Where-Object { $_.HardwareID -match "RKCP35[1-6]\d" } | Format-Table FriendlyName, Status, ConfigManagerErrorCode
```

Expected: at least RKCP3550 (RVD0) is present, `Status = OK`, `ConfigManagerErrorCode = 0`.

- [ ] **Step 6: Commit**

```bash
git add driver/rkmpp/
git commit -m "phase1: rkmpp — IOCTL_RKMPP_GET_CAPS handler"
```

---

### Task 9: User-mode `rkmpp_caps_dump` — TDD'd against a fake device

**Files:**
- Create: `tests/harness/CMakeLists.txt`
- Create: `tests/harness/rkmpp_caps_dump/CMakeLists.txt`
- Create: `tests/harness/rkmpp_caps_dump/main.cpp`
- Create: `tests/harness/rkmpp_caps_dump/test_main.cpp`

The tool has two responsibilities — enumeration and pretty-printing — and we TDD the pretty-printer separately from the SetupAPI calls so we have a real failing test before the real implementation.

- [ ] **Step 1: Top-level `tests/harness/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(rkmpp_harness CXX)
set(CMAKE_CXX_STANDARD 17)
enable_testing()
add_subdirectory(rkmpp_caps_dump)
```

- [ ] **Step 2: Tool `tests/harness/rkmpp_caps_dump/CMakeLists.txt`**

```cmake
add_executable(rkmpp_caps_dump main.cpp)
target_include_directories(rkmpp_caps_dump PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
target_link_libraries(rkmpp_caps_dump PRIVATE setupapi cfgmgr32)

add_executable(rkmpp_caps_dump_test test_main.cpp)
target_include_directories(rkmpp_caps_dump_test
    PRIVATE ${CMAKE_SOURCE_DIR}/../../shared)
add_test(NAME rkmpp_caps_dump_test COMMAND rkmpp_caps_dump_test)
```

- [ ] **Step 3: Write the failing test `test_main.cpp`**

```cpp
/* Unit tests for the pretty-printer in rkmpp_caps_dump. */
#include <cstdio>
#include <sstream>
#include <string>

#include "../../../shared/rkmpp_ioctl.h"

std::string FormatCaps(const RKMPP_CAPS &c);  /* defined in main.cpp */

static int Fail(const char *msg) { std::fprintf(stderr, "FAIL: %s\n", msg); return 1; }

int main() {
    RKMPP_CAPS c{};
    c.StructSize      = sizeof(c);
    c.Hid             = 0x3550;
    c.Uid             = 0;
    c.RevisionWord    = 0xdeadbeef;
    c.SupportedCodecs = RKMPP_CODEC_H264;

    std::string s = FormatCaps(c);
    if (s.find("RKCP3550") == std::string::npos) return Fail("missing HID");
    if (s.find("UID=0")    == std::string::npos) return Fail("missing UID");
    if (s.find("0xdeadbeef") == std::string::npos) return Fail("missing rev");
    if (s.find("H264")     == std::string::npos) return Fail("missing H264 codec");
    return 0;
}
```

- [ ] **Step 4: Run the test, see it fail**

```
cmake -S tests/harness -B build/harness -A ARM64
cmake --build build/harness --config Debug
ctest --test-dir build/harness -C Debug -V
```

Expected: build fails — `FormatCaps` is undefined. (This is the failing-test gate; we'll make it succeed in Step 5.)

- [ ] **Step 5: Implement `main.cpp` with the printer + the SetupAPI loop**

```cpp
/* tests/harness/rkmpp_caps_dump/main.cpp
 * Enumerates GUID_DEVINTERFACE_RKMPP, opens each instance, calls
 * IOCTL_RKMPP_GET_CAPS, and prints a one-line summary per device.
 */
#define UMDF_USING_NTSTATUS
#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <cstdio>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "../../../shared/rkmpp_ioctl.h"

std::string FormatCaps(const RKMPP_CAPS &c)
{
    std::ostringstream os;
    os << "RKCP" << std::hex << c.Hid << std::dec
       << " UID=" << c.Uid
       << " rev=0x" << std::hex << c.RevisionWord << std::dec
       << " codecs=";
    bool first = true;
    auto bit = [&](uint32_t mask, const char *name) {
        if (c.SupportedCodecs & mask) { if (!first) os << "+"; os << name; first = false; }
    };
    bit(RKMPP_CODEC_H264,   "H264");
    bit(RKMPP_CODEC_HEVC,   "HEVC");
    bit(RKMPP_CODEC_VP9,    "VP9");
    bit(RKMPP_CODEC_AV1,    "AV1");
    bit(RKMPP_CODEC_JPEG_D, "JPEG_D");
    bit(RKMPP_CODEC_AVS,    "AVS");
    if (first) os << "(none)";
    return os.str();
}

#ifndef RKMPP_CAPS_DUMP_TEST  /* test_main.cpp re-includes this without main() */
int wmain()
{
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVINTERFACE_RKMPP, nullptr, nullptr,
                                        DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) {
        std::fprintf(stderr, "SetupDiGetClassDevsW failed: %lu\n", GetLastError());
        return 1;
    }

    SP_DEVICE_INTERFACE_DATA ifd{ sizeof(ifd) };
    int idx = 0, found = 0;
    while (SetupDiEnumDeviceInterfaces(set, nullptr, &GUID_DEVINTERFACE_RKMPP,
                                       idx++, &ifd))
    {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &need, nullptr);
        std::vector<uint8_t> buf(need);
        auto *det = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buf.data());
        det->cbSize = sizeof(*det);
        if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, det, need, nullptr, nullptr)) {
            std::fprintf(stderr, "GetDetail failed: %lu\n", GetLastError());
            continue;
        }

        HANDLE h = CreateFileW(det->DevicePath, GENERIC_READ, 0, nullptr,
                               OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            std::fwprintf(stderr, L"open %s failed: %lu\n",
                          det->DevicePath, GetLastError());
            continue;
        }
        RKMPP_CAPS caps{};
        DWORD got = 0;
        if (!DeviceIoControl(h, IOCTL_RKMPP_GET_CAPS, nullptr, 0,
                             &caps, sizeof(caps), &got, nullptr)) {
            std::fwprintf(stderr, L"IOCTL on %s failed: %lu\n",
                          det->DevicePath, GetLastError());
        } else {
            std::printf("%s\n", FormatCaps(caps).c_str());
            found++;
        }
        CloseHandle(h);
    }
    SetupDiDestroyDeviceInfoList(set);
    if (!found) { std::fprintf(stderr, "no rkmpp instances found\n"); return 2; }
    return 0;
}
#endif
```

- [ ] **Step 6: Make `test_main.cpp` compile its own `FormatCaps`**

Update `tests/harness/rkmpp_caps_dump/CMakeLists.txt`:

```cmake
target_compile_definitions(rkmpp_caps_dump_test PRIVATE RKMPP_CAPS_DUMP_TEST)
target_sources(rkmpp_caps_dump_test PRIVATE main.cpp)
```

- [ ] **Step 7: Run the test, see it pass**

```
cmake --build build/harness --config Debug
ctest --test-dir build/harness -C Debug -V
```

Expected: `rkmpp_caps_dump_test ............ Passed`.

- [ ] **Step 8: Commit**

```bash
git add tests/harness/
git commit -m "phase1: rkmpp_caps_dump tool with TDD'd FormatCaps printer"
```

---

### Task 10: End-to-end verification on RK3588

- [ ] **Step 1: Deploy all three drivers**

Run: `pwsh scripts/deploy.ps1 -Target <board>`
Expected: all three pnputil installs report success.

- [ ] **Step 2: Copy `rkmpp_caps_dump.exe` to the target**

```
Copy-Item build\harness\rkmpp_caps_dump\Debug\rkmpp_caps_dump.exe \\<board>\C`$\drvtest\
```

- [ ] **Step 3: Run on the target**

```
Invoke-Command -ComputerName <board> { C:\drvtest\rkmpp_caps_dump.exe }
```

Expected output includes a line like:

```
RKCP3550 UID=0 rev=0x<nonzero> codecs=H264
```

(Other RKCP35xx instances may also print, with `codecs=(none)` in v1.)

- [ ] **Step 4: Document the observed output**

Append a short section to `README.md` under a `## Phase 1 verification` heading, pasting one captured run's output. This locks in the expected baseline for Phase 2 to extend.

- [ ] **Step 5: Commit and tag**

```bash
git add README.md
git commit -m "phase1: end-to-end verification — RVD0 visible to user mode via IOCTL_RKMPP_GET_CAPS"
git tag phase1-done
```

---

## Spec coverage check (self-review)

| Spec section | Phase 1 task |
|---|---|
| §2 — three-driver topology | Tasks 3, 5, 6 (skeletons of all three) |
| §3.1 — rkmpp.sys ACPI matching, MMIO, IOCTL surface | Tasks 6, 7, 8 (skeleton, revision read, IOCTL_RKMPP_GET_CAPS) |
| §3.2 — rkmpp_ccu.sys ACPI matching, ifc registration | Task 5 (version-only ifc) |
| §3.3 — rkiommu.sys ACPI matching, ifc registration | Tasks 3 + 4 (version-only ifc, deploy verify) |
| §5 — repo layout | Task 1 + every later task that adds files |
| §9 — build & deploy | Task 1 + Task 4 (deploy.ps1) |

Out-of-scope-for-Phase-1 (deferred to Phase 2): cluster raise/drop, MMIO IOMMU programming, buffer pool, ALLOC/FREE/SUBMIT/WAIT, register-list jobs, MFT shell, FFmpeg vendoring.
