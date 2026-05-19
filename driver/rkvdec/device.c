/* driver/rkmpp/device.c — per-instance device for rkmpp.sys.
 *
 * Phase 1 responsibilities:
 *   - parse HID + _UID from the ACPI hardware-ID list
 *   - look up the profile
 *   - map MMIO _CRS resource 0
 *   - read the REVISION register and stash it
 *   - register GUID_DEVINTERFACE_RKMPP so user mode can find this instance
 */
#include <initguid.h>
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "profile.h"
#include "devpub.h"
#include "../shared/rkmpp/ifc_client.h"
#include "../shared/rkmpp/bufpool.h"
#include "../shared/acpi_uid.h"
#include "job.h"

/* Per-codec memory window count.  rkvdec2 declares 1 or 2 contiguous
 * regions (we merge to one MmioBase).  AV1 declares 3 separate windows
 * (VCD, CACHE, AFBC) at 64 KB-spaced base addresses with un-allocated
 * phys in between, so they CANNOT be merged: we map each independently
 * and look them up via index. */
#define RKMPP_MAX_MMIO_WINDOWS 1

typedef struct _RKMPP_MMIO_WINDOW {
    PVOID  Base;       /* virt mapping, NULL if slot unused */
    SIZE_T Length;
    PHYSICAL_ADDRESS Phys;
} RKMPP_MMIO_WINDOW;

typedef struct _RKMPP_DEVICE {
    UINT32                 Hid;
    UINT32                 Uid;
    UINT32                 RevisionWord;
    UINT32                 SupportedCodecs;
    RKMPP_CODEC_PERSONALITY Personality;
    /* Primary MMIO base — first window (rkvdec2: link+regs merged;
     * AV1: VCD).  Job/IOCTL paths still address through MmioBase as
     * the codec's main register window.  Additional windows live in
     * Mmios[1..]. */
    PVOID                  MmioBase;
    SIZE_T                 MmioLength;
    RKMPP_MMIO_WINDOW      Mmios[RKMPP_MAX_MMIO_WINDOWS];
    UINT32                 MmioCount;
    RKMPP_IFC_CLIENT       Ifcs;
    RKMPP_JOB_QUEUE        JobQueue;

    /* Phase 3a: IOMMU fault state — written by RkMppOnIommuFault at
     * DISPATCH_LEVEL, read by IOCTL_RKMPP_INJECT_IOMMU_FAULT at any level. */
    volatile LONG          FaultTriggered;   /* set to 1 by the callback */
    volatile LONG          FaultStatusReg;   /* status reg captured */
    volatile LONG64        FaultIova;        /* iova captured */

    /* Per-kick core reset is harmful — BSP only resets after error/timeout
     * (mpp_common.c:2026: if reset_request > 0 mpp_dev_reset).  We mirror
     * that: 1 before first kick, set after error, cleared after reset. */
    volatile LONG          NeedsCoreReset;
    /* Escalation flag set when a failure follows another recent failure —
     * narrow CoreReset (CON40 bits 6..9 toggle) doesn't recover the codec
     * once dec_e is stuck at 1 (mid-decode wedge).  When set, the next
     * JobStart runs the wider FullCoreReset0/1 (PMU idle + NIU + CABAC +
     * CORE bundle) and re-Attaches the IOMMU (FullCoreResetN zeroes
     * DTE_ADDR as a side effect). */
    volatile LONG          NeedsFullReset;
    /* Tracks whether the most recent JobComplete reported failure, so we
     * can detect "two failures in a row" and escalate. */
    volatile LONG          LastJobFailed;
} RKMPP_DEVICE, *PRKMPP_DEVICE;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKMPP_DEVICE, RkMppDeviceGet);

LONG
RkMppQueryAndClearNeedsCoreReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return InterlockedExchange(&ctx->NeedsCoreReset, 0);
}

VOID
RkMppSetNeedsCoreReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    InterlockedExchange(&ctx->NeedsCoreReset, 1);
}

LONG
RkMppQueryAndClearNeedsFullReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return InterlockedExchange(&ctx->NeedsFullReset, 0);
}

VOID
RkMppSetNeedsFullReset(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    InterlockedExchange(&ctx->NeedsFullReset, 1);
    /* Cascade: full reset is a strict superset of core reset, but we
     * also flag the narrow one so any code path that only checks the
     * core flag still triggers SOMETHING. */
    InterlockedExchange(&ctx->NeedsCoreReset, 1);
}

LONG
RkMppExchangeLastJobFailed(_In_ WDFDEVICE Device, LONG NewValue)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return InterlockedExchange(&ctx->LastJobFailed, NewValue);
}

EVT_WDF_DEVICE_PREPARE_HARDWARE     RkMppEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE     RkMppEvtReleaseHardware;
EVT_WDF_OBJECT_CONTEXT_CLEANUP      RkMppEvtDeviceContextCleanup;
EVT_WDF_FILE_CLEANUP                RkMppEvtFileCleanup;
EVT_WDF_FILE_CLOSE                  RkMppEvtFileClose;

/* Phase 3a: IOMMU fault callback — registered in PrepareHardware. */
static VOID RkMppOnIommuFault(_In_ PVOID  ClientCookie,
                               _In_ ULONG64 FaultIova,
                               _In_ ULONG   StatusReg);

/* ISR and DPC live in job.c but are declared via EVT_WDF_INTERRUPT_* here
 * so we can pass them to WdfInterruptCreate in RkMppDeviceCreate. */
extern EVT_WDF_INTERRUPT_ISR  RkMppEvtIsr;
extern EVT_WDF_INTERRUPT_DPC  RkMppEvtDpc;

extern NTSTATUS RkMppQueueInit(_In_ WDFDEVICE Device);  /* in ioctl.c */

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

    /* Configure per-file-object context so bufpool.c can track allocations
     * per open handle.  EvtFileCleanup is invoked while the process is still
     * alive (or we KeStackAttachProcess for safety), before EvtFileClose. */
    WDF_FILEOBJECT_CONFIG foCfg;
    WDF_FILEOBJECT_CONFIG_INIT(&foCfg,
                               WDF_NO_EVENT_CALLBACK,  /* EvtFileCreate */
                               RkMppEvtFileClose,
                               RkMppEvtFileCleanup);

    WDF_OBJECT_ATTRIBUTES foAttr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&foAttr, RKMPP_FILE_CTX);

    WdfDeviceInitSetFileObjectConfig(DeviceInit, &foCfg, &foAttr);

    WDF_OBJECT_ATTRIBUTES attr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr, RKMPP_DEVICE);
    attr.EvtCleanupCallback = RkMppEvtDeviceContextCleanup;

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, &attr, &device);
    if (!NT_SUCCESS(status)) return status;

    status = WdfDeviceCreateDeviceInterface(device, &GUID_DEVINTERFACE_RKMPP, NULL);
    if (!NT_SUCCESS(status)) return status;

    /* Initialise the job queue (spin lock, lists, DPC) before any IOCTL
     * can arrive.  Must be called before RkMppQueueInit so the IOCTL
     * handlers can safely call RkMppGetJobQueue. */
    PRKMPP_DEVICE devCtx = RkMppDeviceGet(device);
    RkMppJobQueueInit(device, &devCtx->JobQueue);

    /* Wire the codec IRQ from EvtDeviceAdd-time with the default config.
     * Calling WdfInterruptCreate from EvtDevicePrepareHardware with
     * explicit InterruptRaw+InterruptTranslated descriptors fails with
     * STATUS_WDF_INVALID_INTERRUPT_CONFIG (0xC020020F) on our ARM64 GIC
     * SPI lines — WDF's strict-binding validation rejects the descriptors
     * even though the same shape works for rkiommu_vdec.  EvtDeviceAdd's
     * lenient path lets WDF auto-bind during normal PnP resource
     * assignment, matching rkiommu_vdec's pattern.
     *
     * Fatal on failure: completion is interrupt-driven exclusively.
     * The previous fall-back poller has been gated off — the polling
     * thread still runs but skips its INT_STATUS poll loop on every
     * KickEvent wake. */
    {
        WDF_INTERRUPT_CONFIG intCfg;
        WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkMppEvtIsr, RkMppEvtDpc);
        NTSTATUS intStatus = WdfInterruptCreate(
            device, &intCfg, WDF_NO_OBJECT_ATTRIBUTES,
            &devCtx->JobQueue.Interrupt);
        if (!NT_SUCCESS(intStatus)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: WdfInterruptCreate failed 0x%08x — "
                       "refusing to load (interrupt-driven completion "
                       "is non-optional)\n",
                       (ULONG)intStatus);
            return intStatus;
        }
    }

    return RkMppQueueInit(device);
}

NTSTATUS
RkMppEvtPrepareHardware(_In_ WDFDEVICE Device,
                        _In_ WDFCMRESLIST ResourcesRaw,
                        _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);  /* WDF auto-binds the IRQ
                                            * descriptor for the interrupt
                                            * registered in DeviceCreate;
                                            * we don't touch raw resources. */
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    NTSTATUS status = RkMppReadAcpiId(Device, &ctx->Hid, &ctx->Uid);
    if (!NT_SUCCESS(status)) return status;

    /* First kick after PnP MUST reset: empirically, a fresh PD power-on
     * leaves the codec FSM in a state where dec_e=1 is accepted but the
     * core never starts (perf[229] doesn't advance, no AXI traffic).
     * Confirmed by skip-first-reset test: zero decode progress.
     * Subsequent kicks only reset after error/timeout (BSP parity via
     * mpp_common.c:2026 — gates mpp_dev_reset on reset_request > 0). */
    InterlockedExchange(&ctx->NeedsCoreReset, 1);

    /* Step 1: walk resources to capture MMIO windows.  The codec IRQ
     * was already registered at EvtDeviceAdd time via WdfInterruptCreate
     * (see RkMppDeviceCreate); WDF auto-binds it against the
     * CmResourceTypeInterrupt entry in this resource list during PnP
     * assignment, so we don't touch the descriptor here. */

    /* RVD0/RVD1 (RKCP3550) declares TWO physically contiguous memory regions
     *   link : 0xFDC38000 / 0x100  ← idx 0..63
     *   regs : 0xFDC38100 / 0x400  ← idx 64..319
     * which we merge into one MmioBase so the BSP idx*4 byte offsets line up.
     *
     * Strategy: take the convex hull and map as a single block.
     * The primary MmioBase always points at Mmios[0].Base. */
    PHYSICAL_ADDRESS mmioLow  = {0};
    PHYSICAL_ADDRESS mmioHigh = {0};
    BOOLEAN          mmioFound = FALSE;
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            PHYSICAL_ADDRESS start = d->u.Memory.Start;
            ULONG len = d->u.Memory.Length;
            PHYSICAL_ADDRESS end;
            end.QuadPart = start.QuadPart + len;
            if (!mmioFound) {
                mmioLow   = start;
                mmioHigh  = end;
                mmioFound = TRUE;
            } else {
                if (start.QuadPart < mmioLow.QuadPart)  mmioLow  = start;
                if (end.QuadPart   > mmioHigh.QuadPart) mmioHigh = end;
            }
        }
        /* CmResourceTypeInterrupt entries are consumed by WDF's auto-bind
         * for the WDFINTERRUPT registered in RkMppDeviceCreate — we don't
         * extract them here. */
    }

    if (!mmioFound) return STATUS_INSUFFICIENT_RESOURCES;
    ULONG mmioLen = (ULONG)(mmioHigh.QuadPart - mmioLow.QuadPart);
    if (mmioLen < 0x800) mmioLen = 0x800;  /* extend to cover cache regs */
    PVOID v = MmMapIoSpaceEx(mmioLow, mmioLen, PAGE_READWRITE | PAGE_NOCACHE);
    if (!v) return STATUS_INSUFFICIENT_RESOURCES;
    ctx->Mmios[0].Base   = v;
    ctx->Mmios[0].Length = mmioLen;
    ctx->Mmios[0].Phys   = mmioLow;
    ctx->MmioCount       = 1;
    ctx->MmioBase        = v;
    ctx->MmioLength      = mmioLen;
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkvdec: HID=RKCP%04x UID=%u MmioBase=phys 0x%llx len 0x%x\n",
               ctx->Hid, ctx->Uid, mmioLow.QuadPart, mmioLen);

    /* WDF interrupt registration moved to RkMppDeviceCreate
     * (EvtDeviceAdd-time) to avoid the STATUS_WDF_INVALID_INTERRUPT_CONFIG
     * failure that the EvtPrepareHardware-time explicit-descriptor path
     * hit on our ARM64 GIC SPI lines.  See the comment block in
     * RkMppDeviceCreate. */

    /* Step 2: open the in-kernel ifcs of rkmpp_ccu.sys and rkiommu.sys.
     * Without them we cannot service a single IOCTL beyond GET_CAPS, so refuse
     * to load (PnP code 31, no bugcheck). */
    status = RkMppOpenIfcs(Device, ctx->Hid, ctx->Uid, &ctx->Ifcs);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp: ccu/iommu ifcs unavailable (0x%08x) — install "
                   "rkmpp_ccu.sys and rkiommu.sys before rkmpp.sys\n", status);
        return STATUS_DEVICE_NOT_READY;
    }

    if (ctx->Ifcs.Iommu.Header.Version != RKIOMMU_IFC_VERSION ||
        ctx->Ifcs.Ccu.Header.Version   != RKMPP_CCU_IFC_VERSION) {
        RkMppCloseIfcs(&ctx->Ifcs);
        return STATUS_REVISION_MISMATCH;
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkmpp: ifcs opened (iommu v%u, ccu v%u)\n",
               ctx->Ifcs.Iommu.Header.Version, ctx->Ifcs.Ccu.Header.Version);

    /* Raise the cluster.  Refcounted; matching DropCluster in ReleaseHardware.
     * Picks the rkvdec0/1 path or the AV1 path based on personality, which
     * we look up from the profile early because the per-codec branch below
     * (where ctx->Personality is finalized) hasn't run yet. */
    PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
    status = ctx->Ifcs.Ccu.RaiseCluster(cookie);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkmpp: RaiseCluster failed 0x%08x\n", status);
        RkMppCloseIfcs(&ctx->Ifcs);
        return status;
    }

    /* Cluster bring-up confirmed working on real hardware as of Phase 3b
     * Task 2 — PD_VCODEC + PD_VDPU + PD_RKVDEC0/1 all power on cleanly,
     * NIU/CCU resets deassert, codec MMIO at 0xFDC38100/0xFDC48100 reads
     * REVISION = 0x53813f05.  Now the rest of PrepareHardware is safe. */
    const RKMPP_PROFILE *p = RkMppFindProfile(ctx->Hid, ctx->Uid);
    if (p) {
        ctx->SupportedCodecs = p->SupportedCodecs;
        ctx->Personality     = p->Personality;
    }

    /* Only touch codec MMIO + IOMMU wiring for devices we actually plan
     * to drive in this phase (decoders only — RVD0/RVD1).  RaiseCluster
     * powers the decoder PDs only; encoder/JPEG/IEP cores live in
     * PD_VENC0/1, etc., which we don't bring up here, so reading their
     * MMIO would SError.  The non-decoder probes still get a profile
     * row (SupportedCodecs=0) so subsequent IOCTLs can report
     * "unsupported"; they just don't get any bring-up activity. */
    if (p && p->SupportedCodecs != 0) {
        /* Wide hang-recovery reset on every PrepareHardware so partial
         * driver reinstall self-heals codec FSM + IOMMU state without
         * needing a full driver-chain reload.  v7 FullCoreReset was
         * RVD0-only; v8 has per-instance narrow variants that skip the
         * shared VDPU NIU reset (CON44 bits 4..6) to avoid disrupting
         * peer codecs.
         *
         * FullCoreResetN zeroes IOMMU DTE_ADDR as a side effect (the
         * AXI/AHB bus reset lives on the same NIU the IOMMU sits
         * behind), so we MUST follow with Reattach to reprogram
         * DTE_ADDR before any further codec activity. */
        if (ctx->Ifcs.CcuOpen) {
            if (ctx->Uid == 0 && ctx->Ifcs.Ccu.FullCoreReset0) {
                ctx->Ifcs.Ccu.FullCoreReset0(ctx->Ifcs.Ccu.Header.Context);
            } else if (ctx->Uid == 1 && ctx->Ifcs.Ccu.FullCoreReset1) {
                ctx->Ifcs.Ccu.FullCoreReset1(ctx->Ifcs.Ccu.Header.Context);
            }
        }
        if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Reattach) {
            NTSTATUS rs = ctx->Ifcs.Iommu.Reattach(ctx->Ifcs.Iommu.Header.Context);
            if (!NT_SUCCESS(rs)) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkmpp: post-Raise Reattach failed 0x%08x\n", rs);
            }
        }

        /* Capture REVISION as ground truth that codec MMIO is alive. */
        ctx->RevisionWord = READ_REGISTER_ULONG(
            (volatile ULONG*)((PUCHAR)ctx->MmioBase + p->RevisionRegOffset));

        /* Wire the IOMMU fault callback.  rkiommu's DPC invokes this when
         * a translation fault posts; we just stash the fault state for the
         * IOCTL surface to surface to userspace.  Safe to register before
         * RkIommuEnable runs — the IRQ stays masked until then.
         *
         * Hardening item: this currently registers globally per device
         * cookie; once we have a job queue, the fault should be attributed
         * to the in-flight job and trigger AssertRvdec{0,1}CoreReset on its core. */
        if (ctx->Ifcs.Iommu.RegisterFaultHandler) {
            ctx->Ifcs.Iommu.RegisterFaultHandler(
                ctx->Ifcs.Iommu.Header.Context,         /* iommu instance */
                WdfDeviceWdmGetDeviceObject(Device),    /* our device, for callback */
                RkMppOnIommuFault);
        }

        /* Pulse the core reset.  RaiseCluster deasserts the bring-up reset
         * bundle (NIU + bus + cabac + core), but UEFI leaves the hardware
         * in an undefined "never reset" state — without a clean
         * assert→deassert pulse the codec sometimes accepts dec_e=1 yet
         * never issues AXI traffic (verified via post-kick IOMMU snapshot:
         * STATUS=idle, INT_RAWSTAT=0, no fault).  Mirror what BSP's
         * platform-init reset_control_bulk_assert+deassert dance does.
         *
         * v7 ifc split: AssertRvdec0CoreReset / DeassertRvdec0CoreReset
         * pulse the CON40 narrow reset bundle (bits 6..9); the matching
         * Rvdec1 pair pulses CON41 bits 6..8.  Dispatch on UID so each
         * codec's PrepareHardware only resets its own bits — matches
         * BSP per-device platform-init reset_control_bulk dance. */
        if (ctx->Uid == 0 &&
            ctx->Ifcs.Ccu.AssertRvdec0CoreReset && ctx->Ifcs.Ccu.DeassertRvdec0CoreReset) {
            ctx->Ifcs.Ccu.AssertRvdec0CoreReset(ctx->Ifcs.Ccu.Header.Context);
            ctx->Ifcs.Ccu.DeassertRvdec0CoreReset(ctx->Ifcs.Ccu.Header.Context);
        } else if (ctx->Uid == 1 &&
                   ctx->Ifcs.Ccu.AssertRvdec1CoreReset && ctx->Ifcs.Ccu.DeassertRvdec1CoreReset) {
            ctx->Ifcs.Ccu.AssertRvdec1CoreReset(ctx->Ifcs.Ccu.Header.Context);
            ctx->Ifcs.Ccu.DeassertRvdec1CoreReset(ctx->Ifcs.Ccu.Header.Context);
        }
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

    /* Stop the poller before anything that takes MMIO out from under it.
     * The poller's tight `READ_REGISTER_ULONG(mmio + IntStatusOffset)`
     * loop runs at PASSIVE_LEVEL with no synchronization against PnP
     * teardown.  If a kick is in flight when PnP-stop fires, the poller
     * is still inside that read when DropCluster powers the codec PD
     * off (→ SError) or MmUnmapIoSpace tears down the kernel VA
     * (→ PAGE_FAULT_IN_NONPAGED_AREA).  Quiesce it first; the second
     * call from EvtDeviceContextCleanup is a no-op once PollerThread
     * is NULL. */
    RkMppJobQueueTeardown(&ctx->JobQueue);

    /* Mirror PrepareHardware in reverse: drop the cluster raise we took
     * there, then release the ifcs and unmap MMIO.
     *
     * Quiesce the IOMMU BEFORE the cluster Drop so that:
     *   - Any in-flight fault IRQ is masked at the device level
     *     (RK_MMU_INT_MASK=0 in RkIommuDisableHw); KMDF's WDF interrupt
     *     vector stays untouched (cross-device WdfInterruptDisable
     *     BSODs on shutdown per commit 9a71579).
     *   - Paging is disabled and DTE_ADDR zeroed while clocks are still
     *     on (PD raised); after Drop the IOMMU MMIO sits in a gated PD
     *     and any access would SError → WHEA UE.
     *
     * Fixes WHEA_UNCORRECTABLE_ERROR observed when reinstalling rkvdec
     * without reinstalling rkiommu_vdec or rkmpp_ccu.
     *
     * Best-effort: if Ifcs.Iommu.Disable is NULL (rkiommu_vdec not yet
     * rebuilt against v6 ifc) or returns failure, log and continue —
     * teardown must always complete. */
    if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Disable) {
        NTSTATUS s = ctx->Ifcs.Iommu.Disable(ctx->Ifcs.Iommu.Header.Context);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkvdec: Iommu.Disable in ReleaseHardware failed 0x%08x — "
                       "continuing teardown\n", s);
        }
    }
    if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.DropCluster) {
        PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
        ctx->Ifcs.Ccu.DropCluster(cookie);
    }
    RkMppCloseIfcs(&ctx->Ifcs);

    for (UINT32 i = 0; i < ctx->MmioCount; i++) {
        if (ctx->Mmios[i].Base) {
            MmUnmapIoSpace(ctx->Mmios[i].Base, ctx->Mmios[i].Length);
            ctx->Mmios[i].Base = NULL;
        }
    }
    ctx->MmioBase   = NULL;
    ctx->MmioLength = 0;
    ctx->MmioCount  = 0;
    return STATUS_SUCCESS;
}

/* Per-device context cleanup — fires when the WDFDEVICE is destroyed
 * (driver unload or device removal).  Stops the polling-completion
 * thread and releases its reference. */
VOID
RkMppEvtDeviceContextCleanup(_In_ WDFOBJECT Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet((WDFDEVICE)Device);
    if (ctx) {
        RkMppJobQueueTeardown(&ctx->JobQueue);
    }
}

/* Reads ACPI _HID and _UID from the device-instance properties.
 * Hardware-ID list may contain multiple IDs (the matching ACPI HID plus
 * `PRP0001` and any compatible IDs). Walk the multi-sz list and pick the
 * first one starting with "ACPI\\RKCP35".
 */
static NTSTATUS RkMppReadAcpiId(_In_ WDFDEVICE Device,
                                _Out_ PUINT32 Hid, _Out_ PUINT32 Uid)
{
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(Device);
    WCHAR buf[1024] = {0};
    ULONG size = 0;

    NTSTATUS status = IoGetDeviceProperty(pdo, DevicePropertyHardwareID,
                                          sizeof(buf), buf, &size);
    if (!NT_SUCCESS(status)) return status;

    /* Walk the multi-sz looking for an entry starting "ACPI\\RKCP35". */
    PCWSTR cursor = buf;
    while (*cursor) {
        size_t len = wcslen(cursor);
        if (len >= 13) {
            if (cursor[0] == L'A' && cursor[1] == L'C' && cursor[2] == L'P' &&
                cursor[3] == L'I' && cursor[4] == L'\\' &&
                cursor[5] == L'R' && cursor[6] == L'K' && cursor[7] == L'C' &&
                cursor[8] == L'P' && cursor[9] == L'3' && cursor[10] == L'5')
            {
                /* Last 4 hex chars from the ID. */
                UINT32 hid = 0;
                for (int i = 9; i < 13; i++) {
                    WCHAR c = cursor[i];
                    UINT32 d;
                    if (c >= L'0' && c <= L'9') d = c - L'0';
                    else if (c >= L'a' && c <= L'f') d = 10 + (c - L'a');
                    else if (c >= L'A' && c <= L'F') d = 10 + (c - L'A');
                    else { hid = 0; break; }
                    hid = (hid << 4) | d;
                }
                if (hid) { *Hid = hid; goto got_hid; }
            }
        }
        cursor += len + 1;
    }
    return STATUS_INVALID_DEVICE_REQUEST;

got_hid:;
    /* Read _UID via IRP_MN_QUERY_ID (BusQueryInstanceID).  For ACPI bus the
     * instance ID is the _UID formatted as a decimal string (e.g. "0", "9").
     * acpi.sys does NOT populate DEVICE_CAPABILITIES.UINumber from _UID
     * reliably, so DevicePropertyUINumber returns 0 for every device. */
    *Uid = RkSharedQueryAcpiUid(pdo);
    return STATUS_SUCCESS;
}

/* EvtFileCreate is WDF_NO_EVENT_CALLBACK; the file context memory is
 * zero-initialised by WDF.  We initialise it lazily on first IOCTL, but
 * it is cleaner to do it in EvtFileClose's counterpart.  Actually the
 * standard pattern: initialise in EvtFileCreate.  Since we used
 * WDF_NO_EVENT_CALLBACK for create, WDF still allocates the context block
 * (zeroed).  RkMppBufAlloc handles a zero-initialised context because
 * RkMppBufFileCtxInit is called from the first alloc path via ioctl.c.
 * EvtFileCleanup frees all outstanding buffers; EvtFileClose is a no-op
 * (WDF requires it when EvtFileCleanup is set). */

VOID
RkMppEvtFileCleanup(_In_ WDFFILEOBJECT FileObject)
{
    /* Initialise context if it was never used (no allocs occurred). */
    PRKMPP_FILE_CTX ctx = RkMppFileGet(FileObject);
    if (!ctx->Device) {
        /* Never initialised — nothing to do. */
        return;
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkmpp: EvtFileCleanup fileobject=%p — draining jobs\n",
               FileObject);

    /* Drain this file-object's outstanding jobs BEFORE freeing its
     * buffer pool.  If a kicked job's bitstream / output / colmv
     * buffers vanish from under the codec mid-DMA, the IOMMU tears
     * down the mappings and the codec's next AXI transaction wedges
     * the cluster (and historically the whole driver).  500ms cap is
     * plenty for a single decode-kick to finish.
     *
     * Only force a core reset on the next kick when we ACTUALLY hit
     * the timeout — if the in-flight finished cleanly, the codec is
     * idle and another session's clean kick shouldn't pay the reset
     * cost.  Forcing reset unconditionally regressed the common case
     * (process exits cleanly post-playback → next session's first
     * kick burned a reset for nothing, and the inter-session timing
     * disrupted ongoing pipelines on other cores). */
    BOOLEAN timedOut = FALSE;
    (void)RkMppJobsDrainOwner(ctx->Device, FileObject, 500, &timedOut);

    LONG sessionErrors = InterlockedExchange(&ctx->ErrorCount, 0);
    PRKMPP_DEVICE devCtx = RkMppDeviceGet(ctx->Device);

    if (timedOut) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkmpp: FileCleanup in-flight wait timed out — "
                   "session-end PD power-cycle below; in-flight work lost\n");
    } else if (sessionErrors > 0) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkmpp: drain clean fileobject=%p but %d error-flagged "
                   "jobs — session-end PD power-cycle below\n",
                   FileObject, sessionErrors);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkmpp: drain done fileobject=%p clean — "
                   "session-end PD power-cycle below\n", FileObject);
    }

    RkMppBufFreeAll(FileObject);

    /* Decide between two session-close hygiene paths:
     *
     *   A. NO other File active on this engine — do the full PD power-
     *      cycle (FullCoreReset0/1 = PD_RKVDEC{0,1} PowerOff → SOFTRST
     *      toggle → PowerOn).  Clears codec FSM + IOMMU walk-cache state
     *      so the next session never inherits anything from this one.
     *      This is the empirical recipe that removes whole classes of
     *      "wedge persists across mpv invocations" failures.
     *
     *   B. ANOTHER File still has Pending / InFlight on this engine —
     *      a second concurrent decode is using us right now.  The wide
     *      PD-cycle would yank the codec and IOMMU out from under their
     *      in-flight DMA, corrupting their stream.  Downgrade to:
     *        - IOMMU Reattach (Disable+Enable) to drop walk-cache state
     *          tied to our just-unmapped iovas; preserves other Files'
     *          domain mappings.
     *        - Flag NeedsCoreReset so the OTHER File's next kick eats
     *          a cheap narrow CON40/CON41 core-reset bundle — softer
     *          cleanse of codec FSM than the full PD-cycle, but it
     *          still scrubs any FSM bits our last kick left behind.
     *      When the last remaining File on this engine closes, path A
     *      runs and performs the full hygiene cycle then.
     *
     * The `timedOut` recovery path stays on A regardless: if our drain
     * failed, the codec is wedged and the other File's next kick was
     * going to fail anyway — better to recover the engine now.
     *
     * Note on race: the otherActive check is not atomic with the heavy
     * path below — a peer can submit between observation and PD-cycle.
     * That's acceptable: if the race-stomp causes the peer's first
     * post-reset kick to error, JobStart's NeedsFullReset recovery
     * path issues a fresh hygiene cycle on the kick after.  The
     * alternative (a Quiescing latch checked in SubmitDense) added
     * complexity without measurable benefit. */
    BOOLEAN otherActive = RkMppJobQueueHasOtherOwner(ctx->Device, FileObject);

    if (devCtx) {
        PRKIOMMU_INTERFACE   iommu = devCtx->Ifcs.IommuOpen ? &devCtx->Ifcs.Iommu : NULL;
        PRKMPP_CCU_INTERFACE ccu   = devCtx->Ifcs.CcuOpen   ? &devCtx->Ifcs.Ccu   : NULL;

        if (otherActive && !timedOut) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                       "rkmpp: FileCleanup with concurrent File on UID=%u — "
                       "skipping IOMMU/CCU touches (would disrupt peer DMA); "
                       "flagging NeedsCoreReset for peer's next kick\n",
                       devCtx->Uid);
            /* Do NOT call Iommu.Reattach here.  It Disable+Enables the
             * IOMMU, opening a ~10µs window with paging OFF — the peer's
             * in-flight AXI transactions during that window go
             * untranslated and fault.  BufFreeAll above already issued
             * per-iova ZAP_CACHE via UnmapMdl, which is the only
             * walk-cache state that needed flushing for our cleanup.
             *
             * The narrow per-codec reset bundle on the peer's next kick
             * scrubs any codec FSM state our last kick left behind, at
             * kick-start timing (codec idle) so it's peer-safe. */
            RkMppSetNeedsCoreReset(ctx->Device);
        } else {
            if (timedOut && otherActive) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkmpp: FileCleanup drain timed out on UID=%u with "
                           "concurrent File active — PD-cycle will disrupt it\n",
                           devCtx->Uid);
            }
            if (iommu && iommu->MaskIrq) {
                iommu->MaskIrq(iommu->Header.Context);
            }
            if (iommu && iommu->Disable) {
                (void)iommu->Disable(iommu->Header.Context);
            }
            if (ccu) {
                if (devCtx->Uid == 0 && ccu->FullCoreReset0) {
                    ccu->FullCoreReset0(ccu->Header.Context);
                } else if (devCtx->Uid == 1 && ccu->FullCoreReset1) {
                    ccu->FullCoreReset1(ccu->Header.Context);
                }
            }
            if (iommu && iommu->Enable) {
                NTSTATUS rs = iommu->Enable(iommu->Header.Context);
                if (!NT_SUCCESS(rs)) {
                    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                               "rkmpp: post-FileCleanup IOMMU Enable failed 0x%08x — "
                               "next session will lazy-Enable on first MapMdl\n", rs);
                }
            }
            if (iommu && iommu->UnmaskIrq) {
                iommu->UnmaskIrq(iommu->Header.Context);
            }

            /* Power-cycle subsumes both narrow CoreReset and wide
             * FullReset that JobStart would otherwise apply to the
             * next session's first kick — clear the flags. */
            (void)RkMppQueryAndClearNeedsCoreReset(ctx->Device);
            (void)RkMppQueryAndClearNeedsFullReset(ctx->Device);
        }
    }

    /* LastJobFailed is per-device; clear regardless of path so it doesn't
     * leak across to the next kick on either File. */
    (void)RkMppExchangeLastJobFailed(ctx->Device, 0);
}

VOID
RkMppEvtFileClose(_In_ WDFFILEOBJECT FileObject)
{
    UNREFERENCED_PARAMETER(FileObject);
    /* Nothing to do; cleanup was done in EvtFileCleanup. */
}

/* ---------------------------------------------------------------------------
 * RkMppOnIommuFault — RKIOMMU_FAULT_CALLBACK invoked by rkiommu.sys DPC
 * (DISPATCH_LEVEL) when the IOMMU raises a page-fault interrupt.
 *
 * ClientCookie is the PDEVICE_OBJECT passed to RegisterFaultHandler; we
 * recover the WDFDEVICE via WdfWdmDeviceGetWdfDeviceHandle, then the
 * per-device context.  Atomic stores ensure safe cross-level visibility.
 * --------------------------------------------------------------------------- */
static VOID
RkMppOnIommuFault(_In_ PVOID   ClientCookie,
                  _In_ ULONG64 FaultIova,
                  _In_ ULONG   StatusReg)
{
    PDEVICE_OBJECT wdmDev = (PDEVICE_OBJECT)ClientCookie;
    WDFDEVICE wdfDev = WdfWdmDeviceGetWdfDeviceHandle(wdmDev);
    if (!wdfDev) return;
    PRKMPP_DEVICE ctx = RkMppDeviceGet(wdfDev);
    if (!ctx) return;

    InterlockedExchange64((LONG64*)&ctx->FaultIova, (LONG64)FaultIova);
    InterlockedExchange((LONG*)&ctx->FaultStatusReg, (LONG)StatusReg);
    InterlockedExchange((LONG*)&ctx->FaultTriggered, 1);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp: IOMMU fault iova=0x%llx status=0x%lx\n",
               (unsigned long long)FaultIova, (unsigned long)StatusReg);
}

/* Accessor used by bufpool.c to reach the rkiommu interface without
 * exposing the full RKMPP_DEVICE structure outside device.c. */
PRKIOMMU_INTERFACE
RkMppGetIommuIfc(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return &ctx->Ifcs.Iommu;
}

/* Accessor used by job.c to reach the CCU interface (raise/drop cluster). */
PRKMPP_CCU_INTERFACE
RkMppGetCcuIfc(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return &ctx->Ifcs.Ccu;
}

/* Accessor used by job.c to reach the MMIO base for ISR register reads. */
PVOID
RkMppGetMmioBase(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return ctx->MmioBase;
}

/* Accessor used by job.c for offset bounds checking on register writes.
 * MmioLength is SIZE_T but every codec MMIO range we map is < 4 GiB, so
 * a ULONG return is fine (bounds-clamped on the off chance). */
ULONG
RkMppGetMmioLength(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    if (!ctx) return 0;
    return (ctx->MmioLength > MAXULONG) ? MAXULONG : (ULONG)ctx->MmioLength;
}

/* Accessor used by job.c to reach the job queue. */
PRKMPP_JOB_QUEUE
RkMppGetJobQueue(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return &ctx->JobQueue;
}

/* Codec-personality accessor — selects the kick path in job.c. */
RKMPP_CODEC_PERSONALITY
RkMppGetPersonality(_In_ WDFDEVICE Device)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    return ctx->Personality;
}

/* Auxiliary MMIO window accessor.  Index 0 is the primary (same as
 * MmioBase); 1..N-1 hit additional windows declared in _CRS.  Returns
 * NULL if the slot is unused.  AV1 uses indices 0=VCD, 1=CACHE, 2=AFBC. */
PVOID
RkMppGetMmioWindow(_In_ WDFDEVICE Device,
                   _In_ UINT32 Index,
                   _Out_opt_ PULONG Length)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    if (Index >= ctx->MmioCount) {
        if (Length) *Length = 0;
        return NULL;
    }
    if (Length) {
        SIZE_T l = ctx->Mmios[Index].Length;
        *Length = (l > MAXULONG) ? MAXULONG : (ULONG)l;
    }
    return ctx->Mmios[Index].Base;
}

void RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    Out->Hid             = ctx->Hid;
    Out->Uid             = ctx->Uid;
    Out->RevisionWord    = ctx->RevisionWord;
    Out->SupportedCodecs = ctx->SupportedCodecs;
}

/* Accessor used by ioctl.c to read the atomically-maintained IOMMU fault
 * state written by RkMppOnIommuFault.  Uses InterlockedOr(x,0) as a
 * barrier-safe atomic load on both x86 and ARM64. */
void RkMppGetFaultState(_In_ WDFDEVICE Device, _Out_ RKMPP_FAULT_STATE *Out)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    Out->Triggered  = InterlockedOr((LONG*)&ctx->FaultTriggered, 0);
    Out->StatusReg  = InterlockedOr((LONG*)&ctx->FaultStatusReg, 0);
    Out->FaultIova  = InterlockedOr64((LONG64*)&ctx->FaultIova, 0);
}
