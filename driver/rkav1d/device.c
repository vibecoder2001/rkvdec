/* driver/rkav1d/device.c — per-instance device for rkav1d.sys.
 *
 * Phase 1 responsibilities:
 *   - parse HID + _UID from the ACPI hardware-ID list
 *   - look up the profile
 *   - map MMIO _CRS resources (3 windows: VCD, CACHE, AFBC)
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

/* AV1D (RKCP3560) declares THREE non-contiguous memory regions
 *   VCD   : 0xFDC70000 / 0x800
 *   CACHE : 0xFDC80000 / 0x400
 *   AFBC  : 0xFDC90000 / 0x400
 * with unallocated phys gaps in between — must be mapped independently. */
#define RKMPP_MAX_MMIO_WINDOWS 3

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
    /* Primary MMIO base — first window (VCD).  Job/IOCTL paths still
     * address through MmioBase as the codec's main register window.
     * Additional windows live in Mmios[1..]. */
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

EVT_WDF_DEVICE_PREPARE_HARDWARE     RkMppEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE     RkMppEvtReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY             RkMppEvtD0Entry;
EVT_WDF_DEVICE_D0_EXIT              RkMppEvtD0Exit;
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
    pnp.EvtDeviceD0Entry         = RkMppEvtD0Entry;
    pnp.EvtDeviceD0Exit          = RkMppEvtD0Exit;
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
     * The AV1 ACPI declares 3 interrupt resources in order: VCD (140),
     * CACHE (139), AFBC (138).  Only VCD signals decode completion (see
     * BSP mpp_av1dec.c:623 av1dec_vcd_irq); CACHE/AFBC are declared but
     * never `request_irq`-ed by BSP.  Default WdfInterruptCreate
     * auto-binds against the first available CmResourceTypeInterrupt
     * descriptor in PnP-assignment order, which matches our ACPI's
     * declaration order — so the first call claims VCD.
     *
     * Calling from EvtDevicePrepareHardware with explicit
     * InterruptRaw+InterruptTranslated fails 0xC020020F on ARM64 GIC
     * SPI lines (see RkMppDeviceCreate comment in driver/rkvdec/device.c
     * for the WDF validator-path explanation).
     *
     * Fatal on failure: completion is interrupt-driven exclusively.
     * The previous fall-back poller has been gated off. */
    {
        WDF_INTERRUPT_CONFIG intCfg;
        WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkMppEvtIsr, RkMppEvtDpc);
        NTSTATUS intStatus = WdfInterruptCreate(
            device, &intCfg, WDF_NO_OBJECT_ATTRIBUTES,
            &devCtx->JobQueue.Interrupt);
        if (!NT_SUCCESS(intStatus)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkav1d: WdfInterruptCreate failed 0x%08x — "
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
     * (VCD/interrupt 140 — the only one BSP uses for completion) was
     * already registered at EvtDeviceAdd time via WdfInterruptCreate
     * (see RkMppDeviceCreate); WDF auto-binds it against the first
     * CmResourceTypeInterrupt entry in this resource list during PnP
     * assignment, so we don't touch interrupt descriptors here. */

    /* AV1D (RKCP3560) declares THREE non-contiguous regions
     *   VCD   : 0xFDC70000 / 0x800
     *   CACHE : 0xFDC80000 / 0x400
     *   AFBC  : 0xFDC90000 / 0x400
     * with unallocated phys gaps in between — must be mapped independently.
     * Map each CmResourceTypeMemory descriptor into its own slot of Mmios[].
     * The primary MmioBase always points at Mmios[0].Base (VCD window). */
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            PHYSICAL_ADDRESS start = d->u.Memory.Start;
            ULONG len = d->u.Memory.Length;

            if (ctx->MmioCount >= RKMPP_MAX_MMIO_WINDOWS) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkav1d: AV1 declares >%u memory windows, "
                           "ignoring extras\n", RKMPP_MAX_MMIO_WINDOWS);
                continue;
            }
            PVOID v = MmMapIoSpaceEx(start, len,
                                     PAGE_READWRITE | PAGE_NOCACHE);
            if (!v) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                           "rkav1d: window[%u] map failed phys=0x%llx "
                           "len=0x%x\n",
                           ctx->MmioCount,
                           (ULONGLONG)start.QuadPart, len);
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            ctx->Mmios[ctx->MmioCount].Base   = v;
            ctx->Mmios[ctx->MmioCount].Length = len;
            ctx->Mmios[ctx->MmioCount].Phys   = start;
            ctx->MmioCount++;
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                       "rkav1d: window[%u] phys=0x%llx len=0x%x va=%p\n",
                       ctx->MmioCount - 1,
                       (ULONGLONG)start.QuadPart, len, v);
        }
        /* CmResourceTypeInterrupt entries are consumed by WDF's auto-bind
         * for the WDFINTERRUPT registered in RkMppDeviceCreate — we don't
         * extract them here.  The first interrupt resource (VCD/140) is
         * what WDF claims; CACHE/AFBC are declared but unused. */
    }

    if (ctx->MmioCount == 0) return STATUS_INSUFFICIENT_RESOURCES;
    ctx->MmioBase   = ctx->Mmios[0].Base;
    ctx->MmioLength = ctx->Mmios[0].Length;

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkav1d: HID=RKCP%04x UID=%u MmioBase=phys 0x%llx len 0x%zx "
               "windows=%u\n",
               ctx->Hid, ctx->Uid,
               (ULONGLONG)ctx->Mmios[0].Phys.QuadPart,
               ctx->MmioLength, ctx->MmioCount);

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
                   "rkav1d: ccu/iommu ifcs unavailable (0x%08x) — install "
                   "rkmpp_ccu.sys and rkiommu.sys before rkav1d.sys\n", status);
        return STATUS_DEVICE_NOT_READY;
    }

    if (ctx->Ifcs.Iommu.Header.Version != RKIOMMU_IFC_VERSION ||
        ctx->Ifcs.Ccu.Header.Version   != RKMPP_CCU_IFC_VERSION) {
        RkMppCloseIfcs(&ctx->Ifcs);
        return STATUS_REVISION_MISMATCH;
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkav1d: ifcs opened (iommu v%u, ccu v%u)\n",
               ctx->Ifcs.Iommu.Header.Version, ctx->Ifcs.Ccu.Header.Version);

    /* Raise the AV1 cluster.  Refcounted; matching DropAv1Cluster in
     * ReleaseHardware. */
    PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
    if (!ctx->Ifcs.Ccu.RaiseAv1Cluster) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkav1d: CCU has no RaiseAv1Cluster — update rkmpp_ccu.sys\n");
        RkMppCloseIfcs(&ctx->Ifcs);
        return STATUS_NOT_SUPPORTED;
    }
    status = ctx->Ifcs.Ccu.RaiseAv1Cluster(cookie);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkav1d: RaiseAv1Cluster failed 0x%08x\n", status);
        RkMppCloseIfcs(&ctx->Ifcs);
        return status;
    }

    /* Cluster bring-up confirmed working on real hardware as of Phase 3b
     * Task 2 — PD_AV1 powers on cleanly, codec MMIO reads
     * REG0_VERSION=0x80019000.  Now the rest of PrepareHardware is safe. */
    const RKMPP_PROFILE *p = RkMppFindProfile(ctx->Hid, ctx->Uid);
    if (p) {
        ctx->SupportedCodecs = p->SupportedCodecs;
        ctx->Personality     = p->Personality;
    }

    /* Only touch codec MMIO + IOMMU wiring for devices we actually plan
     * to drive in this phase.  RaiseAv1Cluster powers the AV1 PD only.
     * The non-decoder probes still get a profile row (SupportedCodecs=0)
     * so subsequent IOCTLs can report "unsupported"; they just don't get
     * any bring-up activity. */
    if (p && p->SupportedCodecs != 0) {
        /* Wide hang-recovery reset on every PrepareHardware so partial
         * driver reinstall self-heals AV1 codec FSM + IOMMU state
         * without needing a full driver-chain reload.  v8 FullAv1Reset
         * cycles the CON68 CRU bundle (PMU bus-idle on PD_AV1, assert,
         * udelay(5), deassert, release) without touching the shared
         * SOFTRST_CON44 NIU bits — those are shared with other
         * VDPU-child codecs and a single-codec hang-recovery must not
         * disrupt peer codecs mid-decode.
         *
         * FullAv1Reset zeroes IOMMU DTE_ADDR as a side effect, so we
         * MUST follow with Reattach to reprogram DTE_ADDR before any
         * further AV1 codec activity. */
        if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.FullAv1Reset) {
            ctx->Ifcs.Ccu.FullAv1Reset(ctx->Ifcs.Ccu.Header.Context);
        }
        if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Reattach) {
            NTSTATUS rs = ctx->Ifcs.Iommu.Reattach(ctx->Ifcs.Iommu.Header.Context);
            if (!NT_SUCCESS(rs)) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkav1d: post-Raise Reattach failed 0x%08x\n", rs);
            }
        }

        /* Capture REVISION as ground truth that codec MMIO is alive. */
        ctx->RevisionWord = READ_REGISTER_ULONG(
            (volatile ULONG*)((PUCHAR)ctx->MmioBase + p->RevisionRegOffset));

        /* Wire the IOMMU fault callback.  rkiommu's DPC invokes this when
         * a translation fault posts; we just stash the fault state for the
         * IOCTL surface to surface to userspace.  Safe to register before
         * RkIommuEnable runs — the IRQ stays masked until then. */
        if (ctx->Ifcs.Iommu.RegisterFaultHandler) {
            ctx->Ifcs.Iommu.RegisterFaultHandler(
                ctx->Ifcs.Iommu.Header.Context,         /* iommu instance */
                WdfDeviceWdmGetDeviceObject(Device),    /* our device, for callback */
                RkMppOnIommuFault);
        }

        /* NOTE: No per-instance core-reset pulse for RKCP3560 — the
         * AV1 core reset lines are not wired in rkmpp_ccu.sys for this
         * SoC variant.  The NeedsCoreReset flag still drives the
         * per-kick prev-mask invalidation path in job.c. */
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkav1d: HID=RKCP%04x UID=%u rev=0x%08x codecs=0x%08x\n",
               ctx->Hid, ctx->Uid, ctx->RevisionWord, ctx->SupportedCodecs);
    return STATUS_SUCCESS;
}

NTSTATUS
RkMppEvtReleaseHardware(_In_ WDFDEVICE Device, _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);

    /* Stop the poller before DropAv1Cluster / MmUnmapIoSpace — otherwise
     * an in-flight kick's READ_REGISTER_ULONG on IntStatus races the PD
     * drop / VA unmap and bugchecks.  Idempotent vs the cleanup-path
     * call from EvtDeviceContextCleanup. */
    RkMppJobQueueTeardown(&ctx->JobQueue);

    /* Mirror PrepareHardware in reverse: drop the AV1 cluster raise we took
     * there, then release the ifcs and unmap MMIO. */
    if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.DropAv1Cluster) {
        PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);
        ctx->Ifcs.Ccu.DropAv1Cluster(cookie);
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

/* ---------------------------------------------------------------------------
 * EvtDeviceD0Entry — fires after PrepareHardware on first power-up, and on
 * every D3→D0 transition thereafter.  Re-arms the AV1 cluster from a gated
 * state: ungate leaf clocks, re-enable the IOMMU's AHB paging + IRQ, then
 * resume the job queue so any work that was Pending across D3 restarts.
 *
 * Option A (BSP runtime-PM parity): PD_AV1 stays raised across D3 — only
 * leaf clocks gate.  So the first D0Entry after PrepareHardware is mostly
 * a no-op: Raise already left clocks ungated, the IRQ is unmasked by
 * default, and the Iommu domain may not yet exist (sibling PnP ordering —
 * tolerated below).
 *
 * Ordering invariant: Ungate clocks BEFORE touching IOMMU MMIO.
 * --------------------------------------------------------------------------- */
NTSTATUS
RkMppEvtD0Entry(_In_ WDFDEVICE Device, _In_ WDF_POWER_DEVICE_STATE PreviousState)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);

    /* Step 1: ungate AV1 leaf clocks.  Idempotent — RaiseAv1Cluster already
     * ungated as part of bring-up, so the first call after PrepareHardware
     * is a hi-word-mask no-op. */
    if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.UngateAv1LeafClocks) {
        NTSTATUS s = ctx->Ifcs.Ccu.UngateAv1LeafClocks(cookie);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkav1d: D0Entry UngateAv1LeafClocks failed 0x%08x\n", s);
            /* Continue — IOMMU/codec MMIO is still gated, but failing here
             * would leave the device in an undefined state.  Surface as
             * error log + best-effort continue. */
        }
    }

    /* Step 2: re-enable IOMMU.  Sibling-PnP-ordering subtlety — if
     * rkiommu_av1d's PrepareHardware hasn't run yet (siblings have no
     * ordering guarantee), the Domain is NULL and Enable returns
     * STATUS_DEVICE_NOT_READY.  In that case the lazy-enable path in
     * RkIommuMapMdl will arm the IOMMU on the first kick after both
     * PrepareHardwares complete — log only, do NOT fail D0Entry. */
    if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Enable) {
        NTSTATUS s = ctx->Ifcs.Iommu.Enable(ctx->Ifcs.Iommu.Header.Context);
        if (!NT_SUCCESS(s)) {
            if (s == STATUS_DEVICE_NOT_READY) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                           "rkav1d: D0Entry Iommu.Enable deferred — sibling "
                           "rkiommu_av1d not yet ready (lazy enable on first "
                           "MapMdl)\n");
            } else {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                           "rkav1d: D0Entry Iommu.Enable failed 0x%08x\n", s);
                return s;
            }
        }
    }

    /* Step 3: IOMMU IRQ is power-managed by KMDF — the framework
     * auto-disconnects across D0->D3 and auto-reconnects on D3->D0.
     * Calling WdfInterruptEnable here would double-enable (cross-device,
     * racing KMDF's auto-reconnect) and BSODs on shutdown.  The
     * Iommu IRQ-unmask ifc method is retained in v6 of rkiommu_ifc for
     * non-D-state callers (hang recovery, etc.).
     * See R7 in docs/superpowers/plans/2026-05-10-av1-d0entry-d0exit-review.md. */

    /* Step 4: resume the job queue — clears the suppression flag set by
     * Quiesce and restarts any Pending chain that didn't get drained. */
    RkMppJobQueueResume(Device, &ctx->JobQueue);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkav1d: D0Entry done (prev=%u)\n", PreviousState);
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * EvtDeviceD0Exit — fires on D0→D3 transition (and before ReleaseHardware
 * on final power-down).  Tears down the AV1 cluster into a safe gated
 * state, in the strict order:
 *
 *   Quiesce (drain in-flight kick) → IOMMU Disable → Gate clocks
 *
 * Quiesce MUST be first: an in-flight kick poll mid-D0Exit will hang if
 * clocks gate underneath it.  Disable MUST happen while clocks are still
 * on: it writes AHB_CONTROL=0 which is an MMIO op.  This is Design X.
 *
 * IOMMU IRQ mask/unmask is intentionally absent — KMDF auto-disconnects
 * power-managed interrupts across D-state transitions.  See R7 in
 * docs/superpowers/plans/2026-05-10-av1-d0entry-d0exit-review.md.
 *
 * Best-effort: every step is NULL-checked + logs but never short-circuits.
 * The D-state transition MUST complete (return SUCCESS) even if individual
 * steps fail — failing D0Exit would leave the device stuck in D0 with the
 * power-manager unable to recover.
 * --------------------------------------------------------------------------- */
NTSTATUS
RkMppEvtD0Exit(_In_ WDFDEVICE Device, _In_ WDF_POWER_DEVICE_STATE TargetState)
{
    PRKMPP_DEVICE ctx = RkMppDeviceGet(Device);
    PVOID cookie = WdfDeviceWdmGetDeviceObject(Device);

    /* Step 1: drain in-flight job + suppress new starts.  Even on timeout,
     * we proceed — the alternative is leaving clocks ungated in D3, which
     * defeats the whole D-state effort. */
    NTSTATUS qs = RkMppJobQueueQuiesce(&ctx->JobQueue, 500);
    if (qs == STATUS_TIMEOUT) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkav1d: D0Exit Quiesce timed out — proceeding with "
                   "gate sequence (in-flight kick may be lost)\n");
    } else if (!NT_SUCCESS(qs)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkav1d: D0Exit Quiesce status 0x%08x — continuing\n", qs);
    }

    /* Step 2: IOMMU IRQ is power-managed by KMDF — the framework
     * auto-disconnects across D0->D3 and auto-reconnects on D3->D0.
     * Calling WdfInterruptDisable here would double-disable (cross-device,
     * racing KMDF's auto-disconnect) and BSODs on shutdown.  The
     * Iommu IRQ-mask ifc method is retained in v6 of rkiommu_ifc for
     * non-D-state callers (hang recovery, etc.).
     * See R7 in docs/superpowers/plans/2026-05-10-av1-d0entry-d0exit-review.md. */

    /* Step 3: disable IOMMU (writes AHB_CONTROL=0).  Clocks are still on. */
    if (ctx->Ifcs.IommuOpen && ctx->Ifcs.Iommu.Disable) {
        NTSTATUS s = ctx->Ifcs.Iommu.Disable(ctx->Ifcs.Iommu.Header.Context);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                       "rkav1d: D0Exit Iommu.Disable failed 0x%08x\n", s);
        }
    }

    /* Step 4: gate AV1 leaf clocks.  AFTER Disable returns — no more MMIO
     * after this point until the next D0Entry. */
    if (ctx->Ifcs.CcuOpen && ctx->Ifcs.Ccu.GateAv1LeafClocks) {
        NTSTATUS s = ctx->Ifcs.Ccu.GateAv1LeafClocks(cookie);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                       "rkav1d: D0Exit GateAv1LeafClocks failed 0x%08x\n", s);
        }
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkav1d: D0Exit done (target=%u)\n", TargetState);
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
               "rkav1d: EvtFileCleanup fileobject=%p — draining jobs\n",
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

    /* Two-tier session-end recovery (mirrors BSP's
     * `rkvdec2_vdpu382_reset` → `rkvdec2_reset` escalation):
     *
     *   - Drain timeout → HARD tier: wide CRU bundle reset
     *     (PMU idle + CON40 bits 2..9 + CON44 bits 4..6 + idle release).
     *     Required because a stuck in-flight kick has the codec FSM in
     *     a state nothing softer can recover.  Cost: also resets the
     *     AXI/AHB/NIU bus blocks the IOMMU sits on, which is why the
     *     post-reset Reattach is critical.
     *
     *   - Clean drain but session had error-flagged jobs → SOFT tier:
     *     IOMMU force-reset only (RK_MMU_CMD_FORCE_RESET).  Resets the
     *     MMU's internal walk caches / prefetcher / fault state without
     *     touching CRU bits.  Safe for cross-codec hand-off — observed
     *     2026-05-03 that the wide reset breaks HEVC's NIU routing
     *     when invoked between H.264 and HEVC sessions.
     *
     *   - Clean drain, no errors → just the standard `Reattach` below. */
    LONG sessionErrors = InterlockedExchange(&ctx->ErrorCount, 0);
    PRKMPP_DEVICE devCtx = RkMppDeviceGet(ctx->Device);

    if (timedOut) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                   "rkav1d: FileCleanup in-flight wait timed out — "
                   "wide CRU reset + WDF restart for full PD cycle\n");
        RkMppSetNeedsCoreReset(ctx->Device);
        if (devCtx && devCtx->Ifcs.CcuOpen &&
            devCtx->Ifcs.Ccu.FullAv1Reset) {
            /* v8: AV1-specific hang-recovery reset (CON68 bundle). */
            devCtx->Ifcs.Ccu.FullAv1Reset(
                devCtx->Ifcs.Ccu.Header.Context);
        }
        /* Wide CRU reset clears CON40 reset bits but leaves PD power on,
         * so codec FSM state that survives a reset (AXI write buffer in
         * flight, internal pipeline registers not in the CRU bundle) can
         * still wedge subsequent decodes.  Empirically the disable+enable
         * cycle on RKCP3560 always recovers — that's a full PD cycle via
         * EvtReleaseHardware → DropAv1Cluster (refcount → 0 → PMU power off)
         * → EvtPrepareHardware → RaiseAv1Cluster (PMU power on).
         * Replicate that programmatically via WdfDeviceSetFailed with
         * AttemptRestart: WDF reports failure to PnP, the framework
         * unloads + reloads the driver, which power-cycles the codec.
         * In-flight work is lost — but the alternative is a system that
         * needs Device Manager intervention to recover. */
        WdfDeviceSetFailed(ctx->Device, WdfDeviceFailedAttemptRestart);
    } else if (sessionErrors > 0) {
        /* Originally invoked the soft-tier `Iommu.ForceReset` here, but
         * issuing RK_MMU_CMD_FORCE_RESET on this SoC wedges the hardware
         * (followup decode IOCTLs hang).  The Linux BSP's
         * rk_iommu_force_reset wraps the same MMU op with additional
         * pipeline quiescing (codec stall + power-domain off, see
         * mpp_dev_reset) that we don't yet replicate; without that
         * sequencing the soft tier is unsafe.  Defer to the existing
         * NeedsCoreReset path: the next session's first kick will run
         * the wide CRU `FullCoreReset` via RkMppJobStart, which is
         * verified safe.  The unconditional `Reattach` below still
         * drops the IOMMU walk cache, which is enough isolation for
         * the common case (errored decode but no driver-side timeout).
         *
         * Re-enable the soft tier when we replicate BSP's stall +
         * power-down + clock-gate sequencing around the FORCE_RESET. */
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkav1d: drain clean fileobject=%p but %d error-flagged "
                   "jobs — flagging next-kick wide reset (soft tier disabled)\n",
                   FileObject, sessionErrors);
        RkMppSetNeedsCoreReset(ctx->Device);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkav1d: drain done fileobject=%p clean\n", FileObject);
    }

    /* Capture peer-active state BEFORE BufFreeAll: a peer can be mid-DMA
     * right now from its own iovas, and we must not touch the IOMMU
     * (Reattach = Disable+Enable opens a ~10µs window with paging OFF —
     * peer's in-flight AXI transactions then go untranslated and fault).
     * BufFreeAll's per-buffer UnmapMdl already issues ZAP_CACHE on this
     * File's iovas, which is the only walk-cache state that needs
     * flushing for our cleanup — the belt-and-suspenders Reattach was
     * redundant for that purpose and unsafe for peers. */
    BOOLEAN otherActive = RkMppJobQueueHasOtherOwner(ctx->Device, FileObject);

    RkMppBufFreeAll(FileObject);

    if (devCtx && devCtx->Ifcs.IommuOpen && devCtx->Ifcs.Iommu.Reattach) {
        if (otherActive) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                       "rkav1d: FileCleanup with concurrent File active — "
                       "skipping Reattach (would disrupt peer DMA); per-iova "
                       "ZAP_CACHE from UnmapMdl already flushed our walk-cache\n");
            /* Flag a narrow per-codec reset for the peer's next kick so
             * any FSM state our last kick left behind is scrubbed. */
            RkMppSetNeedsCoreReset(ctx->Device);
        } else {
            /* No peer — safe to Reattach.  Drops the entire IOMMU walk
             * cache (cross-AU pollution from this session's iovas, the
             * "kill-mid-decode wedge" hardening from
             * memory:rkmpp_kernel_security_todos.md item 9). */
            NTSTATUS rs = devCtx->Ifcs.Iommu.Reattach(
                devCtx->Ifcs.Iommu.Header.Context);
            if (!NT_SUCCESS(rs)) {
                DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                           "rkav1d: post-drain Reattach failed 0x%08x\n", rs);
            }
        }
    }
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
               "rkav1d: IOMMU fault iova=0x%llx status=0x%lx\n",
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

/* Auxiliary MMIO window accessor.  Index 0 is the primary VCD window
 * (same as MmioBase); 1=CACHE, 2=AFBC.  Returns NULL if the slot is
 * unused. */
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
