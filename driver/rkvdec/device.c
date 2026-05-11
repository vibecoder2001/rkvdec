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
EVT_WDF_OBJECT_CONTEXT_CLEANUP      RkMppEvtDeviceContextCleanup;
EVT_WDF_FILE_CLEANUP                RkMppEvtFileCleanup;
EVT_WDF_FILE_CLOSE                  RkMppEvtFileClose;

/* Phase 3a: IOMMU fault callback — registered in PrepareHardware. */
static VOID RkMppOnIommuFault(_In_ PVOID  ClientCookie,
                               _In_ ULONG64 FaultIova,
                               _In_ ULONG   StatusReg);

/* ISR and DPC live in job.c but are declared via EVT_WDF_INTERRUPT_* here
 * so we can pass them to WdfInterruptCreate in PrepareHardware. */
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

    return RkMppQueueInit(device);
}

NTSTATUS
RkMppEvtPrepareHardware(_In_ WDFDEVICE Device,
                        _In_ WDFCMRESLIST ResourcesRaw,
                        _In_ WDFCMRESLIST ResourcesTranslated)
{
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

    /* Step 1: walk resources to capture MMIO windows AND the raw+translated
     * descriptors for the first interrupt.  ARM64 GIC line interrupts require
     * the descriptors to be passed explicitly to WdfInterruptCreate; the
     * default auto-bind path fails with STATUS_WDF_INVALID_INTERRUPT_CONFIG
     * (0xC020020F) — confirmed empirically on first hardware bring-up. */
    PCM_PARTIAL_RESOURCE_DESCRIPTOR irqRaw   = NULL;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR irqTrans = NULL;

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
        } else if (d->Type == CmResourceTypeInterrupt && !irqTrans) {
            irqTrans = d;
            irqRaw   = WdfCmResourceListGetDescriptor(ResourcesRaw, i);
        }
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

    /* Connect the WDF interrupt with explicit raw + translated descriptors.
     * Phase 3a: ISR/DPC are wired but the hardware kick path (Phase 3b) is
     * what actually causes ISRs to fire. */
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
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkmpp: WdfInterruptCreate failed 0x%08x\n",
                       intStatus);
            /* Non-fatal in Phase 3a — Phase 3b real-kick path makes it fatal. */
        }
    }

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
                   "rkmpp: FileCleanup in-flight wait timed out — "
                   "wide CRU reset + WDF restart for full PD cycle\n");
        RkMppSetNeedsCoreReset(ctx->Device);
        if (devCtx && devCtx->Ifcs.CcuOpen) {
            /* v8: per-codec hang-recovery dispatch on UID. */
            if (devCtx->Uid == 0 && devCtx->Ifcs.Ccu.FullCoreReset0) {
                devCtx->Ifcs.Ccu.FullCoreReset0(
                    devCtx->Ifcs.Ccu.Header.Context);
            } else if (devCtx->Uid == 1 && devCtx->Ifcs.Ccu.FullCoreReset1) {
                devCtx->Ifcs.Ccu.FullCoreReset1(
                    devCtx->Ifcs.Ccu.Header.Context);
            }
        }
        /* Wide CRU reset clears CON40 reset bits but leaves PD power on,
         * so codec FSM state that survives a reset (AXI write buffer in
         * flight, internal pipeline registers not in the CRU bundle) can
         * still wedge subsequent decodes.  Empirically the disable+enable
         * cycle on RKCP3550 always recovers — that's a full PD cycle via
         * EvtReleaseHardware → DropCluster (refcount → 0 → PMU power off)
         * → EvtPrepareHardware → RaiseCluster (PMU power on).
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
                   "rkmpp: drain clean fileobject=%p but %d error-flagged "
                   "jobs — flagging next-kick wide reset (soft tier disabled)\n",
                   FileObject, sessionErrors);
        RkMppSetNeedsCoreReset(ctx->Device);
    } else {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
                   "rkmpp: drain done fileobject=%p clean\n",
                   FileObject);
    }

    RkMppBufFreeAll(FileObject);

    /* Reattach the IOMMU domain on every session close, regardless of
     * whether drain timed out.  Linux's `mpp_iommu_dev_deactivate` is
     * called on every IRQ completion; we don't have per-IRQ deactivate
     * machinery, but reattaching at session-close gives an equivalent
     * guarantee that the next session never inherits walk-cache state
     * from a prior session.  This is cheap (one Disable + Enable on the
     * IOMMU, ~10 µs) and is the architectural answer to the
     * kill-mid-decode wedge documented in
     * memory:rkmpp_kernel_security_todos.md item 9.
     *
     * Order matters: must run AFTER BufFreeAll so the buffer iovas have
     * been UnmapMdl'd from the page tables.  Reattach preserves the
     * domain (and any iovas still mapped by other sessions) — only the
     * hardware-side walk caches are flushed. */
    if (devCtx && devCtx->Ifcs.IommuOpen &&
        devCtx->Ifcs.Iommu.Reattach) {
        NTSTATUS rs = devCtx->Ifcs.Iommu.Reattach(
            devCtx->Ifcs.Iommu.Header.Context);
        if (!NT_SUCCESS(rs)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_WARNING_LEVEL,
                       "rkmpp: post-drain Reattach failed 0x%08x\n", rs);
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
