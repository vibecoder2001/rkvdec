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
#include "ifc_client.h"
#include "bufpool.h"
#include "job.h"

typedef struct _RKMPP_DEVICE {
    UINT32                 Hid;
    UINT32                 Uid;
    UINT32                 RevisionWord;
    UINT32                 SupportedCodecs;
    PVOID                  MmioBase;
    SIZE_T                 MmioLength;
    RKMPP_IFC_CLIENT       Ifcs;
    RKMPP_JOB_QUEUE        JobQueue;

    /* Phase 3a: IOMMU fault state — written by RkMppOnIommuFault at
     * DISPATCH_LEVEL, read by IOCTL_RKMPP_INJECT_IOMMU_FAULT at any level. */
    volatile LONG          FaultTriggered;   /* set to 1 by the callback */
    volatile LONG          FaultStatusReg;   /* status reg captured */
    volatile LONG64        FaultIova;        /* iova captured */
} RKMPP_DEVICE, *PRKMPP_DEVICE;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RKMPP_DEVICE, RkMppDeviceGet);

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
static UINT32 RkMppQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo);

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

    /* Step 1: walk resources to capture the MMIO base AND the raw+translated
     * descriptors for the first interrupt.  ARM64 GIC line interrupts require
     * the descriptors to be passed explicitly to WdfInterruptCreate; the
     * default auto-bind path fails with STATUS_WDF_INVALID_INTERRUPT_CONFIG
     * (0xC020020F) — confirmed empirically on first hardware bring-up. */
    PCM_PARTIAL_RESOURCE_DESCRIPTOR irqRaw   = NULL;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR irqTrans = NULL;

    /* RVD0/RVD1 (RKCP3550) declare TWO memory regions in _CRS:
     *   link : 0xFDC38000 / 0x100  ← idx 0..63   (common bank + kick)
     *   regs : 0xFDC38100 / 0x400  ← idx 64..319 (codec params + addrs)
     *
     * They are physically contiguous, so we map [min_start, max_end) as
     * one block — that gets us a single MmioBase whose offsets line up
     * with the BSP's idx*4 byte offsets (idx 10 = 0x28 lands on the kick
     * register, idx 224 = 0x380 lands on INT_STATUS, etc.).
     *
     * Single-region devices (encoder cores, etc.) take the obvious path. */
    PHYSICAL_ADDRESS mmioLow  = {0};
    PHYSICAL_ADDRESS mmioHigh = {0};
    BOOLEAN          mmioFound = FALSE;
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type == CmResourceTypeMemory) {
            PHYSICAL_ADDRESS start = d->u.Memory.Start;
            PHYSICAL_ADDRESS end;
            end.QuadPart = start.QuadPart + d->u.Memory.Length;
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
    /* RVD0/RVD1 (RKCP3550): the ACPI _CRS only declares the lower 0x500
     * bytes of the IP's register file (link 0x100 + regs 0x400), but the
     * BSP DTS exposes the full 0x800 — the extra 0x300 holds the rkvdec2
     * internal cache configuration registers (CACHE0/1/2_SIZE at
     * 0x51C/0x55C/0x59C, CLR_CACHE0/1/2 at 0x510/0x550/0x590, MAX_READS
     * at 0x518) which the kernel BSP run path must program before kick.
     * Force the mapping to the full 0x800 so those writes can land.
     * Firmware fix is to extend the _CRS — until then this runtime
     * widening keeps us moving. */
    if (ctx->Hid == 0x3550 && mmioLen < 0x800) {
        mmioLen = 0x800;
    }
    ctx->MmioBase   = MmMapIoSpaceEx(mmioLow, mmioLen,
                                     PAGE_READWRITE | PAGE_NOCACHE);
    ctx->MmioLength = mmioLen;
    if (!ctx->MmioBase) return STATUS_INSUFFICIENT_RESOURCES;
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp: HID=RKCP%04x UID=%u MmioBase=phys 0x%llx len 0x%x\n",
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
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkmpp: ifcs opened (iommu v%u, ccu v%u)\n",
               ctx->Ifcs.Iommu.Header.Version, ctx->Ifcs.Ccu.Header.Version);

    /* Raise the cluster.  Refcounted; matching DropCluster in ReleaseHardware.
     * The cluster stays raised for the device's lifetime in v1; idle-timeout
     * drop is a future hardening item. */
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
    }

    /* Only touch codec MMIO + IOMMU wiring for devices we actually plan
     * to drive in this phase (decoders only — RVD0/RVD1).  RaiseCluster
     * powers the decoder PDs only; encoder/JPEG/IEP cores live in
     * PD_VENC0/1, etc., which we don't bring up here, so reading their
     * MMIO would SError.  The non-decoder probes still get a profile
     * row (SupportedCodecs=0) so subsequent IOCTLs can report
     * "unsupported"; they just don't get any bring-up activity. */
    if (p && p->SupportedCodecs != 0) {
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
         * to the in-flight job and trigger AssertCoreReset on its core. */
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
         * NOTE: AssertCoreReset currently only targets RVD0 (CON40 bit 9).
         * Gate this on HID==0x3550 && UID==0 to avoid resetting RVD0 from
         * RVD1's bring-up.  Per-core reset for RVD1 is a later task. */
        if (ctx->Hid == 0x3550 && ctx->Uid == 0 &&
            ctx->Ifcs.Ccu.AssertCoreReset && ctx->Ifcs.Ccu.DeassertCoreReset) {
            ctx->Ifcs.Ccu.AssertCoreReset(ctx->Ifcs.Ccu.Header.Context);
            ctx->Ifcs.Ccu.DeassertCoreReset(ctx->Ifcs.Ccu.Header.Context);
        }
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
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
     * there, then release the ifcs and unmap MMIO. */
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
    *Uid = RkMppQueryAcpiUid(pdo);
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

/* IRP_MN_QUERY_ID(BusQueryInstanceID) helper.  Sends a synchronous IRP up
 * the device's PnP stack to retrieve the per-enumerator instance ID and
 * parses it as a decimal integer.  For ACPI devices this returns _UID. */
typedef struct _RKMPP_QID_CTX { KEVENT Done; NTSTATUS Status; } RKMPP_QID_CTX;

_Use_decl_annotations_
static IO_COMPLETION_ROUTINE RkMppQidCompletion;
static NTSTATUS RkMppQidCompletion(PDEVICE_OBJECT DeviceObject, PIRP Irp, PVOID Ctx)
{
    UNREFERENCED_PARAMETER(DeviceObject);
    RKMPP_QID_CTX *c = (RKMPP_QID_CTX *)Ctx;
    NT_ASSERT(c != NULL);
    _Analysis_assume_(c != NULL);
    c->Status = Irp->IoStatus.Status;
    KeSetEvent(&c->Done, IO_NO_INCREMENT, FALSE);
    return STATUS_MORE_PROCESSING_REQUIRED;
}

static UINT32 RkMppQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo)
{
    PDEVICE_OBJECT topDev = IoGetAttachedDeviceReference(Pdo);
    if (!topDev) return 0;

    PIRP irp = IoAllocateIrp(topDev->StackSize, FALSE);
    if (!irp) {
        ObDereferenceObject(topDev);
        return 0;
    }

    RKMPP_QID_CTX ctx;
    KeInitializeEvent(&ctx.Done, NotificationEvent, FALSE);
    ctx.Status = STATUS_NOT_SUPPORTED;

    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    IoSetCompletionRoutine(irp, RkMppQidCompletion, &ctx, TRUE, TRUE, TRUE);

    PIO_STACK_LOCATION sl = IoGetNextIrpStackLocation(irp);
    sl->MajorFunction = IRP_MJ_PNP;
    sl->MinorFunction = IRP_MN_QUERY_ID;
    sl->Parameters.QueryId.IdType = BusQueryInstanceID;

    NTSTATUS status = IoCallDriver(topDev, irp);
    if (status == STATUS_PENDING) {
        KeWaitForSingleObject(&ctx.Done, Executive, KernelMode, FALSE, NULL);
        status = ctx.Status;
    }

    UINT32 uid = 0;
    if (NT_SUCCESS(status)) {
        PCWSTR instId = (PCWSTR)irp->IoStatus.Information;
        if (instId) {
            UNICODE_STRING us;
            RtlInitUnicodeString(&us, instId);
            /* acpi.sys formats numeric _UID values as hex without "0x"
             * prefix.  _UID=10 returns "A", _UID=11 returns "B", etc.
             * Parse base 16 — single-digit UIDs (0-9) match both bases. */
            ULONG val = 0;
            if (NT_SUCCESS(RtlUnicodeStringToInteger(&us, 16, &val))) {
                uid = val;
            }
            ExFreePool((PVOID)instId);  /* PnP requires caller to free */
        }
    }

    IoFreeIrp(irp);
    ObDereferenceObject(topDev);
    return uid;
}

VOID
RkMppEvtFileCleanup(_In_ WDFFILEOBJECT FileObject)
{
    /* Initialise context if it was never used (no allocs occurred). */
    PRKMPP_FILE_CTX ctx = RkMppFileGet(FileObject);
    if (!ctx->Device) {
        /* Never initialised — nothing to do. */
        return;
    }
    RkMppBufFreeAll(FileObject);
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
