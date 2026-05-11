/* driver/rkiommu_vdec/device.c — per-instance RKIOMMU_DEVICE lifecycle.
 *
 * Responsibilities:
 *   - parse HID + _UID from the ACPI hardware-ID multi-sz list
 *   - allocate an RKIOMMU_DOMAIN (page directory + IOVA bitmap)
 *   - map the MMIO window from _CRS resource 0
 *   - connect the IRQ (WdfInterruptCreate)
 *   - register itself in the global g_deviceList so MapMdl can find it
 *   - expose RkIommuEnable() for lazy paging activation
 *
 * Register offsets used here are defined in pgtable.h and sourced from
 *   torvalds/linux drivers/iommu/rockchip-iommu.c (master, 2024).
 */
#include <ntddk.h>
#include <wdf.h>
#include "device.h"
#include "../shared/iommu/pgtable.h"
#include "../shared/iommu/fault.h"
#include "../shared/acpi_uid.h"

/* ---------------------------------------------------------------------------
 * Global instance list — all RKIOMMU_DEVICE instances live here.
 * Protected by g_deviceListLock (DISPATCH_LEVEL spin lock).
 * --------------------------------------------------------------------------- */
LIST_ENTRY  g_deviceList;
KSPIN_LOCK  g_deviceListLock;

static BOOLEAN g_listInitialized = FALSE;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
EVT_WDF_DEVICE_PREPARE_HARDWARE  RkIommuEvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  RkIommuEvtReleaseHardware;

extern NTSTATUS RkIommuRegisterIfc(_In_ WDFDEVICE Device);  /* in ifc.c */

/* ---------------------------------------------------------------------------
 * ACPI HID/UID parse
 *
 * Walks the multi-sz hardware-ID list looking for "ACPI\RKCP35xx".
 * Extracts the last 4 hex digits as the HID (e.g. RKCP3570 → 0x3570).
 * Reads _UID via IRP_MN_QUERY_ID(BusQueryInstanceID) (DevicePropertyUINumber
 * is not reliably populated by acpi.sys).
 * --------------------------------------------------------------------------- */
static NTSTATUS
RkIommuReadAcpiId(_In_ WDFDEVICE Device,
                  _Out_ PUINT32 Hid,
                  _Out_ PUINT32 Uid)
{
    PDEVICE_OBJECT pdo = WdfDeviceWdmGetPhysicalDevice(Device);
    WCHAR buf[1024] = {0};
    ULONG size = 0;

    NTSTATUS status = IoGetDeviceProperty(pdo, DevicePropertyHardwareID,
                                          sizeof(buf), buf, &size);
    if (!NT_SUCCESS(status)) return status;

    PCWSTR cursor = buf;
    while (*cursor) {
        size_t len = wcslen(cursor);
        if (len >= 13 &&
            cursor[0] == L'A' && cursor[1] == L'C' && cursor[2] == L'P' &&
            cursor[3] == L'I' && cursor[4] == L'\\' &&
            cursor[5] == L'R' && cursor[6] == L'K' && cursor[7] == L'C' &&
            cursor[8] == L'P' && cursor[9] == L'3' && cursor[10] == L'5')
        {
            UINT32 hid = 0;
            for (int i = 9; i < 13; i++) {
                WCHAR  c = cursor[i];
                UINT32 d;
                if      (c >= L'0' && c <= L'9') d = (UINT32)(c - L'0');
                else if (c >= L'a' && c <= L'f') d = 10u + (UINT32)(c - L'a');
                else if (c >= L'A' && c <= L'F') d = 10u + (UINT32)(c - L'A');
                else { hid = 0; break; }
                hid = (hid << 4) | d;
            }
            if (hid) {
                *Hid = hid;
                /* IRP_MN_QUERY_ID(BusQueryInstanceID) returns the _UID for
                 * ACPI devices.  DevicePropertyUINumber is not reliably
                 * populated by acpi.sys, so we use the IRP path. */
                *Uid = RkSharedQueryAcpiUid(pdo);
                return STATUS_SUCCESS;
            }
        }
        cursor += len + 1;
    }
    return STATUS_INVALID_DEVICE_REQUEST;
}

/* ---------------------------------------------------------------------------
 * RkIommuDisable — pair to RkIommuEnable.  Mirrors Linux's
 * rockchip_iommu_disable: mask IRQs, command DISABLE_PAGING, zero
 * DTE_ADDR.  All MMU instances are touched.  After this returns, the
 * IOMMU is in the same state as before any RkIommuEnable was ever
 * called — page-table contents preserved (Domain is not freed) but the
 * hardware no longer holds any walk-cache state for them.
 *
 * Idempotent: calling on an already-disabled IOMMU returns SUCCESS
 * without touching MMIO.
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuDisableHw(PRKIOMMU_DEVICE Dev)
{
    if (!Dev || !Dev->MmioBase) return STATUS_DEVICE_NOT_READY;
    if (!Dev->PagingEnabled)    return STATUS_SUCCESS;

    int n_cfg = (Dev->MmioLength >= 0x80) ? 2 : 1;

    /* STALL all instances first so any in-flight AXI traffic completes
     * or is held until we finish the disable sequence. */
    for (int mi = 0; mi < n_cfg; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_ENABLE_STALL);
        KeStallExecutionProcessor(20);
    }

    for (int mi = 0; mi < n_cfg; mi++) {
        volatile UCHAR *base = Dev->MmioBase + (mi * 0x40);

        /* Mask IRQs so the disable sequence doesn't fire a spurious
         * fault interrupt when the codec's last AXI gets translated
         * with a paging-disabled engine. */
        WRITE_REGISTER_ULONG((volatile ULONG*)(base + RK_MMU_INT_MASK), 0u);

        /* DISABLE_PAGING. */
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_COMMAND),
            RK_MMU_CMD_DISABLE_PAGING);
        KeStallExecutionProcessor(20);

        /* Zero DTE_ADDR — drops the binding to the page directory.
         * Critical: the next RkIommuEnable will write the original
         * Dev->Domain->PdPhys back, forcing the IOMMU to re-fetch the
         * directory and rebuild walk caches from scratch.  Without
         * this zero step, attaching the same domain is treated as a
         * no-op by some implementations (we observed walk-cache
         * residue surviving a plain Enable→Disable→Enable). */
        WRITE_REGISTER_ULONG((volatile ULONG*)(base + RK_MMU_DTE_ADDR), 0u);
    }

    Dev->PagingEnabled = FALSE;

    /* UN-STALL all instances. */
    for (int mi = 0; mi < n_cfg; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_DISABLE_STALL);
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL,
               "rkiommu_vdec: disabled (HID=RKCP%04x UID=%u cfg=%d)\n",
               Dev->Hid, Dev->Uid, n_cfg);
    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuEnable — activate IOMMU paging on the hardware.
 *
 * Phase 3a: real MMIO programming, with three BSP-mandated flags:
 *   FlagDisableMmuReset   — skip the MMU reset command (RK3588 rkvdec/enc)
 *   FlagEnableCmdRetry    — retry ENABLE_PAGING up to 3 times with status poll
 *   FlagShootdownEntire   — ZAP_CACHE used for TLB flush (set in MapMdl/UnmapMdl)
 *
 * Called lazily by MapMdl on the first successful map.
 * --------------------------------------------------------------------------- */
_Use_decl_annotations_
NTSTATUS RkIommuEnableHw(PRKIOMMU_DEVICE Dev)
{
    if (!Dev->MmioBase || !Dev->Domain) return STATUS_DEVICE_NOT_READY;
    if (Dev->PagingEnabled)             return STATUS_SUCCESS;

    /* RK3588 rkvdec0_mmu / rkvdec1_mmu have TWO MMU instances per
     * codec, one for read-port one for write-port.  DT:
     *   reg = <0x0 0xfdc38700 0x0 0x40>, <0x0 0xfdc38740 0x0 0x40>;
     * BSP rockchip-iommu.c walks `iommu->num_mmu` and writes EVERY
     * config register to ALL bases.  We were configuring only the
     * first — bitstream READS went through MMU#0 and translated
     * correctly, but DECOUT WRITES went through MMU#1 (paging-disabled
     * by default) and were silently dropped.  Fix: write the entire
     * enable sequence to both bases when MmioLength >= 0x80. */
    /* Configure both MMU instances when present (RK3588 codec MMUs come
     * in pairs).  Wrap the entire enable sequence in STALL / UN-STALL
     * commands like BSP rk_iommu_enable does — without stall, the
     * partially-configured MMU sees in-flight codec AXI traffic and
     * lands in undefined state (we observed kernel BSOD / SoC abort). */
    int n_cfg   = (Dev->MmioLength >= 0x80) ? 2 : 1;
    int n_paged = n_cfg;

    /* STALL all instances before config. */
    for (int mi = 0; mi < n_cfg; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_ENABLE_STALL);
        KeStallExecutionProcessor(50);
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu_vdec: HID=RKCP%04x UID=%u (MmioLength=0x%x) cfg=%d paged=%d\n",
               Dev->Hid, Dev->Uid, Dev->MmioLength, n_cfg, n_paged);

    /* Configure ALL MMU instances (n_mmu) — RK3588 codec MMUs come in
     * pairs (read-port + write-port).  See comment above for details. */
    for (int mi = 0; mi < n_cfg; mi++) {
        volatile UCHAR *base = Dev->MmioBase + (mi * 0x40);

        /* 1. Optionally reset the MMU. */
        if (!Dev->FlagDisableMmuReset) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG*)(base + RK_MMU_COMMAND),
                RK_MMU_CMD_RESET);
            KeStallExecutionProcessor(50);
        }

        /* 2. Program the page-directory base. */
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_DTE_ADDR),
            Dev->Domain->PdPhys);

        /* 3. Zap any stale TLB cache from a previous session. */
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_COMMAND),
            RK_MMU_CMD_ZAP_CACHE);

        /* 4. Enable IRQs. */
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_INT_MASK),
            RK_MMU_IRQ_MASK);

        /* 5. AUTO_GATING workaround. */
        ULONG ag = READ_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_AUTO_GATING));
        ag |= (1u << 31);   /* DISABLE_FETCH_DTE_TIME_LIMIT */
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(base + RK_MMU_AUTO_GATING),
            ag);
    }

    /* Diagnostic: read back DTE_ADDR + STATUS + INT_MASK to verify the
     * MMIO writes above actually landed.  If the IOMMU is mis-clocked, the
     * read returns 0xFFFFFFFF or stale; if DTE_ADDR readback != PdPhys,
     * the IOMMU isn't in our page table. */
    {
        ULONG dteRb  = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_DTE_ADDR));
        ULONG statRb = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_STATUS));
        ULONG maskRb = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_INT_MASK));
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: pre-enable readback HID=RKCP%04x UID=%u "
                   "DTE_ADDR=0x%08x (want 0x%08x) STATUS=0x%08x INT_MASK=0x%08x\n",
                   Dev->Hid, Dev->Uid, dteRb, Dev->Domain->PdPhys, statRb, maskRb);
    }

    /* 6. Enable paging on ALL MMU instances. */
    ULONG attempts = Dev->FlagEnableCmdRetry ? 3u : 1u;
    for (ULONG i = 0; i < attempts; i++) {
        for (int mi = 0; mi < n_paged; mi++) {
            WRITE_REGISTER_ULONG(
                (volatile ULONG*)(Dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
                RK_MMU_CMD_ENABLE_PAGING);
        }
        ULONG st0 = READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + RK_MMU_STATUS));
        ULONG st1 = (n_cfg > 1) ? READ_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + 0x40 + RK_MMU_STATUS)) : 0u;
        if ((st0 & RK_MMU_STATUS_PAGING_ENABLED)) {
            Dev->PagingEnabled = TRUE;
            /* UN-STALL all instances to release in-flight AXI traffic. */
            for (int mi = 0; mi < n_cfg; mi++) {
                WRITE_REGISTER_ULONG(
                    (volatile ULONG*)(Dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
                    RK_MMU_CMD_DISABLE_STALL);
            }
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu_vdec: paging enabled (HID=RKCP%04x UID=%u attempt=%u st0=0x%x st1=0x%x cfg=%d paged=%d)\n",
                       Dev->Hid, Dev->Uid, i + 1u, st0, st1, n_cfg, n_paged);
            return STATUS_SUCCESS;
        }
        KeStallExecutionProcessor(20);
    }

    /* Failure path: also UN-STALL so the IOMMU doesn't stay frozen. */
    for (int mi = 0; mi < n_cfg; mi++) {
        WRITE_REGISTER_ULONG(
            (volatile ULONG*)(Dev->MmioBase + (mi * 0x40) + RK_MMU_COMMAND),
            RK_MMU_CMD_DISABLE_STALL);
    }
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu_vdec: paging-enable timeout (HID=RKCP%04x UID=%u)\n",
               Dev->Hid, Dev->Uid);
    return STATUS_DEVICE_HARDWARE_ERROR;
}

/* ---------------------------------------------------------------------------
 * EvtPrepareHardware
 * --------------------------------------------------------------------------- */
NTSTATUS
RkIommuEvtPrepareHardware(_In_ WDFDEVICE Device,
                           _In_ WDFCMRESLIST ResourcesRaw,
                           _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesRaw);

    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(Device);

    NTSTATUS status = RkIommuReadAcpiId(Device, &ctx->Hid, &ctx->Uid);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: failed to read ACPI ID (0x%08x)\n", status);
        return status;
    }

    /* UIDs 7-10 are RE0M/RE1M/RD0M/RD1M — codec IOMMUs carry the three
     * BSP _DSD flags.  Block IOMMUs (UIDs 0-6) do not. */
    BOOLEAN codecIommu = (ctx->Uid >= 7 && ctx->Uid <= 10);
    ctx->FlagDisableMmuReset = codecIommu;
    ctx->FlagEnableCmdRetry  = codecIommu;
    ctx->FlagShootdownEntire = codecIommu;

    /* RK3588 codec MMUs come in pairs.  ACPI declares two Memory regions
     *   0xFDC38700/0x40 + 0xFDC38740/0x40
     * which we want to map as ONE contiguous 0x80 block so num_mmu=2
     * detection downstream works.  Walk all memory resources, pick the
     * first one as base, and extend Length by every adjacent region. */
    ULONG count = WdfCmResourceListGetCount(ResourcesTranslated);
    PHYSICAL_ADDRESS basePhys = {0};
    ULONG totalLen = 0;
    for (ULONG i = 0; i < count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d =
            WdfCmResourceListGetDescriptor(ResourcesTranslated, i);
        if (d->Type != CmResourceTypeMemory) continue;
        if (totalLen == 0) {
            basePhys = d->u.Memory.Start;
            totalLen = d->u.Memory.Length;
        } else if (d->u.Memory.Start.QuadPart ==
                   basePhys.QuadPart + totalLen) {
            totalLen += d->u.Memory.Length;
        } else {
            /* Non-adjacent — would need a separate ioremap.  Log a
             * warning and ignore for now; only the rk3588 paired-MMU
             * layout matters for our scope. */
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu_vdec: non-adjacent mem resource at 0x%llx ignored\n",
                       d->u.Memory.Start.QuadPart);
        }
    }
    if (totalLen) {
        ctx->MmioBase = (volatile UCHAR*)MmMapIoSpaceEx(
            basePhys, totalLen, PAGE_READWRITE | PAGE_NOCACHE);
        ctx->MmioLength = totalLen;
    }
    if (!ctx->MmioBase) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: no MMIO resource\n");
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    /* Allocate the v2 IOMMU domain (page directory + IOVA bitmap) */
    status = RkIommuDomainCreateVdec(&ctx->Domain);
    if (!NT_SUCCESS(status)) {
        MmUnmapIoSpace((PVOID)ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase = NULL;
        return status;
    }

    /* Register in the global list */
    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    InsertTailList(&g_deviceList, &ctx->ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu_vdec: RKCP%04x UID=%u ready, MMIO=%p PdPhys=0x%08x\n",
               ctx->Hid, ctx->Uid, ctx->MmioBase, ctx->Domain->PdPhys);

    /* Phase 2: leave IOMMU paging disabled until first client maps.
     * RkIommuEnable(ctx) will be called lazily from the MapMdl path. */

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * EvtReleaseHardware
 * --------------------------------------------------------------------------- */
NTSTATUS
RkIommuEvtReleaseHardware(_In_ WDFDEVICE Device,
                           _In_ WDFCMRESLIST ResourcesTranslated)
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(Device);

    /* Remove from global list */
    KIRQL irql;
    KeAcquireSpinLock(&g_deviceListLock, &irql);
    RemoveEntryList(&ctx->ListEntry);
    InitializeListHead(&ctx->ListEntry);
    KeReleaseSpinLock(&g_deviceListLock, irql);

    /* Do NOT touch MMIO here.  By the time EvtReleaseHardware runs (driver
     * uninstall, surprise-removal, sleep), the codec power-domain that gates
     * this IOMMU's MMIO may already have been torn down by the rkmpp / ccu
     * stack — touching the registers in that window trips a synchronous
     * external abort and bugcheck 0x124 (WHEA_UNCORRECTABLE_ERROR, sub 0x12).
     *
     * The page-table RAM is freed by RkIommuDomainDestroy below, and the
     * IOMMU hardware state is reset by RkIommuEnable on the next load, so
     * skipping the disable-paging / int-mask writes here costs nothing. */
    ctx->PagingEnabled = FALSE;

    if (ctx->Domain) {
        RkIommuDomainDestroy(ctx->Domain);
        ctx->Domain = NULL;
    }

    if (ctx->MmioBase) {
        MmUnmapIoSpace((PVOID)ctx->MmioBase, ctx->MmioLength);
        ctx->MmioBase   = NULL;
        ctx->MmioLength = 0;
    }

    return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * RkIommuDeviceCreate — called from EvtDeviceAdd
 * --------------------------------------------------------------------------- */
NTSTATUS
RkIommuDeviceCreate(_Inout_ PWDFDEVICE_INIT DeviceInit)
{
    /* Ensure the global list is initialized (idempotent, called once) */
    if (!g_listInitialized) {
        InitializeListHead(&g_deviceList);
        KeInitializeSpinLock(&g_deviceListLock);
        g_listInitialized = TRUE;
    }

    WDF_PNPPOWER_EVENT_CALLBACKS pnp;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnp);
    pnp.EvtDevicePrepareHardware = RkIommuEvtPrepareHardware;
    pnp.EvtDeviceReleaseHardware = RkIommuEvtReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnp);

    WDF_OBJECT_ATTRIBUTES attr;
    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attr, RKIOMMU_DEVICE);

    WDFDEVICE device;
    NTSTATUS status = WdfDeviceCreate(&DeviceInit, &attr, &device);
    if (!NT_SUCCESS(status)) return status;

    PRKIOMMU_DEVICE ctx = RkIommuDeviceGet(device);
    RtlZeroMemory(ctx, sizeof(*ctx));
    InitializeListHead(&ctx->ListEntry);

    /* Read ACPI _HID/_UID up front so RkIommuRegisterIfc below publishes
     * the correct (Hid, Uid) in the in-kernel interface struct.  WDF
     * snapshots the interface contents at AddQueryInterface time, so
     * filling these in later (e.g. from EvtPrepareHardware) is too late —
     * consumers would see Hid=0/Uid=0 and the topology match in
     * rkmpp/ifc_client.c would never succeed. */
    {
        NTSTATUS sId = RkIommuReadAcpiId(device, &ctx->Hid, &ctx->Uid);
        if (!NT_SUCCESS(sId)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkiommu_vdec: early ACPI ID read failed (0x%08x)\n", sId);
            return sId;
        }
    }

    /* Wire the IOMMU fault interrupt.  Phase 3a saw spurious WHEA-ing on
     * IRQ entry, but that was a symptom of the broader codec MMIO gating
     * issue (the ISR touched not-yet-clocked MMIO).  With Phase 3b's
     * cluster bring-up landed (REVISION reads cleanly), the path is safe.
     *
     * Note: per the BSP DSD `rockchip,master-handle-irq = 1`, RD0M/RD1M's
     * fault interrupts are routed to the master rkvdec core's IRQ rather
     * than the IOMMU's own line.  The vector here may therefore never
     * fire for those instances; that's expected, not a bug.  Other IOMMU
     * UIDs (VPMU, JDMU, ...) follow the standard path. */
    WDF_INTERRUPT_CONFIG intCfg;
    WDF_INTERRUPT_CONFIG_INIT(&intCfg, RkIommuEvtIsr, RkIommuEvtDpc);

    status = WdfInterruptCreate(device, &intCfg,
                                WDF_NO_OBJECT_ATTRIBUTES,
                                &ctx->Interrupt);
    if (!NT_SUCCESS(status)) {
        DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                   "rkiommu_vdec: WdfInterruptCreate failed (0x%08x)\n", status);
        /* Non-fatal: many of our IOMMU instances will never fire an IRQ
         * (master-handle-irq), so refusing to load on this would deny
         * service for the working ones.  Continue without an ISR. */
    }

    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
               "rkiommu_vdec: device created\n");

    return RkIommuRegisterIfc(device);
}
