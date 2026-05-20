/* driver/shared/acpi_uid.c — ACPI _UID query via IRP_MN_QUERY_ID. */
#include <ntddk.h>
#include <wdf.h>
#include "acpi_uid.h"

typedef struct _RK_QID_CTX { KEVENT Done; NTSTATUS Status; } RK_QID_CTX;

_Use_decl_annotations_
static IO_COMPLETION_ROUTINE RkSharedQidCompletion;
static NTSTATUS RkSharedQidCompletion(PDEVICE_OBJECT DeviceObject, PIRP Irp, PVOID Ctx)
{
    UNREFERENCED_PARAMETER(DeviceObject);
    RK_QID_CTX *c = (RK_QID_CTX *)Ctx;
    NT_ASSERT(c != NULL);
    _Analysis_assume_(c != NULL);
    c->Status = Irp->IoStatus.Status;
    KeSetEvent(&c->Done, IO_NO_INCREMENT, FALSE);
    return STATUS_MORE_PROCESSING_REQUIRED;
}

UINT32 RkSharedQueryAcpiUid(_In_ PDEVICE_OBJECT Pdo)
{
    PDEVICE_OBJECT topDev = IoGetAttachedDeviceReference(Pdo);
    if (!topDev) return 0;

    PIRP irp = IoAllocateIrp(topDev->StackSize, FALSE);
    if (!irp) { ObDereferenceObject(topDev); return 0; }

    RK_QID_CTX ctx;
    KeInitializeEvent(&ctx.Done, NotificationEvent, FALSE);
    ctx.Status = STATUS_NOT_SUPPORTED;
    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    IoSetCompletionRoutine(irp, RkSharedQidCompletion, &ctx, TRUE, TRUE, TRUE);

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
            ULONG val = 0;
            if (NT_SUCCESS(RtlUnicodeStringToInteger(&us, 16, &val)))
                uid = val;
            ExFreePool((PVOID)instId);
        }
    }
    IoFreeIrp(irp);
    ObDereferenceObject(topDev);
    return uid;
}

NTSTATUS RkSharedReadAcpiHidUid(_In_ WDFDEVICE Device,
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
        /* Accept any four hex digits after "ACPI\RKCP".  The old
         * per-driver copies hardcoded `cursor[9]=='3' && cursor[10]=='5'`
         * (so only RKCP35xx); a hypothetical RKCP36xx variant would
         * silently fail to ID, returning STATUS_INVALID_DEVICE_REQUEST
         * and aborting PrepareHardware.  The single arbiter for which
         * HIDs the driver supports is the .INF match list — not the
         * parser. */
        if (len >= 13 &&
            cursor[0] == L'A' && cursor[1] == L'C' && cursor[2] == L'P' &&
            cursor[3] == L'I' && cursor[4] == L'\\' &&
            cursor[5] == L'R' && cursor[6] == L'K' && cursor[7] == L'C' &&
            cursor[8] == L'P')
        {
            UINT32 hid = 0;
            BOOLEAN ok = TRUE;
            for (int i = 9; i < 13; i++) {
                WCHAR  c = cursor[i];
                UINT32 d;
                if      (c >= L'0' && c <= L'9') d = (UINT32)(c - L'0');
                else if (c >= L'a' && c <= L'f') d = 10u + (UINT32)(c - L'a');
                else if (c >= L'A' && c <= L'F') d = 10u + (UINT32)(c - L'A');
                else { ok = FALSE; break; }
                hid = (hid << 4) | d;
            }
            if (ok && hid) {
                *Hid = hid;
                *Uid = RkSharedQueryAcpiUid(pdo);
                return STATUS_SUCCESS;
            }
        }
        cursor += len + 1;
    }
    return STATUS_INVALID_DEVICE_REQUEST;
}
