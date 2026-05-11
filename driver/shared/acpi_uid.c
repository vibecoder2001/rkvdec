/* driver/shared/acpi_uid.c — ACPI _UID query via IRP_MN_QUERY_ID. */
#include <ntddk.h>
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
