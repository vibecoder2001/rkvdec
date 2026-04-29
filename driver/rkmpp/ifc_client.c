/* driver/rkmpp/ifc_client.c — opens rkiommu.sys and rkmpp_ccu.sys ifcs. */
#include "ifc_client.h"

/* Per-IRP completion gate. */
typedef struct _RKMPP_QI_CTX { KEVENT Done; NTSTATUS Status; } RKMPP_QI_CTX;

static IO_COMPLETION_ROUTINE RkMppQiCompletion;

_Use_decl_annotations_
static NTSTATUS RkMppQiCompletion(PDEVICE_OBJECT DeviceObject, PIRP Irp, PVOID Context)
{
    UNREFERENCED_PARAMETER(DeviceObject);
    RKMPP_QI_CTX *c = (RKMPP_QI_CTX*)Context;
    NT_ASSERT(c != NULL);
    _Analysis_assume_(c != NULL);
    c->Status = Irp->IoStatus.Status;
    KeSetEvent(&c->Done, IO_NO_INCREMENT, FALSE);
    return STATUS_MORE_PROCESSING_REQUIRED;
}

static NTSTATUS QueryByGuid(_In_ const GUID *Guid,
                            _In_ USHORT IfcVersion,
                            _Out_writes_bytes_(BufLen) PVOID Buf,
                            _In_ USHORT BufLen)
{
    PWSTR symlinks = NULL;
    NTSTATUS status = IoGetDeviceInterfaces((LPGUID)Guid, NULL, 0, &symlinks);
    if (!NT_SUCCESS(status)) return status;
    if (!symlinks || !*symlinks) {
        if (symlinks) ExFreePool(symlinks);
        return STATUS_NOT_FOUND;
    }

    NTSTATUS final = STATUS_NOT_FOUND;
    PWSTR cursor = symlinks;
    while (*cursor) {
        UNICODE_STRING name;
        RtlInitUnicodeString(&name, cursor);

        PFILE_OBJECT fo = NULL;
        PDEVICE_OBJECT devObj = NULL;
        status = IoGetDeviceObjectPointer(&name, FILE_READ_DATA, &fo, &devObj);
        if (!NT_SUCCESS(status)) {
            cursor += wcslen(cursor) + 1;
            continue;
        }

        PIRP irp = IoAllocateIrp(devObj->StackSize, FALSE);
        if (!irp) {
            ObDereferenceObject(fo);
            cursor += wcslen(cursor) + 1;
            continue;
        }
        RKMPP_QI_CTX ctx;
        KeInitializeEvent(&ctx.Done, NotificationEvent, FALSE);
        ctx.Status = STATUS_NOT_SUPPORTED;

        irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
        IoSetCompletionRoutine(irp, RkMppQiCompletion, &ctx, TRUE, TRUE, TRUE);

        PIO_STACK_LOCATION sl = IoGetNextIrpStackLocation(irp);
        sl->MajorFunction = IRP_MJ_PNP;
        sl->MinorFunction = IRP_MN_QUERY_INTERFACE;
        sl->Parameters.QueryInterface.InterfaceType             = (LPGUID)Guid;
        sl->Parameters.QueryInterface.Size                      = BufLen;
        sl->Parameters.QueryInterface.Version                   = IfcVersion;
        sl->Parameters.QueryInterface.Interface                 = (PINTERFACE)Buf;
        sl->Parameters.QueryInterface.InterfaceSpecificData     = NULL;

        status = IoCallDriver(devObj, irp);
        if (status == STATUS_PENDING) {
            KeWaitForSingleObject(&ctx.Done, Executive, KernelMode, FALSE, NULL);
            status = ctx.Status;
        }
        IoFreeIrp(irp);
        ObDereferenceObject(fo);

        if (NT_SUCCESS(status)) {
            final = STATUS_SUCCESS;
            break;
        }

        cursor += wcslen(cursor) + 1;
    }

    ExFreePool(symlinks);
    return final;
}

NTSTATUS RkMppOpenIfcs(_In_ WDFDEVICE Device, _Out_ RKMPP_IFC_CLIENT *Out)
{
    UNREFERENCED_PARAMETER(Device);  /* ClientCookie identification deferred */
    RtlZeroMemory(Out, sizeof(*Out));

    NTSTATUS s = QueryByGuid(&GUID_DEVINTERFACE_RKIOMMU,
                             RKIOMMU_IFC_VERSION,
                             &Out->Iommu, sizeof(Out->Iommu));
    if (!NT_SUCCESS(s)) return s;
    Out->IommuOpen = TRUE;

    s = QueryByGuid(&GUID_DEVINTERFACE_RKMPP_CCU,
                    RKMPP_CCU_IFC_VERSION,
                    &Out->Ccu, sizeof(Out->Ccu));
    if (!NT_SUCCESS(s)) {
        if (Out->Iommu.Header.InterfaceDereference)
            Out->Iommu.Header.InterfaceDereference(Out->Iommu.Header.Context);
        Out->IommuOpen = FALSE;
        return s;
    }
    Out->CcuOpen = TRUE;
    return STATUS_SUCCESS;
}

VOID RkMppCloseIfcs(_Inout_ RKMPP_IFC_CLIENT *c)
{
    if (c->IommuOpen && c->Iommu.Header.InterfaceDereference)
        c->Iommu.Header.InterfaceDereference(c->Iommu.Header.Context);
    if (c->CcuOpen && c->Ccu.Header.InterfaceDereference)
        c->Ccu.Header.InterfaceDereference(c->Ccu.Header.Context);
    RtlZeroMemory(c, sizeof(*c));
}
