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

/* Issues IRP_MN_QUERY_INTERFACE on a single device. */
static NTSTATUS QueryOne(_In_ PDEVICE_OBJECT devObj,
                         _In_ const GUID *Guid,
                         _In_ USHORT IfcVersion,
                         _Out_writes_bytes_(BufLen) PVOID Buf,
                         _In_ USHORT BufLen)
{
    PIRP irp = IoAllocateIrp(devObj->StackSize, FALSE);
    if (!irp) return STATUS_INSUFFICIENT_RESOURCES;

    RKMPP_QI_CTX ctx;
    KeInitializeEvent(&ctx.Done, NotificationEvent, FALSE);
    ctx.Status = STATUS_NOT_SUPPORTED;

    irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    IoSetCompletionRoutine(irp, RkMppQiCompletion, &ctx, TRUE, TRUE, TRUE);

    PIO_STACK_LOCATION sl = IoGetNextIrpStackLocation(irp);
    sl->MajorFunction = IRP_MJ_PNP;
    sl->MinorFunction = IRP_MN_QUERY_INTERFACE;
    sl->Parameters.QueryInterface.InterfaceType         = (LPGUID)Guid;
    sl->Parameters.QueryInterface.Size                  = BufLen;
    sl->Parameters.QueryInterface.Version               = IfcVersion;
    sl->Parameters.QueryInterface.Interface             = (PINTERFACE)Buf;
    sl->Parameters.QueryInterface.InterfaceSpecificData = NULL;

    NTSTATUS status = IoCallDriver(devObj, irp);
    if (status == STATUS_PENDING) {
        KeWaitForSingleObject(&ctx.Done, Executive, KernelMode, FALSE, NULL);
        status = ctx.Status;
    }
    IoFreeIrp(irp);
    return status;
}

/* Issues IRP_MN_QUERY_INTERFACE for Guid against the first matching
 * device interface.  On success the caller-owned PFILE_OBJECT is
 * returned via *OutFileObj — the caller MUST hold this reference for
 * as long as it uses the function pointers in Buf, because the pointers
 * land in the provider's .text section and the binary will unload as
 * soon as nothing keeps its DEVICE_OBJECT alive. */
static NTSTATUS QueryByGuid(_In_ const GUID *Guid,
                            _In_ USHORT IfcVersion,
                            _Out_writes_bytes_(BufLen) PVOID Buf,
                            _In_ USHORT BufLen,
                            _Out_ PFILE_OBJECT *OutFileObj)
{
    *OutFileObj = NULL;

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

        status = QueryOne(devObj, Guid, IfcVersion, Buf, BufLen);

        if (NT_SUCCESS(status)) {
            /* Hand the file-object reference to the caller — they pin
             * the provider's binary by holding it. */
            *OutFileObj = fo;
            final = STATUS_SUCCESS;
            break;
        }

        ObDereferenceObject(fo);
        cursor += wcslen(cursor) + 1;
    }

    ExFreePool(symlinks);
    return final;
}

/* v1 client→IOMMU topology — must match driver/rkiommu/topology.c.
 * RKCP3550 (rkvdec0/1 cores) → RKCP3570 with the matching UID:
 *   RVD0 (3550, UID 0) → RD0M (3570, UID 9)
 *   RVD1 (3550, UID 1) → RD1M (3570, UID 10)
 * Returns 0,0 if no entry. */
static VOID LookupIommuForClient(UINT32 ClientHid, UINT32 ClientUid,
                                 UINT32 *IommuHid, UINT32 *IommuUid)
{
    *IommuHid = 0;
    *IommuUid = 0;
    if (ClientHid == 0x3550) {
        *IommuHid = 0x3570;
        *IommuUid = (ClientUid == 0) ? 9 : 10;
    }
}

/* Iterate every published rkiommu device interface and pick the one
 * whose Hid/Uid matches (TargetHid, TargetUid).  Returns the file-
 * object reference + populated interface struct on success. */
static NTSTATUS QueryIommuByHidUid(UINT32 TargetHid, UINT32 TargetUid,
                                   PRKIOMMU_INTERFACE Out,
                                   PFILE_OBJECT *OutFileObj)
{
    *OutFileObj = NULL;
    PWSTR symlinks = NULL;
    NTSTATUS status = IoGetDeviceInterfaces((LPGUID)&GUID_DEVINTERFACE_RKIOMMU,
                                            NULL, 0, &symlinks);
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
        if (!NT_SUCCESS(IoGetDeviceObjectPointer(&name, FILE_READ_DATA,
                                                  &fo, &devObj))) {
            cursor += wcslen(cursor) + 1;
            continue;
        }

        RKIOMMU_INTERFACE ifc;
        RtlZeroMemory(&ifc, sizeof(ifc));
        status = QueryOne(devObj, &GUID_DEVINTERFACE_RKIOMMU,
                          RKIOMMU_IFC_VERSION, &ifc, sizeof(ifc));
        if (NT_SUCCESS(status) &&
            ifc.Hid == TargetHid && ifc.Uid == TargetUid) {
            *Out         = ifc;
            *OutFileObj  = fo;
            final        = STATUS_SUCCESS;
            break;
        }
        if (NT_SUCCESS(status) && ifc.Header.InterfaceDereference) {
            ifc.Header.InterfaceDereference(ifc.Header.Context);
        }
        ObDereferenceObject(fo);
        cursor += wcslen(cursor) + 1;
    }

    ExFreePool(symlinks);
    return final;
}

NTSTATUS RkMppOpenIfcs(_In_ WDFDEVICE Device,
                       _In_ UINT32 ClientHid,
                       _In_ UINT32 ClientUid,
                       _Out_ RKMPP_IFC_CLIENT *Out)
{
    UNREFERENCED_PARAMETER(Device);
    RtlZeroMemory(Out, sizeof(*Out));

    /* Topology-aware iommu pick: rkmpp's own (Hid, Uid) → the iommu
     * instance that owns its translation domain.  Without this we'd
     * pick the first rkiommu IoGetDeviceInterfaces returns (typically
     * VPMU UID 0), which has no translations for our buffers, and the
     * codec hardware silently never decodes. */
    UINT32 iommuHid = 0, iommuUid = 0;
    LookupIommuForClient(ClientHid, ClientUid, &iommuHid, &iommuUid);
    NTSTATUS s;
    if (iommuHid != 0) {
        s = QueryIommuByHidUid(iommuHid, iommuUid,
                               &Out->Iommu, &Out->IommuFileObj);
        if (!NT_SUCCESS(s)) {
            DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL,
                       "rkmpp: no rkiommu RKCP%04x UID=%u for client "
                       "RKCP%04x UID=%u — falling back to first match\n",
                       iommuHid, iommuUid, ClientHid, ClientUid);
        }
    } else {
        s = STATUS_NOT_FOUND;
    }
    if (!NT_SUCCESS(s)) {
        /* Fallback for clients we don't have a topology entry for: any
         * rkiommu instance.  Used by non-decoder probes that don't
         * actually run jobs. */
        s = QueryByGuid(&GUID_DEVINTERFACE_RKIOMMU,
                        RKIOMMU_IFC_VERSION,
                        &Out->Iommu, sizeof(Out->Iommu),
                        &Out->IommuFileObj);
        if (!NT_SUCCESS(s)) return s;
    }
    Out->IommuOpen = TRUE;

    s = QueryByGuid(&GUID_DEVINTERFACE_RKMPP_CCU,
                    RKMPP_CCU_IFC_VERSION,
                    &Out->Ccu, sizeof(Out->Ccu),
                    &Out->CcuFileObj);
    if (!NT_SUCCESS(s)) {
        if (Out->Iommu.Header.InterfaceDereference)
            Out->Iommu.Header.InterfaceDereference(Out->Iommu.Header.Context);
        if (Out->IommuFileObj) {
            ObDereferenceObject(Out->IommuFileObj);
            Out->IommuFileObj = NULL;
        }
        Out->IommuOpen = FALSE;
        return s;
    }
    Out->CcuOpen = TRUE;
    return STATUS_SUCCESS;
}

VOID RkMppCloseIfcs(_Inout_ RKMPP_IFC_CLIENT *c)
{
    /* Drop interface refs before file-object refs so the provider's
     * Dereference callback runs while its binary is still loaded. */
    if (c->IommuOpen && c->Iommu.Header.InterfaceDereference)
        c->Iommu.Header.InterfaceDereference(c->Iommu.Header.Context);
    if (c->CcuOpen && c->Ccu.Header.InterfaceDereference)
        c->Ccu.Header.InterfaceDereference(c->Ccu.Header.Context);
    if (c->IommuFileObj) ObDereferenceObject(c->IommuFileObj);
    if (c->CcuFileObj)   ObDereferenceObject(c->CcuFileObj);
    RtlZeroMemory(c, sizeof(*c));
}
