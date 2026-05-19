/* driver/rkvdec/ioctl.c — IOCTL surface for rkvdec.sys (H.264 + H.265).
 *
 * Surfaces:
 *   IOCTL_RKMPP_GET_CAPS / ALLOC_BUFFER / FREE_BUFFER
 *   IOCTL_RKMPP_SUBMIT_DENSE_JOB / PEEK_DENSE_JOB / WAIT_JOB
 *
 * The legacy sparse IOCTL_RKMPP_SUBMIT_JOB / PEEK_JOB are NOT handled
 * here — rkvdec is dense-only.  rkav1d (separate driver binary) still
 * uses the sparse path because its SWREG layout doesn't match the
 * vdpu34x bank split.
 */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "devpub.h"
#include "../shared/rkmpp/bufpool.h"
#include "job.h"

/* Provided by device.c */
extern void    RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out);
extern void    RkMppGetFaultState(_In_ WDFDEVICE Device, _Out_ RKMPP_FAULT_STATE *Out);
extern BOOLEAN RkMppIsIommuAttached(_In_ WDFDEVICE Device);  /* Phase 4 (Task 4.2) */

EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL RkMppEvtIoDeviceControl;

NTSTATUS RkMppQueueInit(_In_ WDFDEVICE Device)
{
    /* Parallel dispatch: required for concurrent decode sessions on the
     * same engine.  Sequential mode serialised ALL IRPs across all File
     * handles, so stream 1's blocking WAIT_JOB (up to 1000ms) would
     * monopolise the queue and stall every IOCTL from stream 2 — even
     * its initial GET_CAPS / ALLOC_BUFFER calls.
     *
     * Handler reentrancy: SUBMIT/PEEK/WAIT/JobBufferInUse all hold the
     * per-queue spinlock; bufpool ALLOC/FREE hold the per-File spinlock;
     * GET_CAPS and INJECT_IOMMU_FAULT are read-only.  Multiple WAIT_JOB
     * callers block on per-job KEVENTs without holding any queue resource. */
    WDF_IO_QUEUE_CONFIG cfg;
    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&cfg, WdfIoQueueDispatchParallel);
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
    NTSTATUS   status = STATUS_INVALID_DEVICE_REQUEST;
    ULONG_PTR  info   = 0;
    WDFDEVICE  device = WdfIoQueueGetDevice(Queue);

    /* Phase 4 (Task 4.2): when master rkiommu has cascade-detached us,
     * refuse any IOCTL that requires IOMMU.  GET_CAPS is the exception —
     * it only reads device-context fields, no IOMMU needed; allowing it
     * lets MFT discover us during the transient detach window without
     * spurious errors. */
    if (!RkMppIsIommuAttached(device) && IoControlCode != IOCTL_RKMPP_GET_CAPS) {
        WdfRequestComplete(Request, STATUS_DEVICE_NOT_READY);
        return;
    }

    switch (IoControlCode) {

    /* ---- GET_CAPS ---------------------------------------------------- */
    case IOCTL_RKMPP_GET_CAPS: {
        if (OutputBufferLength < sizeof(RKMPP_CAPS)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        PRKMPP_CAPS out;
        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                (PVOID*)&out, NULL);
        if (NT_SUCCESS(status)) {
            RKMPP_DEVICE_PUBLIC pub;
            RkMppGetPublic(device, &pub);

            RtlZeroMemory(out, sizeof(*out));
            out->StructSize      = sizeof(*out);
            out->Hid             = pub.Hid;
            out->Uid             = pub.Uid;
            out->RevisionWord    = pub.RevisionWord;
            out->SupportedCodecs = pub.SupportedCodecs;
            info = sizeof(*out);
        }
        break;
    }

    /* ---- ALLOC_BUFFER ------------------------------------------------ */
    case IOCTL_RKMPP_ALLOC_BUFFER: {
        RKMPP_ALLOC_BUFFER_IN  *in;
        RKMPP_ALLOC_BUFFER_OUT *out;

        status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in),
                                               (PVOID*)&in, NULL);
        if (!NT_SUCCESS(status)) break;

        if (in->StructSize < sizeof(*in)) {
            status = STATUS_INVALID_PARAMETER;
            break;
        }

        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                (PVOID*)&out, NULL);
        if (!NT_SUCCESS(status)) break;

        /* Initialise the per-file context on first use. */
        WDFFILEOBJECT file = WdfRequestGetFileObject(Request);
        PRKMPP_FILE_CTX fctx = RkMppFileGet(file);
        if (!fctx->Device) {
            RkMppBufFileCtxInit(file, device);
        }

        status = RkMppBufAlloc(device, file, in, out);
        if (NT_SUCCESS(status))
            info = sizeof(*out);
        break;
    }

    /* ---- FREE_BUFFER ------------------------------------------------- */
    case IOCTL_RKMPP_FREE_BUFFER: {
        RKMPP_FREE_BUFFER_IN *in;
        status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in),
                                               (PVOID*)&in, NULL);
        if (!NT_SUCCESS(status)) break;

        WDFFILEOBJECT file = WdfRequestGetFileObject(Request);
        PRKMPP_FILE_CTX fctx = RkMppFileGet(file);
        if (!fctx->Device) {
            /* No buffers ever allocated on this file handle. */
            status = STATUS_NOT_FOUND;
            break;
        }

        if (RkMppJobBufferInUse(device, file, in->BufferHandle)) {
            status = STATUS_DEVICE_BUSY;
            break;
        }

        status = RkMppBufFree(file, in->BufferHandle);
        break;
    }

    /* ---- SUBMIT_DENSE_JOB -------------------------------------------- */
    case IOCTL_RKMPP_SUBMIT_DENSE_JOB: {
        if (InputBufferLength < sizeof(RKMPP_SUBMIT_DENSE_JOB_IN) ||
            OutputBufferLength < sizeof(RKMPP_SUBMIT_DENSE_JOB_OUT)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        RKMPP_SUBMIT_DENSE_JOB_IN  *in;
        RKMPP_SUBMIT_DENSE_JOB_OUT *out;
        status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in),
                                               (PVOID*)&in, NULL);
        if (!NT_SUCCESS(status)) break;
        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                (PVOID*)&out, NULL);
        if (!NT_SUCCESS(status)) break;
        WDFFILEOBJECT file = WdfRequestGetFileObject(Request);
        if (!file) {
            status = STATUS_INVALID_PARAMETER;
            break;
        }
        status = RkMppJobSubmitDense(WdfIoQueueGetDevice(Queue), file, in, out);
        if (NT_SUCCESS(status)) info = sizeof(*out);
        break;
    }

    /* ---- PEEK_DENSE_JOB ---------------------------------------------- */
    case IOCTL_RKMPP_PEEK_DENSE_JOB: {
        if (InputBufferLength < sizeof(RKMPP_PEEK_JOB_IN) ||
            OutputBufferLength < sizeof(RKMPP_PEEK_DENSE_JOB_OUT)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        RKMPP_PEEK_JOB_IN        *in;
        RKMPP_PEEK_DENSE_JOB_OUT *out;
        status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in),
                                               (PVOID*)&in, NULL);
        if (!NT_SUCCESS(status)) break;
        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                (PVOID*)&out, NULL);
        if (!NT_SUCCESS(status)) break;
        WDFFILEOBJECT file = WdfRequestGetFileObject(Request);
        if (!file) {
            status = STATUS_INVALID_PARAMETER;
            break;
        }
        status = RkMppJobPeekDense(WdfIoQueueGetDevice(Queue), file,
                                   in->JobId, out);
        if (NT_SUCCESS(status)) info = sizeof(*out);
        break;
    }

    /* ---- WAIT_JOB ---------------------------------------------------- */
    case IOCTL_RKMPP_WAIT_JOB: {
        if (InputBufferLength < sizeof(RKMPP_WAIT_JOB_IN) ||
            OutputBufferLength < sizeof(RKMPP_WAIT_JOB_OUT)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        RKMPP_WAIT_JOB_IN  *in;
        RKMPP_WAIT_JOB_OUT *out;
        status = WdfRequestRetrieveInputBuffer(Request, sizeof(*in),
                                               (PVOID*)&in, NULL);
        if (!NT_SUCCESS(status)) break;
        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                 (PVOID*)&out, NULL);
        if (!NT_SUCCESS(status)) break;
        WDFFILEOBJECT file = WdfRequestGetFileObject(Request);
        if (!file) {
            status = STATUS_INVALID_PARAMETER;
            break;
        }
        status = RkMppJobWait(WdfIoQueueGetDevice(Queue),
                              file, in->JobId, in->TimeoutMs, out);
        if (NT_SUCCESS(status)) info = sizeof(*out);
        break;
    }

    /* ---- INJECT_IOMMU_FAULT ----------------------------------------- */
    case IOCTL_RKMPP_INJECT_IOMMU_FAULT: {
        if (OutputBufferLength < sizeof(RKMPP_FAULT_RESULT)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        RKMPP_FAULT_RESULT *out;
        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                (PVOID*)&out, NULL);
        if (!NT_SUCCESS(status)) break;

        /* Phase 3a scaffolding: the actual fault injection (writing a bad iova
         * to a codec register and asserting the kick bit) requires the real
         * hardware-kick path that lands in Phase 3b.  Until then, this IOCTL
         * just reports whatever fault state has been recorded by the registered
         * callback (typically nothing, unless something else triggered an
         * IOMMU fault — which is itself a useful diagnostic).
         *
         * TODO (Phase 3b): submit a job that programs RKVDEC_DMA_SRC with
         * iova=0xDEADB000 and asserts the start bit, then poll the
         * FaultTriggered flag for up to 100ms. */
        RKMPP_FAULT_STATE fs;
        RkMppGetFaultState(device, &fs);
        out->Triggered = (UINT32)fs.Triggered;
        out->StatusReg = (UINT32)fs.StatusReg;
        out->FaultIova = (UINT64)fs.FaultIova;
        info   = sizeof(*out);
        status = STATUS_SUCCESS;
        break;
    }

    default:
        status = STATUS_INVALID_DEVICE_REQUEST;
        break;
    }

    WdfRequestCompleteWithInformation(Request, status, info);
}
