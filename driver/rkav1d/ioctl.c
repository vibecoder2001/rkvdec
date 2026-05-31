/* driver/rkav1d/ioctl.c — IOCTL surface for rkav1d.sys.
 * Phase 1: IOCTL_RKMPP_GET_CAPS.
 * Phase 2: IOCTL_RKMPP_ALLOC_BUFFER, IOCTL_RKMPP_FREE_BUFFER,
 *           IOCTL_RKMPP_SUBMIT_JOB, IOCTL_RKMPP_WAIT_JOB.
 */
#include <ntddk.h>
#include <wdf.h>

#include "../../shared/rkmpp_ioctl.h"
#include "../../shared/rkiommu_ifc.h"
#include "devpub.h"
#include "../shared/rkmpp/bufpool.h"
#include "job.h"

/* Provided by device.c */
extern void RkMppGetPublic(_In_ WDFDEVICE Device, _Out_ RKMPP_DEVICE_PUBLIC *Out);
extern void RkMppGetFaultState(_In_ WDFDEVICE Device, _Out_ RKMPP_FAULT_STATE *Out);
extern PRKIOMMU_INTERFACE RkMppGetIommuIfc(_In_ WDFDEVICE Device);

/* AV1 IOMMU-attach gate mirroring rkvdec/device.c::RkMppIsIommuAttached.
 * rkav1d's device context doesn't track an explicit IommuAttached flag,
 * but the rkiommu_av1d provider only fills the interface (MapMdl, etc.)
 * after attach completes — so a NULL Header.Context (or NULL MapMdl)
 * means we haven't bound yet.  See review finding #5. */
static BOOLEAN RkAv1dIsIommuAttached(_In_ WDFDEVICE Device)
{
    PRKIOMMU_INTERFACE i = RkMppGetIommuIfc(Device);
    return (i != NULL) && (i->Header.Context != NULL) && (i->MapMdl != NULL);
}

EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL RkMppEvtIoDeviceControl;

/* Predicate bridge for RkMppBufFreeIfNotInUse — see rkvdec/ioctl.c. */
typedef struct _RKMPP_FREE_INUSE_CTX {
    WDFDEVICE     Device;
    WDFFILEOBJECT File;
} RKMPP_FREE_INUSE_CTX;

static BOOLEAN
RkMppFreeBufInUseCb(_In_ PVOID Ctx, _In_ UINT64 Cookie)
{
    RKMPP_FREE_INUSE_CTX *c = (RKMPP_FREE_INUSE_CTX *)Ctx;
    return RkMppJobBufferInUse(c->Device, c->File, Cookie);
}

NTSTATUS RkMppQueueInit(_In_ WDFDEVICE Device)
{
    /* Parallel dispatch: required for concurrent AV1 decode sessions on
     * the same engine.  Sequential mode serialised ALL IRPs across all
     * File handles, so stream 1's blocking WAIT_JOB would monopolise the
     * queue and stall every IOCTL from stream 2 — even its initial
     * GET_CAPS / ALLOC_BUFFER calls.
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

    /* Refuse IOCTLs that require IOMMU before the rkiommu_av1d provider
     * has attached.  GET_CAPS is exempt (read-only device-context fields).
     * Mirrors rkvdec/ioctl.c.  See review finding #5. */
    if (IoControlCode != IOCTL_RKMPP_GET_CAPS &&
        !RkAv1dIsIommuAttached(device)) {
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

        /* Upper-bound StructSize too — see rkvdec/ioctl.c for the
         * rationale.  Review I1. */
        if (in->StructSize != sizeof(*in)) {
            status = (in->StructSize > sizeof(*in))
                     ? STATUS_REVISION_MISMATCH
                     : STATUS_INVALID_PARAMETER;
            break;
        }

        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(*out),
                                                (PVOID*)&out, NULL);
        if (!NT_SUCCESS(status)) break;

        /* Initialise the per-file context on first use.  Race-safe via
         * sentinel-pointer election; see review finding #2 and
         * rkvdec/ioctl.c for the matching pattern. */
        WDFFILEOBJECT file = WdfRequestGetFileObject(Request);
        PRKMPP_FILE_CTX fctx = RkMppFileGet(file);
        if (!fctx->Device) {
            PVOID sentinel = (PVOID)(LONG_PTR)-1;
            PVOID prev = InterlockedCompareExchangePointer(
                (PVOID volatile *)&fctx->Device, sentinel, NULL);
            if (prev == NULL) {
                RkMppBufFileCtxInit(file, device);
                InterlockedExchangePointer((PVOID volatile *)&fctx->Device,
                                           device);
            } else {
                while (fctx->Device == sentinel) {
                    KeStallExecutionProcessor(1);
                }
            }
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

        /* Reject cookie 0 sentinel.  Review I10. */
        if (in->BufferHandle == 0) {
            status = STATUS_INVALID_PARAMETER;
            break;
        }

        /* Atomic in-use check + remove-from-list in one file-lock hold,
         * closing the window a concurrent SubmitJob could slip through.
         * See rkvdec/ioctl.c for the full rationale (review finding #3). */
        RKMPP_FREE_INUSE_CTX inUseCtx;
        inUseCtx.Device = device;
        inUseCtx.File   = file;
        status = RkMppBufFreeIfNotInUse(file, in->BufferHandle,
                                        RkMppFreeBufInUseCb, &inUseCtx);
        break;
    }

    /* ---- SUBMIT_JOB -------------------------------------------------- */
    case IOCTL_RKMPP_SUBMIT_JOB: {
        if (InputBufferLength < sizeof(RKMPP_SUBMIT_JOB_IN) ||
            OutputBufferLength < sizeof(RKMPP_SUBMIT_JOB_OUT)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        RKMPP_SUBMIT_JOB_IN  *in;
        RKMPP_SUBMIT_JOB_OUT *out;
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
        status = RkMppJobSubmit(WdfIoQueueGetDevice(Queue), file, in, out);
        if (NT_SUCCESS(status)) info = sizeof(*out);
        break;
    }

    /* ---- PEEK_JOB ---------------------------------------------------- */
    case IOCTL_RKMPP_PEEK_JOB: {
        if (InputBufferLength < sizeof(RKMPP_PEEK_JOB_IN) ||
            OutputBufferLength < sizeof(RKMPP_PEEK_JOB_OUT)) {
            status = STATUS_BUFFER_TOO_SMALL;
            break;
        }
        RKMPP_PEEK_JOB_IN  *in;
        RKMPP_PEEK_JOB_OUT *out;
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
        status = RkMppJobPeek(WdfIoQueueGetDevice(Queue), file, in->JobId, out);
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
