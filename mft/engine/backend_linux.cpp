/* mft/engine/backend_linux.cpp — DecodeEngineBackend implementation
 * over the BSP /dev/mpp_service surface (mpp_svc.c) and
 * /dev/dma_heap/system-uncached.  Symmetric counterpart to
 * mft/engine/backend_windows.cpp.
 *
 * Ownership model:
 *   - LinuxBackend_New(width, height) allocates a backend value + ctx
 *     and returns a pointer the caller hands to
 *     DecodeEngine_InitWithBackend.
 *   - LinuxBackend_Free releases both (called after Shutdown).
 *
 * Buffer handle convention: on Linux there is no opaque kernel handle —
 * the dma_fd IS the cookie the regbuilder writes into RegWrite entries
 * (via H264BufferRefs).  AllocBuf sets `out->handle = (uint64_t)dma_fd`
 * and `out->dma_fd = dma_fd`; SubmitH264 forwards every known fd to the
 * kernel's session FD tracker (MppSvc_ImportFds) and to the cmd=0x4001
 * BufferHandle table (MppSvc_Submit's buf_map).
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#include "decode_engine_backend.h"
#include "mpp_svc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* DPB pool of 16 + ~8 codec-scratch buffers; pad generously.  Anything
 * higher would mean the MFT layer leaked an alloc — assert in debug. */
#define LB_MAX_BUFS 64

struct LinuxBackendCtx {
    int      svc_fd;
    uint32_t width;
    uint32_t height;

    /* dma-fd registry — ImportFds replays the full set whenever new
     * buffers were added since the last submit (kernel re-imports are
     * idempotent; the cost is small and avoids a per-submit dirty list). */
    int  fds[LB_MAX_BUFS];
    int  n_fds;
    int  fds_dirty;

    /* IRQ readback buffer: kernel copies the codec's IRQ status word
     * here on poll.  Lifetime: until SubmitH264 returns.  Stored in ctx
     * so the kernel pointer stays stable across the submit→poll pair. */
    uint32_t irq_readback[MPP_IRQ_READBACK_WORDS];
};

static void register_fd(LinuxBackendCtx *ctx, int fd) {
    if (ctx->n_fds >= LB_MAX_BUFS) {
        fprintf(stderr, "backend_linux: fd registry full (%d) — bump LB_MAX_BUFS\n",
                LB_MAX_BUFS);
        return;
    }
    ctx->fds[ctx->n_fds++] = fd;
    ctx->fds_dirty = 1;
}

static void unregister_fd(LinuxBackendCtx *ctx, int fd) {
    for (int i = 0; i < ctx->n_fds; i++) {
        if (ctx->fds[i] == fd) {
            ctx->fds[i] = ctx->fds[--ctx->n_fds];
            return;
        }
    }
}

/* ---- vtable functions -------------------------------------------------- */

static int LinuxBe_Open(void *vctx, DecodeEngineCodec codec) {
    LinuxBackendCtx *ctx = (LinuxBackendCtx *)vctx;
    if (codec != DE_CODEC_H264) {
        fprintf(stderr, "backend_linux: only H.264 supported (got codec=%d)\n",
                (int)codec);
        return 1;
    }
    ctx->svc_fd = MppSvc_Open();
    if (ctx->svc_fd < 0) return 1;
    if (MppSvc_SendCodecInfo(ctx->svc_fd, ctx->width, ctx->height) < 0) {
        MppSvc_Close(ctx->svc_fd);
        ctx->svc_fd = -1;
        return 1;
    }
    return 0;
}

static void LinuxBe_Close(void *vctx) {
    LinuxBackendCtx *ctx = (LinuxBackendCtx *)vctx;
    if (ctx->svc_fd >= 0) {
        MppSvc_Close(ctx->svc_fd);
        ctx->svc_fd = -1;
    }
}

static int LinuxBe_AllocBuf(void *vctx, uint32_t size,
                            DecodeEngineBufUsage usage,
                            DecodeEngineBuf *out) {
    (void)usage;   /* dma_heap allocation is uniform for H.264 right now */
    LinuxBackendCtx *ctx = (LinuxBackendCtx *)vctx;
    MppSvcBuf b = {0};
    if (MppSvc_AllocBuf(size, &b) < 0) return 1;

    out->dma_fd  = b.dma_fd;
    out->handle  = (uint64_t)b.dma_fd;   /* fd is the regbuilder cookie */
    out->iova    = 0;                    /* kernel resolves at submit */
    out->user_va = b.cpu_va;
    out->size    = (uint32_t)b.size;

    register_fd(ctx, b.dma_fd);
    return 0;
}

static void LinuxBe_FreeBuf(void *vctx, DecodeEngineBuf *buf) {
    LinuxBackendCtx *ctx = (LinuxBackendCtx *)vctx;
    if (!buf->user_va) return;
    MppSvcBuf b;
    b.dma_fd = buf->dma_fd;
    b.cpu_va = buf->user_va;
    b.size   = buf->size;
    unregister_fd(ctx, buf->dma_fd);
    MppSvc_FreeBuf(&b);
    DecodeEngineBuf zero = {0};
    *buf = zero;
}

/* Convert a dense-bank output back to the sparse RKMPP_REG_WRITE list
 * the BSP MppSvc surface expects.  The BSP rkvdec driver kernel side
 * already walks every reg in [s..e] for each bank request (mpp_write_req,
 * mpp_rkvdec2.c:359) and writes zeros for unset slots, so we only need
 * to emit non-zero values + the iova-substitution slots here. */
static uint32_t dense_to_sparse(const H26xDenseOutput *in,
                                H264RegWriteList *list) {
    list->count = 0;

    /* Bitmap of register indices that are filled by an iova slot — skip
     * them in the plain-walk so we don't double-emit. */
    uint32_t iova_idx_set[10] = {0};   /* covers idx 0..319 */
    for (uint32_t i = 0; i < in->IovaSlotCount; i++) {
        uint32_t idx = in->IovaSlots[i].RegIdx;
        if (idx < 320) iova_idx_set[idx >> 5] |= (1u << (idx & 31));
    }

    auto add_plain = [&](uint32_t idx, uint32_t val) {
        if (list->count >= RKMPP_MAX_REG_WRITES) return;
        RKMPP_REG_WRITE *w = &list->entries[list->count++];
        w->Offset       = idx * 4u;
        w->Value        = val;
        w->BufferHandle = 0;
        w->IovaOffset   = 0;
        w->Reserved     = 0;
    };
    auto walk = [&](uint32_t first, uint32_t n, const uint32_t *src) {
        for (uint32_t k = 0; k < n; k++) {
            uint32_t idx = first + k;
            if (iova_idx_set[idx >> 5] & (1u << (idx & 31))) continue;
            if (src[k] == 0) continue;
            add_plain(idx, src[k]);
        }
    };
    walk(RKMPP_DENSE_COMMON_FIRST,  RKMPP_DENSE_COMMON_WORDS,
         in->Bank.Common);
    walk(RKMPP_DENSE_CPARAM_FIRST,  RKMPP_DENSE_CPARAM_WORDS,
         in->Bank.CodecParams);
    walk(RKMPP_DENSE_CADDR_FIRST,   RKMPP_DENSE_CADDR_WORDS,
         in->Bank.CommonAddr);
    walk(RKMPP_DENSE_CODADDR_FIRST, RKMPP_DENSE_CODADDR_WORDS,
         in->Bank.CodecAddr);
    walk(RKMPP_DENSE_HIPOC_FIRST,   RKMPP_DENSE_HIPOC_WORDS,
         in->Bank.HighPoc);
    walk(RKMPP_DENSE_STAT_FIRST,    RKMPP_DENSE_STAT_WORDS,
         in->Bank.Stat);

    /* iova-substitution entries: kernel resolves BufferHandle → iova at
     * submit time and stamps into the register list before the kick. */
    for (uint32_t i = 0; i < in->IovaSlotCount; i++) {
        if (list->count >= RKMPP_MAX_REG_WRITES) break;
        RKMPP_REG_WRITE *w = &list->entries[list->count++];
        w->Offset       = in->IovaSlots[i].RegIdx * 4u;
        w->Value        = 0;
        w->BufferHandle = in->IovaSlots[i].BufferHandle;
        w->IovaOffset   = in->IovaSlots[i].IovaOffset;
        w->Reserved     = 0;
    }

    /* Append the kick last, mirroring the sparse path's ordering. */
    if (in->KickValue != 0) {
        add_plain(RKMPP_DENSE_KICK_REG_IDX, in->KickValue);
    }
    return list->count;
}

static int LinuxBe_SubmitDense(void *vctx, const H26xDenseOutput *in,
                               uint32_t timeout_ms, uint32_t *hw_status) {
    LinuxBackendCtx *ctx = (LinuxBackendCtx *)vctx;

    if (ctx->fds_dirty) {
        if (MppSvc_ImportFds(ctx->svc_fd, ctx->fds, ctx->n_fds) < 0)
            return 1;
        ctx->fds_dirty = 0;
    }

    /* Build buf_map from every registered fd (handle = fd).  Sending
     * unused fds is harmless — the kernel just ignores them when no
     * register references them. */
    MppSvcBufMap buf_map[LB_MAX_BUFS];
    for (int i = 0; i < ctx->n_fds; i++) {
        buf_map[i].handle = (uint64_t)ctx->fds[i];
        buf_map[i].dma_fd = ctx->fds[i];
    }

    H264RegWriteList list;
    dense_to_sparse(in, &list);

    memset(ctx->irq_readback, 0, sizeof(ctx->irq_readback));
    if (MppSvc_Submit(ctx->svc_fd, &list, buf_map, ctx->n_fds,
                      ctx->irq_readback, ctx->width, ctx->height) < 0)
        return 1;

    int rc = MppSvc_Poll(ctx->svc_fd, timeout_ms, ctx->irq_readback);
    if (hw_status) *hw_status = ctx->irq_readback[0];
    /* MppSvc_Poll returns 0 on RDY, 1 on timeout, -1 on error.  Caller
     * (decode_engine.cpp) interprets non-zero rc + RDY-bit in hw_status
     * the same way the Windows backend reports wout.Status. */
    return rc;
}

/* ---- factory ---------------------------------------------------------- */

extern "C" DecodeEngineBackend *LinuxBackend_New(uint32_t width, uint32_t height)
{
    LinuxBackendCtx *ctx = (LinuxBackendCtx *)calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;
    ctx->svc_fd = -1;
    ctx->width  = width;
    ctx->height = height;

    DecodeEngineBackend *be =
        (DecodeEngineBackend *)calloc(1, sizeof(*be));
    if (!be) { free(ctx); return NULL; }

    be->ctx        = ctx;
    be->Open       = LinuxBe_Open;
    be->Close      = LinuxBe_Close;
    be->AllocBuf   = LinuxBe_AllocBuf;
    be->FreeBuf    = LinuxBe_FreeBuf;
    be->SubmitH264 = LinuxBe_SubmitH264;
    return be;
}

extern "C" void LinuxBackend_Free(DecodeEngineBackend *be)
{
    if (!be) return;
    free(be->ctx);
    free(be);
}
