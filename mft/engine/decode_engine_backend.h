/* mft/engine/decode_engine_backend.h
 *
 * Pluggable backend interface for DecodeEngine — abstracts the
 * kernel/device boundary so the same mft/engine + parser_glue + dpb +
 * regbuilder + packed_tables stack runs on:
 *   - Windows  (rkmpp.sys IOCTL_RKMPP_*; backend_windows.cpp)
 *   - Linux    (/dev/mpp_service ioctl;  backend_linux.cpp)
 *
 * Scope (decode-engine-backend-split.md):
 *   - All codec Init paths route AllocBuf/FreeBuf through this vtable.
 *   - Only the H.264 Submit path is abstracted.  H.265 and AV1 retain
 *     direct DeviceIoControl calls in decode_engine{,_av1}.cpp; their
 *     backend split is a future scope.
 *
 * Linkage: C so a future Linux backend or a `.c` consumer (the existing
 * linux_mpp_decode harness) can include this header without a C++ ABI
 * dependency.
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* H26xDenseOutput: brought in by regbuilder_dense.h (C-linkage). */
#include "../regbuilder_dense.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Plain-C buffer descriptor populated by AllocBuf and consumed by Submit
 * + the engine's user-space packers.  All backends fill `handle` (which
 * round-trips back into Submit's RegWrite entries via BufferHandle), the
 * mapped `user_va`, and the rounded-up `size`.  `iova` is informational
 * (Windows kernel reports it; Linux can leave 0).  `dma_fd` is set on
 * Linux backends (-1 / 0 on Windows) so the Linux Submit can build the
 * cmd=0x4001 BufferHandle table the BSP service expects. */
typedef struct DecodeEngineBuf {
    uint64_t handle;
    uint64_t iova;
    void    *user_va;
    uint32_t size;
    int      dma_fd;
} DecodeEngineBuf;

typedef enum DecodeEngineBufUsage {
    DE_BUF_BITSTREAM_INPUT = 1,
    DE_BUF_REFERENCE_FRAME = 2,
    DE_BUF_OUTPUT_FRAME    = 3,
    DE_BUF_SCRATCH         = 4,
} DecodeEngineBufUsage;

/* Codec enum mirror — kept in sync with `enum class Codec` in
 * decode_engine.h.  Plain int field on the vtable so C consumers don't
 * need the C++ header. */
typedef enum DecodeEngineCodec {
    DE_CODEC_H264 = 0,
    DE_CODEC_H265 = 1,
} DecodeEngineCodec;

typedef struct DecodeEngineBackend {
    /* Per-instance state owned by the backend.  Engine treats as opaque. */
    void *ctx;

    /* Open the per-codec device + verify capability bit.  Returns 0 on
     * success; non-zero leaves the engine to fail Init.  Backend may
     * carry the resulting device handle inside ctx. */
    int  (*Open)(void *ctx, DecodeEngineCodec codec);

    /* Release device + any per-instance resources.  Idempotent. */
    void (*Close)(void *ctx);

    /* Allocate / free a DMA-coherent buffer mapped into the calling
     * process.  AllocBuf returns 0 on success and fills `*out`; non-zero
     * on failure (engine prints diagnostics).  FreeBuf is a no-op on
     * already-freed buffers (handle == 0). */
    int  (*AllocBuf)(void *ctx, uint32_t size, DecodeEngineBufUsage usage,
                     DecodeEngineBuf *out);
    void (*FreeBuf )(void *ctx, DecodeEngineBuf *buf);

    /* Submit one rkvdec2 decode kick + wait for completion (synchronous).
     * `out` is the dense-bank output from H264BuildDenseRegs (or
     * H265BuildDenseRegs).  Used for both H.264 and H.265 — the bank
     * layout is identical at the vdpu34x level, only the codec-param
     * and codec-addr bank contents differ.
     *
     * Returns 0 if the wait completed without a hard error; *hw_status
     * receives the codec's HardwareStatus register (caller checks the
     * RDY bit to determine whether output is available).  Non-zero
     * returns mean the IOCTL itself failed. */
    int  (*SubmitDense)(void *ctx,
                        const H26xDenseOutput *out,
                        uint32_t timeout_ms,
                        uint32_t *hw_status);
} DecodeEngineBackend;

/* Default Windows backend — wraps rkmpp.sys IOCTLs.  Per-engine value
 * so each DecodeEngine can hold its own device handle (the H.265 path
 * still reaches into the handle directly via its non-vtable submit).
 *
 * `device_storage` is a pointer to a `HANDLE`-sized slot owned by the
 * engine; Open writes the opened device into it, AllocBuf/FreeBuf/
 * SubmitH264 read it back.  Letting the engine own the slot means the
 * H.265 / AV1 paths that haven't been ported to the vtable yet can
 * still call DeviceIoControl(e->device, ...) directly. */
void WindowsBackend_Init(DecodeEngineBackend *out, void *device_storage);

#ifdef __cplusplus
}  /* extern "C" */
#endif
