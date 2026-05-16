/* mpp_svc.h — Linux mpp_service + dma_heap backend */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* Include Windows shims before any mft header */
#include "winshim.h"
#include "mft/regbuilder_h264.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One DMA-buf allocation: heap fd + cpu mmap + size */
typedef struct {
    int    dma_fd;
    void  *cpu_va;
    size_t size;
} MppSvcBuf;

/* Maps an RKMPP_REG_WRITE BufferHandle cookie to the corresponding dma_fd.
 * Caller builds this table from the same handles passed to H264BuildRegisterList. */
typedef struct {
    uint64_t handle;
    int      dma_fd;
} MppSvcBufMap;

/* mpp_service wire protocol structs and constants */
#include <sys/ioctl.h>
#include <linux/ioctl.h>

struct MppReqV1 {
    uint32_t cmd;
    uint32_t flag;
    uint32_t size;
    uint32_t offset;
    uint64_t data_ptr;
};

/* IOCTL — from mpp_common.c: MPP_IOC_CFG_V1 = _IOW('v', 1, unsigned int) */
#define MPP_IOC_CFG_V1          _IOW('v', 1, unsigned int)

/* Command codes — from BSP mpp_common.h */
#define MPP_CMD_INIT_CLIENT_TYPE    0x100
#define MPP_CMD_SET_REG_WRITE       0x200
#define MPP_CMD_SET_REG_READ        0x201
#define MPP_CMD_SET_REG_ADDR_OFFSET 0x202
#define MPP_CMD_SET_RCB_INFO        0x203
#define MPP_CMD_POLL_HW_FINISH      0x300
#define MPP_CMD_TRANS_FD_TO_IOVA    0x401

/* Client type — from mpp_dev_defs.h: VPU_CLIENT_RKVDEC = 9 */
#define VPU_CLIENT_RKVDEC           9

/* Protocol flags — from mpp_common.h */
#define MPP_FLAGS_MULTI_MSG         0x00000001
#define MPP_FLAGS_LAST_MSG          0x00000002
#define MPP_FLAGS_REG_OFFSET_ALONE  0x00000010  /* same value as MPP_FLAGS_REG_NO_OFFSET */

/*
 * IRQ readback buffer size (words).
 * The kernel copies RKVDEC2 hardware register range [896..952) to the
 * buffer passed as data_ptr in SET_REG_READ.  readback[0] is the
 * IRQ status word (RKVDEC_REG_INT_EN, index 224).
 */
#define MPP_IRQ_READBACK_WORDS  (56 / 4)

/* MPP_CMD_SEND_CODEC_INFO = MPP_CMD_CONTROL_BASE + 3 = 0x403.
 * Must be sent once per session (before the first decode) so the kernel sets
 * task->pixels = width * height, which gates CLK_MODE_ADVANCED selection.
 * Without it the clock stays at CLK_MODE_NORMAL, which is too slow for
 * 1920×1080 and the hardware watchdog fires (irq=0x23 TIMEOUT_STA). */
#define MPP_CMD_SEND_CODEC_INFO     0x403

/* Open /dev/mpp_service and initialize the session for RKVDEC H.264 decode.
 * Returns a valid fd on success, -1 on error (with perror output). */
int  MppSvc_Open(void);
void MppSvc_Close(int svc_fd);

/*
 * Send MPP_CMD_SEND_CODEC_INFO to the session so the kernel knows the
 * frame dimensions.  Call once after MppSvc_Open, before the first Submit.
 *
 * Payload: 3 × codec_info_elem {type u32, flag u32, data u64}:
 *   {1, 1, width}   DEC_INFO_WIDTH  / CODEC_INFO_FLAG_NUMBER
 *   {2, 1, height}  DEC_INFO_HEIGHT / CODEC_INFO_FLAG_NUMBER
 *   {3, 2, "h264"}  DEC_INFO_FORMAT / CODEC_INFO_FLAG_STRING
 */
int  MppSvc_SendCodecInfo(int svc_fd, uint32_t width, uint32_t height);

/*
 * Import DMA-buf fds into the session's IOMMU DMA tracker so that
 * mpp_translate_reg_address can translate (IovaOffset<<10)|fd register words.
 * fds[] is an array of n dma_fd values; after the call the kernel has mapped
 * each fd into the session and will resolve them during SET_REG_WRITE processing.
 * Returns 0 on success, -1 on error.
 */
int  MppSvc_ImportFds(int svc_fd, const int *fds, int n);

/* Allocate an uncached DMA-buf via /dev/dma_heap/system-uncached.
 * cpu_va is mmap'd PROT_READ|PROT_WRITE for CPU access.
 * Returns 0 on success, -1 on error. */
int  MppSvc_AllocBuf(size_t size, MppSvcBuf *out);
void MppSvc_FreeBuf(MppSvcBuf *b);

/*
 * Build MppReqV1 messages from rl and submit in ONE MPP_IOC_CFG_V1 ioctl.
 *
 * IOVA entries (BufferHandle != 0) are encoded as (IovaOffset << 10) | dma_fd
 * in the register word.  The kernel's built-in trans_tbl_h264d handles
 * fd → IOVA translation.
 *
 * irq_readback: caller-allocated uint32_t[MPP_IRQ_READBACK_WORDS].
 *   The kernel will copy IRQ status registers here after HW finishes.
 *   Must remain valid until MppSvc_Poll returns.
 *
 * Returns 0 on success, -1 on error.
 */
int  MppSvc_Submit(int svc_fd, const H264RegWriteList *rl,
                   const MppSvcBufMap *buf_map, int n_bufs,
                   uint32_t *irq_readback,
                   uint32_t width, uint32_t height);

/*
 * Wait for hardware completion via MPP_IOC_CFG_V1 + MPP_CMD_POLL_HW_FINISH.
 * After this returns, irq_readback[0] holds the hardware IRQ status word.
 * Returns 0 = success (RDY bit set), 1 = timeout, -1 = error.
 */
int  MppSvc_Poll(int svc_fd, uint32_t timeout_ms, const uint32_t *irq_readback);

#ifdef __cplusplus
}  /* extern "C" */
#endif
