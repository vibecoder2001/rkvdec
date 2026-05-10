# Linux mpp_service H.264 Decode Harness — Design Spec

**Date:** 2026-05-09  
**Goal:** Bisect open H.264 bugs (row-556 IDR divergence, P-frame output leak) by running our
user-mode pipeline (parser_glue + regbuilder_h264 + h264_packed_tables + dpb) against the BSP
kernel driver on RK3588 Linux. If decoded pixels match ffmpeg CPU output on Linux/BSP, bugs are
in `rkmpp.sys` (kernel/IOMMU/buffer-cache); if the same corruption appears, bugs are in our
user-mode code.

H.264 only. H.265 and AV1 work in Windows and are out of scope.

---

## 1. Files

```
tests/harness/linux_mpp_decode/
  mpp_svc.h              — DMA-buf alloc + mpp_service submit/poll API
  mpp_svc.c              — platform backend (dma_heap + /dev/mpp_service IOCTLs)
  linux_mpp_decode.c     — main: AU walker + parser/DPB/regbuilder + output dump
  winshim.h              — Windows-type aliases (uint32_t etc.) for mft headers
  Makefile               — builds on rk ARM64 with gcc/g++
```

No changes to `mft/` or `shared/`. All existing user-mode code is consumed as-is.

---

## 2. mpp_svc API

```c
/* One DMA-buf allocation: heap fd + cpu mmap + size. */
typedef struct {
    int    dma_fd;
    void  *cpu_va;
    size_t size;
} MppSvcBuf;

/* Open /dev/mpp_service and send MPP_CMD_INIT_CLIENT_TYPE for H.264 decoder.
 * Returns session fd on success, -1 on error. */
int  MppSvc_Open(void);
void MppSvc_Close(int svc_fd);

/* Allocate an uncached DMA-buf via /dev/dma_heap/system-uncached.
 * cpu_va is mmap'd RW for CPU uploads (bitstream, packed tables). */
int  MppSvc_AllocBuf(size_t size, MppSvcBuf *out);
void MppSvc_FreeBuf(MppSvcBuf *b);

/* Buffer-handle → dma_fd mapping table for MppSvc_Submit. */
typedef struct { uint64_t handle; int dma_fd; } MppSvcBufMap;

/* Translate H264RegWriteList into MppReqV1 array and call MPP_IOC_SET_REQ.
 *
 * Entries where BufferHandle == 0  → cmd=0x200 (direct reg write)
 * Entries where BufferHandle != 0  → cmd=0x202 (IOVA substitution pair)
 *   One cmd=0x202 sub-message per distinct BufferHandle; MppReqV1.offset
 *   is set to the dma_fd from buf_map for that handle; payload is an
 *   array of (reg_index_u32, byte_offset) pairs.
 *
 * This matches the wire format the BSP mpp_service kernel driver expects
 * and is identical to what the BSP shim log captures. */
int  MppSvc_Submit(int svc_fd, const H264RegWriteList *rl,
                   const MppSvcBufMap *buf_map, int n_bufs);

/* Poll for completion via MPP_IOC_GET_REQ / MPP_CMD_POLL_HW_FINISH.
 * Returns 0 = success, 1 = timeout, -1 = error.
 * *hw_status is the hardware status register value on return. */
int  MppSvc_Poll(int svc_fd, uint32_t timeout_ms, uint32_t *hw_status);
```

### Protocol constants

Confirm exact values from both sides during implementation:
- `~/linux-rockchip/drivers/video/rockchip/mpp/` — kernel driver, IOCTL numbers, cmd enum
- `~/mpp/osal/driver/` or `~/mpp/mpp/hal/rkdec/` — userspace mpp library, same wire format

Expected:
- `MPP_IOC_SET_REQ` = `_IOW('M', 0, struct MppReqV1)`
- `MPP_IOC_GET_REQ` = `_IOWR('M', 1, struct MppReqV1)`
- `MPP_CMD_INIT_CLIENT_TYPE` = 0x100
- `MPP_CMD_SET_REG_WRITE`    = 0x200
- `MPP_CMD_SET_REG_READ`     = 0x201
- `MPP_CMD_SET_REG_ADDR_OFFSET` = 0x202
- `MPP_CMD_POLL_HW_FINISH`   = 0x300

---

## 3. Buffer pool

Allocated once at init, persistent across all AUs (same layout as Windows `DecodeEngine`):

| Buffer        | Size                            | Notes |
|---------------|---------------------------------|-------|
| cabac_init    | 3712 B                          | uploaded once at init |
| pps_table     | `RKH264_SPSPPS_UNIT_SIZE`       | overwritten each AU (or cached) |
| rps_table     | `RKH264_RPS_SIZE` (384 B)       | overwritten each AU |
| rcb           | from `H264GetRcbBufferSizes()`  | scratch, never read by CPU |
| error_ref     | 1 × frame_size                  | fallback ref, zero-filled |
| pool_output   | 16 × frame_size                 | DPB output slots |
| pool_colmv    | 16 × colmv_size                 | per-slot colmv |
| bitstream     | 2 MiB                           | refilled per AU |

`frame_size = width_aligned × height_aligned × 3/2` (NV12).  
`colmv_size = (W/16) × (H/16) × 16` bytes (one vector per macroblock).

---

## 4. Main harness flow

CLI: `linux_mpp_decode <stream.h264> <width> <height> [out.yuv]`

```
Init:
  MppSvc_Open()
  allocate persistent buffers
  upload cabac_init table

Per-AU loop:
  h264_au_next()                           — find next slice AU
  H264ParseAccessUnit()
  Dpb_Select()                             — pick output slot, build ref lists
  memcpy AU bytes → bitstream.cpu_va
  H264PackFrameRps()  → rps_table.cpu_va
  H264PackSpsPpsUnit() → pps_table.cpu_va  — skip if SPS/PPS unchanged
  H264BuildRegisterList()                  → H264RegWriteList
  MppSvc_Submit() + MppSvc_Poll()
  if hw_status == 0:
      fwrite(pool_output[slot].cpu_va, frame_size, 1, out_fp)
  Dpb_OnDecodeComplete()

Shutdown:
  free all buffers
  MppSvc_Close()
```

**Output order:** decode (bitstream) order. No reorder queue in Part 1.

---

## 5. Part 2 — display-order reorder (deferred)

After Part 1 confirms pixel correctness for non-B-frame content, Part 2 ports
`DecodeEngine_OnDecodeComplete` + `reorder_q` / `ready_q` to emit frames in POC order.
This lets us rule out the reorder queue itself contributing to the B-frame corruption seen
on Windows.

---

## 6. Makefile

```makefile
CXX      = g++
CC       = gcc
CFLAGS   = -O2 -Wall -I../../.. -I../../../mft
CXXFLAGS = $(CFLAGS) -std=c++17
MFT      = ../../..

OBJS_CXX = parser_glue.o regbuilder_h264.o h264_packed_tables.o dpb.o
OBJS_C   = linux_mpp_decode.o mpp_svc.o

parser_glue.o:       $(MFT)/mft/parser_glue.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
regbuilder_h264.o:   $(MFT)/mft/regbuilder_h264.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
h264_packed_tables.o: $(MFT)/mft/h264_packed_tables.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
dpb.o:               $(MFT)/mft/dpb.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

linux_mpp_decode: $(OBJS_CXX) $(OBJS_C)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f linux_mpp_decode *.o
```

`winshim.h` re-aliases Windows integer typedefs (`UINT32 → uint32_t`, etc.) and any
`#define` guards needed by mft headers. Pattern matches existing harnesses.

---

## 7. Success criteria

Primary reference: **ffmpeg hardware decode via BSP rkmpp stack** on the same Linux box.
Both paths hit the same kernel driver and the same hardware, so any divergence between
our harness output and ffmpeg-rkmpp output is isolated to user-mode code only
(our regbuilder/packed-tables/DPB vs librockchip-mpp's).

```
ffmpeg -vcodec h264_rkmpp -i dancing_nobf.h264 -pix_fmt nv12 ref_hw.yuv
```

Secondary reference: ffmpeg CPU software decode (`-vcodec h264`, libavcodec) as a
sanity check that the hardware itself is producing correct pixels.

```
ffmpeg -vcodec h264 -i dancing_nobf.h264 -pix_fmt nv12 ref_sw.yuv
```

- **Part 1 (non-B):** `linux_mpp_decode dancing_nobf.h264 W H out.yuv` matches
  `ref_hw.yuv` byte-for-byte → our user-mode pipeline is correct; any Windows divergence
  is in `rkmpp.sys`.
  If it diverges from `ref_hw.yuv` but matches the Windows output → bug is in our
  user-mode code, same on both platforms.
- **Part 1 (B-frame):** Same comparison with `dancing.h264`; result definitively
  assigns B-frame corruption to user-mode or kernel/driver.
- **Part 2:** With reorder enabled, frame order in `out.yuv` matches ffmpeg display order.
