# Linux mpp_service H.264 Decode Harness — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `tests/harness/linux_mpp_decode/` — a Linux harness that runs our user-mode H.264 pipeline (parser_glue + regbuilder_h264 + h264_packed_tables + dpb) against `/dev/mpp_service` on RK3588, producing NV12 output to compare against ffmpeg-rkmpp.

**Architecture:** A thin `mpp_svc.c` backend replaces the Windows IOCTL layer: it allocates DMA-bufs via `/dev/dma_heap/system-uncached`, translates `H264RegWriteList` entries into `MppReqV1` sub-messages (cmd=0x200 for plain writes, cmd=0x202 for IOVA substitution), and submits to `/dev/mpp_service`. The main harness `linux_mpp_decode.c` drives the same parser→DPB→regbuilder pipeline as `winreplay_h264_diff.c` but hits real hardware. All work is on `rk` (the ARM64 Linux host at 192.168.0.180, user `vibecoder`).

**Tech Stack:** gcc/g++ on ARM64 Linux, `/dev/mpp_service` (BSP mpp_service kernel driver), `/dev/dma_heap/system-uncached`, mft C++ sources (parser_glue, regbuilder_h264, h264_packed_tables, dpb).

**Reference streams:** `tests/data/` on the Windows dev box; scp them to `~/streams/` on rk before starting.

---

## File Map

```
tests/harness/linux_mpp_decode/
  devioctl.h          — stub for Windows devioctl.h (prevents build failure)
  winshim.h           — Windows integer typedef aliases for mft headers
  mpp_svc.h           — DMA-buf + mpp_service public API
  mpp_svc.c           — backend implementation
  linux_mpp_decode.c  — main: AU walker + H.264 pipeline + NV12 dump
  Makefile            — builds on rk ARM64
```

No changes to `mft/`, `shared/`, or any existing file.

---

## Register bank layout (from `winreplay_h264_diff.c` kBanks[])

The rkvdec2 H.264 register file is grouped into these banks. Use this table for both
`cmd=0x200` writes and `cmd=0x201` readback:

| Bank | cmd   | MppReqV1.offset | size (B) | reg range |
|------|-------|-----------------|----------|-----------|
| 0    | 0x200 | 32              | 100      | 8..32     |
| 1    | 0x200 | 256             | 196      | 64..112   |
| 2    | 0x200 | 512             | 60       | 128..142  |
| 3    | 0x200 | 640             | 152      | 160..197  |
| 4    | 0x200 | 800             | 20       | 200..204  |
| 5    | 0x200 | 1024            | 88       | 256..277  |
| IRQ  | 0x201 | 896             | 56       | 224..237  |

`RKVDEC2_REG_INT_EN = 224 * 4 = 896`. Success = bit 2 set (`RKVDEC2_INT_DEC_RDY_STA`).
Error = bits 4/5 set (ERROR_STA / TIMEOUT_STA). From `mft/rkvdec2_h264_regs.h`.

---

## Task 1: Research mpp_service protocol from BSP source on rk

**Files:**
- Read: `~/linux-rockchip/drivers/video/rockchip/mpp/mpp_service.h`
- Read: `~/linux-rockchip/drivers/video/rockchip/mpp/mpp_cmd.h` (or `mpp_dev_common.h`)
- Read: `~/mpp/osal/driver/mpp_device.c`
- Create: `tests/harness/linux_mpp_decode/mpp_protocol_notes.txt` (scratch, not committed)

- [ ] **Step 1: On rk, find the MppReqV1 struct and IOCTL numbers**

```bash
grep -rn "MppReqV1\|MPP_IOC_SET_REQ\|MPP_IOC_GET_REQ\|MPP_IOC_BASE" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ | head -40
grep -rn "MppReqV1\|MPP_IOC_SET_REQ\|MPP_IOC_GET_REQ" \
    ~/mpp/osal/driver/ | head -40
```

Expected: find struct with `cmd, flag, size, offset, data_ptr` fields and the `_IOW`/`_IOWR` ioctl numbers.

- [ ] **Step 2: Find MPP_CMD_* constants**

```bash
grep -rn "MPP_CMD_\|0x100\|0x200\|0x201\|0x202\|0x300\|INIT_CLIENT" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ | grep -i "define\|enum" | head -40
```

Expected: confirm `MPP_CMD_INIT_CLIENT_TYPE=0x100`, `MPP_CMD_SET_REG_WRITE=0x200`,
`MPP_CMD_SET_REG_ADDR_OFFSET=0x202`, `MPP_CMD_POLL_HW_FINISH=0x300`.

- [ ] **Step 3: Confirm INIT_CLIENT_TYPE payload for RKVDEC2 H.264**

```bash
grep -rn "VPU_CLIENT_RKVDEC2\|RKVDEC2\|h264\|VpuClientType\|MppClientType\|CLIENT_TYPE" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ | head -30
grep -rn "VPU_CLIENT_RKVDEC2\|RKVDEC2\|VpuClientType" ~/mpp/osal/ | head -20
```

Expected: find `VPU_CLIENT_RKVDEC2 = 6` (or similar enum value). Record the value.

- [ ] **Step 4: Confirm cmd=0x202 payload format (reg_idx, byte_offset u32 pairs)**

```bash
grep -rn "0x202\|ADDR_OFFSET\|reg_idx\|trans_tbl\|iova.*offset\|offset.*iova" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ | head -30
```

Expected: confirm each pair is `(uint32_t reg_word_index, uint32_t byte_offset)` and
`MppReqV1.offset` for cmd=0x202 contains the DMA-buf fd number.

- [ ] **Step 5: Confirm cmd=0x202 MppReqV1.offset = dma_fd**

```bash
grep -rn "dma_buf\|fd.*offset\|offset.*fd\|import.*fd" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ | head -20
```

- [ ] **Step 6: Find the MPP_FLAGS_LAST_CMD flag value**

```bash
grep -rn "LAST_CMD\|MPP_FLAGS\|0x0001\|last.*flag" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ | head -20
grep -rn "LAST_CMD\|MPP_FLAGS" ~/mpp/osal/driver/ | head -20
```

- [ ] **Step 7: Confirm MPP_IOC_GET_REQ blocks until hardware finish**

```bash
grep -rn "GET_REQ\|poll_hw\|wait.*finish\|ioctl.*get" \
    ~/linux-rockchip/drivers/video/rockchip/mpp/ \
    ~/mpp/osal/driver/ | head -30
```

- [ ] **Step 8: Record findings in scratch notes**

Write `mpp_protocol_notes.txt` with the confirmed values. Anything that differs from
the expected values in this plan must be corrected before Task 4.

---

## Task 2: Scaffold — devioctl.h stub + winshim.h + Makefile + empty stubs

**Files:**
- Create: `tests/harness/linux_mpp_decode/devioctl.h`
- Create: `tests/harness/linux_mpp_decode/winshim.h`
- Create: `tests/harness/linux_mpp_decode/mpp_svc.h`
- Create: `tests/harness/linux_mpp_decode/mpp_svc.c` (stubs)
- Create: `tests/harness/linux_mpp_decode/linux_mpp_decode.c` (stub main)
- Create: `tests/harness/linux_mpp_decode/Makefile`

The problem: `mft/regbuilder_h264.h` includes `shared/rkmpp_ioctl.h` which includes
`<devioctl.h>` (Windows-only). On Linux, the `-I.` flag makes the compiler find our stub
`devioctl.h` before any system path.

- [ ] **Step 1: Create devioctl.h**

```c
/* devioctl.h — Linux stub: prevents shared/rkmpp_ioctl.h from failing */
#pragma once
#include <stdint.h>
#define CTL_CODE(dev,func,method,access) 0u
#define METHOD_BUFFERED     0
#define METHOD_IN_DIRECT    1
#define METHOD_OUT_DIRECT   2
#define METHOD_NEITHER      3
#define FILE_READ_ACCESS    1
#define FILE_WRITE_ACCESS   2
#define FILE_ANY_ACCESS     0
```

- [ ] **Step 2: Create winshim.h**

```c
/* winshim.h — Windows integer typedef aliases for Linux builds of mft/ */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint8_t   UINT8;
typedef uint16_t  UINT16;
typedef uint32_t  UINT32;
typedef uint64_t  UINT64;
typedef int8_t    INT8;
typedef int16_t   INT16;
typedef int32_t   INT32;
typedef int64_t   INT64;
typedef uint8_t   BOOLEAN;
typedef void     *PVOID;
typedef uint32_t  NTSTATUS;
typedef size_t    SIZE_T;

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif
#ifndef STATUS_SUCCESS
#define STATUS_SUCCESS ((NTSTATUS)0)
#endif

/* DEFINE_GUID: rkmpp_ioctl.h declares the device interface GUID.
 * On Linux it generates no code. */
#define DEFINE_GUID(name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
    /* GUID stub: unused on Linux */
```

- [ ] **Step 3: Create mpp_svc.h**

```c
/* mpp_svc.h — Linux mpp_service + dma_heap backend */
#pragma once
#include <stdint.h>
#include <stddef.h>

/* Include Windows shims before any mft header */
#include "winshim.h"
#include "mft/regbuilder_h264.h"

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

/* Open /dev/mpp_service and initialize the session for RKVDEC2 H.264 decode.
 * Returns a valid fd on success, -1 on error (with perror output). */
int  MppSvc_Open(void);
void MppSvc_Close(int svc_fd);

/* Allocate an uncached DMA-buf via /dev/dma_heap/system-uncached.
 * cpu_va is mmap'd PROT_READ|PROT_WRITE for CPU access.
 * Returns 0 on success, -1 on error. */
int  MppSvc_AllocBuf(size_t size, MppSvcBuf *out);
void MppSvc_FreeBuf(MppSvcBuf *b);

/* Translate rl into MppReqV1 sub-messages and submit to svc_fd via
 * MPP_IOC_SET_REQ.  Does NOT kick hardware — caller must call MppSvc_Kick.
 * buf_map[0..n_bufs-1] maps each BufferHandle in rl to a dma_fd.
 * Returns 0 on success, -1 on error. */
int  MppSvc_Submit(int svc_fd, const H264RegWriteList *rl,
                   const MppSvcBufMap *buf_map, int n_bufs);

/* Send the cmd=0x300 LAST_CMD kick that triggers hardware decode.
 * Call after MppSvc_Submit. Returns 0 on success, -1 on error. */
int  MppSvc_Kick(int svc_fd);

/* Wait for hardware completion via MPP_IOC_GET_REQ.
 * *irq_status receives the raw RKVDEC2_REG_INT_EN readback word.
 * Returns 0 = success (RDY bit set), 1 = timeout, -1 = error. */
int  MppSvc_Poll(int svc_fd, uint32_t timeout_ms, uint32_t *irq_status);
```

- [ ] **Step 4: Create mpp_svc.c stub**

```c
/* mpp_svc.c */
#include "mpp_svc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>

int  MppSvc_Open(void)          { return -1; /* TODO */ }
void MppSvc_Close(int fd)       { (void)fd; }
int  MppSvc_AllocBuf(size_t sz, MppSvcBuf *b) { (void)sz; (void)b; return -1; }
void MppSvc_FreeBuf(MppSvcBuf *b) { (void)b; }
int  MppSvc_Submit(int fd, const H264RegWriteList *rl,
                   const MppSvcBufMap *m, int n) {
    (void)fd; (void)rl; (void)m; (void)n; return -1;
}
int  MppSvc_Kick(int fd) { (void)fd; return -1; }
int  MppSvc_Poll(int fd, uint32_t ms, uint32_t *s) {
    (void)fd; (void)ms; (void)s; return -1;
}
```

- [ ] **Step 5: Create linux_mpp_decode.c stub**

```c
/* linux_mpp_decode.c */
#include "winshim.h"
#include "mft/parser_glue.h"
#include "mft/regbuilder_h264.h"
#include "mft/h264_packed_tables.h"
#include "mft/dpb.h"
#include "mpp_svc.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    fprintf(stderr, "linux_mpp_decode: stub\n");
    return 0;
}
```

- [ ] **Step 6: Create Makefile**

```makefile
CXX      = g++
CC       = gcc
ROOT     = ../../..
# -I. so <devioctl.h> resolves to our stub before any system path
CFLAGS   = -O2 -Wall -I. -I$(ROOT) -I$(ROOT)/mft
CXXFLAGS = $(CFLAGS) -std=c++17

OBJS_CXX = parser_glue.o regbuilder_h264.o h264_packed_tables.o dpb.o
OBJS_C   = linux_mpp_decode.o mpp_svc.o

parser_glue.o:        $(ROOT)/mft/parser_glue.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
regbuilder_h264.o:    $(ROOT)/mft/regbuilder_h264.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
h264_packed_tables.o: $(ROOT)/mft/h264_packed_tables.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
dpb.o:                $(ROOT)/mft/dpb.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

linux_mpp_decode: $(OBJS_CXX) $(OBJS_C)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f linux_mpp_decode *.o
```

- [ ] **Step 7: Copy files to rk and smoke-build**

```bash
# On dev box — scp the harness directory to rk
scp -r tests/harness/linux_mpp_decode vibecoder@192.168.0.180:~/linux_mpp_decode/

# On rk
ssh rk
cd ~/linux_mpp_decode
make
```

Expected: `linux_mpp_decode` binary built with no errors. It prints "stub" and exits 0.

- [ ] **Step 8: Commit scaffold files**

```bash
git add tests/harness/linux_mpp_decode/
git commit -m "harness: scaffold linux_mpp_decode — stubs + winshim + Makefile"
```

---

## Task 3: mpp_svc — DMA-buf allocation

**Files:**
- Modify: `tests/harness/linux_mpp_decode/mpp_svc.c`

`/dev/dma_heap/system-uncached` is the uncached heap. The kernel provides
`struct dma_heap_allocation_data` in `<linux/dma-heap.h>` and ioctl
`DMA_HEAP_IOCTL_ALLOC`. On some BSP kernels the path may be
`/dev/dma_heap/system` — confirm with `ls /dev/dma_heap/` on rk.

- [ ] **Step 1: On rk, verify dma_heap device path**

```bash
ls /dev/dma_heap/
```

Expected: see `system-uncached` or `system`. Use the uncached variant for codec buffers.

- [ ] **Step 2: Implement MppSvc_AllocBuf**

Replace the stub in `mpp_svc.c`:

```c
static int g_heap_fd = -1;

static int heap_open(void) {
    if (g_heap_fd >= 0) return 0;
    g_heap_fd = open("/dev/dma_heap/system-uncached", O_RDWR | O_CLOEXEC);
    if (g_heap_fd < 0) {
        /* Fallback: some BSP kernels use "system" only */
        g_heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    }
    if (g_heap_fd < 0) { perror("open /dev/dma_heap"); return -1; }
    return 0;
}

int MppSvc_AllocBuf(size_t size, MppSvcBuf *out) {
    if (heap_open() < 0) return -1;
    /* Round up to page size */
    size_t aligned = (size + 4095) & ~(size_t)4095;
    struct dma_heap_allocation_data alloc = {
        .len      = aligned,
        .fd_flags = O_RDWR | O_CLOEXEC,
    };
    if (ioctl(g_heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc) < 0) {
        perror("DMA_HEAP_IOCTL_ALLOC"); return -1;
    }
    void *va = mmap(NULL, aligned, PROT_READ | PROT_WRITE,
                    MAP_SHARED, alloc.fd, 0);
    if (va == MAP_FAILED) {
        perror("mmap dma_buf"); close(alloc.fd); return -1;
    }
    out->dma_fd = alloc.fd;
    out->cpu_va = va;
    out->size   = aligned;
    return 0;
}

void MppSvc_FreeBuf(MppSvcBuf *b) {
    if (!b) return;
    if (b->cpu_va && b->cpu_va != MAP_FAILED)
        munmap(b->cpu_va, b->size);
    if (b->dma_fd >= 0)
        close(b->dma_fd);
    b->cpu_va = NULL;
    b->dma_fd = -1;
}
```

- [ ] **Step 3: Write a smoke test for allocation**

Add a temporary `main` to `mpp_svc.c` (guarded by `#ifdef MPP_SVC_TEST_ALLOC`):

```c
#ifdef MPP_SVC_TEST_ALLOC
int main(void) {
    MppSvcBuf b = {0};
    if (MppSvc_AllocBuf(4096, &b) < 0) { fprintf(stderr, "alloc failed\n"); return 1; }
    fprintf(stderr, "alloc OK: dma_fd=%d cpu_va=%p size=%zu\n",
            b.dma_fd, b.cpu_va, b.size);
    /* Write a pattern and read it back */
    uint32_t *p = (uint32_t *)b.cpu_va;
    p[0] = 0xDEADBEEF;
    if (p[0] != 0xDEADBEEF) { fprintf(stderr, "readback FAIL\n"); return 1; }
    fprintf(stderr, "readback OK\n");
    MppSvc_FreeBuf(&b);
    return 0;
}
#endif
```

- [ ] **Step 4: Build and run the alloc smoke test on rk**

```bash
# On rk
cd ~/linux_mpp_decode
cp ~/rkvdec/tests/harness/linux_mpp_decode/mpp_svc.c .
gcc -O2 -DMPP_SVC_TEST_ALLOC -I. -I~/rkvdec -I~/rkvdec/mft \
    mpp_svc.c -o test_alloc
./test_alloc
```

Expected:
```
alloc OK: dma_fd=4 cpu_va=0x... size=4096
readback OK
```

- [ ] **Step 5: Remove the test main, commit**

```bash
git add tests/harness/linux_mpp_decode/mpp_svc.c
git commit -m "harness: mpp_svc DMA-buf allocation via dma_heap"
```

---

## Task 4: mpp_svc — session open + codec init

**Files:**
- Modify: `tests/harness/linux_mpp_decode/mpp_svc.c`
- Modify: `tests/harness/linux_mpp_decode/mpp_svc.h` (add protocol constants)

Use the confirmed values from Task 1. The expected values below are correct for the
RK3588 BSP mpp_service — adjust if Task 1 found different numbers.

- [ ] **Step 1: Add MppReqV1 struct and constants to mpp_svc.h**

Add this block to `mpp_svc.h` after the existing includes:

```c
/* ---- mpp_service wire protocol ---- *
 * Verify these against ~/linux-rockchip/drivers/video/rockchip/mpp/
 * and ~/mpp/osal/driver/mpp_device.c (Task 1). */
#include <sys/ioctl.h>
#include <linux/ioctl.h>

struct MppReqV1 {
    uint32_t cmd;
    uint32_t flag;
    uint32_t size;
    uint32_t offset;
    uint64_t data_ptr;
};

/* IOCTL numbers — confirm from BSP mpp_service.h */
#define MPP_IOC_BASE            'M'
#define MPP_IOC_SET_REQ         _IOW(MPP_IOC_BASE, 0, struct MppReqV1)
#define MPP_IOC_GET_REQ         _IOWR(MPP_IOC_BASE, 1, struct MppReqV1)

/* Command codes — confirm from BSP mpp_cmd.h */
#define MPP_CMD_INIT_CLIENT_TYPE    0x100
#define MPP_CMD_SET_REG_WRITE       0x200
#define MPP_CMD_SET_REG_READ        0x201
#define MPP_CMD_SET_REG_ADDR_OFFSET 0x202
#define MPP_CMD_POLL_HW_FINISH      0x300

/* Client type for RKVDEC2 H.264 decode — confirm from BSP VpuClientType enum */
#define VPU_CLIENT_RKVDEC2          6

/* Flag set on the final sub-message to trigger the hw kick */
#define MPP_FLAGS_LAST_CMD          0x0001
```

- [ ] **Step 2: Implement MppSvc_Open**

```c
int MppSvc_Open(void) {
    int fd = open("/dev/mpp_service", O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("open /dev/mpp_service"); return -1; }

    /* Announce codec type: RKVDEC2 H.264 decoder */
    uint32_t client_type = VPU_CLIENT_RKVDEC2;
    struct MppReqV1 req = {
        .cmd      = MPP_CMD_INIT_CLIENT_TYPE,
        .flag     = 0,
        .size     = sizeof(client_type),
        .offset   = 0,
        .data_ptr = (uint64_t)(uintptr_t)&client_type,
    };
    if (ioctl(fd, MPP_IOC_SET_REQ, &req) < 0) {
        perror("MPP_CMD_INIT_CLIENT_TYPE"); close(fd); return -1;
    }
    return fd;
}

void MppSvc_Close(int svc_fd) {
    if (svc_fd >= 0) close(svc_fd);
}
```

- [ ] **Step 3: Smoke test Open/Close on rk**

Temporarily add to `linux_mpp_decode.c` main:

```c
int svc = MppSvc_Open();
if (svc < 0) { fprintf(stderr, "MppSvc_Open failed\n"); return 1; }
fprintf(stderr, "MppSvc_Open OK: fd=%d\n", svc);
MppSvc_Close(svc);
fprintf(stderr, "MppSvc_Close OK\n");
```

```bash
# On rk: sync files, rebuild, run
make && ./linux_mpp_decode
```

Expected:
```
MppSvc_Open OK: fd=5
MppSvc_Close OK
```

If `open("/dev/mpp_service")` fails with ENOENT, check `ls /dev/mpp*` on rk and
update the path.

- [ ] **Step 4: Commit**

```bash
git add tests/harness/linux_mpp_decode/mpp_svc.h tests/harness/linux_mpp_decode/mpp_svc.c
git commit -m "harness: mpp_svc Open/Close with INIT_CLIENT_TYPE"
```

---

## Task 5: mpp_svc — Submit (translate H264RegWriteList → MppReqV1)

**Files:**
- Modify: `tests/harness/linux_mpp_decode/mpp_svc.c`

`H264RegWriteList.entries[i].Offset` is a byte offset from SWREG[0] (no 0x100 prefix —
the regbuilder emits raw register offsets, kernel handles the MMIO base). This offset
maps directly to `MppReqV1.offset` for cmd=0x200.

For cmd=0x202 IOVA substitution pairs: `(entry.Offset/4, entry.IovaOffset)` as u32 pairs,
one MppReqV1 per distinct buffer handle (= per distinct dma_fd).

Use the fixed bank table from the plan header for cmd=0x200 grouping.

- [ ] **Step 1: Add the bank table to mpp_svc.c**

```c
/* Register banks — matches kBanks[] in winreplay_h264_diff.c.
 * MppReqV1.offset = byte offset from SWREG[0]; size = bytes in bank. */
typedef struct { uint32_t offset; uint32_t size; uint32_t cmd; } RegBank;
static const RegBank kBanks[] = {
    {  32, 100, MPP_CMD_SET_REG_WRITE },  /* regs   8.. 32 */
    { 256, 196, MPP_CMD_SET_REG_WRITE },  /* regs  64..112 */
    { 512,  60, MPP_CMD_SET_REG_WRITE },  /* regs 128..142 */
    { 640, 152, MPP_CMD_SET_REG_WRITE },  /* regs 160..197 */
    { 800,  20, MPP_CMD_SET_REG_WRITE },  /* regs 200..204 */
    {1024,  88, MPP_CMD_SET_REG_WRITE },  /* regs 256..277 */
    { 896,  56, MPP_CMD_SET_REG_READ  },  /* regs 224..237 — IRQ status */
};
#define N_BANKS (sizeof(kBanks)/sizeof(kBanks[0]))
#define BANK_MAX_WORDS (196/4)  /* largest bank is 196 B = 49 words */
```

- [ ] **Step 2: Implement the helper that sends one MppReqV1**

```c
static int send_req(int svc_fd, uint32_t cmd, uint32_t flag,
                    uint32_t size, uint32_t offset, void *data) {
    struct MppReqV1 req = {
        .cmd      = cmd,
        .flag     = flag,
        .size     = size,
        .offset   = offset,
        .data_ptr = (uint64_t)(uintptr_t)data,
    };
    if (ioctl(svc_fd, MPP_IOC_SET_REQ, &req) < 0) {
        perror("MPP_IOC_SET_REQ"); return -1;
    }
    return 0;
}
```

- [ ] **Step 3: Implement MppSvc_Submit**

```c
int MppSvc_Submit(int svc_fd, const H264RegWriteList *rl,
                  const MppSvcBufMap *buf_map, int n_bufs) {
    /* --- cmd=0x200 banks: plain writes -------------------------------- */
    for (size_t b = 0; b < N_BANKS; b++) {
        if (kBanks[b].cmd != MPP_CMD_SET_REG_WRITE) continue;
        uint32_t first_byte = kBanks[b].offset;
        uint32_t last_byte  = first_byte + kBanks[b].size;
        uint32_t words      = kBanks[b].size / 4;
        uint32_t bank_data[BANK_MAX_WORDS];
        memset(bank_data, 0, words * 4);

        for (uint32_t i = 0; i < rl->count; i++) {
            const RKMPP_REG_WRITE *e = &rl->entries[i];
            if (e->BufferHandle != 0) continue;  /* IOVA sub — skip for now */
            if (e->Offset < first_byte || e->Offset >= last_byte) continue;
            bank_data[(e->Offset - first_byte) / 4] = e->Value;
        }
        if (send_req(svc_fd, MPP_CMD_SET_REG_WRITE, 0,
                     words * 4, first_byte, bank_data) < 0)
            return -1;
    }

    /* --- cmd=0x201 readback spec: IRQ bank ----------------------------
     * Specifies which register range to read back after hardware finish.
     * The actual buffer is provided in MppSvc_Poll's MPP_IOC_GET_REQ call.
     * data_ptr=0 here: we're registering the spec (offset+size), not the
     * destination buffer.  Confirm this interpretation from BSP source
     * in Task 1; if the kernel expects a live buffer here, make it static. */
    for (size_t b = 0; b < N_BANKS; b++) {
        if (kBanks[b].cmd != MPP_CMD_SET_REG_READ) continue;
        if (send_req(svc_fd, MPP_CMD_SET_REG_READ, 0,
                     kBanks[b].size, kBanks[b].offset, NULL) < 0)
            return -1;
    }

    /* --- cmd=0x202: IOVA substitution, one per distinct buffer -------- */
    /* Collect all unique handles from the reg list */
    uint64_t seen_handles[RKMPP_MAX_REG_WRITES];
    int      n_seen = 0;
    for (uint32_t i = 0; i < rl->count; i++) {
        if (!rl->entries[i].BufferHandle) continue;
        uint64_t h = rl->entries[i].BufferHandle;
        int dup = 0;
        for (int j = 0; j < n_seen; j++) if (seen_handles[j] == h) { dup = 1; break; }
        if (!dup && n_seen < RKMPP_MAX_REG_WRITES) seen_handles[n_seen++] = h;
    }

    for (int si = 0; si < n_seen; si++) {
        uint64_t handle = seen_handles[si];

        /* Look up dma_fd for this handle */
        int dma_fd = -1;
        for (int j = 0; j < n_bufs; j++) {
            if (buf_map[j].handle == handle) { dma_fd = buf_map[j].dma_fd; break; }
        }
        if (dma_fd < 0) {
            fprintf(stderr, "mpp_svc: no buf_map entry for handle %llx\n",
                    (unsigned long long)handle);
            return -1;
        }

        /* Build (reg_word_index, byte_offset_within_buf) pairs */
        uint32_t pairs[RKMPP_MAX_REG_WRITES * 2];
        int n_pairs = 0;
        for (uint32_t i = 0; i < rl->count; i++) {
            const RKMPP_REG_WRITE *e = &rl->entries[i];
            if (e->BufferHandle != handle) continue;
            pairs[n_pairs * 2 + 0] = e->Offset / 4;   /* register word index */
            pairs[n_pairs * 2 + 1] = e->IovaOffset;    /* byte offset in buffer */
            n_pairs++;
        }

        /* Send cmd=0x202: MppReqV1.offset = dma_fd */
        if (send_req(svc_fd, MPP_CMD_SET_REG_ADDR_OFFSET, 0,
                     n_pairs * 8, (uint32_t)dma_fd, pairs) < 0)
            return -1;
    }

    return 0;
}
```

- [ ] **Step 4: Implement MppSvc_Kick**

```c
int MppSvc_Kick(int svc_fd) {
    struct MppReqV1 req = {
        .cmd  = MPP_CMD_POLL_HW_FINISH,
        .flag = MPP_FLAGS_LAST_CMD,
        .size = 0, .offset = 0, .data_ptr = 0,
    };
    if (ioctl(svc_fd, MPP_IOC_SET_REQ, &req) < 0) {
        perror("MPP_CMD_POLL_HW_FINISH (kick)"); return -1;
    }
    return 0;
}
```

- [ ] **Step 5: Commit**

```bash
git add tests/harness/linux_mpp_decode/mpp_svc.c tests/harness/linux_mpp_decode/mpp_svc.h
git commit -m "harness: mpp_svc Submit + Kick (reg-list → MppReqV1 translation)"
```

---

## Task 6: mpp_svc — Poll (wait for hardware completion)

**Files:**
- Modify: `tests/harness/linux_mpp_decode/mpp_svc.c`

Poll reads back the IRQ status register (RKVDEC2_REG_INT_EN = 224 * 4 = 896).
Success = `RKVDEC2_INT_DEC_RDY_STA` (bit 2). Timeout = `RKVDEC2_INT_DEC_TIMEOUT_STA` (bit 5).

- [ ] **Step 1: Implement MppSvc_Poll**

```c
/* IRQ status bits from mft/rkvdec2_h264_regs.h */
#define RKVDEC2_INT_DEC_RDY_STA     (1u << 2)
#define RKVDEC2_INT_DEC_ERROR_STA   (1u << 4)
#define RKVDEC2_INT_DEC_TIMEOUT_STA (1u << 5)
#define RKVDEC2_REG_INT_EN_BYTE     (224 * 4)   /* byte offset from SWREG[0] */
/* IRQ bank: offset=896, size=56 B, 14 words. INT_EN is word 0 of the IRQ bank. */
#define IRQ_BANK_OFFSET             896u
#define IRQ_BANK_WORDS              (56u / 4u)

int MppSvc_Poll(int svc_fd, uint32_t timeout_ms, uint32_t *irq_status) {
    uint32_t readback[IRQ_BANK_WORDS];
    memset(readback, 0, sizeof(readback));

    /* MPP_IOC_GET_REQ: blocks until hardware raises IRQ or times out.
     * data_ptr points to the buffer that the kernel fills with the
     * register readback specified by the earlier cmd=0x201 sub-message. */
    struct MppReqV1 req = {
        .cmd      = MPP_CMD_POLL_HW_FINISH,
        .flag     = MPP_FLAGS_LAST_CMD,
        .size     = sizeof(readback),
        .offset   = IRQ_BANK_OFFSET,
        .data_ptr = (uint64_t)(uintptr_t)readback,
    };
    if (ioctl(svc_fd, MPP_IOC_GET_REQ, &req) < 0) {
        perror("MPP_IOC_GET_REQ"); return -1;
    }
    (void)timeout_ms;   /* kernel handles timeout internally */

    /* readback[0] is RKVDEC2_REG_INT_EN (first word of IRQ bank at off=896) */
    uint32_t sta = readback[0];
    if (irq_status) *irq_status = sta;

    if (sta & RKVDEC2_INT_DEC_TIMEOUT_STA) return 1;  /* timeout */
    if (sta & RKVDEC2_INT_DEC_RDY_STA)    return 0;   /* success */
    /* ERROR_STA or unexpected: treat as error */
    fprintf(stderr, "mpp_svc: poll unexpected status 0x%08x\n", sta);
    return -1;
}
```

> **Note:** If `MPP_IOC_GET_REQ` semantics differ (e.g., it expects a different payload
> or the readback buffer address is passed differently), confirm from `~/mpp/osal/driver/`
> and adjust accordingly. The critical invariant is: after this ioctl returns 0, the
> hardware has finished and pool_output.cpu_va contains valid NV12.

- [ ] **Step 2: Commit**

```bash
git add tests/harness/linux_mpp_decode/mpp_svc.c
git commit -m "harness: mpp_svc Poll (MPP_IOC_GET_REQ + IRQ status check)"
```

---

## Task 7: linux_mpp_decode — buffer pool init

**Files:**
- Modify: `tests/harness/linux_mpp_decode/linux_mpp_decode.c`

All persistent buffers (CABAC, pps_table, rps_table, rcb, error_ref, DPB pool) are
allocated once and reused across AUs. Buffer handles for `H264BuildRegisterList` are
the `dma_fd` values cast to `uint64_t`.

- [ ] **Step 1: Add sizing constants and HarnessCtx struct**

```c
#include "winshim.h"
#include "mft/parser_glue.h"
#include "mft/regbuilder_h264.h"
#include "mft/h264_packed_tables.h"
#include "mft/dpb.h"
#include "mpp_svc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BITSTREAM_BUF_SIZE  (2u * 1024u * 1024u)   /* 2 MiB */
#define DPB_SLOTS           DPB_MAX_SLOTS           /* 16 */

typedef struct {
    int          svc_fd;
    uint32_t     width, height;

    MppSvcBuf    bitstream;
    MppSvcBuf    cabac_init;
    MppSvcBuf    pps_table;
    MppSvcBuf    rps_table;
    MppSvcBuf    scaling_list;
    MppSvcBuf    rcb;
    MppSvcBuf    error_ref;
    MppSvcBuf    pool_output[DPB_SLOTS];
    MppSvcBuf    pool_colmv[DPB_SLOTS];

    DpbPoolEntry dpb_pool[DPB_SLOTS];
    DpbCtx       dpb;

    H264RcbInfo  rcb_info[RKH264_RCB_COUNT];
} HarnessCtx;
```

- [ ] **Step 2: Implement HarnessCtx_Init**

```c
static uint32_t nv12_frame_size(uint32_t w, uint32_t h) {
    uint32_t mb_w = (w + 15) / 16;
    uint32_t mb_h = (h + 15) / 16;
    uint32_t stride = mb_w * 16;
    uint32_t y_size = stride * (mb_h * 16);
    return y_size + y_size / 2;  /* NV12: Y + UV half-height */
}

static uint32_t colmv_size(uint32_t w, uint32_t h) {
    uint32_t mb_w = (w + 15) / 16;
    uint32_t mb_h = (h + 15) / 16;
    return mb_w * mb_h * 16;
}

static int alloc_or_die(const char *label, size_t size, MppSvcBuf *b) {
    if (MppSvc_AllocBuf(size, b) < 0) {
        fprintf(stderr, "alloc_or_die: %s failed (%zu bytes)\n", label, size);
        return -1;
    }
    return 0;
}

static int HarnessCtx_Init(HarnessCtx *ctx, uint32_t w, uint32_t h) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->width  = w;
    ctx->height = h;

    ctx->svc_fd = MppSvc_Open();
    if (ctx->svc_fd < 0) return -1;

    uint32_t frame_sz = nv12_frame_size(w, h);
    uint32_t cmv_sz   = colmv_size(w, h);
    uint32_t rcb_total = H264GetRcbBufferSizes(ctx->rcb_info, w, h);

    if (alloc_or_die("bitstream",  BITSTREAM_BUF_SIZE,           &ctx->bitstream)    < 0) return -1;
    if (alloc_or_die("cabac_init", RKH264_CABAC_INIT_SIZE +
                                   RKH264_TABLE_TAIL_PAD,        &ctx->cabac_init)   < 0) return -1;
    if (alloc_or_die("pps_table",  RKH264_SPSPPS_UNIT_SIZE +
                                   RKH264_TABLE_TAIL_PAD,        &ctx->pps_table)    < 0) return -1;
    if (alloc_or_die("rps_table",  RKH264_RPS_SIZE +
                                   RKH264_TABLE_TAIL_PAD,        &ctx->rps_table)    < 0) return -1;
    if (alloc_or_die("scaling",    RKH264_SCALING_LIST_SIZE +
                                   RKH264_TABLE_TAIL_PAD,        &ctx->scaling_list) < 0) return -1;
    if (alloc_or_die("rcb",        rcb_total,                    &ctx->rcb)          < 0) return -1;
    if (alloc_or_die("error_ref",  frame_sz,                     &ctx->error_ref)    < 0) return -1;

    for (int i = 0; i < DPB_SLOTS; i++) {
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "pool_output[%d]", i);
        if (alloc_or_die(lbl, frame_sz, &ctx->pool_output[i]) < 0) return -1;
        snprintf(lbl, sizeof(lbl), "pool_colmv[%d]", i);
        if (alloc_or_die(lbl, cmv_sz,   &ctx->pool_colmv[i])  < 0) return -1;

        /* DPB pool entries use dma_fd as the buffer handle */
        ctx->dpb_pool[i].output_frame = (uint64_t)ctx->pool_output[i].dma_fd;
        ctx->dpb_pool[i].colmv        = (uint64_t)ctx->pool_colmv[i].dma_fd;
    }
    Dpb_Init(&ctx->dpb, ctx->dpb_pool, DPB_SLOTS);

    /* Upload CABAC init table (static, upload once) */
    size_t n_words;
    const uint32_t *cabac = H264GetCabacInitTable(&n_words);
    memcpy(ctx->cabac_init.cpu_va, cabac, n_words * 4);
    memset((uint8_t *)ctx->cabac_init.cpu_va + n_words * 4, 0, RKH264_TABLE_TAIL_PAD);

    /* Zero out error_ref so it's a valid black frame */
    memset(ctx->error_ref.cpu_va, 0, ctx->error_ref.size);

    /* Zero scaling_list = flat 16 */
    memset(ctx->scaling_list.cpu_va, 0, ctx->scaling_list.size);

    return 0;
}

static void HarnessCtx_Shutdown(HarnessCtx *ctx) {
    MppSvc_FreeBuf(&ctx->bitstream);
    MppSvc_FreeBuf(&ctx->cabac_init);
    MppSvc_FreeBuf(&ctx->pps_table);
    MppSvc_FreeBuf(&ctx->rps_table);
    MppSvc_FreeBuf(&ctx->scaling_list);
    MppSvc_FreeBuf(&ctx->rcb);
    MppSvc_FreeBuf(&ctx->error_ref);
    for (int i = 0; i < DPB_SLOTS; i++) {
        MppSvc_FreeBuf(&ctx->pool_output[i]);
        MppSvc_FreeBuf(&ctx->pool_colmv[i]);
    }
    MppSvc_Close(ctx->svc_fd);
    ctx->svc_fd = -1;
}
```

- [ ] **Step 3: Write a smoke test that inits and shuts down**

Temporarily add to `main`:

```c
int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <width> <height>\n", argv[0]); return 1; }
    HarnessCtx ctx;
    if (HarnessCtx_Init(&ctx, atoi(argv[1]), atoi(argv[2])) < 0) return 1;
    fprintf(stderr, "HarnessCtx_Init OK — all %d DPB slots + tables allocated\n",
            DPB_SLOTS);
    HarnessCtx_Shutdown(&ctx);
    fprintf(stderr, "HarnessCtx_Shutdown OK\n");
    return 0;
}
```

```bash
# On rk (1920x1080 as a quick test)
make && ./linux_mpp_decode 1920 1080
```

Expected: "HarnessCtx_Init OK — all 16 DPB slots + tables allocated"

- [ ] **Step 4: Commit**

```bash
git add tests/harness/linux_mpp_decode/linux_mpp_decode.c
git commit -m "harness: linux_mpp_decode buffer pool init + shutdown"
```

---

## Task 8: linux_mpp_decode — AU loop + NV12 output dump

**Files:**
- Modify: `tests/harness/linux_mpp_decode/linux_mpp_decode.c`

AU walker is copied from `winreplay_h264_diff.c`. Each AU feeds the full
parser→DPB→regbuilder→Submit→Kick→Poll pipeline and writes the output slot to file.

- [ ] **Step 1: Add Annex-B AU walker (copy from winreplay_h264_diff.c)**

```c
static size_t find_start_code(const uint8_t *buf, size_t len, size_t from) {
    for (size_t i = from; i + 3 <= len; i++) {
        if (buf[i]==0 && buf[i+1]==0 && buf[i+2]==1) return i + 3;
        if (i + 4 <= len &&
            buf[i]==0 && buf[i+1]==0 && buf[i+2]==0 && buf[i+3]==1) return i + 4;
    }
    return SIZE_MAX;
}

static int h264_nal_is_slice(uint8_t h) {
    uint8_t t = h & 0x1F;
    return t == 1 || t == 5;
}

typedef struct { const uint8_t *buf; size_t len; size_t pos; } NalIter;

static int h264_au_next(NalIter *it, size_t *au_off, size_t *au_len,
                        size_t *slice_off) {
    if (it->pos >= it->len) return 0;
    size_t first_sc = find_start_code(it->buf, it->len, it->pos);
    if (first_sc == SIZE_MAX) return 0;
    size_t sc_start = first_sc - 3;
    if (sc_start > 0 && it->buf[sc_start-1] == 0) sc_start--;
    size_t nh = first_sc, end = it->len;
    size_t slice_nh = 0;
    int found = 0;
    while (nh < it->len) {
        if (h264_nal_is_slice(it->buf[nh])) {
            slice_nh = nh;
            size_t nxt = find_start_code(it->buf, it->len, nh + 1);
            if (nxt == SIZE_MAX) end = it->len;
            else { end = nxt - 3; if (end > 0 && it->buf[end-1] == 0) end--; }
            found = 1; break;
        }
        size_t nxt = find_start_code(it->buf, it->len, nh + 1);
        if (nxt == SIZE_MAX) break;
        nh = nxt;
    }
    if (!found) return 0;
    *au_off   = sc_start;
    *au_len   = end - sc_start;
    if (slice_off) {
        size_t s = slice_nh - 3;
        *slice_off = s;
    }
    it->pos = end;
    return 1;
}
```

- [ ] **Step 2: Add the per-AU decode function**

```c
static int decode_one_au(HarnessCtx *ctx, const uint8_t *au, size_t au_len,
                         size_t slice_off, FILE *out_fp) {
    static uint8_t scratch[2u << 20];
    static H264ParseResult parsed;
    static DpbSelection sel;
    static H264RegWriteList rl;

    /* 1. Parse */
    H264ParseStatus ps = H264ParseAccessUnit(au, au_len,
                                             scratch, sizeof(scratch), &parsed);
    if (ps != H264_PARSE_OK || !parsed.has_slice) {
        fprintf(stderr, "  parse status %d has_slice=%d — skip\n", ps, parsed.has_slice);
        return 0;
    }

    /* 2. DPB select (void — no return value, same as winreplay_h264_diff.c) */
    memset(&sel, 0, sizeof(sel));
    Dpb_Select(&ctx->dpb, &parsed, &sel);

    /* Copy DPB entries into parsed.decode.dpb (regbuilder reads them there) */
    memcpy(parsed.decode.dpb, sel.dpb_entries, sizeof(sel.dpb_entries));

    /* 3. Upload bitstream (slice region only) */
    size_t slice_len = au_len - slice_off;
    if (slice_len > BITSTREAM_BUF_SIZE) {
        fprintf(stderr, "  AU too large: %zu bytes\n", slice_len);
        return -1;
    }
    memcpy(ctx->bitstream.cpu_va, au + slice_off, slice_len);

    /* 4. Pack RPS + PPS tables */
    H264PackFrameRps((uint8_t *)ctx->rps_table.cpu_va,
                     parsed.decode.frame_num,
                     parsed.sps.log2_max_frame_num_minus4,
                     sel.dpb_entries, sel.ref_lists);

    H264PackSpsPpsUnit((uint8_t *)ctx->pps_table.cpu_va,
                       &parsed.sps, &parsed.pps,
                       sel.dpb_entries, /*field_pic=*/0);

    /* 5. Build buffer refs (dma_fd as handle) */
    H264BufferRefs bufs;
    memset(&bufs, 0, sizeof(bufs));
    bufs.bitstream         = (uint64_t)ctx->bitstream.dma_fd;
    bufs.bitstream_offset  = 0;
    bufs.bitstream_size    = (uint32_t)slice_len;
    bufs.output_frame      = (uint64_t)ctx->pool_output[sel.current_slot].dma_fd;
    bufs.colmv_cur         = (uint64_t)ctx->pool_colmv[sel.current_slot].dma_fd;
    bufs.error_ref         = (uint64_t)ctx->error_ref.dma_fd;
    bufs.pps_table         = (uint64_t)ctx->pps_table.dma_fd;
    bufs.rps_table         = (uint64_t)ctx->rps_table.dma_fd;
    bufs.cabac_init_table  = (uint64_t)ctx->cabac_init.dma_fd;
    bufs.scaling_list      = (uint64_t)ctx->scaling_list.dma_fd;
    for (int i = 0; i < RKH264_RCB_COUNT; i++) {
        bufs.rcb[i]        = (uint64_t)ctx->rcb.dma_fd;
        bufs.rcb_offset[i] = ctx->rcb_info[i].offset;
    }
    for (int i = 0; i < DPB_SLOTS; i++) {
        bufs.refs[i]      = (uint64_t)ctx->pool_output[i].dma_fd;
        bufs.ref_colmv[i] = (uint64_t)ctx->pool_colmv[i].dma_fd;
    }

    /* 6. Build register list */
    memset(&rl, 0, sizeof(rl));
    H264RegBuildStatus rs = H264BuildRegisterList(&parsed, &bufs,
                                                  sel.current_slot, &rl);
    if (rs != H264_REGBUILD_OK) {
        fprintf(stderr, "  H264BuildRegisterList failed: %d\n", rs);
        return -1;
    }

    /* 7. Build buf_map: every dma_fd that appears as a BufferHandle */
    MppSvcBufMap buf_map[64];
    int n_map = 0;
    /* Add all known buffers (handles = dma_fds for us) */
#define ADD_MAP(fd_val) do { \
    buf_map[n_map].handle = (uint64_t)(fd_val); \
    buf_map[n_map].dma_fd = (int)(fd_val); \
    n_map++; } while(0)
    ADD_MAP(ctx->bitstream.dma_fd);
    ADD_MAP(ctx->pool_output[sel.current_slot].dma_fd);
    ADD_MAP(ctx->pool_colmv[sel.current_slot].dma_fd);
    ADD_MAP(ctx->error_ref.dma_fd);
    ADD_MAP(ctx->pps_table.dma_fd);
    ADD_MAP(ctx->rps_table.dma_fd);
    ADD_MAP(ctx->cabac_init.dma_fd);
    ADD_MAP(ctx->scaling_list.dma_fd);
    ADD_MAP(ctx->rcb.dma_fd);
    for (int i = 0; i < DPB_SLOTS; i++) {
        ADD_MAP(ctx->pool_output[i].dma_fd);
        ADD_MAP(ctx->pool_colmv[i].dma_fd);
    }
#undef ADD_MAP

    /* 8. Submit + kick + poll */
    if (MppSvc_Submit(ctx->svc_fd, &rl, buf_map, n_map) < 0) return -1;
    if (MppSvc_Kick(ctx->svc_fd) < 0) return -1;
    uint32_t irq_sta = 0;
    int poll_rc = MppSvc_Poll(ctx->svc_fd, 2000, &irq_sta);
    if (poll_rc != 0) {
        fprintf(stderr, "  Poll failed: rc=%d irq=0x%08x\n", poll_rc, irq_sta);
        Dpb_OnDecodeComplete(&ctx->dpb);
        return -1;
    }

    /* 9. Dump NV12 frame to output file */
    if (out_fp) {
        uint32_t frame_sz = nv12_frame_size(ctx->width, ctx->height);
        fwrite(ctx->pool_output[sel.current_slot].cpu_va, 1, frame_sz, out_fp);
    }

    Dpb_OnDecodeComplete(&ctx->dpb);
    return 0;
}
```

- [ ] **Step 3: Implement main**

```c
static uint8_t *read_file(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(n);
    fread(buf, 1, n, f); fclose(f);
    *out_len = n; return buf;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <stream.h264> <width> <height> [out.yuv]\n",
                argv[0]);
        return 1;
    }
    uint32_t W = (uint32_t)atoi(argv[2]);
    uint32_t H = (uint32_t)atoi(argv[3]);
    FILE *out_fp = NULL;
    if (argc >= 5) {
        out_fp = fopen(argv[4], "wb");
        if (!out_fp) { perror(argv[4]); return 1; }
    }

    size_t bs_len;
    uint8_t *bs = read_file(argv[1], &bs_len);
    if (!bs) return 1;

    HarnessCtx ctx;
    if (HarnessCtx_Init(&ctx, W, H) < 0) { free(bs); return 1; }

    NalIter it = { .buf = bs, .len = bs_len, .pos = 0 };
    int au_idx = 0, decoded = 0, failed = 0;
    while (1) {
        size_t au_off, au_len, slice_off;
        if (!h264_au_next(&it, &au_off, &au_len, &slice_off)) break;
        fprintf(stderr, "AU %d: off=%zu len=%zu slice_off=%zu\n",
                au_idx, au_off, au_len, slice_off);
        int rc = decode_one_au(&ctx, bs + au_off, au_len, slice_off, out_fp);
        if (rc == 0) decoded++;
        else         failed++;
        au_idx++;
    }

    fprintf(stderr, "Done: %d decoded, %d failed\n", decoded, failed);
    HarnessCtx_Shutdown(&ctx);
    if (out_fp) fclose(out_fp);
    free(bs);
    return failed > 0 ? 1 : 0;
}
```

- [ ] **Step 4: Commit**

```bash
git add tests/harness/linux_mpp_decode/linux_mpp_decode.c
git commit -m "harness: linux_mpp_decode AU loop + NV12 output dump"
```

---

## Task 9: End-to-end smoke — first IDR frame

**Files:** (no new files — testing existing code on rk)

- [ ] **Step 1: Confirm stream dimensions**

```bash
# On rk — check what streams are available
ls ~/streams/
ffprobe ~/streams/dancing_nobf.h264 2>&1 | grep "Video:"
```

Note the exact width and height (e.g., 1920x1080).

- [ ] **Step 2: Decode the first IDR frame only**

```bash
cd ~/linux_mpp_decode
./linux_mpp_decode ~/streams/dancing_nobf.h264 1920 1080 first_idr.yuv 2>&1 | head -20
```

Expected: "AU 0: off=0 len=... slice_off=..." then "Done: N decoded, 0 failed"

If it fails, check `irq=0x...` value:
- `0x04` = RDY (success)
- `0x20` = TIMEOUT → hardware not decoding; check register list with `winreplay_h264_diff`
- `0x10` = ERROR → bitstream or table issue

- [ ] **Step 3: Verify the output frame is non-zero**

```bash
# Quick check: first and last 32 bytes should not be all zeros for a real frame
xxd first_idr.yuv | head -4
xxd first_idr.yuv | tail -4
```

- [ ] **Step 4: Compare first frame against ffmpeg-rkmpp**

```bash
# Generate reference: ffmpeg hardware decode, first frame only
ffmpeg -vcodec h264_rkmpp -i ~/streams/dancing_nobf.h264 \
    -frames:v 1 -pix_fmt nv12 -f rawvideo ref_idr.yuv 2>/dev/null

# Byte-exact compare (may differ if strides differ — see note)
cmp first_idr.yuv ref_idr.yuv && echo "MATCH" || echo "DIFFER"
```

> **Note on strides:** Our output frame is `mb_w*16 × mb_h*16` NV12.  ffmpeg may use
> different padding. If `cmp` says DIFFER, check sizes first:
> ```bash
> ls -la first_idr.yuv ref_idr.yuv
> ```
> If sizes differ, adjust `nv12_frame_size` or use a PSNR/diff comparison:
> ```bash
> ffmpeg -f rawvideo -pix_fmt nv12 -s 1920x1080 -i first_idr.yuv \
>        -f rawvideo -pix_fmt nv12 -s 1920x1080 -i ref_idr.yuv \
>        -lavfi psnr -f null - 2>&1 | grep PSNR
> ```
> A correct IDR frame should have PSNR > 40 dB (near-lossless vs HW reference).
> The row-556 bug shows as ~30 dB with visible corruption past row 556.

- [ ] **Step 5: Commit test artifacts (stream list, not the yuv files)**

```bash
git add tests/harness/linux_mpp_decode/
git commit -m "harness: linux_mpp_decode end-to-end smoke verified on rk"
```

---

## Task 10: Full stream + B-frame bisection comparison

**Files:** (no new files)

- [ ] **Step 1: Decode full non-B-frame stream**

```bash
# On rk
./linux_mpp_decode ~/streams/dancing_nobf.h264 1920 1080 ours_nobf.yuv 2>&1 | tail -5
```

- [ ] **Step 2: Generate ffmpeg-rkmpp reference for full stream**

```bash
ffmpeg -vcodec h264_rkmpp -i ~/streams/dancing_nobf.h264 \
    -pix_fmt nv12 -f rawvideo ref_nobf.yuv 2>/dev/null
```

- [ ] **Step 3: Compare full streams**

```bash
cmp ours_nobf.yuv ref_nobf.yuv && echo "FULL MATCH" || {
    echo "DIFFER at byte $(cmp -l ours_nobf.yuv ref_nobf.yuv | head -1 | awk '{print $1}')"
}
```

If they match → our user-mode code is correct for non-B streams; any Windows divergence
is in `rkmpp.sys`.

If they differ in the same pattern as Windows → bug is in our user-mode code on both
platforms.

- [ ] **Step 4: Decode B-frame stream**

```bash
./linux_mpp_decode ~/streams/dancing.h264 1920 1080 ours_bframe.yuv 2>&1 | tail -5

ffmpeg -vcodec h264_rkmpp -i ~/streams/dancing.h264 \
    -pix_fmt nv12 -f rawvideo ref_bframe.yuv 2>/dev/null
```

- [ ] **Step 5: Compare B-frame output**

```bash
# Frame-by-frame PSNR to identify which frames diverge
FRAME_SIZE=$((1920 * 1080 * 3 / 2))
ffmpeg -f rawvideo -pix_fmt nv12 -s 1920x1080 -i ours_bframe.yuv \
       -f rawvideo -pix_fmt nv12 -s 1920x1080 -i ref_bframe.yuv \
       -lavfi "psnr=stats_file=psnr.log" -f null - 2>/dev/null
grep "mse_avg" psnr.log | awk '{print NR, $0}' | head -30
```

Frames with PSNR < 30 dB indicate active corruption. Record which frame indices are
corrupt and whether the pattern matches what Windows shows.

- [ ] **Step 6: Record bisection result in h264_cavlc_idr_divergence.md**

Update `docs/` or create a new memory note:
- "Linux/BSP matches ffmpeg-rkmpp → bug in rkmpp.sys" OR
- "Linux/BSP shows same corruption → bug in user-mode (regbuilder/DPB/packed tables)"

```bash
git commit -am "harness: record bisection result — [Linux matches / Linux diverges]"
```
