/* mpp_svc.c */
#include "mpp_svc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>

static int g_heap_fd = -1;

static int heap_open(void) {
    if (g_heap_fd >= 0) return 0;
    g_heap_fd = open("/dev/dma_heap/system-uncached", O_RDWR | O_CLOEXEC);
    if (g_heap_fd < 0)
        g_heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (g_heap_fd < 0) { perror("open /dev/dma_heap"); return -1; }
    return 0;
}

int MppSvc_Open(void) {
    int fd = open("/dev/mpp_service", O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("open /dev/mpp_service"); return -1; }

    uint32_t client_type = VPU_CLIENT_RKVDEC;
    struct MppReqV1 req = {
        .cmd      = MPP_CMD_INIT_CLIENT_TYPE,
        .flag     = MPP_FLAGS_LAST_MSG | MPP_FLAGS_REG_OFFSET_ALONE,
        .size     = sizeof(client_type),
        .offset   = 0,
        .data_ptr = (uint64_t)(uintptr_t)&client_type,
    };
    if (ioctl(fd, MPP_IOC_CFG_V1, &req) < 0) {
        perror("MPP_CMD_INIT_CLIENT_TYPE"); close(fd); return -1;
    }
    return fd;
}

void MppSvc_Close(int svc_fd) {
    if (svc_fd >= 0) close(svc_fd);
}

int MppSvc_AllocBuf(size_t size, MppSvcBuf *out) {
    out->dma_fd = -1;
    out->cpu_va = NULL;
    out->size   = 0;
    if (heap_open() < 0) return -1;
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
    b->size   = 0;
}

int MppSvc_SendCodecInfo(int svc_fd, uint32_t width, uint32_t height) {
    /* Each element: { __u32 type, __u32 flag, __u64 data } = 16 bytes */
    struct CodecInfoElem {
        uint32_t type;
        uint32_t flag;
        uint64_t data;
    };
    struct CodecInfoElem elems[3];
    elems[0].type = 1;  /* DEC_INFO_WIDTH */
    elems[0].flag = 1;  /* CODEC_INFO_FLAG_NUMBER */
    elems[0].data = width;
    elems[1].type = 2;  /* DEC_INFO_HEIGHT */
    elems[1].flag = 1;
    elems[1].data = height;
    elems[2].type = 3;  /* DEC_INFO_FORMAT */
    elems[2].flag = 2;  /* CODEC_INFO_FLAG_STRING */
    elems[2].data = 0x34363268ULL; /* "h264\0..." stored as little-endian u64 */

    struct MppReqV1 req = {
        .cmd      = MPP_CMD_SEND_CODEC_INFO,
        .flag     = MPP_FLAGS_LAST_MSG | MPP_FLAGS_REG_OFFSET_ALONE,
        .size     = sizeof(elems),
        .offset   = 0,
        .data_ptr = (uint64_t)(uintptr_t)elems,
    };
    if (ioctl(svc_fd, MPP_IOC_CFG_V1, &req) < 0) {
        perror("MppSvc_SendCodecInfo");
        return -1;
    }
    return 0;
}

int MppSvc_ImportFds(int svc_fd, const int *fds, int n) {
    if (n <= 0 || n > 60) {
        fprintf(stderr, "MppSvc_ImportFds: bad count %d\n", n);
        return -1;
    }
    uint32_t data[60];
    for (int i = 0; i < n; i++)
        data[i] = (uint32_t)fds[i];

    struct MppReqV1 req = {
        .cmd      = MPP_CMD_TRANS_FD_TO_IOVA,
        .flag     = MPP_FLAGS_LAST_MSG | MPP_FLAGS_REG_OFFSET_ALONE,
        .size     = (uint32_t)(n * sizeof(uint32_t)),
        .offset   = 0,
        .data_ptr = (uint64_t)(uintptr_t)data,
    };
    if (ioctl(svc_fd, MPP_IOC_CFG_V1, &req) < 0) {
        perror("MppSvc_ImportFds MPP_CMD_TRANS_FD_TO_IOVA");
        return -1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Register bank layout — mirrors the RKVDEC2 SWREG windows used by mpp-new.
 * Offsets and sizes are byte counts from SWREG[0] (no 0x100 prefix — that
 * prefix is the Windows ACPI MMIO mapping offset, not used on Linux).
 * ---------------------------------------------------------------------------*/
typedef struct { uint32_t offset; uint32_t size; uint32_t cmd; } RegBank;
static const RegBank kBanks[] = {
    {  32, 100, MPP_CMD_SET_REG_WRITE },   /* SWREG  8..32  (control) */
    { 256, 196, MPP_CMD_SET_REG_WRITE },   /* SWREG 64..112 (params)  */
    { 512,  60, MPP_CMD_SET_REG_WRITE },   /* SWREG128..142 (addrs)   */
    { 640, 152, MPP_CMD_SET_REG_WRITE },   /* SWREG160..197 (dpb)     */
    { 800,  20, MPP_CMD_SET_REG_WRITE },   /* SWREG200..204           */
    {1024,  88, MPP_CMD_SET_REG_WRITE },   /* SWREG256..277           */
    { 896,  56, MPP_CMD_SET_REG_READ  },   /* SWREG224..237 (IRQ)     */
};
#define N_BANKS (sizeof(kBanks)/sizeof(kBanks[0]))
#define BANK_MAX_WORDS (196/4)  /* largest bank = 196 B = 49 words */

/* Set MPPDBG=1 in the environment to enable per-submit register dumps. */
static int s_mpp_dbg_frame = 0;

static void dump_reglist(const H264RegWriteList *rl,
                         const MppSvcBufMap *buf_map, int n_bufs,
                         const uint32_t bank_words[][BANK_MAX_WORDS])
{
    fprintf(stderr, "=== MppSvc_Submit REG DUMP (frame %d) ===\n",
            s_mpp_dbg_frame);
    fprintf(stderr, "  rl->count = %u entries\n", rl->count);
    for (uint32_t i = 0; i < rl->count; i++) {
        const RKMPP_REG_WRITE *e = &rl->entries[i];
        uint32_t sw = e->Offset / 4;
        if (e->BufferHandle != 0) {
            int fd = -1;
            for (int j = 0; j < n_bufs; j++) {
                if (buf_map[j].handle == e->BufferHandle) { fd = buf_map[j].dma_fd; break; }
            }
            uint32_t enc = ((uint32_t)e->IovaOffset << 10) | (uint32_t)fd;
            fprintf(stderr, "  [%3u] SW%-3u off=0x%04x IOVA fd=%-3d iovaOff=%-6u enc=0x%08x\n",
                    i, sw, e->Offset, fd, e->IovaOffset, enc);
        } else {
            fprintf(stderr, "  [%3u] SW%-3u off=0x%04x val=0x%08x\n",
                    i, sw, e->Offset, e->Value);
        }
    }

    fprintf(stderr, "--- Bank non-zero words ---\n");
    for (uint32_t b = 0; b < N_BANKS; b++) {
        if (kBanks[b].cmd == MPP_CMD_SET_REG_READ) continue;
        uint32_t nw = kBanks[b].size / 4;
        for (uint32_t w = 0; w < nw; w++) {
            if (bank_words[b][w]) {
                uint32_t sw = kBanks[b].offset / 4 + w;
                fprintf(stderr, "  Bank%-u SW%-3u=0x%08x  (fd=%u off=%u)\n",
                        b, sw, bank_words[b][w],
                        bank_words[b][w] & 0x3ffu,
                        bank_words[b][w] >> 10);
            }
        }
    }
    fprintf(stderr, "=========================================\n");
}

int MppSvc_Submit(int svc_fd, const H264RegWriteList *rl,
                  const MppSvcBufMap *buf_map, int n_bufs,
                  uint32_t *irq_readback,
                  uint32_t width, uint32_t height)
{
    struct MppReqV1 reqs[N_BANKS + 1];  /* +1 for SET_RCB_INFO */
    (void)height;
    uint32_t bank_words[N_BANKS][BANK_MAX_WORDS];
    uint32_t i, b;
    int n_reqs = 0;

    /* Validate every IOVA entry has a buf_map entry */
    for (i = 0; i < rl->count; i++) {
        uint64_t h = rl->entries[i].BufferHandle;
        if (!h) continue;
        int found = 0;
        for (int j = 0; j < n_bufs; j++) {
            if (buf_map[j].handle == h) { found = 1; break; }
        }
        if (!found) {
            fprintf(stderr, "MppSvc_Submit: no buf_map for handle %llx\n",
                    (unsigned long long)h);
            return -1;
        }
    }

    for (b = 0; b < N_BANKS; b++) {
        const RegBank *bank = &kBanks[b];

        if (bank->cmd == MPP_CMD_SET_REG_READ) {
            /* IRQ readback spec: data_ptr receives hardware regs after HW finish */
            reqs[n_reqs++] = (struct MppReqV1){
                .cmd      = MPP_CMD_SET_REG_READ,
                .flag     = MPP_FLAGS_MULTI_MSG,
                .size     = bank->size,
                .offset   = bank->offset,
                .data_ptr = (uint64_t)(uintptr_t)irq_readback,
            };
            continue;
        }

        uint32_t n_words = bank->size / 4;
        if (n_words > BANK_MAX_WORDS) {
            fprintf(stderr, "MppSvc_Submit: bank %u too large\n", b);
            return -1;
        }
        memset(bank_words[b], 0, n_words * sizeof(uint32_t));

        uint32_t bank_end = bank->offset + bank->size;
        for (i = 0; i < rl->count; i++) {
            const RKMPP_REG_WRITE *e = &rl->entries[i];
            if (e->Offset < bank->offset || e->Offset >= bank_end)
                continue;
            uint32_t word_idx = (e->Offset - bank->offset) / 4;

            if (e->BufferHandle != 0) {
                /*
                 * IOVA entry: encode as (byte_offset << 10) | dma_fd.
                 * The kernel's mpp_translate_reg_address will:
                 *   fd    = reg & 0x3ff
                 *   offset = reg >> 10
                 *   reg   = iommu_iova(fd) + offset
                 */
                int dma_fd = -1;
                for (int j = 0; j < n_bufs; j++) {
                    if (buf_map[j].handle == e->BufferHandle) {
                        dma_fd = buf_map[j].dma_fd;
                        break;
                    }
                }
                bank_words[b][word_idx] = ((uint32_t)e->IovaOffset << 10)
                                        | (uint32_t)dma_fd;
            } else {
                bank_words[b][word_idx] = e->Value;
            }
        }

        reqs[n_reqs++] = (struct MppReqV1){
            .cmd      = MPP_CMD_SET_REG_WRITE,
            .flag     = MPP_FLAGS_MULTI_MSG,
            .size     = bank->size,
            .offset   = bank->offset,
            .data_ptr = (uint64_t)(uintptr_t)bank_words[b],
        };
    }

    if (n_reqs == 0) {
        fprintf(stderr, "MppSvc_Submit: no messages to send\n");
        return -1;
    }
    /* n_reqs should be N_BANKS(7) + 1(RCB) = 8 before the RCB append below */

    /*
     * RCB (Reference Cache Buffer) info — tells the kernel to override RCB
     * scratch registers SW133..142 with SRAM addresses (rcb_iova=0xFFF00000).
     * Without this, mpp_set_rcbbuf finds rcb_inf->cnt==0 and leaves the
     * registers pointing at DRAM, which has insufficient bandwidth for the
     * CABAC entropy scratch (STRMD_ROW) and triggers the HW watchdog.
     *
     * Sizes are from rk3588s.dtsi rockchip,rcb-info; order matches BSP HAL.
     * Flags must NOT include REG_OFFSET_ALONE (0x10) — that bit accumulates
     * into msgs->flags and would switch existing IOVA registers to raw-fd
     * (NO_OFFSET) mode, breaking our (IovaOffset<<10)|fd encoding.
     */
    /*
     * H.264 RCB SRAM elements — sized per BSP h264d_refine_rcb_size at the
     * current resolution.  Order matches BSP rcb_priority[] minus INTER_ROW
     * (always skipped: "may conflict with other buffer in ddr").
     *
     * For width <= 4096px no-MBAFF 8-bit, only DBLK_ROW and INTRA_ROW are
     * non-zero — sending the others remaps SRAM for registers the HW
     * doesn't use at this resolution and the watchdog fires (irq=0x23).
     */
    struct RcbElem { uint32_t index; uint32_t size; };
    struct RcbElem rcb_elems[8];
    int n_rcb = 0;

    const uint32_t bit_depth = 8;
    const uint32_t mbaff = 0;
    const uint32_t w_aligned = (width + 15u) & ~15u;
    #define RCB_BYTES(bits) ((((bits) + 7u) / 8u + 63u) & ~63u)

    /* Priority order: DBLK, INTRA, SAO, INTER, FBC, TRANSD_ROW, STRMD,
     * INTER_COL, FILT_COL, TRANSD_COL.  INTER_ROW is skipped. */
    /* DBLK_ROW (SW139) */
    {
        uint32_t bits = w_aligned * (2u + (mbaff ? 12u : 6u) * bit_depth);
        if (bits) { rcb_elems[n_rcb++] = (struct RcbElem){139, RCB_BYTES(bits)}; }
    }
    /* INTRA_ROW (SW133) */
    {
        uint32_t bits = w_aligned * 44u;
        if (bits) { rcb_elems[n_rcb++] = (struct RcbElem){133, RCB_BYTES(bits)}; }
    }
    /* SAO_ROW (SW140) — h264 sets to 0 */
    /* INTER_ROW (SW137) — skipped by priority mode */
    /* FBC_ROW (SW141) — only when fbc_e and chroma>1, h264 we don't enable */
    /* TRANSD_ROW (SW134) — only when width > 8192 */
    /* STRMD_ROW (SW136) — only when width > 4096 */
    if (w_aligned > 4096u) {
        uint32_t bits = ((w_aligned + 15u) / 16u) * 154u * (mbaff ? 2u : 1u);
        rcb_elems[n_rcb++] = (struct RcbElem){136, RCB_BYTES(bits)};
    }
    /* INTER_COL (SW138) — h264 sets to 0 */
    /* FILT_COL (SW142) — h264 sets to 0 */
    /* TRANSD_COL (SW135) — only when height > 8192 */
    #undef RCB_BYTES

    reqs[n_reqs++] = (struct MppReqV1){
        .cmd      = MPP_CMD_SET_RCB_INFO,
        .flag     = MPP_FLAGS_MULTI_MSG,
        .size     = (uint32_t)(n_rcb * sizeof(rcb_elems[0])),
        .offset   = 0,
        .data_ptr = (uint64_t)(uintptr_t)rcb_elems,
    };

    /*
     * All messages carry MULTI_MSG.  The last message adds LAST_MSG but NOT
     * REG_OFFSET_ALONE: that flag aliases MPP_FLAGS_REG_NO_OFFSET (0x10),
     * which would make mpp_translate_reg_address treat full 32-bit register
     * values as fd numbers instead of (offset<<10)|fd pairs.
     */
    reqs[n_reqs - 1].flag |= MPP_FLAGS_LAST_MSG;

    s_mpp_dbg_frame++;
    if (s_mpp_dbg_frame == 1)
        fprintf(stderr, "MppSvc_Submit: frame1 n_reqs=%d\n", n_reqs);
    if (getenv("MPPDBG"))
        dump_reglist(rl, buf_map, n_bufs,
                     (const uint32_t (*)[BANK_MAX_WORDS])bank_words);

    if (ioctl(svc_fd, MPP_IOC_CFG_V1, &reqs[0]) < 0) {
        perror("MppSvc_Submit MPP_IOC_CFG_V1");
        return -1;
    }
    return 0;
}

/* IRQ status bits from rkvdec2 kernel driver / our register header */
#define RKVDEC2_INT_DEC_RDY_STA     (1u << 2)
#define RKVDEC2_INT_DEC_TIMEOUT_STA (1u << 5)

int MppSvc_Poll(int svc_fd, uint32_t timeout_ms, const uint32_t *irq_readback) {
    struct MppReqV1 req = {
        .cmd      = MPP_CMD_POLL_HW_FINISH,
        .flag     = MPP_FLAGS_LAST_MSG | MPP_FLAGS_REG_OFFSET_ALONE,
        .size     = 0,
        .offset   = 0,
        .data_ptr = 0,
    };
    if (ioctl(svc_fd, MPP_IOC_CFG_V1, &req) < 0) {
        perror("MppSvc_Poll MPP_IOC_CFG_V1"); return -1;
    }
    (void)timeout_ms;

    /* irq_readback[0..13] = SWREG224..237 populated by kernel via SET_REG_READ */
    if (getenv("MPPDBG")) {
        fprintf(stderr, "  IRQ readback (SW224..%u):",
                224u + MPP_IRQ_READBACK_WORDS - 1);
        for (int k = 0; k < MPP_IRQ_READBACK_WORDS; k++)
            fprintf(stderr, " %08x", irq_readback[k]);
        fprintf(stderr, "\n");
    }
    uint32_t sta = irq_readback[0];
    if (sta & RKVDEC2_INT_DEC_TIMEOUT_STA) {
        fprintf(stderr, "mpp_svc: TIMEOUT irq=0x%08x\n", sta);
        return 1;
    }
    if (sta & RKVDEC2_INT_DEC_RDY_STA)    return 0;
    fprintf(stderr, "mpp_svc: poll unexpected status 0x%08x\n", sta);
    return -1;
}
