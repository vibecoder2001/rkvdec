/* Replay variant: split shared fd9 into 3 separate buffers (CABAC, PPS, RPS)
 * — mimicking our Windows driver's allocation pattern.  Goal: see if the
 * codec works when those 3 tables are at different iovas. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/dma-heap.h>

struct mpp_msg_v1 { uint32_t cmd, flags, size, offset; uint64_t data_ptr; };
#define MPP_IOC_CFG_V1   _IOW('v', 1, uint32_t)
#define MPP_FLAGS_MULTI  (1u << 0)
#define MPP_FLAGS_LAST   (1u << 1)

#define DUMP_DIR "/tmp/mpp_dump"

/* Original 7 buffers + we add 2 more separate buffers for PPS and RPS
 * by carving them out of fd9.  fd9 keeps just CABAC data (offset 0). */
#define N_ORIG 7
#define N_SPLIT 2  /* additional pps + rps */
#define N_ALLOC (N_ORIG + N_SPLIT)

static const struct { const char *name; size_t len; uint32_t bsp_fd; } orig[N_ORIG] = {
    { "alloc0_fd9_len77824.bin",   77824,  9  },
    { "alloc1_fd10_len2097152.bin", 2097152, 10 },
    { "alloc2_fd11_len1843200.bin", 1843200, 11 },
    { "alloc3_fd12_len491520.bin", 491520, 12 },
    { "alloc4_fd13_len124928.bin", 124928, 13 },
    { "alloc5_fd14_len124928.bin", 124928, 14 },
    { "alloc6_fd15_len124928.bin", 124928, 15 },
};
static int our_fd[N_ALLOC];

static void *load_blob(const char *path, size_t expected_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "open %s: %s\n", path, strerror(errno)); return NULL; }
    void *buf = malloc(expected_len);
    if (fread(buf, 1, expected_len, f) != expected_len) {
        fprintf(stderr, "short read %s\n", path); free(buf); fclose(f); return NULL;
    }
    fclose(f);
    return buf;
}
static int load_msg(const char *name, void **out_buf, size_t *out_len) {
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", DUMP_DIR, name);
    struct stat st;
    if (stat(path, &st) < 0) return -1;
    *out_buf = load_blob(path, st.st_size);
    *out_len = st.st_size;
    return *out_buf ? 0 : -1;
}
static int alloc_dh(int dh, size_t len) {
    struct dma_heap_allocation_data req = { .len = len, .fd_flags = O_RDWR };
    if (ioctl(dh, DMA_HEAP_IOCTL_ALLOC, &req) < 0) { perror("alloc"); return -1; }
    return req.fd;
}
static void put_blob(int fd, size_t len, const void *src) {
    void *m = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (m == MAP_FAILED) { perror("mmap"); return; }
    memcpy(m, src, len);
    munmap(m, len);
}
static int send_chain(int mpp_fd, struct mpp_msg_v1 *msgs, int nmsgs) {
    if (nmsgs > 1) {
        for (int i = 0; i < nmsgs; i++) {
            msgs[i].flags |= MPP_FLAGS_MULTI;
            if (i == nmsgs - 1) msgs[i].flags |= MPP_FLAGS_LAST;
        }
    }
    int rv = ioctl(mpp_fd, MPP_IOC_CFG_V1, msgs);
    if (rv < 0) { fprintf(stderr, "chain rv=%d errno=%d (%s)\n", rv, errno, strerror(errno)); return -1; }
    return 0;
}

int main(int argc, char **argv) {
    int dh = open("/dev/dma_heap/system-uncached", O_RDWR);
    int mpp = open("/dev/mpp_service", O_RDWR);
    if (dh < 0 || mpp < 0) { perror("open"); return 1; }
    {
        uint32_t client = 9;
        struct mpp_msg_v1 m = { .cmd=0x100, .size=4, .data_ptr=(uintptr_t)&client };
        if (send_chain(mpp, &m, 1) < 0) return 1;
    }
    /* Load fd9's contents (76KB blob with CABAC at 0, PPS at 0x1000, RPS at 0x5000). */
    char p[512]; snprintf(p, sizeof(p), "%s/%s", DUMP_DIR, orig[0].name);
    uint8_t *fd9_blob = load_blob(p, orig[0].len);
    if (!fd9_blob) return 1;

    /* Allocate the 7 original buffers (fd9 will have only the first 4KB
     * of CABAC table — we'll point reg197 at it; PPS and RPS go to the
     * two new separate buffers). */
    for (int i = 0; i < N_ORIG; i++) {
        our_fd[i] = alloc_dh(dh, orig[i].len);
        if (our_fd[i] < 0) return 1;
        printf("orig alloc[%d] len=%zu fd=%d\n", i, orig[i].len, our_fd[i]);
        snprintf(p, sizeof(p), "%s/%s", DUMP_DIR, orig[i].name);
        void *src = load_blob(p, orig[i].len);
        if (!src) return 1;
        put_blob(our_fd[i], orig[i].len, src);
        free(src);
    }

    /* New: separate PPS buffer (4 KiB) with PPS data from fd9+0x1000 */
    our_fd[N_ORIG + 0] = alloc_dh(dh, 4096);
    printf("split alloc[7=PPS] fd=%d\n", our_fd[N_ORIG + 0]);
    put_blob(our_fd[N_ORIG + 0], 4096, fd9_blob + 0x1000);

    /* New: separate RPS buffer (4 KiB) with RPS data from fd9+0x5000 */
    our_fd[N_ORIG + 1] = alloc_dh(dh, 4096);
    printf("split alloc[8=RPS] fd=%d\n", our_fd[N_ORIG + 1]);
    put_blob(our_fd[N_ORIG + 1], 4096, fd9_blob + 0x5000);

    free(fd9_blob);

    /* Build the kick chain.  msg33.3 is the H.264 ADDR bank (idx 160..197).
     * BSP's data has fd9 in slots for PPS (idx 161), RPS (idx 163), CABAC (idx 197).
     * We rewrite PPS and RPS to point at our new separate fds, with offset 0.
     * cmd=0x202 SET_REG_ADDR_OFFSET also has entries for PPS (idx 161, off 0x1000)
     * and RPS (idx 163, off 0x5000) — those become offset 0 since our new buffers
     * start at offset 0. */
    static const char *kick_files[] = {
        "msg32.0_cmd403_off0000_size48.bin",
        "msg33.0_cmd200_off0032_size100.bin",
        "msg33.1_cmd200_off0256_size196.bin",
        "msg33.2_cmd200_off0512_size60.bin",
        "msg33.3_cmd200_off0640_size152.bin",
        "msg33.4_cmd200_off0800_size20.bin",
        "msg33.5_cmd200_off1024_size88.bin",
        "msg33.6_cmd201_off0896_size56.bin",
        "msg33.7_cmd202_off0000_size88.bin",
    };
    static const struct { uint32_t cmd, offset, size; } kick_msgs[] = {
        { 0x403, 0,    48  },
        { 0x200, 32,   100 },
        { 0x200, 256,  196 },
        { 0x200, 512,  60  },
        { 0x200, 640,  152 },
        { 0x200, 800,  20  },
        { 0x200, 1024, 88  },
        { 0x201, 896,  56  },
        { 0x202, 0,    88  },
    };

    /* codec info first */
    {
        void *buf; size_t len;
        if (load_msg(kick_files[0], &buf, &len) < 0) return 1;
        struct mpp_msg_v1 m = { .cmd = kick_msgs[0].cmd, .size = kick_msgs[0].size,
                                .offset = kick_msgs[0].offset, .data_ptr = (uintptr_t)buf };
        send_chain(mpp, &m, 1);
        free(buf);
    }

    /* Build chain with surgical fd substitutions */
    void *bufs[9] = {0};
    size_t lens[9] = {0};
    struct mpp_msg_v1 chain[9];
    int nchain = 0;
    for (int i = 1; i < 9; i++) {
        if (load_msg(kick_files[i], &bufs[i], &lens[i]) < 0) return 1;

        if (kick_msgs[i].cmd == 0x200 && kick_msgs[i].offset == 512) {
            /* idx 128..142: translate fd9..fd15 -> our_fd[0..6] */
            uint32_t *u = (uint32_t*)bufs[i];
            for (size_t j = 0; j < lens[i] / 4; j++)
                if (u[j] >= 9 && u[j] <= 15) u[j] = (uint32_t)our_fd[u[j] - 9];
        } else if (kick_msgs[i].cmd == 0x200 && kick_msgs[i].offset == 640) {
            /* idx 160..197: PPS @ word 1, RPS @ word 3, CABAC @ word 37 */
            uint32_t *u = (uint32_t*)bufs[i];
            /* default fd substitution */
            for (size_t j = 0; j < lens[i] / 4; j++)
                if (u[j] >= 9 && u[j] <= 15) u[j] = (uint32_t)our_fd[u[j] - 9];
            /* override PPS slot to point at our split PPS buffer */
            u[1] = (uint32_t)our_fd[N_ORIG + 0];
            u[3] = (uint32_t)our_fd[N_ORIG + 1];
        } else if (kick_msgs[i].cmd == 0x202) {
            /* SET_REG_ADDR_OFFSET: pairs of (reg_idx, byte_offset).
             * Our split PPS / RPS are at offset 0 in their own buffers
             * — clear the 0x1000 / 0x5000 offsets. */
            uint32_t *u = (uint32_t*)bufs[i];
            for (size_t j = 0; j + 1 < lens[i] / 4; j += 2) {
                if (u[j] == 0xa1 || u[j] == 0xa3) u[j+1] = 0;  /* PPS / RPS */
            }
        }
        chain[nchain].cmd    = kick_msgs[i].cmd;
        chain[nchain].flags  = 0;
        chain[nchain].size   = kick_msgs[i].size;
        chain[nchain].offset = kick_msgs[i].offset;
        chain[nchain].data_ptr = (uintptr_t)bufs[i];
        nchain++;
    }
    if (send_chain(mpp, chain, nchain) < 0) return 1;
    {
        struct mpp_msg_v1 poll = { .cmd = 0x300 };
        send_chain(mpp, &poll, 1);
    }

    /* Read output */
    void *m = mmap(NULL, orig[2].len, PROT_READ, MAP_SHARED, our_fd[2], 0);
    size_t nz = 0;
    for (size_t i = 0; i < orig[2].len; i++) if (((uint8_t*)m)[i]) nz++;
    printf("output: nonzero=%zu\n", nz);
    FILE *of = fopen("/tmp/replay_split_out.yuv", "wb");
    if (of) { fwrite(m, 1, 1382400, of); fclose(of); }
    return 0;
}
