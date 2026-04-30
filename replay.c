/* Standalone replay of a captured BSP H.264 decode against /dev/mpp_service.
 * Reads pre-decode dmabuf contents and ioctl payloads from a dump dir,
 * replays them on a fresh session, writes the resulting output frame.
 */
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
#define N_ALLOC 7

static const struct { const char *name; size_t len; } alloc_descs[N_ALLOC] = {
    { "alloc0_fd9_len77824.bin",   77824   },
    { "alloc1_fd10_len2097152.bin", 2097152 },
    { "alloc2_fd11_len1843200.bin", 1843200 },
    { "alloc3_fd12_len491520.bin", 491520  },
    { "alloc4_fd13_len124928.bin", 124928  },
    { "alloc5_fd14_len124928.bin", 124928  },
    { "alloc6_fd15_len124928.bin", 124928  },
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
    if (stat(path, &st) < 0) { fprintf(stderr, "stat %s: %s\n", path, strerror(errno)); return -1; }
    *out_buf = load_blob(path, st.st_size);
    *out_len = st.st_size;
    return *out_buf ? 0 : -1;
}

static void translate_fds(uint32_t *u, size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (u[i] >= 9 && u[i] <= 15) {
            int idx = u[i] - 9;
            u[i] = (uint32_t)our_fd[idx];
        }
    }
}

static int send_chain(int mpp_fd, struct mpp_msg_v1 *msgs, int nmsgs) {
    if (nmsgs > 1) {
        for (int i = 0; i < nmsgs; i++) {
            msgs[i].flags |= MPP_FLAGS_MULTI;
            if (i == nmsgs - 1) msgs[i].flags |= MPP_FLAGS_LAST;
        }
    }
    int rv = ioctl(mpp_fd, MPP_IOC_CFG_V1, msgs);
    if (rv < 0) {
        int e = errno;
        fprintf(stderr, "ioctl chain rv=%d errno=%d (%s)\n", rv, e, strerror(e));
        return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    int dh = open("/dev/dma_heap/system-uncached", O_RDWR);
    int mpp = open("/dev/mpp_service", O_RDWR);
    if (dh < 0 || mpp < 0) { perror("open"); return 1; }

    {
        uint32_t client = 9;
        struct mpp_msg_v1 m = { .cmd=0x100, .flags=0, .size=4, .offset=0,
                                .data_ptr=(uintptr_t)&client };
        if (send_chain(mpp, &m, 1) < 0) { fprintf(stderr, "INIT fail\n"); return 1; }
    }

    for (int i = 0; i < N_ALLOC; i++) {
        struct dma_heap_allocation_data req = {
            .len = alloc_descs[i].len, .fd_flags = O_RDWR
        };
        if (ioctl(dh, DMA_HEAP_IOCTL_ALLOC, &req) < 0) { perror("alloc"); return 1; }
        our_fd[i] = req.fd;
        printf("alloc[%d] len=%zu fd=%d\n", i, alloc_descs[i].len, our_fd[i]);

        char path[512];
        snprintf(path, sizeof(path), "%s/%s", DUMP_DIR, alloc_descs[i].name);
        void *src = load_blob(path, alloc_descs[i].len);
        if (!src) return 1;

        void *m = mmap(NULL, alloc_descs[i].len, PROT_READ | PROT_WRITE,
                       MAP_SHARED, our_fd[i], 0);
        if (m == MAP_FAILED) { perror("mmap"); return 1; }
        memcpy(m, src, alloc_descs[i].len);
        munmap(m, alloc_descs[i].len);
        free(src);
    }

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
    static const struct { uint32_t cmd, offset, size, flags; } kick_msgs[] = {
        { 0x403, 0,    48,  0x2  },
        { 0x200, 32,   100, 0x1  },
        { 0x200, 256,  196, 0x1  },
        { 0x200, 512,  60,  0x1  },
        { 0x200, 640,  152, 0x1  },
        { 0x200, 800,  20,  0x1  },
        { 0x200, 1024, 88,  0x1  },
        { 0x201, 896,  56,  0x1  },
        { 0x202, 0,    88,  0x11 },
    };

    {
        void *buf; size_t len;
        if (load_msg(kick_files[0], &buf, &len) < 0) return 1;
        struct mpp_msg_v1 m = {
            .cmd = kick_msgs[0].cmd, .flags = 0,
            .size = kick_msgs[0].size, .offset = kick_msgs[0].offset,
            .data_ptr = (uintptr_t)buf
        };
        if (send_chain(mpp, &m, 1) < 0) return 1;
        free(buf);
    }

    void *bufs[9] = {0};
    size_t lens[9] = {0};
    struct mpp_msg_v1 chain[9];
    int nchain = 0;
    for (int i = 1; i < 9; i++) {
        if (load_msg(kick_files[i], &bufs[i], &lens[i]) < 0) return 1;
        if (kick_msgs[i].cmd == 0x200 &&
            (kick_msgs[i].offset == 512 || kick_msgs[i].offset == 640)) {
            translate_fds((uint32_t*)bufs[i], lens[i] / 4);
        }
        chain[nchain].cmd = kick_msgs[i].cmd;
        chain[nchain].flags = 0;
        chain[nchain].size = kick_msgs[i].size;
        chain[nchain].offset = kick_msgs[i].offset;
        chain[nchain].data_ptr = (uintptr_t)bufs[i];
        nchain++;
    }
    printf("submitting %d-msg chain\n", nchain);
    if (send_chain(mpp, chain, nchain) < 0) return 1;

    {
        struct mpp_msg_v1 poll = { .cmd = 0x300, .flags = 0, .size = 0,
                                   .offset = 0, .data_ptr = 0 };
        if (send_chain(mpp, &poll, 1) < 0) return 1;
    }

    {
        void *m = mmap(NULL, alloc_descs[2].len, PROT_READ, MAP_SHARED, our_fd[2], 0);
        if (m == MAP_FAILED) { perror("mmap output"); return 1; }
        size_t nz = 0;
        const uint8_t *b = (const uint8_t*)m;
        for (size_t i = 0; i < alloc_descs[2].len; i++) if (b[i]) nz++;
        printf("output buffer: nonzero bytes = %zu / %zu\n", nz, alloc_descs[2].len);
        FILE *f = fopen("/tmp/replay_out.yuv", "wb");
        if (f) { fwrite(m, 1, 1382400, f); fclose(f); }
        munmap(m, alloc_descs[2].len);
    }
    return 0;
}
