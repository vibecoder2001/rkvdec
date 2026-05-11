/* linux_mpp_decode.c — H.264 decode harness for /dev/mpp_service on RK3588 Linux.
 *
 * Usage: linux_mpp_decode <stream.h264> <width> <height> [out.yuv]
 *
 * Feeds each Annex-B access unit through our parser→DPB→regbuilder pipeline
 * and submits to the BSP mpp_service kernel driver.  NV12 output frames are
 * written to out.yuv in decode (bitstream) order.
 *
 * Reference comparison: ffmpeg -vcodec h264_rkmpp -i stream.h264 -pix_fmt nv12 ref.yuv
 * If out.yuv matches ref.yuv, our user-mode pipeline is correct; H.264 bugs
 * are in rkmpp.sys (Windows kernel driver), not in parser/regbuilder/dpb.
 */
#include "winshim.h"
#include "mft/parser_glue.h"
#include "mft/regbuilder_h264.h"
#include "mft/h264_packed_tables.h"
#include "mft/dpb.h"
#include "mpp_svc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

/* ---- Buffer sizing -------------------------------------------------------- */

#define BITSTREAM_BUF_SIZE  (2u * 1024u * 1024u)
#define DPB_SLOTS           DPB_MAX_SLOTS

static uint32_t nv12_frame_size(uint32_t w, uint32_t h) {
    uint32_t mb_w = (w + 15) / 16;
    uint32_t mb_h = (h + 15) / 16;
    uint32_t stride = mb_w * 16;
    uint32_t y_size = stride * (mb_h * 16);
    return y_size + y_size / 2;
}

static uint32_t colmv_size(uint32_t w, uint32_t h) {
    /* Per BSP vdpu34x_get_colmv_size with H264 params:
     *   ctu_size=16, colmv_size=4, colmv_byte=16, compress=1.
     *   segment_w = 64*4*4/16 = 64;  segment_h = 16
     *   seg_cnt_w = align(w,64)/64;  seg_cnt_h = align(h,16)/16
     *   head    = align(seg_cnt_w,16) * seg_cnt_h
     *   payload = seg_cnt_w * seg_cnt_h * 64 * 16
     *   total   = align(head + payload, 128)
     * Frame-mode-only h264 streams set compress=1; we size for that. */
    uint32_t seg_cnt_w = (w + 63) / 64;
    uint32_t seg_cnt_h = (h + 15) / 16;
    uint32_t head    = ((seg_cnt_w + 15) & ~15u) * seg_cnt_h;
    uint32_t payload = seg_cnt_w * seg_cnt_h * 64 * 16;
    uint32_t total   = head + payload;
    return (total + 127u) & ~127u;
}

/* ---- HarnessCtx ----------------------------------------------------------- */

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
    if (MppSvc_SendCodecInfo(ctx->svc_fd, w, h) < 0) return -1;

    uint32_t frame_sz  = nv12_frame_size(w, h);
    uint32_t cmv_sz    = colmv_size(w, h);
    uint32_t rcb_total = H264GetRcbBufferSizes(ctx->rcb_info, w, h);

    if (alloc_or_die("bitstream",  BITSTREAM_BUF_SIZE,                              &ctx->bitstream)    < 0) return -1;
    if (alloc_or_die("cabac_init", RKH264_CABAC_INIT_SIZE + RKH264_TABLE_TAIL_PAD,  &ctx->cabac_init)   < 0) return -1;
    if (alloc_or_die("pps_table",  RKH264_SPSPPS_UNIT_SIZE + RKH264_TABLE_TAIL_PAD, &ctx->pps_table)    < 0) return -1;
    if (alloc_or_die("rps_table",  RKH264_RPS_SIZE + RKH264_TABLE_TAIL_PAD,         &ctx->rps_table)    < 0) return -1;
    if (alloc_or_die("scaling",    RKH264_SCALING_LIST_SIZE + RKH264_TABLE_TAIL_PAD, &ctx->scaling_list) < 0) return -1;
    if (alloc_or_die("rcb",        rcb_total,                                        &ctx->rcb)          < 0) return -1;
    if (alloc_or_die("error_ref",  frame_sz,                                         &ctx->error_ref)    < 0) return -1;

    for (int i = 0; i < DPB_SLOTS; i++) {
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "pool_output[%d]", i);
        if (alloc_or_die(lbl, frame_sz, &ctx->pool_output[i]) < 0) return -1;
        snprintf(lbl, sizeof(lbl), "pool_colmv[%d]", i);
        if (alloc_or_die(lbl, cmv_sz,   &ctx->pool_colmv[i])  < 0) return -1;

        ctx->dpb_pool[i].output_frame = (uint64_t)ctx->pool_output[i].dma_fd;
        ctx->dpb_pool[i].colmv        = (uint64_t)ctx->pool_colmv[i].dma_fd;
    }
    Dpb_Init(&ctx->dpb, ctx->dpb_pool, DPB_SLOTS);

    /* Import every DMA fd into the session so mpp_translate_reg_address works. */
    {
        int all_fds[7 + DPB_SLOTS * 2];
        int n = 0;
        all_fds[n++] = ctx->bitstream.dma_fd;
        all_fds[n++] = ctx->cabac_init.dma_fd;
        all_fds[n++] = ctx->pps_table.dma_fd;
        all_fds[n++] = ctx->rps_table.dma_fd;
        all_fds[n++] = ctx->scaling_list.dma_fd;
        all_fds[n++] = ctx->rcb.dma_fd;
        all_fds[n++] = ctx->error_ref.dma_fd;
        for (int i = 0; i < DPB_SLOTS; i++) {
            all_fds[n++] = ctx->pool_output[i].dma_fd;
            all_fds[n++] = ctx->pool_colmv[i].dma_fd;
        }
        if (MppSvc_ImportFds(ctx->svc_fd, all_fds, n) < 0) return -1;
    }

    fprintf(stderr, "FDs: svc=%d bitstream=%d cabac=%d pps=%d rps=%d "
            "scaling=%d rcb=%d error_ref=%d pool_out[0]=%d pool_cmv[0]=%d\n",
            ctx->svc_fd, ctx->bitstream.dma_fd, ctx->cabac_init.dma_fd,
            ctx->pps_table.dma_fd, ctx->rps_table.dma_fd,
            ctx->scaling_list.dma_fd, ctx->rcb.dma_fd, ctx->error_ref.dma_fd,
            ctx->pool_output[0].dma_fd, ctx->pool_colmv[0].dma_fd);

    size_t n_words;
    const uint32_t *cabac = H264GetCabacInitTable(&n_words);
    memcpy(ctx->cabac_init.cpu_va, cabac, n_words * 4);
    memset((uint8_t *)ctx->cabac_init.cpu_va + n_words * 4, 0, RKH264_TABLE_TAIL_PAD);

    memset(ctx->error_ref.cpu_va, 0, ctx->error_ref.size);
    /* Initialize scaling list to H.264 flat-16 defaults; overwritten per-frame
     * when the stream carries an explicit scaling matrix. */
    H264PackScalingList((uint8_t *)ctx->scaling_list.cpu_va, NULL, 0);

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

/* ---- Annex-B AU walker ---------------------------------------------------- */

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

/* ---- Per-AU decode -------------------------------------------------------- */

/* Out parameters reported by decode_one_au — let the caller drive output
 * reordering (display order != decode order for B-frame streams). */
typedef struct DecodedAuInfo {
    int32_t  top_poc;
    uint8_t  is_idr;
    uint8_t  has_frame;       /* 1 if pool_output[slot] holds a fresh frame */
    uint32_t max_num_ref_frames;
    /* Caller supplies a destination buffer of nv12_frame_size bytes; we
     * memcpy the codec output here so the pool slot can be reused. */
    uint8_t *yuv_dst;
    uint32_t yuv_size;
} DecodedAuInfo;

static int decode_one_au(HarnessCtx *ctx, const uint8_t *au, size_t au_len,
                         size_t slice_off, DecodedAuInfo *info) {
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

    /* 2. DPB select */
    memset(&sel, 0, sizeof(sel));
    Dpb_Select(&ctx->dpb, &parsed, &sel);

    /* Copy DPB entries into decode_params.dpb so regbuilder can read them */
    memcpy(parsed.decode.dpb, sel.dpb_entries, sizeof(sel.dpb_entries));

    /* 3. Upload bitstream — full AU (SPS/PPS/SEI/slice) so the HW pre-parser
     * walks all NALs.  BSP HAL passes the full input packet length to reg016. */
    (void)slice_off;
    size_t slice_len = au_len;
    if (slice_len > BITSTREAM_BUF_SIZE) {
        fprintf(stderr, "  AU too large: %zu bytes\n", slice_len);
        return -1;
    }
    memcpy(ctx->bitstream.cpu_va, au, slice_len);
    /* Zero a 64-byte tail pad — some BSP paths align reg016 length up and
     * memset the tail; harmless if HW reads it on read-ahead. */
    if (slice_len + 64 <= BITSTREAM_BUF_SIZE)
        memset((uint8_t *)ctx->bitstream.cpu_va + slice_len, 0, 64);

    /* 4. Pack RPS + PPS tables */
    H264PackFrameRps((uint8_t *)ctx->rps_table.cpu_va,
                     parsed.decode.frame_num,
                     parsed.sps.log2_max_frame_num_minus4,
                     sel.dpb_entries, sel.ref_lists);

    H264PackSpsPpsUnit((uint8_t *)ctx->pps_table.cpu_va,
                       &parsed.sps, &parsed.pps,
                       sel.dpb_entries, /*field_pic=*/0);

    H264PackScalingList((uint8_t *)ctx->scaling_list.cpu_va,
                        parsed.has_scaling_matrix ? &parsed.scaling_matrix : NULL,
                        parsed.has_scaling_matrix);

    /* 5. Build buffer refs */
    H264BufferRefs bufs;
    memset(&bufs, 0, sizeof(bufs));
    bufs.bitstream        = (uint64_t)ctx->bitstream.dma_fd;
    bufs.bitstream_offset = 0;
    bufs.bitstream_size   = (uint32_t)((slice_len + 3u) & ~3u); /* align to 4 */
    bufs.output_frame     = (uint64_t)ctx->pool_output[sel.current_slot].dma_fd;
    bufs.colmv_cur        = (uint64_t)ctx->pool_colmv[sel.current_slot].dma_fd;
    bufs.error_ref        = (uint64_t)ctx->error_ref.dma_fd;
    bufs.pps_table        = (uint64_t)ctx->pps_table.dma_fd;
    bufs.rps_table        = (uint64_t)ctx->rps_table.dma_fd;
    bufs.cabac_init_table = (uint64_t)ctx->cabac_init.dma_fd;
    bufs.scaling_list     = (uint64_t)ctx->scaling_list.dma_fd;
    for (int i = 0; i < (int)RKH264_RCB_COUNT; i++) {
        bufs.rcb[i]        = (uint64_t)ctx->rcb.dma_fd;
        bufs.rcb_offset[i] = ctx->rcb_info[i].offset;
    }
    /* Use the compact-ordered iovas Dpb_Select filled in sel.refs/ref_colmv
     * — they match sel.dpb_entries[] so reg99..102 (which the regbuilder
     * derives from dpb_entries) lines up with reg164..179 / reg181..196.
     * A 1:1 dpb_slot→pool_slot mapping aliases ref[current_slot] onto
     * output_frame and HW reads its own decode output (timeout watchdog). */
    for (int i = 0; i < DPB_SLOTS; i++) {
        bufs.refs[i]      = sel.refs[i];
        bufs.ref_colmv[i] = sel.ref_colmv[i];
    }

    /* 6. Build register list */
    memset(&rl, 0, sizeof(rl));
    H264RegBuildStatus rs = H264BuildRegisterList(&parsed, &bufs,
                                                  sel.current_slot, &rl);
    if (rs != H264_REGBUILD_OK) {
        fprintf(stderr, "  H264BuildRegisterList failed: %d\n", rs);
        return -1;
    }

    /* 7. Build buf_map: map every dma_fd that may appear as a BufferHandle */
    MppSvcBufMap buf_map[64];
    int n_map = 0;
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

    /* 8. Submit + poll.  irq_buf is populated by the kernel via SET_REG_READ. */
    uint32_t irq_buf[MPP_IRQ_READBACK_WORDS];
    memset(irq_buf, 0, sizeof(irq_buf));
    if (MppSvc_Submit(ctx->svc_fd, &rl, buf_map, n_map, irq_buf,
                      ctx->width, ctx->height) < 0) return -1;
    int poll_rc = MppSvc_Poll(ctx->svc_fd, 2000, irq_buf);
    uint32_t irq_sta = irq_buf[0];
    if (poll_rc != 0) {
        fprintf(stderr, "  Poll failed: rc=%d irq=0x%08x\n", poll_rc, irq_sta);
        Dpb_OnDecodeComplete(&ctx->dpb);
        return -1;
    }

    /* 9. Hand the decoded NV12 back to the caller for reorder. */
    if (info && info->yuv_dst) {
        uint32_t frame_sz = nv12_frame_size(ctx->width, ctx->height);
        if (info->yuv_size >= frame_sz)
            memcpy(info->yuv_dst, ctx->pool_output[sel.current_slot].cpu_va,
                   frame_sz);
        info->has_frame = 1;
    }
    if (info) {
        info->top_poc = parsed.decode.top_field_order_cnt;
        info->is_idr  = (parsed.decode.flags &
                         V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) != 0;
        info->max_num_ref_frames = parsed.sps.max_num_ref_frames
                                       ? parsed.sps.max_num_ref_frames : 1;
    }

    Dpb_OnDecodeComplete(&ctx->dpb);
    return 0;
}

/* ---- Display-order reorder queue ----------------------------------------- *
 * Mirrors DecodeEngine reorder_q semantics (mft/engine/decode_engine.cpp):
 *   - Push (poc, yuv) onto the queue per Submit.
 *   - While queue.size() > max_num_reorder_pics, spill the lowest-POC
 *     entry to out_fp.
 *   - On IDR boundary, spill the entire queue first (POC resets at IDR;
 *     pre-IDR tail frames have higher POCs than post-IDR frames and would
 *     otherwise stay stuck).
 *   - On EOF, drain everything in POC order.
 *
 * Buffer ownership: each queue entry owns a malloc'd YUV buffer of
 * nv12_frame_size bytes.  No external pool holds. */
typedef struct ReorderEntry {
    int32_t  poc;
    uint8_t *yuv;          /* malloc'd, nv12_frame_size bytes */
} ReorderEntry;

static int g_emit_idx = 0;
static void reorder_spill_lowest(ReorderEntry *q, int *n, FILE *out_fp,
                                 uint32_t frame_sz) {
    if (*n == 0) return;
    int min_i = 0;
    for (int i = 1; i < *n; i++)
        if (q[i].poc < q[min_i].poc) min_i = i;
    if (out_fp) fwrite(q[min_i].yuv, 1, frame_sz, out_fp);
    if (getenv("LINUXMPP_REORDER_DBG"))
        fprintf(stderr, "  emit[%d] poc=%d\n", g_emit_idx, q[min_i].poc);
    g_emit_idx++;
    free(q[min_i].yuv);
    /* Shift remaining entries down. */
    for (int i = min_i; i < *n - 1; i++) q[i] = q[i + 1];
    (*n)--;
}

/* ---- main ----------------------------------------------------------------- */

static uint8_t *read_file(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t *)malloc((size_t)n);
    if (!buf) { fclose(f); return NULL; }
    fread(buf, 1, (size_t)n, f); fclose(f);
    *out_len = (size_t)n; return buf;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <stream.h264> <width> <height> [out.yuv] [--frames N]\n",
                argv[0]);
        return 1;
    }
    uint32_t W = (uint32_t)atoi(argv[2]);
    uint32_t H = (uint32_t)atoi(argv[3]);
    FILE *out_fp = NULL;
    int max_frames = INT_MAX;
    int first_opt = 4;
    if (argc >= 5 && argv[4][0] != '-') {
        out_fp = fopen(argv[4], "wb");
        if (!out_fp) { perror(argv[4]); return 1; }
        first_opt = 5;
    }
    for (int i = first_opt; i < argc; i++) {
        if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            max_frames = atoi(argv[++i]);
    }

    size_t bs_len;
    uint8_t *bs = read_file(argv[1], &bs_len);
    if (!bs) return 1;

    HarnessCtx ctx;
    if (HarnessCtx_Init(&ctx, W, H) < 0) { free(bs); return 1; }

    NalIter it = { .buf = bs, .len = bs_len, .pos = 0 };
    int au_idx = 0, decoded = 0, failed = 0;
    uint32_t frame_sz = nv12_frame_size(W, H);

    /* Reorder queue: at most max_num_ref_frames+1 entries simultaneously
     * for sane streams, but size to DPB_SLOTS for safety. */
    ReorderEntry q[DPB_SLOTS + 1];
    int  n_q = 0;
    uint32_t max_reorder = 1;   /* Updated from SPS on first slice. */

    while (au_idx < max_frames) {
        size_t au_off, au_len, slice_off;
        if (!h264_au_next(&it, &au_off, &au_len, &slice_off)) break;
        size_t rel_slice = slice_off - au_off;
        fprintf(stderr, "AU %d: off=%zu len=%zu slice_off=%zu (rel=%zu)\n",
                au_idx, au_off, au_len, slice_off, rel_slice);

        DecodedAuInfo info = {0};
        info.yuv_dst  = (uint8_t *)malloc(frame_sz);
        info.yuv_size = frame_sz;
        if (!info.yuv_dst) { fprintf(stderr, "OOM\n"); failed++; au_idx++; continue; }

        int rc = decode_one_au(&ctx, bs + au_off, au_len, rel_slice, &info);
        if (rc != 0 || !info.has_frame) {
            free(info.yuv_dst);
            failed++;
            au_idx++;
            continue;
        }
        decoded++;

        /* Spec-compliant H.264 reorder for output: see DecodeEngine
         * reorder_q (mft/engine/decode_engine.cpp). */
        max_reorder = info.max_num_ref_frames;

        /* IDR boundary: POC resets at IDR (h264_idr_reorder_fix memory).
         * Drain pre-IDR tail in POC order before pushing the IDR itself. */
        if (info.is_idr) {
            while (n_q > 0)
                reorder_spill_lowest(q, &n_q, out_fp, frame_sz);
        }

        q[n_q].poc = info.top_poc;
        q[n_q].yuv = info.yuv_dst;   /* transfer ownership */
        n_q++;

        while ((uint32_t)n_q > max_reorder)
            reorder_spill_lowest(q, &n_q, out_fp, frame_sz);

        au_idx++;
    }

    /* Drain remaining queued frames in POC order. */
    while (n_q > 0)
        reorder_spill_lowest(q, &n_q, out_fp, frame_sz);

    fprintf(stderr, "Done: %d decoded, %d failed\n", decoded, failed);
    HarnessCtx_Shutdown(&ctx);
    if (out_fp) fclose(out_fp);
    free(bs);
    return failed > 0 ? 1 : 0;
}
