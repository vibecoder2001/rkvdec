/* winreplay_h264_diff.c — Linux harness that drives our user-mode H.264
 * pipeline (parser_glue + h264_packed_tables + regbuilder_h264 + dpb)
 * against a captured BSP shim log and diffs the produced register bank
 * values reg-by-reg.
 *
 * Mirrors winreplay_h265_diff.c.  Use:
 *   winreplay_h264_diff <bitstream.h264> <mpp.shim.h264.log> [--au N]
 *
 * SPDX-License-Identifier: BSD-2-Clause-Patent
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

#include "winshim.h"
#include "mft/parser_glue.h"
#include "mft/regbuilder_h264.h"
#include "mft/h264_packed_tables.h"
#include "mft/rkvdec2_h264_regs.h"
#include "mft/dpb.h"

/* ---- Shim-log parser (identical layout to HEVC harness) ---------- */
#define MAX_SUBMSGS_PER_AU 16
#define MAX_AUS            64

typedef struct ShimSubMsg {
    uint32_t cmd, flags, size, offset, words;
    uint32_t data[512];
} ShimSubMsg;

typedef struct ShimAU {
    int        n_subs;
    ShimSubMsg subs[MAX_SUBMSGS_PER_AU];
} ShimAU;

typedef struct ShimLog {
    int    n_aus;
    ShimAU aus[MAX_AUS];
} ShimLog;

static int parse_shim_log(const char *path, ShimLog *out) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    out->n_aus = 0;
    ShimAU *cur_au = NULL;
    ShimSubMsg *cur_sub = NULL;
    int started = 0;
    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "MPP IOCTL")) { cur_sub = NULL; continue; }
        unsigned cmd, flags, size, off;
        unsigned long long data_ptr;
        if (sscanf(line, " [sub %*d] cmd=0x%x flags=0x%x size=%u offset=%u data=0x%llx",
                   &cmd, &flags, &size, &off, &data_ptr) == 5) {
            cur_sub = NULL;
            if (cmd == 0x200 && off == 32) {
                if (out->n_aus >= MAX_AUS) { fclose(f); return -1; }
                cur_au = &out->aus[out->n_aus++];
                cur_au->n_subs = 0;
                started = 1;
            }
            if (cmd == 0x300) { cur_au = NULL; started = 0; continue; }
            if (!cur_au || !started) continue;
            if (cmd != 0x200 && cmd != 0x201 && cmd != 0x202) continue;
            if (cur_au->n_subs >= MAX_SUBMSGS_PER_AU) continue;
            cur_sub = &cur_au->subs[cur_au->n_subs++];
            cur_sub->cmd = cmd; cur_sub->flags = flags;
            cur_sub->size = size; cur_sub->offset = off;
            cur_sub->words = 0;
            memset(cur_sub->data, 0, sizeof(cur_sub->data));
            continue;
        }
        if (cur_sub) {
            const char *p = line;
            while (*p == ' ') p++;
            if (*p != '[') continue;
            int idx;
            char *e = NULL;
            if (sscanf(p, "[%d]", &idx) != 1) continue;
            const char *q = strchr(p, ']'); if (!q) continue;
            q++;
            for (int j = 0; j < 8; j++) {
                while (*q == ' ') q++;
                if (!isxdigit((unsigned char)*q)) break;
                unsigned long v = strtoul(q, &e, 16);
                if (e == q) break;
                if (idx + j < (int)(sizeof(cur_sub->data)/4))
                    cur_sub->data[idx + j] = (uint32_t)v;
                q = e;
            }
            uint32_t end = (uint32_t)idx + 8;
            if (end > cur_sub->words) cur_sub->words = end;
        }
    }
    fclose(f);
    for (int a = 0; a < out->n_aus; a++) {
        for (int s = 0; s < out->aus[a].n_subs; s++) {
            uint32_t cap = out->aus[a].subs[s].size / 4;
            if (out->aus[a].subs[s].words > cap)
                out->aus[a].subs[s].words = cap;
        }
    }
    return 0;
}

/* ---- Annex-B H.264 NAL framing + AU collation -------------------- */
typedef struct nal_iter {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;
} nal_iter;

static size_t find_start_code(const uint8_t *buf, size_t len, size_t from) {
    for (size_t i = from; i + 3 <= len; i++) {
        if (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1) return i + 3;
        if (i + 4 <= len && buf[i] == 0 && buf[i+1] == 0 &&
            buf[i+2] == 0 && buf[i+3] == 1) return i + 4;
    }
    return SIZE_MAX;
}

/* H.264 slice NAL types: 1 (non-IDR) and 5 (IDR). */
static int h264_nal_is_slice(uint8_t hdr0) {
    uint8_t t = hdr0 & 0x1F;
    return t == 1 || t == 5;
}

static int h264_au_next(nal_iter *it, size_t *au_off, size_t *au_len,
                        size_t *slice_off) {
    if (it->pos >= it->len) return 0;
    size_t first_sc = find_start_code(it->buf, it->len, it->pos);
    if (first_sc == SIZE_MAX) return 0;
    size_t sc_start = first_sc - 3;
    if (sc_start > 0 && it->buf[sc_start - 1] == 0) sc_start--;
    size_t nh = first_sc;
    int found = 0;
    size_t end = it->len;
    size_t slice_nh = 0;
    while (nh < it->len) {
        if (h264_nal_is_slice(it->buf[nh])) {
            slice_nh = nh;
            size_t nxt = find_start_code(it->buf, it->len, nh + 1);
            if (nxt == SIZE_MAX) end = it->len;
            else { end = nxt - 3; if (end > 0 && it->buf[end - 1] == 0) end--; }
            found = 1; break;
        }
        size_t nxt = find_start_code(it->buf, it->len, nh + 1);
        if (nxt == SIZE_MAX) break;
        nh = nxt;
    }
    if (!found) return 0;
    *au_off = sc_start;
    *au_len = end - sc_start;
    if (slice_off) {
        size_t s = slice_nh - 3;
        *slice_off = s;
    }
    it->pos = end;
    return 1;
}

/* ---- Bank descriptors (H.264 cmd=0x200/0x201 sub-messages) -------- *
 * From mpp.shim.h264.dma.log, AU 0:
 *   sub 0: cmd=0x200 off=32   size=100  regs  8.. 32
 *   sub 1: cmd=0x200 off=256  size=196  regs 64..112
 *   sub 2: cmd=0x200 off=512  size=60   regs 128..142
 *   sub 3: cmd=0x200 off=640  size=152  regs 160..197
 *   sub 4: cmd=0x200 off=800  size=20   regs 200..204
 *   sub 5: cmd=0x200 off=1024 size=88   regs 256..277
 *   sub 6: cmd=0x201 off=896  size=56   regs 224..237
 */
typedef struct BankSpec { uint32_t off, size, first_idx, cmd; } BankSpec;
static const BankSpec kBanks[] = {
    {  32, 100,   8, 0x200 },
    { 256, 196,  64, 0x200 },
    { 512,  60, 128, 0x200 },
    { 640, 152, 160, 0x200 },
    { 800,  20, 200, 0x200 },
    {1024,  88, 256, 0x200 },
    { 896,  56, 224, 0x201 },
};
#define N_BANKS (sizeof(kBanks)/sizeof(kBanks[0]))

/* ---- Diff one bank ---------------------------------------------- */
static int diff_bank(const char *label,
                     const uint32_t *expected, uint32_t exp_words,
                     const uint32_t *got,      uint32_t got_words,
                     uint32_t first_idx,
                     const uint8_t *iova_mask)
{
    uint32_t n = exp_words > got_words ? exp_words : got_words;
    int diffs = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t e = (i < exp_words) ? expected[i] : 0;
        uint32_t g = (i < got_words ) ? got[i]      : 0;
        if (iova_mask && iova_mask[i]) continue;
        if (e != g) {
            if (diffs == 0) printf("  [bank %s, base reg%u]\n", label, first_idx);
            printf("    reg%u (sub-w%u): bsp=0x%08x  ours=0x%08x  XOR=0x%08x\n",
                   first_idx + i, i, e, g, e ^ g);
            diffs++;
        }
    }
    return diffs;
}

static uint8_t *read_file_path(const char *path, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return NULL; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t*)malloc(n);
    fread(buf, 1, n, f); fclose(f);
    *out_len = n;
    return buf;
}

static void mark_iova_positions(const H264RegWriteList *rl,
                                uint32_t bank_first_idx, uint32_t bank_size,
                                uint8_t *mask)
{
    uint32_t n_words = bank_size / 4;
    memset(mask, 0, n_words);
    for (uint32_t i = 0; i < rl->count; i++) {
        const RKMPP_REG_WRITE *e = &rl->entries[i];
        if (!e->BufferHandle) continue;
        uint32_t idx = e->Offset / 4;
        if (idx >= bank_first_idx && idx < bank_first_idx + n_words)
            mask[idx - bank_first_idx] = 1;
    }
}

static void splat_bank(const H264RegWriteList *rl,
                       uint32_t bank_first_idx, uint32_t bank_size,
                       uint32_t *out)
{
    uint32_t n = bank_size / 4;
    memset(out, 0, n * 4);
    for (uint32_t i = 0; i < rl->count; i++) {
        const RKMPP_REG_WRITE *e = &rl->entries[i];
        uint32_t idx = e->Offset / 4;
        if (idx < bank_first_idx || idx >= bank_first_idx + n) continue;
        if (e->BufferHandle) out[idx - bank_first_idx] = 0xDEADBEEFu;
        else                 out[idx - bank_first_idx] = e->Value;
    }
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <bitstream.h264> <mpp.shim.h264.log> [--au N]\n", argv[0]);
        return 1;
    }
    int target_au = -1;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--au") && i + 1 < argc) target_au = atoi(argv[++i]);
    }

    size_t bs_len;
    uint8_t *bs = read_file_path(argv[1], &bs_len);
    if (!bs) return 1;

    static ShimLog log;
    if (parse_shim_log(argv[2], &log)) return 1;
    fprintf(stderr, "shim log: %d AUs captured\n", log.n_aus);

    static H264ParseResult parsed;
    memset(&parsed, 0, sizeof(parsed));
    static uint8_t scratch[2 * 65536];

    static DpbPoolEntry pool[DPB_MAX_SLOTS];
    for (int i = 0; i < DPB_MAX_SLOTS; i++) {
        pool[i].output_frame = 0x100000ULL + i * 0x10000ULL;
        pool[i].colmv        = 0x800000ULL + i * 0x10000ULL;
    }
    static DpbCtx dpb;
    Dpb_Init(&dpb, pool, DPB_MAX_SLOTS);

    nal_iter it = { .buf = bs, .len = bs_len, .pos = 0 };
    int au_idx = 0;
    int total_diffs = 0;

    H264RcbInfo rcb_info[10];

    while (1) {
        size_t au_off, au_len, slice_off;
        if (!h264_au_next(&it, &au_off, &au_len, &slice_off)) break;
        if (au_idx >= log.n_aus) break;

        H264ParseStatus ps = H264ParseAccessUnit(bs + au_off, au_len,
                                                 scratch, sizeof(scratch), &parsed);
        if (ps != H264_PARSE_OK || !parsed.has_slice) {
            fprintf(stderr, "AU %d: parse status %d, has_slice=%d — skipping\n",
                    au_idx, ps, parsed.has_slice);
            au_idx++;
            continue;
        }

        DpbSelection sel;
        memset(&sel, 0, sizeof(sel));
        Dpb_Select(&dpb, &parsed, &sel);

        /* Copy DPB selection into the parser's decode_params for the
         * regbuilder (it consumes parsed.decode.dpb directly). */
        memcpy(parsed.decode.dpb, sel.dpb_entries, sizeof(sel.dpb_entries));

        H264BufferRefs bufs;
        memset(&bufs, 0, sizeof(bufs));
        size_t slice_region_off = slice_off - au_off;
        size_t slice_region_len = au_len - slice_region_off;
        bufs.bitstream       = 1;
        bufs.bitstream_size  = (uint32_t)slice_region_len;
        bufs.bitstream_offset= 0;
        bufs.output_frame    = 2;
        bufs.colmv_cur       = 3;
        bufs.error_ref       = 0;
        uint32_t W = ((uint32_t)parsed.sps.pic_width_in_mbs_minus1 + 1) * 16u;
        uint32_t Hp = ((uint32_t)parsed.sps.pic_height_in_map_units_minus1 + 1) * 16u;
        H264GetRcbBufferSizes(rcb_info, W, Hp);
        for (int i = 0; i < 10; i++) {
            bufs.rcb[i] = 4;
            bufs.rcb_offset[i] = rcb_info[i].offset;
        }
        bufs.pps_table        = 5;
        bufs.rps_table        = 6;
        bufs.scaling_list     = 0;
        bufs.cabac_init_table = 7;
        /* Synthetic IOVA pattern that mirrors BSP's near_index fill:
         * sel.refs[i] now carries a real DPB slot iova (own iova for valid
         * slots, near-propagated iova for empties).  We translate the per-
         * slot pool[].output_frame back into the synthetic 8+slot value
         * that the diff harness uses. */
        for (int i = 0; i < 16; i++) {
            uint64_t out_iova = sel.refs[i];
            uint64_t cmv_iova = sel.ref_colmv[i];
            int      out_slot = -1;
            int      cmv_slot = -1;
            for (int s = 0; s < DPB_MAX_SLOTS; s++) {
                if (pool[s].output_frame == out_iova) out_slot = s;
                if (pool[s].colmv        == cmv_iova) cmv_slot = s;
            }
            bufs.refs[i]      = (out_slot >= 0) ? (8u  + (uint64_t)out_slot) : 0;
            bufs.ref_colmv[i] = (cmv_slot >= 0) ? (24u + (uint64_t)cmv_slot) : 0;
        }

        static H264RegWriteList rl;
        memset(&rl, 0, sizeof(rl));
        H264RegBuildStatus rs = H264BuildRegisterList(&parsed, &bufs,
                                                      sel.current_slot, &rl);
        if (rs != H264_REGBUILD_OK) {
            fprintf(stderr, "AU %d: regbuild fail %d\n", au_idx, rs);
            au_idx++;
            continue;
        }

        Dpb_OnDecodeComplete(&dpb);

        if (target_au >= 0 && au_idx != target_au) {
            au_idx++;
            continue;
        }

        const char *st = "I";
        if (parsed.slice.slice_type == V4L2_H264_SLICE_TYPE_P) st = "P";
        else if (parsed.slice.slice_type == V4L2_H264_SLICE_TYPE_B) st = "B";
        printf("\n==== AU %d (frame_num=%u, POC=%d, %s, %s) — diff vs shim AU %d ====\n",
               au_idx, parsed.decode.frame_num,
               (int32_t)parsed.decode.top_field_order_cnt,
               (parsed.decode.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)
                   ? "IDR" : "non-IDR", st, au_idx);

        ShimAU *shim = &log.aus[au_idx];
        for (size_t b = 0; b < N_BANKS; b++) {
            uint32_t expected[100] = {0};
            uint32_t got[100];
            uint8_t  iova_mask[100];
            uint32_t exp_words = 0;
            uint32_t cap = kBanks[b].size / 4;
            int found = 0;
            for (int s = 0; s < shim->n_subs; s++) {
                if (shim->subs[s].cmd != kBanks[b].cmd) continue;
                if (shim->subs[s].offset != kBanks[b].off) continue;
                if (shim->subs[s].size != kBanks[b].size) continue;
                memcpy(expected, shim->subs[s].data, kBanks[b].size);
                exp_words = kBanks[b].size / 4;
                found = 1;
                break;
            }
            if (!found) {
                printf("  [bank @off=%u size=%u]: no matching shim sub — skipped\n",
                       kBanks[b].off, kBanks[b].size);
                continue;
            }
            splat_bank(&rl, kBanks[b].first_idx, kBanks[b].size, got);
            mark_iova_positions(&rl, kBanks[b].first_idx, kBanks[b].size, iova_mask);

            char label[24];
            snprintf(label, sizeof(label), "off=%u", kBanks[b].off);
            int d = diff_bank(label, expected, exp_words, got, cap,
                              kBanks[b].first_idx, iova_mask);
            total_diffs += d;
            if (d == 0)
                printf("  [bank %s base reg%u] OK (%u words match)\n",
                       label, kBanks[b].first_idx, cap);
        }

        au_idx++;
    }

    printf("\n==== TOTAL DIFFS: %d ====\n", total_diffs);
    return total_diffs ? 1 : 0;
}
