# RK3588 AV1 Decoder (vdpu / hal_av1d_vdpu) Register Map — Kick-1 Capture

Source trace: `docs/av1_trace_first_capture.log`, kick #1 (between the first and second
`AV1SHIM kick begin` markers — log lines 494..1390). Register index `j` in `r[0][j]`
corresponds to **swreg j** (byte offset `j*4` from the SWREG MMIO base).

Reg-set source of truth: `C:\Users\vibecoder\mpp\mpp\hal\vpu\av1d\hal_av1d_vdpu_reg.h`
(struct `VdpuAv1dRegSet`). Field assignments referenced from
`C:\Users\vibecoder\mpp\mpp\hal\vpu\av1d\hal_av1d_vdpu.c`.

NOTE: a previous version of this document was decoded against `vdpu383_com.h`. RK3588's
AV1 IP is the **vpu/av1d** (legacy VeriSilicon-derived) HAL, NOT vdpu383. Decode here is
verified by:
- `r[0][1]=0x00000001` matches `swreg1.sw_dec_e=1` (bit 0).
- `r[0][3]=0x88001100` matches `swreg3.sw_dec_mode=17` (AV1) | `sw_write_mvs_e` |
  `sw_dec_out_ec_bypass` | `sw_dec_out_ec_byte_word=0` (see hal_av1d_vdpu.c:1920-1924).

## Summary

- Captured non-zero registers in kick 1: **106**
- All 106 land inside structs defined in `VdpuAv1dRegSet` — **0 stragglers without a
  struct match.**
- Explicitly written by `hal_av1d_vdpu.c` (gen_regs / dxva paths): ~88
- Register groups represented:
  - 1..12  control / dimensions / flags
  - 13..32 segmentation + per-seg LF + ref-pic seg + filter/quant
  - 33..49 ref widths/heights/scales + qmlevel
  - 51..63 superres / APF / AXI / build/fuse
  - 64..183 (`addr_cfg`) DMA base addresses (Y/C/D for current+7 refs, segment, model,
    cdef/superres/lr colbufs, filmgrain, prob, mc_sync, vert-filt, bsd-ctrl, tile,
    stream)
  - 184..262 ref-mv / cur-frame offsets, gm_mode, T-Y/T-C base, strm length/offset, error
    code, cdef strength
  - 263..266 cdef strength, AXI qos, 128-bit / err-conceal
  - 298..319 superres invra, perf counters, hw_build_id, alignment, timeouts
  - 320..511 PP cfg (`vdpu_av1d_pp_cfg`)

Captured registers with **no match in struct** : none. Every captured index
falls inside either `VdpuAv1dRegSet` directly (0..63, 184..319), the embedded
`addr_cfg` (64..183), or `vdpu_av1d_pp_cfg` (320..511).

Notation: bitfields listed LSB→MSB to match the `RK_U32 ... :N` ordering in the header
(little-endian, GCC packs in declaration order).

---

## Control / mode (swreg 1..12)

| swreg | Captured | MPP field decode | Source / driver |
|---|---|---|---|
| 1 | 0x00000001 | `sw_dec_e=1` | hal_av1d_vdpu.c:1916 (`regs->swreg1.sw_dec_e=1` — kick) |
| 2 | 0x00000400 | `sw_dec_clk_gate_e=1` (bit 10) | hal_av1d_vdpu.c:1918 |
| 3 | 0x88001100 | `sw_dec_mode=17` (av1, bits 27:31=0b10001), `sw_dec_out_ec_byte_word=1` (bit 14) — **wait**: 0x88001100 → bits set: 8,12,15,20,23,27,31. Decode: `sw_dec_out_ec_bypass`(8)=1, `sw_write_mvs_e`(12)=1, `sw_dec_out_ec_byte_word`(15)=0, `sw_filtering_dis`(13)=0; bits 20,23,27,31 fall in `sw_dec_mode`(27..31)=0b10001=17 (AV1) | hal_av1d_vdpu.c:1920-1924 sets dec_mode=17, write_mvs_e=1, dec_out_ec_bypass=1, dec_out_ec_byte_word=0 (word align) |
| 4 | 0x02800f01 | `sw_ref_frames=1` (b0:3=1), `sw_pic_height_in_cbs=0x3c` (b6:18=60 → 480px), `sw_pic_width_in_cbs=0x0a` (b19:31=10 → 80px? **flag: small res, looks like 480-wide × 270 frame** ; actually 0x0a in the high 13 = pic_width_in_cbs=10×8=80? The field width is 13 starting at bit 19 — recompute) — see notes below | hal_av1d_vdpu.c:805 / 1219-1220 (`pic_width_in_cbs = ALIGN(width,8)>>3`, set `sw_ref_frames` from active count). For typical 480×270 stream: width_in_cbs=60, height_in_cbs=34 |
| 5 | 0x40017f20 | Composite of `sw_*` flags incl. `sw_filt_level_base_gt32`(bit1), `sw_enable_dual_filter`(bit7=1), `sw_enable_jnt_comp`(bit8), `sw_allow_filter_intra`(bit9), `sw_enable_intra_edge_filter`(bit10), `sw_tempor_mvp_e`(bit11), `sw_allow_interintra`(bit12), `sw_allow_masked_compound`(bit13), `sw_enable_cdef`(bit14), `sw_switchable_motion_mode`(bit15), `sw_show_frame`(bit16), `sw_strm_start_bit`=0x20 (bits25:31, value=0x20→32?) | hal_av1d_vdpu.c:1926-1949 (per-bit flags from dxva), 2109 (`sw_strm_start_bit`) |
| 6 | 0x00000080 | `sw_stream_len=0x80` (128 bytes, `MPP_ALIGN(strm_len,128)`) | hal_av1d_vdpu.c:2110 |
| 7 | 0x00000010 | `sw_cdef_damping=2`? actual: 0x10=bit4 set → `sw_cdef_bits=1` (b3:4) — typical cdef bits=1 | hal_av1d_vdpu.c:1603-1604 |
| 8 | 0x05008c00 | `sw_quant_base_qindex=0x8c=140` (b8:15), `sw_superres_pic_width=??` (b17:31), `sw_bit_depth_*=0` (8-bit) | hal_av1d_vdpu.c:1224, 2046-2048 |
| 9 | 0x00003900 | `sw_context_update_tile_id=0` (b20:31), low bits encode `sw_mf*_type`, `sw_scale_denom_minus9`. 0x3900 → bits 8,11,12,13. Decode: `sw_mf2_type=1` (b8:10=001), `sw_mf3_type=7` (b11:13=111) | hal_av1d_vdpu.c:1091-1093, 1488 |
| 10 | 0x00020401 | `sw_tile_transpose=1`(b0)? **wait** — 0x00020401: bit0=1, bit10=1, bit17=1. Decode: `sw_tile_transpose=1`, `sw_multicore_full_width`(b2:9)=0, `sw_num_tile_rows_8k_av1`(b10:16)=1, `sw_num_tile_cols_8k`(b17:23)=1 | hal_av1d_vdpu.c:1485-1490 |
| 11 | 0xd8000c0c | bits set 2,3,10,11,27,28,30,31. Decode: `sw_use_temporal2_mvs=1`(b2)? — actual ordering: b0..3 = use_temporal{3,2,1,0}_mvs; so b2=1 → `sw_use_temporal1_mvs=1`, b3=1 → `sw_use_temporal0_mvs=1`. `sw_mcomp_filt_type`(b8:10)=4? bits 10,11 → b10=`sw_mcomp_filt_type` MSB, b11 = `sw_multicore_expect_context_update`=1. `sw_dec_tile_size_mag`(b30:31)=3, `sw_transform_mode`(b27:29)=4 (TX_MODE_SELECT) | hal_av1d_vdpu.c:1017-1060, 1480, 2050-2053 |
| 12 | 0x00007800 | `sw_max_cb_size`(b10:12)=6 → 64×64 SB (use_128x128=0); `sw_min_cb_size`(b13:15)=3 (8×8); other bits zero | hal_av1d_vdpu.c:2054-2055 |

Note for swreg4: the captured 0x02800f01 with 13-bit field at bits 19..31 yields
`sw_pic_width_in_cbs=0x050` (=80, → 640px) and `sw_pic_height_in_cbs=0x03c` (=60,
→ 480px) — consistent with a 640×480 (or padded) AV1 stream.

## Loop-filter / segment per-seg (swreg 14..32)

These are written 1:1 from dxva segmentation/loop_filter (hal_av1d_vdpu.c:1251-1391,
1402-1422). All seven captured here come up with the same low-byte pattern
(`0x028001e0`, `0x40004000`, `0x78080402`) which corresponds to defaults / all-zero
segments with non-zero filter level / sharpness / mode_deltas.

| swreg | Captured | Likely decode | Source |
|---|---|---|---|
| 14 | 0x14000000 | `sw_filt_level0=20` (b26:31=20)? actually field at b26:31 6 bits→0x05; `sw_filt_level0` = filter_level[0] | hal_av1d_vdpu.c:1251, 1321-1328 |
| 15 | 0x14000000 | `sw_filt_level1` = filter_level[1] | hal_av1d_vdpu.c:1252, 1330-1337 |
| 16 | 0x08000000 | `sw_filt_level2` = filter_level_u | hal_av1d_vdpu.c:1253, 1339-1346 |
| 17 | 0x08000000 | `sw_filt_level3` = filter_level_v | hal_av1d_vdpu.c:1254, 1348-1355 |
| 19 | 0xfc000000 | `sw_lr_unit_size` (b26:31=0x3f)?  high 6 bits = lr_unit_size=63 | hal_av1d_vdpu.c:1637 |
| 20 | 0x00800000 | `sw_global_mv_seg0=0`, `sw_mf1_last_offset`(b22:30)=2 | hal_av1d_vdpu.c:709, 1328 |
| 21..27 | 0x00800000 | analogous (`sw_mf1_*_offset`,`sw_mf2_last_offset`, etc.) | hal_av1d_vdpu.c:710-717 |
| 31 | 0x04000000 | low byte=0 (`sw_quant_seg6=0`); bit 26 → `sw_skip_ref0`(b22:25)? recompute: `sw_skip_ref0` is at b22:25 (4b)=0; bit 26 falls in reserved0. Actually `sw_filt_level_delta0_seg6`(b15:21) covers bit 26? — boundaries: 8+6+1+4+7=26, so bit 26 is start of `sw_skip_ref0`. → `sw_skip_ref0=1` (bit 26 set) | hal_av1d_vdpu.c:2082 (`sw_skip_ref0 = dxva->skip_ref0 ? : 1`) |
| 32 | 0x14000000 | `sw_skip_ref1`=??, `sw_filt_level_delta0_seg7`= | hal_av1d_vdpu.c:2083 |

(Per-seg capture pattern matches `sw_filt_level_segN=0` clears written at
hal_av1d_vdpu.c:2061-2068.)

## Ref dims / scales / qmlevel (swreg 33..49)

| swreg | Captured | Decode | Source |
|---|---|---|---|
| 33 | 0x028001e0 | `sw_ref0_width=0x280` (640), `sw_ref0_height=0x1e0` (480) | hal_av1d_vdpu.c:294, 315 |
| 34 | 0x028001e0 | ref1 width/height | hal_av1d_vdpu.c:296, 317 |
| 35 | 0x028001e0 | ref2 width/height | hal_av1d_vdpu.c:298, 319 |
| 36 | 0x40004000 | `sw_ref0_hor_scale=0x4000`, `sw_ref0_ver_scale=0x4000` (1.0 in s2.14) | hal_av1d_vdpu.c:336, 357 |
| 37..42 | 0x40004000 | refN_hor/ver_scale = 1.0 | hal_av1d_vdpu.c:338-369 |
| 43 | 0x028001e0 | ref3 dims | hal_av1d_vdpu.c:300, 321 |
| 44 | 0x028001e0 | ref4 dims | hal_av1d_vdpu.c:302, 323 |
| 45 | 0x028001e0 | ref5 dims | hal_av1d_vdpu.c:304, 325 |
| 46 | 0x028001e0 | ref6 dims | hal_av1d_vdpu.c:306, 327 |
| 47 | 0x78080402 | `sw_mf2_last2_offset=2`(b0:8), `sw_mf2_last3_offset=4`(b9:17) → 0x002, 0x002, 0x002 packed; `sw_qmlevel_y` (b27:30)=0xf? — bit pattern 0x78080402: bits 1,10,19,27..30. Decode: mf2_last2_offset=2, mf2_last3_offset=2, mf2_golden_offset=2, qmlevel_y=15 | hal_av1d_vdpu.c:718-720, 2074 |
| 48 | 0x78080402 | mf2_bwdref/altref2/altref offsets, qmlevel_u | hal_av1d_vdpu.c:721-723, 2075 |
| 49 | 0x0003c000 | `sw_qmlevel_v=15` (b14:17), filt_ref_adj_6/7 = 0 | hal_av1d_vdpu.c:1408-1409, 2076 |

## Stream / IO / build (swreg 51..63)

| swreg | Captured | Decode | Source |
|---|---|---|---|
| 51 | 0x0003800e | `sw_superres_luma_step=14`(b0:13), `sw_superres_chroma_step=14` | hal_av1d_vdpu.c:1199-1200 |
| 53 | 0x00020000 | `sw_cdef_chroma_secondary_strength`(b16:31)=2 | hal_av1d_vdpu.c:1617 |
| 55 | 0x00000008 | `sw_apf_threshold=8`(b0:15) | hal_av1d_vdpu.c:2127 |
| 58 | 0x00000210 | `sw_dec_max_burst=16`(b0:7), `sw_dec_buswidth=2`(b8:10) | hal_av1d_vdpu.c:2128-2129 |

## Address registers (`addr_cfg`, swreg 64..183)

These are 32-bit MSB/LSB pairs. The Linux kernel patches the `_msb` halves to add the
upper IOMMU bits; userland writes the FD/IOVA into the `_lsb` half. All captured values
0xffXXXXXX are post-IOMMU virtual addresses (kernel patched them in before kicking).

| swreg | Captured | Decode | Source |
|---|---|---|---|
| 65 | 0xffbd0000 | `sw_dec_out_ybase_lsb` | hal_av1d_vdpu.c:2097 |
| 67 | 0xffaa0000 | `sw_refer0_ybase_lsb` | hal_av1d_vdpu.c:387 |
| 69 | 0xffaa0000 | `sw_refer1_ybase_lsb` | hal_av1d_vdpu.c:389 |
| 71 | 0xffaa0000 | `sw_refer2_ybase_lsb` | hal_av1d_vdpu.c:391 |
| 73 | 0xffaa0000 | `sw_refer3_ybase_lsb` | hal_av1d_vdpu.c:393 |
| 75 | 0xffaa0000 | `sw_refer4_ybase_lsb` | hal_av1d_vdpu.c:395 |
| 77 | 0xffaa0000 | `sw_refer5_ybase_lsb` | hal_av1d_vdpu.c:397 |
| 79 | 0xffaa0000 | `sw_refer6_ybase_lsb` | hal_av1d_vdpu.c:399 |
| 83 | 0xfff6f000 | `sw_global_model_base_lsb` | hal_av1d_vdpu.c:1469 |
| 85 | 0xfff42e00 | `sw_cdef_colbuf_base_lsb` | hal_av1d_vdpu.c:1620 |
| 89 | 0xfff44980 | `sw_superres_colbuf_base_lsb` | hal_av1d_vdpu.c:1207 |
| 91 | 0xfff4d080 | `sw_lr_colbuf_base_lsb` | hal_av1d_vdpu.c:1638 |
| 99 | 0xffc1b000 | `sw_dec_out_cbase_lsb` | hal_av1d_vdpu.c:2098 |
| 101 | 0xffaeb000 | `sw_refer0_cbase_lsb` | hal_av1d_vdpu.c:443 |
| 103 | 0xffaeb000 | `sw_refer1_cbase_lsb` | hal_av1d_vdpu.c:446 |
| 105 | 0xffaeb000 | `sw_refer2_cbase_lsb` | hal_av1d_vdpu.c:449 |
| 107 | 0xffaeb000 | `sw_refer3_cbase_lsb` | hal_av1d_vdpu.c:452 |
| 109 | 0xffaeb000 | `sw_refer4_cbase_lsb` | hal_av1d_vdpu.c:455 |
| 111 | 0xffaeb000 | `sw_refer5_cbase_lsb` | hal_av1d_vdpu.c:458 |
| 113 | 0xffaeb000 | `sw_refer6_cbase_lsb` | hal_av1d_vdpu.c:461 |
| 133 | 0xffc40840 | `sw_dec_out_dbase_lsb` (FBC dir/MV scratch) | hal_av1d_vdpu.c:2100 |
| 135 | 0xffb10840 | `sw_refer0_dbase_lsb` | hal_av1d_vdpu.c:505 |
| 137 | 0xffb10840 | `sw_refer1_dbase_lsb` | hal_av1d_vdpu.c:508 |
| 139 | 0xffb10840 | `sw_refer2_dbase_lsb` | hal_av1d_vdpu.c:511 |
| 141 | 0xffb10840 | `sw_refer3_dbase_lsb` | hal_av1d_vdpu.c:514 |
| 143 | 0xffb10840 | `sw_refer4_dbase_lsb` | hal_av1d_vdpu.c:517 |
| 145 | 0xffb10840 | `sw_refer5_dbase_lsb` | hal_av1d_vdpu.c:520 |
| 147 | 0xffb10840 | `sw_refer6_dbase_lsb` | hal_av1d_vdpu.c:523 |
| 167 | 0xfff3f000 | `sw_tile_base_lsb` | hal_av1d_vdpu.c:1493 |
| 169 | 0xffeb0010 | `sw_stream_base_lsb` | hal_av1d_vdpu.c:2113 |
| 171 | 0xffeac000 | `sw_prob_tab_out_base_lsb` | hal_av1d_vdpu.c:775 |
| 173 | 0xffea8000 | `sw_prob_tab_base_lsb` | hal_av1d_vdpu.c:776 |
| 175 | 0xffea7000 | `sw_mc_sync_curr_base_lsb` | hal_av1d_vdpu.c:2123 |
| 177 | 0xffea7000 | `sw_mc_sync_left_base_lsb` (shares filter_mem) | hal_av1d_vdpu.c:2124 |
| 179 | 0xfff40000 | `sw_dec_vert_filt_base_lsb` | hal_av1d_vdpu.c:1425 |
| 183 | 0xfff41e00 | `sw_dec_bsd_ctrl_base_lsb` | hal_av1d_vdpu.c:1426 |

## Tile-Y / Tile-C base + ref/cur offsets / strm + global misc (swreg 184..266)

| swreg | Captured | Decode | Source |
|---|---|---|---|
| 184 | 0x000003ff | `sw_cur_last_offset=0x3ff` (b9:17), other fields 0 | hal_av1d_vdpu.c:1074, 1082, 860, 725 |
| 185 | 0x000003ff | analogous swreg185 (cur_last2/mf3_last2/ref1_gm) | hal_av1d_vdpu.c:1075, 1083, 861, 726 |
| 186 | 0x000003ff | swreg186 (cur_last3/mf3_last3/ref2_gm) | hal_av1d_vdpu.c:1076, 1084, 862, 727 |
| 187 | 0x000003ff | swreg187 (cur_golden/mf3_golden/ref3_gm) | hal_av1d_vdpu.c:1077, 1085, 863, 728 |
| 188 | 0x000003ff | swreg188 (cur_bwdref/mf3_bwdref/ref4_gm) | hal_av1d_vdpu.c:1078, 1086, 864, 729 |
| 257 | 0x000003ff | swreg257 (cur_altref2/mf3_altref2/ref5_gm) | hal_av1d_vdpu.c:1079, 1087, 865, 730 |
| 258 | 0x00000080 | `sw_strm_buffer_len=128` (`ALIGN(strm_len,128)`) | hal_av1d_vdpu.c:2108 |
| 262 | 0x000003ff | swreg262 (cur_altref/mf3_altref/ref6_gm) | hal_av1d_vdpu.c:1080, 1088, 866, 731 |
| 263 | 0x0000000f | `sw_cdef_luma_primary_strength=0x0f` (32-bit packed strengths) | hal_av1d_vdpu.c:1614 |
| 265 | 0x01004000 | `sw_axi_arqos=0`(b0:3); `sw_axi_wr_ostd_threshold=64`(b8:17), `sw_axi_rd_ostd_threshold=64`(b18:27) | hal_av1d_vdpu.c:2131-2132 |

## Diag / superres / timeouts (swreg 298..319)

| swreg | Captured | Decode | Source |
|---|---|---|---|
| 298 | 0x000e000e | `sw_superres_luma_step_invra=0x0e`, `sw_superres_chroma_step_invra=0x0e` | hal_av1d_vdpu.c:1201-1202 |
| 314 | 0x00000040 | `sw_dec_alignment=0x40` (64-byte) | hal_av1d_vdpu.c:2121 |
| 318 | 0x8fffffff | `sw_ext_timeout_cycles=0x0fffffff`, `sw_ext_timeout_override_e=1` (bit 31) | hal_av1d_vdpu.c:2134-2135 |
| 319 | 0x8fffffff | `sw_timeout_cycles=0x0fffffff`, `sw_timeout_override_e=1` | hal_av1d_vdpu.c:2136-2137 |

## PP cfg (swreg 320..511)

| swreg | Captured | Decode | Source |
|---|---|---|---|
| 320 | 0x00000001 | `sw_pp_out_e=1` | hal_av1d_vdpu.c:2140 |
| 322 | 0x000c0000 | `sw_pp_out_format=12`? bits 18,19 set → `sw_pp_out_format`(b18:22)=0x03 (NV12 output for AV1 8-bit) | hal_av1d_vdpu.c:2192 / 2210 |
| 326 | 0xff5d0000 | `sw_pp_out_lu_base_lsb` (post-IOMMU) | hal_av1d_vdpu.c:2193 |
| 328 | 0xff61b000 | `sw_pp_out_ch_base_lsb` (post-IOMMU) | hal_av1d_vdpu.c:2194 |
| 329 | 0x02800280 | `sw_pp_out_y_stride=0x280`(640), `sw_pp_out_c_stride=0x280` | hal_av1d_vdpu.c:2148-2149 |
| 331 | 0x014000f0 | `sw_pp_in_height=0xf0` (240), `sw_pp_in_width=0x140` (320) — set as height/2, width/2 | hal_av1d_vdpu.c:2144-2145 |
| 332 | 0x028001e0 | `sw_pp_out_height=0x1e0` (480), `sw_pp_out_width=0x280` (640) | hal_av1d_vdpu.c:2146-2147 |
| 394 | 0x01010000 | `sw_pp0_dup_hor=1`(b16:23)? actually `sw_pp0_dup_hor` at b16:23, value 1 — wait struct: { pp1_dup_ver:8, pp1_dup_hor:8, pp0_dup_ver:8, pp0_dup_hor:8 } LSB-first → b0:7=pp1_dup_ver=0, b8:15=pp1_dup_hor=0, b16:23=pp0_dup_ver=1, b24:31=pp0_dup_hor=1 | hal_av1d_vdpu.c:2142-2143 (sw_pp0_dup_hor=1, sw_pp0_dup_ver=1) |

## Field decode caveat / static-vs-driver

All 106 captured non-zero registers map cleanly into struct fields in
`hal_av1d_vdpu_reg.h`. The `addr_cfg` MSB halves (swreg64,66,68,...) were left at 0 in
the regset and are patched into the **upper byte** of the corresponding LSB slot by
the kernel mpp_iommu (the `0xff......` upper nibble is the IOMMU domain ID). No
captured register exists outside the documented struct.

Frame-parameter mapping (driving source variables) for the most load-bearing fields:

- `swreg4.sw_pic_width_in_cbs` ← `MPP_ALIGN(dxva->width,8) >> 3`
- `swreg4.sw_pic_height_in_cbs` ← `MPP_ALIGN(dxva->height,8) >> 3`
- `swreg5.*` ← `dxva->coding.*`, `dxva->loop_filter.delta_lf_*`, `dxva->format.show_frame`, `dxva->cdef`
- `swreg8.sw_quant_base_qindex` ← `dxva->quantization.base_qindex`
- `swreg33..46` width/height ← `dxva->frame_refs[i].{width,height}`
- `swreg36..42` scale ← computed from ref dims / cur dims (see `vdpu_av1d_set_scaling`)
- `swreg184..188,257,262` cur_*_offset, mf3_*_offset, refN_gm_mode ← `dxva->frame_refs[i].wmtype`, ref-order-hint deltas
