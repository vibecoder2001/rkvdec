#!/usr/bin/env python3
"""
av1_align_kicks — match BSP-captured kicks to dav1d output frames by
                   distinguishing register fields, then emit a per-kick
                   diff between our regbuilder output and BSP's capture.

Inputs:
  1. BSP register trace produced by the AV1SHIM-instrumented kernel,
     containing N kicks worth of `AV1SHIM r[0][NNN]=VVVVVVVV` lines.
  2. Output of av1_regbuild_diff for the same input file (with
     RKMPP_AV1_SKIP=0 — i.e. ALL dav1d output frames, including the
     keyframe).  Each "# kick N" block in that output gives us our
     regbuilder's emit for the Nth dav1d output picture.

Matching key: swreg8 bits[12:19] = sw_quant_base_qindex (AV1 yac).
Each dav1d picture's quant.yac is unique enough across the 4 kicks
in our test streams to pin BSP→dav1d alignment uniquely.

Output: a per-kick diff table showing bit-exact matches and
distinguishable mismatches.
"""

import re
import sys
from collections import defaultdict

DMA_IDX = set([
    65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,101,103,105,
    107,109,111,113,133,135,137,139,141,143,145,147,167,169,171,173,
    175,177,179,183,190,192,194,196,198,200,202,204,224,226,228,230,
    232,234,236,238,326,328,339,341,348,350,505,507,
])

KICK_RE = re.compile(r"AV1SHIM kick begin")
REG_RE  = re.compile(r"AV1SHIM r\[0\]\[(\d+)\]=([0-9a-f]+)")
OURK_RE = re.compile(r"# kick (\d+):.*qp_y=(\d+)")
OURR_RE = re.compile(r"AV1SHIM r\[0\]\[(\d+)\]=([0-9a-f]+)")
OURB_RE = re.compile(r"# kick (\d+) begin")


def parse_bsp_trace(path):
    """Return list-of-dict: per-kick {idx: value}.  All non-zero swregs."""
    kicks = []
    cur = None
    for line in open(path):
        if KICK_RE.search(line):
            if cur is not None:
                kicks.append(cur)
            cur = {}
            continue
        if cur is None:
            continue
        m = REG_RE.search(line)
        if m:
            idx = int(m.group(1))
            val = int(m.group(2), 16)
            if val != 0 and idx < 512:
                cur[idx] = val
    if cur is not None and cur:
        kicks.append(cur)
    return kicks


def parse_ours(path):
    """Return list of (kick_idx, qp_y, {idx: value})."""
    out = []
    cur = None
    cur_qp = None
    for line in open(path):
        m = OURK_RE.search(line)
        if m:
            cur_qp = int(m.group(2))
            continue
        m = OURB_RE.search(line)
        if m:
            cur = (int(m.group(1)), cur_qp, {})
            continue
        if cur is None:
            continue
        if "# kick" in line and "end" in line:
            out.append(cur)
            cur = None
            continue
        m = OURR_RE.search(line)
        if m:
            idx = int(m.group(1))
            val = int(m.group(2), 16)
            if val != 0:
                cur[2][idx] = val
    return out


def qp_of_kick(kick):
    """Extract sw_quant_base_qindex from swreg8 bits 8-15.
    Layout (per regbuilder_av1_reg.h):
        sw_scaling_shift       : 4
        sw_bit_depth_c_minus8  : 2
        sw_bit_depth_y_minus8  : 2
        sw_quant_base_qindex   : 8   <-- bits 8-15
        sw_idr_pic_e           : 1
        sw_superres_pic_width  : 15
    """
    v = kick.get(8, 0)
    return (v >> 8) & 0xff


def main():
    if len(sys.argv) < 3:
        print("usage: av1_align_kicks <bsp_trace.log> <ours.log>",
              file=sys.stderr)
        return 2
    bsp = parse_bsp_trace(sys.argv[1])
    ours = parse_ours(sys.argv[2])

    print(f"BSP captured {len(bsp)} hardware kicks")
    print(f"dav1d emitted {len(ours)} output pictures from our regbuilder\n")

    # Map BSP kicks to dav1d picture indices via QP.  For each BSP kick,
    # find dav1d pictures with the matching qp.
    print("Per-BSP-kick QP fingerprints (sw_quant_base_qindex):")
    bsp_qps = []
    for k_idx, k in enumerate(bsp):
        q = qp_of_kick(k)
        nz = len([1 for v in k.values() if v])
        bsp_qps.append(q)
        print(f"  bsp_kick {k_idx}: qp={q:3d}  ({nz} non-zero regs)")
    print()

    print("dav1d pictures grouped by QP:")
    by_qp = defaultdict(list)
    for o in ours:
        by_qp[o[1]].append(o[0])
    for q in sorted(by_qp):
        marker = " ← matches BSP" if q in bsp_qps else ""
        print(f"  qp={q:3d}: dav1d frames {by_qp[q]}{marker}")
    print()

    # Best-guess mapping: pick the FIRST dav1d frame matching each BSP qp.
    # Note: when multiple dav1d frames share a qp, this may be wrong;
    # fall back to other distinguishing fields (refresh_frame_flags etc.)
    # for ambiguity resolution in a future revision.
    mapping = []
    used = set()
    for k_idx, q in enumerate(bsp_qps):
        cands = [i for i in by_qp[q] if i not in used]
        chosen = cands[0] if cands else None
        if chosen is not None:
            used.add(chosen)
        mapping.append((k_idx, q, chosen))

    print("Provisional alignment (BSP kick → dav1d frame):")
    for k_idx, q, dav1d_idx in mapping:
        print(f"  bsp_kick {k_idx} (qp={q}) → dav1d frame {dav1d_idx}")
    print()

    # For each successfully-mapped pair, diff non-DMA regs.
    print("=" * 72)
    print("Per-kick reg diff (non-DMA logic regs only)")
    print("=" * 72)
    total_match = 0
    total_diff_logic = 0
    for k_idx, q, dav1d_idx in mapping:
        if dav1d_idx is None:
            continue
        bsp_regs = bsp[k_idx]
        our_regs = ours[dav1d_idx][2]
        # Logic indices = union, excluding DMA
        all_idx = (set(bsp_regs) | set(our_regs)) - DMA_IDX
        match = 0
        miss_bsp_zero = []  # we set, BSP didn't
        miss_our_zero = []  # BSP set, we didn't
        miss_both    = []   # both set, different values
        for idx in sorted(all_idx):
            bv = bsp_regs.get(idx, 0)
            ov = our_regs.get(idx, 0)
            if bv == ov:
                if bv != 0:
                    match += 1
            elif bv == 0:
                miss_bsp_zero.append((idx, ov))
            elif ov == 0:
                miss_our_zero.append((idx, bv))
            else:
                miss_both.append((idx, ov, bv))
        total_match += match
        total_diff_logic += len(miss_bsp_zero) + len(miss_our_zero) + len(miss_both)
        print(f"\nbsp_kick {k_idx} (qp={q}) ↔ dav1d frame {dav1d_idx}: "
              f"{match} bit-exact, {len(miss_bsp_zero)+len(miss_our_zero)+len(miss_both)} divergent")
        for idx, ov, bv in miss_both:
            print(f"  swreg{idx:3d}: ours={ov:08x} bsp={bv:08x}")
        for idx, ov in miss_bsp_zero:
            print(f"  swreg{idx:3d}: ours={ov:08x} bsp=00000000  (we set, BSP didn't)")
        for idx, bv in miss_our_zero:
            print(f"  swreg{idx:3d}: ours=00000000 bsp={bv:08x}  (BSP set, we didn't)")

    print("\n" + "=" * 72)
    print(f"TOTAL: {total_match} bit-exact matches, "
          f"{total_diff_logic} logic divergences across all aligned kicks")
    print("=" * 72)


if __name__ == "__main__":
    sys.exit(main() or 0)
