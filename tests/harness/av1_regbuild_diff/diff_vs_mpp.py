#!/usr/bin/env python3
"""Diff our regbuilder output (BSP-shim format) against MPP-captured
reg_N_in.txt arrays.  Per kick, prints regs where values differ.

DMA-base regs (lsb/msb of buffer addresses) carry FDs that necessarily
differ between our harness (uses 0x10/0x11/0x20+i) and MPP capture
(uses MPP's actual buffer FDs).  Mask them so diff focuses on the
non-DMA reg-fill correctness.

Usage:
    python diff_vs_mpp.py <our_dump.txt> <capture_dir>
"""
import sys, re

# trans_tbl_av1_vcd[] DMA lsb indices.  We mask both lsb and lsb-1 (msb).
DMA_LSB = [
    65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
    97, 99, 101, 103, 105, 107, 109, 111, 113, 133, 135, 137, 139,
    141, 143, 145, 147, 167, 169, 171, 173, 175, 177, 179, 183, 190,
    192, 194, 196, 198, 200, 202, 204, 224, 226, 228, 230, 232, 234,
    236, 238, 326, 328, 339, 341, 348, 350, 505, 507,
]
DMA_MASK = set(DMA_LSB) | set(i - 1 for i in DMA_LSB)

def parse_our(path):
    """Returns {kick_idx: {reg_idx: value}}."""
    out, cur, kick = {}, None, None
    with open(path) as f:
        for line in f:
            m = re.match(r'# kick (\d+) begin', line)
            if m:
                kick = int(m.group(1)); cur = {}; out[kick] = cur; continue
            m = re.match(r'# kick (\d+) end', line)
            if m: cur = None; continue
            if cur is None: continue
            m = re.match(r'AV1SHIM r\[0\]\[(\d+)\]=([0-9a-f]+)', line)
            if m:
                cur[int(m.group(1))] = int(m.group(2), 16)
    return out

def parse_mpp(path):
    """Returns {reg_idx: value} for one kick's reg_N_in.txt."""
    out = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'reg\[\s*(\d+)\] = ([0-9a-f]+)', line)
            if m:
                idx = int(m.group(1)); v = int(m.group(2), 16)
                if v != 0:  # match shim's "non-zero only" emit
                    out[idx] = v
    return out

def main():
    if len(sys.argv) != 3:
        print(__doc__); sys.exit(2)
    our_path, cap_dir = sys.argv[1], sys.argv[2].rstrip('/').rstrip('\\')
    ours = parse_our(our_path)
    total_match = 0
    total_diff = 0
    # reg[5] top 7 bits = sw_strm_start_bit, applied post-regbuilder in
    # the HW kick path (see decode_engine_av1.cpp). The bare regbuilder
    # dump leaves them at 0; mask before comparing.
    REG5_STRM_START_MASK = 0x7F << 25
    for kick in sorted(ours):
        cap = parse_mpp(f"{cap_dir}/reg_{kick}_in.txt")
        ours_k = ours[kick]
        # Combine reg sets, mask DMA, compare
        all_idx = (set(ours_k) | set(cap)) - DMA_MASK
        diffs = []
        match = 0
        for idx in sorted(all_idx):
            ov = ours_k.get(idx, 0)
            mv = cap.get(idx, 0)
            if idx == 5:
                ov &= ~REG5_STRM_START_MASK
                mv &= ~REG5_STRM_START_MASK
            if ov == mv:
                match += 1
            else:
                diffs.append((idx, ov, mv))
        total_match += match
        total_diff += len(diffs)
        print(f"=== kick {kick}: match={match} diff={len(diffs)} ===")
        for idx, ov, mv in diffs[:50]:
            print(f"  reg[{idx:3d}]  ours={ov:08x}  mpp={mv:08x}  xor={ov ^ mv:08x}")
        if len(diffs) > 50:
            print(f"  ... {len(diffs) - 50} more")
    print(f"\nTOTAL: {total_match} matching, {total_diff} divergent (DMA-masked)")

if __name__ == '__main__':
    main()
