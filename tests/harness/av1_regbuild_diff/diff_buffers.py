#!/usr/bin/env python3
"""Diff our AV1 input buffer dumps against BSP MPP captures.

Usage:
    python diff_buffers.py <our_dump_dir> <bsp_capture_dir> [kick]

Prints per-buffer match summary; for diffs, shows first 10 differing
hex words.  Targets `prob_tbl`, `prob_tbl_out`, `global_mode`,
`tile_info`, `stream` for kick 0 by default.
"""
import os, sys

KINDS = ["prob_tbl", "prob_tbl_out", "global_mode", "tile_info", "stream"]

def parse_hex_lines(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try: out.append(int(line, 16))
            except ValueError: return None
    return out

def diff_one(our_path, bsp_path):
    if not os.path.exists(our_path):
        return f"missing ours: {our_path}"
    if not os.path.exists(bsp_path):
        return f"missing bsp:  {bsp_path}"

    if os.path.basename(our_path).startswith("stream_"):
        a = open(our_path, "rb").read()
        b = open(bsp_path, "rb").read()
        if a == b: return f"  MATCH ({len(a)} bytes)"
        diff_off = next((i for i in range(min(len(a), len(b))) if a[i] != b[i]), -1)
        return f"  DIFF: ours={len(a)}B bsp={len(b)}B first-diff-byte=@{diff_off}"

    a = parse_hex_lines(our_path)
    b = parse_hex_lines(bsp_path)
    if a is None or b is None:
        return f"  PARSE_ERROR (text format mismatch)"
    if a == b:
        return f"  MATCH ({len(a)} words)"
    diffs = [(i, x, y) for i, (x, y) in enumerate(zip(a, b)) if x != y]
    if len(a) != len(b):
        sz = f" len ours={len(a)} bsp={len(b)}"
    else:
        sz = ""
    msg = [f"  DIFF: {len(diffs)} word(s) differ{sz}"]
    for i, x, y in diffs[:10]:
        msg.append(f"    [{i:5d}] ours={x:08x}  bsp={y:08x}  xor={x ^ y:08x}")
    if len(diffs) > 10:
        msg.append(f"    ... {len(diffs) - 10} more")
    return "\n".join(msg)

def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(2)
    ours, bsp = sys.argv[1], sys.argv[2]
    kick = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    print(f"=== diff kick {kick}: ours={ours}  vs  bsp={bsp} ===")
    for kind in KINDS:
        fname = f"{kind}_{kick}.txt"
        print(f"{kind}:")
        print(diff_one(os.path.join(ours, fname),
                       os.path.join(bsp, fname)))

if __name__ == "__main__":
    main()
