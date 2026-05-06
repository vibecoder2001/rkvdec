#!/bin/bash
# av1_hw_align_inject — leverages av1_align_kicks.py alignment to
# inject our regbuilder's output for the dav1d frame that matches a
# given BSP kick.  Three modes:
#
#   --bsp-only:  inject ONLY where our regbuilder's value matches BSP
#                exactly (sanity: should not change decode output)
#   --our-only:  inject all of our regbuilder's non-zero values
#                (full test of our regbuilder vs BSP behavior)
#   --diff-one IDX:  inject only swreg=IDX (binary-search a single
#                    differing register)
#
# Usage: av1_hw_align_inject <ivf> <bsp_kick_idx> <mode>
set -e
IVF=$1
BSP_KICK=$2
MODE=$3
[ -z "$MODE" ] && { echo "usage: $0 <ivf> <bsp_kick_idx> <mode> [--diff-one IDX]" >&2; exit 2; }

# Capture fresh BSP trace for this input
sudo dmesg -c > /dev/null
ffmpeg -hide_banner -y -loglevel error -c:v av1_rkmpp_decoder -i "$IVF" -f rawvideo -pix_fmt yuv420p /tmp/baseline.yuv 2>/dev/null
sudo dmesg | grep AV1SHIM > /tmp/bsp_aligned.log
~/av1/bin/av1_regbuild_diff "$IVF" > /tmp/our_aligned.log 2>/dev/null

# Use the python alignment to build the override blobs
python3 - "$BSP_KICK" "$MODE" "$4" <<'PYEOF'
import sys, re, struct
from collections import defaultdict

bsp_kick_idx = int(sys.argv[1])
mode = sys.argv[2]
diff_one_idx = int(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else None

DMA_IDX = set([
    65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,101,103,105,
    107,109,111,113,133,135,137,139,141,143,145,147,167,169,171,173,
    175,177,179,183,190,192,194,196,198,200,202,204,224,226,228,230,
    232,234,236,238,326,328,339,341,348,350,505,507,
])

# Parse BSP trace
bsp_kicks = []
cur = None
for line in open("/tmp/bsp_aligned.log"):
    if "AV1SHIM kick begin" in line:
        if cur is not None: bsp_kicks.append(cur)
        cur = {}
        continue
    if cur is None: continue
    m = re.search(r"AV1SHIM r\[0\]\[(\d+)\]=([0-9a-f]+)", line)
    if m:
        idx = int(m.group(1)); v = int(m.group(2), 16)
        if v != 0 and idx < 512: cur[idx] = v
if cur: bsp_kicks.append(cur)

# Parse our regs
ours = []
cur = None
cur_qp = None
for line in open("/tmp/our_aligned.log"):
    m = re.search(r"# kick (\d+):.*qp_y=(\d+)", line)
    if m:
        cur_qp = int(m.group(2)); continue
    m = re.search(r"# kick (\d+) begin", line)
    if m:
        cur = (int(m.group(1)), cur_qp, {}); continue
    if cur is None: continue
    if "# kick" in line and "end" in line:
        ours.append(cur); cur = None; continue
    m = re.search(r"AV1SHIM r\[0\]\[(\d+)\]=([0-9a-f]+)", line)
    if m:
        idx = int(m.group(1)); v = int(m.group(2), 16)
        if v != 0: cur[2][idx] = v

# Align via QP
def qp_of_bsp(k): return (k.get(8, 0) >> 8) & 0xff
by_qp = defaultdict(list)
for o in ours: by_qp[o[1]].append(o)

bsp = bsp_kicks[bsp_kick_idx]
target_qp = qp_of_bsp(bsp)
cands = by_qp[target_qp]
if not cands:
    print(f"ERROR: no dav1d frame with qp={target_qp} matching bsp_kick {bsp_kick_idx}", file=sys.stderr)
    sys.exit(1)
our_kick_idx, _, our_regs = cands[0]
print(f"alignment: bsp_kick {bsp_kick_idx} (qp={target_qp}) ↔ dav1d frame {our_kick_idx}", file=sys.stderr)

# Build the override
val = [0] * 512
mask = [0] * 16
all_idx = (set(bsp) | set(our_regs)) - DMA_IDX
for idx in all_idx:
    bv = bsp.get(idx, 0); ov = our_regs.get(idx, 0)
    if mode == "--bsp-only":
        # Only inject where ours == BSP (sanity: should be no-op)
        if ov != 0 and ov == bv:
            val[idx] = ov
            mask[idx // 32] |= 1 << (idx % 32)
    elif mode == "--our-only":
        # Inject all our non-zero values
        if ov != 0:
            val[idx] = ov
            mask[idx // 32] |= 1 << (idx % 32)
    elif mode == "--diff-one":
        if idx == diff_one_idx and ov != 0:
            val[idx] = ov
            mask[idx // 32] |= 1 << (idx % 32)

nz = sum(1 for x in val if x)
print(f"override count: {nz} (mode={mode})", file=sys.stderr)

open("/tmp/v.bin","wb").write(struct.pack("<512I", *val))
open("/tmp/m.bin","wb").write(struct.pack("<16I", *mask))
PYEOF

sudo cp /tmp/v.bin /sys/kernel/debug/av1shim/regs
sudo cp /tmp/m.bin /sys/kernel/debug/av1shim/mask
echo $BSP_KICK | sudo tee /sys/kernel/debug/av1shim/skip > /dev/null
echo 1 | sudo tee /sys/kernel/debug/av1shim/apply > /dev/null
echo Y         | sudo tee /sys/kernel/debug/av1shim/enable > /dev/null

ffmpeg -hide_banner -y -loglevel error -c:v av1_rkmpp_decoder -i "$IVF" -f rawvideo -pix_fmt yuv420p /tmp/test.yuv 2>/dev/null

echo N | sudo tee /sys/kernel/debug/av1shim/enable > /dev/null

if cmp -s /tmp/test.yuv /tmp/baseline.yuv; then
    echo "PASS: output bit-exact identical to baseline"
else
    BS=$(sha256sum /tmp/baseline.yuv | cut -d' ' -f1 | head -c 16)
    TS=$(sha256sum /tmp/test.yuv     | cut -d' ' -f1 | head -c 16)
    echo "DIFFER: baseline=$BS  test=$TS"
fi
