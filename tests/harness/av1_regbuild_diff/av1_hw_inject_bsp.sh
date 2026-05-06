#!/bin/bash
# av1_hw_inject_bsp — sanity test: override with BSP-captured values,
# decode again, and verify output bit-exact matches the no-override
# baseline.  Tests that the AV1SHIM override path itself is
# transparent when the values are unchanged.
#
# Args: <ivf_file> <bsp_trace.log> <kick_n_in_trace>
set -e
IVF=$1
TRACE=$2
KICK=$3
[ -z "$KICK" ] && { echo "usage: $0 <ivf> <trace.log> <kick_idx_in_trace>" >&2; exit 2; }

# Parse the BSP trace's kick N (0-based among kick begins) into a
# 512-u32 binary blob and a 16-u32 mask blob.
python3 - "$TRACE" "$KICK" <<'PYEOF'
import sys, re, struct
trace, kick = sys.argv[1], int(sys.argv[2])
# trans_tbl_av1_vcd[] from mpp_av1dec.c — register indices that hold
# DMA addresses (kernel IOMMU-patches them per decode).  We must NOT
# override these or we'd write stale IOVAs from the captured session
# that no longer point to valid buffers.
DMA_IDX = set([
    65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,
    97, 99, 101, 103, 105, 107, 109, 111, 113, 133, 135, 137, 139,
    141, 143, 145, 147, 167, 169, 171, 173, 175, 177, 179, 183, 190,
    192, 194, 196, 198, 200, 202, 204, 224, 226, 228, 230, 232, 234,
    236, 238, 326, 328, 339, 341, 348, 350, 505, 507,
])
in_kick = -1
val = [0] * 512
mask = [0] * 16
for line in open(trace):
    if "AV1SHIM kick begin" in line:
        in_kick += 1
    if in_kick != kick:
        continue
    m = re.search(r'AV1SHIM r\[0\]\[(\d+)\]=([0-9a-f]+)', line)
    if not m: continue
    idx = int(m.group(1)); v = int(m.group(2), 16)
    if v == 0 or idx >= 512 or idx in DMA_IDX: continue
    val[idx] = v
    mask[idx // 32] |= 1 << (idx % 32)
open("/tmp/bsp_val.bin","wb").write(struct.pack("<512I", *val))
open("/tmp/bsp_mask.bin","wb").write(struct.pack("<16I", *mask))
nz = sum(1 for x in val if x)
print(f"loaded {nz} nonzero non-DMA overrides from kick {kick}", file=sys.stderr)
PYEOF

sudo cp /tmp/bsp_val.bin   /sys/kernel/debug/av1shim/regs
sudo cp /tmp/bsp_mask.bin  /sys/kernel/debug/av1shim/mask
echo $((KICK)) | sudo tee /sys/kernel/debug/av1shim/skip > /dev/null
echo Y         | sudo tee /sys/kernel/debug/av1shim/enable > /dev/null

echo "running decode with BSP-equivalent override on kick $KICK..." >&2
ffmpeg -hide_banner -y -loglevel error -c:v av1_rkmpp_decoder -i "$IVF" -f rawvideo -pix_fmt yuv420p /tmp/av1_hw_bsp_inject.yuv

echo N | sudo tee /sys/kernel/debug/av1shim/enable > /dev/null

echo "comparing to baseline (no override)..." >&2
ffmpeg -hide_banner -y -loglevel error -c:v av1_rkmpp_decoder -i "$IVF" -f rawvideo -pix_fmt yuv420p /tmp/av1_hw_baseline.yuv

if cmp -s /tmp/av1_hw_bsp_inject.yuv /tmp/av1_hw_baseline.yuv; then
    echo "PASS: BSP-value override produces bit-exact baseline output"
else
    echo "FAIL: override mechanism corrupts state somehow"
    sha256sum /tmp/av1_hw_bsp_inject.yuv /tmp/av1_hw_baseline.yuv
fi
