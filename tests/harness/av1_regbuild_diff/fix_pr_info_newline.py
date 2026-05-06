#!/usr/bin/env python3
"""Replace literal newline in pr_info string with \\n escape."""
p = "/home/vibecoder/linux-rockchip/drivers/video/rockchip/mpp/mpp_av1dec.c"
with open(p, "rb") as f:
    s = f.read()

# Find the pr_info line with literal newline
needle = b'pr_info("AV1SHIM OVR APPLIED: %u regs patched in cls0 (apply_remaining was %d)\n", applied, av1shim_ovr_apply_remaining);'
replacement = b'pr_info("AV1SHIM OVR APPLIED: %u regs patched in cls0 (apply_remaining was %d)\\n", applied, av1shim_ovr_apply_remaining);'
n = s.count(needle)
print(f"matches: {n}")
if n != 1:
    raise SystemExit(1)
s = s.replace(needle, replacement)
with open(p, "wb") as f:
    f.write(s)
print("patched")
