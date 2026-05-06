#!/usr/bin/env python3
p = "/home/vibecoder/linux-rockchip/drivers/video/rockchip/mpp/mpp_av1dec.c"
with open(p) as f:
    s = f.read()
old = 'pr_info("AV1SHIM OVR APPLIED'
new = 'pr_err("AV1SHIM OVR APPLIED'
print("matches:", s.count(old))
s = s.replace(old, new)
with open(p, "w") as f:
    f.write(s)
print("ok")
