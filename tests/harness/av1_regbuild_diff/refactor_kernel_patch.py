#!/usr/bin/env python3
"""Refactor mpp_av1dec.c: replace the late-write override block with
an early pre-fill that patches task->reg_class[0].data IN PLACE
before BSP's writel loop, so BSP writes our values to MMIO."""
import re

p = "/home/vibecoder/linux-rockchip/drivers/video/rockchip/mpp/mpp_av1dec.c"
src = open(p).read()

new_pre_fill = (
    "\tav1dec_set_l2_cache(dec, task);\n"
    "\tav1dec_set_afbc(dec, task);\n"
    "\n"
    "\t/* SHIM pre-fill: patch task->reg_class[0].data IN PLACE\n"
    "\t * BEFORE the BSP writel loop, so the codec sees our regs\n"
    "\t * via the same write path BSP uses (instead of late-write\n"
    "\t * which corrupts state). */\n"
    "\tif (av1shim_ovr_enabled) {\n"
    "\t\tunsigned j;\n"
    "\t\tu32 *cls0 = (u32 *)task->reg_class[0].data;\n"
    "\t\tunsigned cls0_n = task->reg_class[0].len / sizeof(u32);\n"
    "\t\tmutex_lock(&av1shim_ovr_lock);\n"
    "\t\tif (av1shim_ovr_skip_remaining > 0) {\n"
    "\t\t\tav1shim_ovr_skip_remaining--;\n"
    "\t\t} else if (cls0) {\n"
    "\t\t\tfor (j = 0; j < AV1SHIM_OVR_MAX && j < cls0_n; j++) {\n"
    "\t\t\t\tif (av1shim_ovr_mask[j / 32] & (1u << (j % 32)))\n"
    "\t\t\t\t\tcls0[j] = av1shim_ovr_val[j];\n"
    "\t\t\t}\n"
    "\t\t}\n"
    "\t\tmutex_unlock(&av1shim_ovr_lock);\n"
    "\t}\n"
    "\n"
)

old_pre_fill = "\tav1dec_set_l2_cache(dec, task);\n\tav1dec_set_afbc(dec, task);\n\n"
assert src.count(old_pre_fill) == 1, f"old anchor count {src.count(old_pre_fill)}"
src = src.replace(old_pre_fill, new_pre_fill)

# Remove the late-write block + skip_ovr label.
late_re = re.compile(
    r"\t/\* SHIM: apply userspace register overrides.*?skip_ovr:\n",
    re.DOTALL,
)
src, n = late_re.subn("", src)
print(f"removed {n} late-write blocks")

open(p, "w").write(src)
print("patched")
