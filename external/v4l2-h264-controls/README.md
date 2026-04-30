# V4L2 H.264 stateless decoder controls

Cherry-picked from `linux/include/uapi/linux/v4l2-controls.h`.

These struct definitions form the V4L2 stateless H.264 decoder ABI as
specified by the Linux media subsystem. They are the input shape the
RK3588 BSP register builder (`drivers/video/rockchip/mpp/mpp_rkvdec2_h264.c`)
expects: it reads these structs and produces the rkvdec2 register write
list that drives the hardware.

Keeping field names and bit-flag values identical to Linux makes the BSP
register-builder port a near-mechanical exercise.

## Why this directory exists separately from the kernel

Linux kernel uAPI headers are licensed GPL-2 with the
"Linux-syscall-note" exemption — userspace consumers can redistribute
these struct definitions without inheriting GPL-2. The vendored copy
here is a small subset of that ABI, retyped for MSVC (Linux's
`__u8`/`__s32` → `<stdint.h>`).

## Updating

When Linux's H.264 decoder ABI grows new fields (e.g. when the kernel
adds support for SVC), pick the changes back from
`include/uapi/linux/v4l2-controls.h` (the H.264-tagged sections starting
around `V4L2_H264_SPS_CONSTRAINT_SET0_FLAG`). Don't bring in any
non-H.264 controls from the same header — keep this file scoped.
