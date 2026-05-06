#!/usr/bin/env python3
"""Add a dev_info that prints each request's class/s/e BEFORE writel loop."""
p = "/home/vibecoder/linux-rockchip/drivers/video/rockchip/mpp/mpp_av1dec.c"
with open(p) as f:
    s = f.read()

old = '''	for (i = 0; i < task->w_req_cnt; i++) {
		int class;
		struct mpp_request *req = &task->w_reqs[i];

		for (class = 0; class < hw->reg_class_num; class++) {'''

new = '''	dev_info(mpp->dev, "AV1SHIM REQS: w_req_cnt=%d\\n", task->w_req_cnt);
	for (i = 0; i < task->w_req_cnt; i++) {
		int class;
		struct mpp_request *req = &task->w_reqs[i];

		for (class = 0; class < hw->reg_class_num; class++) {'''

print("matches:", s.count(old))
assert s.count(old) == 1
s = s.replace(old, new)

# Also dump s/e per class
old2 = '''			regs = (u32 *)task->reg_class[class].data;

			mpp_debug(DEBUG_TASK_INFO, "found rd_class %d, base=%08x, s=%d, e=%d\\n",
				  class, base, s, e);
			for (j = s; j < e; j++) {
				if (class == 0 && j == hw->hw.reg_en) {
					en_val = regs[j];
					continue;
				}
				writel_relaxed(regs[j], dec->reg_base[class] + j * sizeof(u32));
			}'''
new2 = '''			regs = (u32 *)task->reg_class[class].data;

			mpp_debug(DEBUG_TASK_INFO, "found rd_class %d, base=%08x, s=%d, e=%d\\n",
				  class, base, s, e);
			dev_info(mpp->dev, "AV1SHIM REQ[%d] class=%d s=%d e=%d\\n", i, class, s, e);
			for (j = s; j < e; j++) {
				if (class == 0 && j == hw->hw.reg_en) {
					en_val = regs[j];
					continue;
				}
				writel_relaxed(regs[j], dec->reg_base[class] + j * sizeof(u32));
			}'''
print("matches2:", s.count(old2))
assert s.count(old2) == 1
s = s.replace(old2, new2)

with open(p, "w") as f:
    f.write(s)
print("ok")
