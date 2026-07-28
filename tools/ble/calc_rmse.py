#!/usr/bin/env python3
"""快速计算 SWP RMSE。直接把终端输出的行粘贴进来，按 Ctrl+Z 结束。"""
import sys, math

data = []
for line in sys.stdin:
    line = line.strip()
    if not line or line == "SWP_DONE":
        break
    # Strip common prefixes like "← SWP:" or "SWP:"
    if "SWP:" in line:
        line = line[line.index("SWP:") + 4:]
    parts = line.split(",")
    if len(parts) == 3:
        try:
            data.append((int(parts[0]), int(parts[1]), int(parts[2])))
        except ValueError:
            pass

if not data:
    print("no data")
    sys.exit(1)

n = len(data)
mse_l = sum((t - al)**2 for t, al, _ in data) / n
mse_r = sum((t - ar)**2 for _, al, ar in data) / n
full = (math.sqrt(mse_l) + math.sqrt(mse_r)) / 2

cs = [(t, al, ar) for t, al, ar in data if t == 300]
if cs:
    nc = len(cs)
    cmse_l = sum((t - al)**2 for t, al, _ in cs) / nc
    cmse_r = sum((t - ar)**2 for t, _, ar in cs) / nc
    cfull = (math.sqrt(cmse_l) + math.sqrt(cmse_r)) / 2
    cmax_l = max(abs(t - al) for t, al, _ in cs)
    cmax_r = max(abs(t - ar) for _, al, ar in cs)
else:
    cfull = cmax_l = cmax_r = 0

print(f"全程RMSE:{full:.1f}  匀速RMSE:{cfull:.1f}  左max:{cmax_l}  右max:{cmax_r}")
