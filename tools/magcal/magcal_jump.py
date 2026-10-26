#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位磁方位角跳变点，打印跳变前后的原始读数"""
import sys
import csv
import math
import numpy as np


def load(path):
    out = []
    with open(path) as fp:
        for d in csv.DictReader(fp):
            out.append({k: float(v) for k, v in d.items()})
    return out


rows = [r for r in load(sys.argv[1]) if r['phase'] == 1]
t = np.array([r['tick'] for r in rows]) / 1000.0
mx = np.array([r['mx'] for r in rows]) / 100.0
my = np.array([r['my'] for r in rows]) / 100.0
mz = np.array([r['mz'] for r in rows]) / 100.0
gz = np.array([r['gz'] for r in rows]) / 100.0

h = np.degrees(np.arctan2(mx, my))
# 相邻帧的一阶差分（不做 unwrap，直接看原始角度的跳变）
dh = np.diff(h)
dh = (dh + 180) % 360 - 180        # 归一到 ±180

# 真实每帧应转 20.45 * 0.0333 = 0.68°
print('每帧理论变化 ≈ +0.68°（陀螺 20.45°/s × 33.3ms）')
print('相邻帧 |Δh| > 5° 的帧数: %d / %d' % (np.sum(np.abs(dh) > 5), len(dh)))
print()

jumps = np.where(np.abs(dh) > 5)[0]
if len(jumps):
    print('=== 跳变点明细（最多前 12 个）===')
    print(' 帧号    t(s)      mx      my      mz     Δh(°)   Δmx    Δmy   方位角')
    for j in jumps[:12]:
        print('  %4d  %6.2f  %7.2f %7.2f %7.2f  %+7.2f  %+6.2f %+6.2f  %7.2f' % (
            j + 1, t[j + 1] - t[0], mx[j + 1], my[j + 1], mz[j + 1], dh[j],
            mx[j + 1] - mx[j], my[j + 1] - my[j], h[j + 1]))

print('\n=== 每 1 秒采样（陀螺累计 vs 磁方位角原始值）===')
gyro = np.concatenate([[0], np.cumsum(gz[1:] * np.diff(t))])
print('  t(s)   陀螺累计   磁方位角   磁-陀螺    mx      my      mz     |m_xy|')
for i in range(0, len(t), 30):
    d = (h[i] - gyro[i])
    d = (d + 180) % 360 - 180
    print('  %5.1f  %+8.1f  %+8.1f  %+8.1f  %6.2f  %6.2f  %6.2f  %6.2f' % (
        t[i] - t[0], gyro[i], h[i], d, mx[i], my[i], mz[i],
        math.hypot(mx[i], my[i])))
