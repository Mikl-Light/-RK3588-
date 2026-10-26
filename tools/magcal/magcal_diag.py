#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：磁航向残差 38° 到底来自哪里
查三件事：
  1) $T 帧率 / tick 间隔是否均匀
  2) 磁读数是否有"阶梯/冻结"（相邻帧完全相同的比例）
  3) 磁方位角速率 vs 陀螺速率的瞬时对比（看是延迟还是尺度）
  4) 分圈误差：每转一圈后累计误差
"""
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


def unwrap(a):
    o = [a[0]]
    for v in a[1:]:
        d = v - o[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        o.append(o[-1] + d)
    return np.array(o)


rows = [r for r in load(sys.argv[1]) if r['phase'] == 1]
t = np.array([r['tick'] for r in rows]) / 1000.0
dt = np.diff(t)
print('=== 1. 时间轴 ===')
print('  帧数 %d  时长 %.1fs  $T 帧率 %.1f Hz' % (len(rows), t[-1] - t[0], (len(t) - 1) / (t[-1] - t[0])))
print('  dt: 均值 %.1f ms  std %.1f ms  min %.0f  max %.0f' % (
    dt.mean() * 1000, dt.std() * 1000, dt.min() * 1000, dt.max() * 1000))
big = np.sum(dt > 0.1)
print('  dt>100ms 的次数: %d（丢帧/卡顿）' % big)

mx = np.array([r['mx'] for r in rows]) / 100.0
my = np.array([r['my'] for r in rows]) / 100.0
mz = np.array([r['mz'] for r in rows]) / 100.0
gz = np.array([r['gz'] for r in rows]) / 100.0  # °/s

print('\n=== 2. 磁读数是否冻结/阶梯 ===')
for name, v in (('mx', mx), ('my', my), ('mz', mz)):
    same = np.sum(np.diff(v) == 0)
    print('  %s: 相邻帧完全相同的比例 %.1f%%  (%d/%d)  唯一值 %d 个' % (
        name, same / len(v) * 100, same, len(v) - 1, len(np.unique(v))))

print('\n=== 3. 瞬时速率对比（磁方位角 vs 陀螺）===')
h = unwrap(list(np.degrees(np.arctan2(mx, my))))
w_mag = np.gradient(h) / np.gradient(t)      # °/s
seg = len(t) // 6
print('   窗口        磁方位角速率  陀螺速率   差')
for i in range(6):
    a, b = i * seg, min((i + 1) * seg, len(t))
    print('  %5.1f-%5.1fs   %+8.2f     %+8.2f   %+6.2f' % (
        t[a] - t[0], t[b - 1] - t[0],
        w_mag[a:b].mean(), gz[a:b].mean(), w_mag[a:b].mean() - gz[a:b].mean()))
print('  全段: 磁 %+.2f °/s   陀螺 %+.2f °/s   比值 %.4f' % (
    w_mag.mean(), gz.mean(), w_mag.mean() / gz.mean()))

print('\n=== 4. 分圈误差（每满 360° 陀螺积分时的磁累计）===')
gyro = np.concatenate([[0], np.cumsum(gz[1:] * dt)])
k = 1
for i in range(1, len(t)):
    if gyro[i] >= 360 * k:
        print('  陀螺 %4.0f° 时  磁累计 %+7.1f°   误差 %+7.1f°' % (gyro[i], h[i], h[i] - gyro[i]))
        k += 1
        if k > 6:
            break
print('  终点: 陀螺 %+.1f°  磁 %+.1f°  误差 %+.1f°' % (gyro[-1], h[-1], h[-1] - gyro[-1]))

print('\n=== 5. 误差随方位角的分布（看是否固定在某个方向偏差大）===')
# 用 gyro 作为真值，把磁-陀螺差按陀螺方位分箱
err = h - gyro
bins = np.linspace(-180, 540, 13)
idx = np.digitize(gyro, bins)
print('  陀螺方位段   样本  平均误差   误差std')
for i in range(1, len(bins)):
    sel = idx == i
    if sel.sum() > 5:
        print('  %+6.0f~%+6.0f  %5d   %+7.2f   %6.2f' % (
            bins[i - 1], bins[i], sel.sum(), err[sel].mean(), err[sel].std()))
