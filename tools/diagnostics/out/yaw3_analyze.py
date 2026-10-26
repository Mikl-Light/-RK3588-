# -*- coding: utf-8 -*-
"""比较三路航向在 360° 旋转中的一致性，找阶跃点。

为什么这么写：
  fused 航向 = laser 基准 + 0.95*laser增量 + 0.05*wheel增量。
  若 laser(rf2o) 在某个朝向 scan-matching 退化，它的 yaw 会出现阶跃；
  fused 因为没有 yaw 跳变保护，会跟着跳，而 wheel(固件磁+陀螺) 是独立的，
  所以"三路差值的突变点"就是 rf2o 出问题的朝向。
"""
import csv
import math
import sys


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


rows = []
with open(sys.argv[1] if len(sys.argv) > 1 else 'yaw3_log.csv') as f:
    for r in csv.DictReader(f):
        rows.append(r)

print('样本 %d 行' % len(rows))

# 累积各路增量
acc = {'f': 0.0, 'l': 0.0, 'w': 0.0}
prev = {}
peak = []
for r in rows:
    t = float(r['t'])
    cur = {'f': float(r['fyaw']), 'l': float(r['lyaw']), 'w': float(r['wyaw'])}
    out = {'t': t}
    for k in ('l', 'w'):
        if k in prev:
            d = wrap(cur[k] - prev[k])
            if abs(d) < 3.0:      # 丢弃 wrap 或掉线造成的伪跳变
                acc[k] += d
        prev[k] = cur[k]
    out['l'] = acc['l']
    out['w'] = acc['w']
    out['facc'] = float(r['yaw_acc'])
    out['d'] = math.degrees(acc['l'] - acc['w'])   # laser 与 wheel 的累积差
    peak.append(out)
    prev = cur

# 打印抽样
print('\n  t(s)   fused累计  laser累计  wheel累计   (laser-wheel)')
step = max(1, len(peak) // 25)
for i in range(0, len(peak), step):
    p = peak[i]
    print('  %5.1f  %8.1f°  %8.1f°  %8.1f°   %+7.1f°'
          % (p['t'], math.degrees(p['facc']), math.degrees(p['l']),
             math.degrees(p['w']), p['d']))

last = peak[-1]
print('\n=== 终值 ===')
print('  fused 累计 %+.1f°' % math.degrees(last['facc']))
print('  laser 累计 %+.1f°' % math.degrees(last['l']))
print('  wheel 累计 %+.1f°' % math.degrees(last['w']))
print('  laser − wheel = %+.1f°' % last['d'])
print('  fused − laser = %+.1f°' % math.degrees(last['facc'] - last['l']))

# 找单帧阶跃最大的几处
print('\n=== laser 单帧最大阶跃（排除 wrap） ===')
jumps = []
for i in range(1, len(rows)):
    d = wrap(float(rows[i]['lyaw']) - float(rows[i - 1]['lyaw']))
    if abs(d) < 3.0:
        jumps.append((abs(d), math.degrees(d), float(rows[i]['t']),
                      math.degrees(float(rows[i]['fyaw']))))
jumps.sort(reverse=True)
for a, d, t, fy in jumps[:6]:
    print('  t=%5.1fs  Δ=%+7.2f°   当时 fused航向=%+7.1f°' % (t, d, fy))

print('\n=== wheel 单帧最大阶跃 ===')
jumps = []
for i in range(1, len(rows)):
    d = wrap(float(rows[i]['wyaw']) - float(rows[i - 1]['wyaw']))
    if abs(d) < 3.0:
        jumps.append((abs(d), math.degrees(d), float(rows[i]['t']),
                      math.degrees(float(rows[i]['fyaw']))))
jumps.sort(reverse=True)
for a, d, t, fy in jumps[:6]:
    print('  t=%5.1fs  Δ=%+7.2f°   当时 fused航向=%+7.1f°' % (t, d, fy))
