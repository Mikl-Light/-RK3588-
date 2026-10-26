#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scan_around.py —— 全向逐角度输出距离，用来找人（或其他近处目标）在哪个方位

为什么需要：
  laser_line.py --probe 只输出"区间 + 最近值"，看不出具体是哪个角度。
  本脚本按固定步长逐角度打印，人站在车前时可以直接读到"人出现在哪一度"。

用法：
  python3 scan_around.py --step 5 --n 5
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class Scanner(Node):
    def __init__(self):
        super().__init__('scan_around')
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.last = None
        self.sub = self.create_subscription(LaserScan, '/scan', self._cb, qos)

    def _cb(self, msg):
        self.last = msg


def dist_at(msg, center_deg, half_deg):
    vals = []
    inc = msg.angle_increment
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            continue
        a = msg.angle_min + i * inc
        ad = math.degrees(a)
        dd = (ad - center_deg + 180.0) % 360.0 - 180.0
        if abs(dd) <= half_deg:
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=5.0)
    ap.add_argument('--n', type=int, default=5, help='每个角度采样帧数')
    ap.add_argument('--half', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    node = Scanner()

    # 等第一帧
    import time
    t0 = time.time()
    while node.last is None and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.last is None:
        print('收不到 /scan')
        return

    print("全向扫描（步长 %.0f°, 每角 %d 帧中位）:" % (a.step, a.n))
    print("  角度      距离        备注")
    rows = []
    c = -180.0
    while c < 180.0:
        vals = []
        for _ in range(a.n):
            t = time.time()
            while time.time() - t < 0.4:
                rclpy.spin_once(node, timeout_sec=0.05)
            d = dist_at(node.last, c, a.half)
            if d is not None:
                vals.append(d)
        if vals:
            vals.sort()
            med = vals[len(vals) // 2]
            spread = vals[-1] - vals[0]
            note = ''
            if 0.70 <= med <= 1.40:
                note = '  <== 1m 附近，可能是人'
            if spread > 0.05:
                note += ' (跳 %.0fmm)' % (spread * 1000)
            print("  %+6.0f°   %6.3f m%s" % (c, med, note))
            rows.append((c, med, spread))
        else:
            print("  %+6.0f°   无回波" % c)
        c += a.step

    # 汇总：1m 附近的方位
    near = [r for r in rows if 0.70 <= r[1] <= 1.40]
    print("\n【1m 附近的方位】", end=' ')
    if near:
        print(', '.join('%+.0f°(%.2fm)' % (r[0], r[1]) for r in near))
    else:
        print('【没有】—— 人可能没站进扫描面，或者距离不在 0.7~1.4m')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
