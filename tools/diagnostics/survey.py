#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""survey.py —— 分扇区勘察周围空间，给自动跑圈定安全边界

为什么需要：
  盲发 cmd_vel 上次撞了。跑图之前必须先知道"往哪个方向能走多远"。
  本车雷达 0° 在车尾、180° 在车头（2026-10-22 定案），所以：
    前方 = 180°，左方 = 270°(或 -90°)，右方 = 90°，后方 = 0°
  （雷达 yaw 已由 base_link->laser 的 TF 补 π，raw 角度仍是这个关系）

输出：
  每个扇区的中位距离 + 最近距离 + 有效点数，并给出最空/最堵方向。
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class Scout(Node):
    def __init__(self):
        super().__init__('survey')
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.last = None
        self.sub = self.create_subscription(LaserScan, '/scan', self._cb, qos)

    def _cb(self, msg):
        self.last = msg


NAMES = {180: '前', 90: '右', 270: '左', 0: '后'}


def sector_stats(msg, center_deg, half_deg):
    vals = []
    inc = msg.angle_increment
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            continue
        ad = math.degrees(msg.angle_min + i * inc) % 360.0
        dd = (ad - center_deg + 180.0) % 360.0 - 180.0
        if abs(dd) <= half_deg:
            vals.append(r)
    if not vals:
        return None, None, 0
    vals.sort()
    return vals[len(vals) // 2], vals[0], len(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=8, help='扇区数量（默认 8 = 每 45°）')
    ap.add_argument('--min-hits', type=int, default=8, help='少于此点数的扇区标记为弱')
    a = ap.parse_args()

    rclpy.init()
    node = Scout()
    while node.last is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    msg = node.last

    print('angle_min=%.1f°  angle_max=%.1f°  inc=%.2f°  点数=%d  量程 %.2f~%.2f m'
          % (math.degrees(msg.angle_min), math.degrees(msg.angle_max),
             math.degrees(msg.angle_increment), len(msg.ranges),
             msg.range_min, msg.range_max))
    print()
    print('%-8s %-10s %-10s %-8s %s' % ('方位', '中位距离', '最近距离', '点数', '判读'))
    print('-' * 62)

    step = 360 // a.n
    best = (None, -1.0)
    worst = (None, 99.0)
    for k in range(a.n):
        c = (180 + k * step) % 360   # 从车头 180° 开始排
        med, mn, cnt = sector_stats(msg, c, step / 2.0)
        if cnt < a.min_hits:
            print('%-8s %-10s %-10s %-8d %s' % ('%d°(%s)' % (c, NAMES.get(c, '')), '无回波', '无回波', cnt, '开阔/无目标'))
            if med is None and 5.0 > best[1]:
                best = (c, 5.0)
            continue
        med_s = '%.2f m' % med
        mn_s = '%.2f m' % mn
        if med > best[1]:
            best = (c, med)
        if mn < worst[1]:
            worst = (c, mn)
        if mn < 0.35:
            tag = '⚠ 很近'
        elif med < 0.8:
            tag = '偏窄'
        else:
            tag = '可走'
        print('%-8s %-10s %-10s %-8d %s' % ('%d°(%s)' % (c, NAMES.get(c, '')), med_s, mn_s, cnt, tag))

    print('-' * 62)
    print('最空方向: %s°  中位 %.2f m' % (best[0], best[1]))
    print('最近障碍: %s°  %.2f m' % (worst[0], worst[1]))

    node.destroy_node()
    rclpy.shutdown()


main()
