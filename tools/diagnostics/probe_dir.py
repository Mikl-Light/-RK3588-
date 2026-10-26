#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_dir.py —— 判定每路 odom 的"位移方向"与"自身航向"是否自洽

判据：
  自洽的 odom，沿车头前进时 位移方向 ≈ 航向（差 0°）。
  差 90° → 位移在另一套坐标系里（固件 X=北/Y=西 vs ROS X=东/Y=北 的经典问题）。
  差 180° → 反装 / 符号翻转。

为什么必须这么测：
  "三路 odom 互相印证"是循环论证 —— 它们可能一起错。
  外部裁判是雷达（车确实朝车头走了，已由 Δr(θ)=-d·cos(θ-θmove) 验证）。
  所以只要把"位移方向"和"自身航向"比，就能定位是哪一路在骗人。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class P(Node):
    def __init__(self):
        super().__init__('probe_dir')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.m = {}
        self.scan = None
        for nm, tp in (('laser', '/odom_laser'), ('wheel', '/odom'), ('fused', '/odom_fused')):
            self.create_subscription(
                Odometry, tp,
                (lambda n: lambda msg: self.m.__setitem__(n, msg))(nm), qos)
        self.create_subscription(LaserScan, '/scan',
                                 lambda msg: setattr(self, 'scan', msg), qos)

    def wait(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def snap(self):
        out = {}
        for nm, msg in self.m.items():
            p = msg.pose.pose
            out[nm] = (p.position.x, p.position.y, yaw_of(p.orientation))
        return out

    def front(self):
        """车头 180±35° 最近距离"""
        if self.scan is None:
            return 99.0
        s = self.scan
        best = 99.0
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            ad = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
            if 145.0 <= ad <= 215.0:
                best = min(best, r)
        return best

    def stop(self):
        t = Twist()
        for _ in range(8):
            self.pub.publish(t)
            time.sleep(0.03)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=0.10)
    ap.add_argument('--dist', type=float, default=0.30)
    a = ap.parse_args()

    rclpy.init()
    n = P()
    n.wait(2.0)

    print('车头前方最近 %.2f m（安全阈值 0.40）' % n.front())
    if n.front() < 0.40:
        print('前方过近，拒绝执行 —— 请先转向开阔方向')
        n.destroy_node()
        rclpy.shutdown()
        return

    s1 = n.snap()
    print('\n=== T1 ===')
    for k in ('laser', 'wheel', 'fused'):
        if k in s1:
            x, y, t = s1[k]
            print('  %-6s (%+.3f, %+.3f)  yaw=%+.1f°' % (k, x, y, math.degrees(t)))

    print('\n前进 %.2f m（%.1fs，纯时间控制）' % (a.dist, a.dist / a.v))
    t = Twist()
    t.linear.x = a.v
    t0 = time.time()
    while time.time() - t0 < a.dist / a.v:
        n.pub.publish(t)
        n.wait(0.05)
    n.stop()
    n.wait(1.2)

    s2 = n.snap()
    print('\n=== T2 ===')
    for k in ('laser', 'wheel', 'fused'):
        if k in s2:
            x, y, t = s2[k]
            print('  %-6s (%+.3f, %+.3f)  yaw=%+.1f°' % (k, x, y, math.degrees(t)))

    print('\n' + '=' * 70)
    print('%-7s %-24s %-10s %-10s %s' % ('来源', '位移Δ', '位移方向', '平均yaw', '偏差'))
    print('-' * 70)
    for k in ('laser', 'wheel', 'fused'):
        if k not in s1 or k not in s2:
            continue
        dx = s2[k][0] - s1[k][0]
        dy = s2[k][1] - s1[k][1]
        d = math.hypot(dx, dy)
        ang = math.degrees(math.atan2(dy, dx))
        ym = math.degrees(s1[k][2] + 0.5 * math.atan2(
            math.sin(s2[k][2] - s1[k][2]), math.cos(s2[k][2] - s1[k][2])))
        dev = (ang - ym + 180.0) % 360.0 - 180.0
        print('%-7s (%+.3f,%+.3f) |Δ|=%.3f  %+7.1f°   %+7.1f°   %+7.1f°'
              % (k, dx, dy, d, ang, ym, dev))

    print('-' * 70)
    print('偏差 ≈ 0°   → 自洽，可直接用于建图')
    print('偏差 ≈ ±90° → 位置在另一套坐标系（北/西 vs 东/北）')
    print('偏差 ≈ ±180°→ 符号翻转 / 反装补偿错')
    print('\n实际走了 %.2f m（时间控制，真值）' % a.dist)

    n.stop()
    n.destroy_node()
    rclpy.shutdown()


main()
