#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""measure_line.py —— 长距离直线测试：比对三路里程计的累积位移

为什么不复用 fusion_line_test.py：
  那个脚本用点云互相关做真值，但网格搜索范围默认 ±0.45m。
  跑 2m 时要么把搜索范围放大到 ±2.5m（几千次匹配，容易超时），
  要么就匹配失败。本脚本改用"轮式 vs 激光"互相校验——
  两者是完全独立的物理量（编码器 vs 激光几何），不共享任何误差源，
  足以判断哪一路可信。

用法：
  python3 measure_line.py --speed 0.2 --dur 10
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class M(Node):
    def __init__(self):
        super().__init__('measure_line')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.odom = None
        self.laser = None
        self.fused = None
        self.create_subscription(Odometry, '/odom', self._o, qos)
        self.create_subscription(Odometry, '/odom_laser', self._l, qos)
        self.create_subscription(Odometry, '/odom_fused', self._f, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _o(self, m):
        self.odom = m

    def _l(self, m):
        self.laser = m

    def _f(self, m):
        self.fused = m

    def wait_all(self, timeout=12.0):
        t = time.time()
        while time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom and self.laser and self.fused:
                return True
        return False

    def snap(self):
        def p(m):
            return (m.pose.pose.position.x, m.pose.pose.position.y)
        return {'odom': p(self.odom), 'laser': p(self.laser), 'fused': p(self.fused)}

    def drive(self, speed, dur, hz=20):
        msg = Twist()
        msg.linear.x = float(speed)
        try:
            t_end = time.time() + dur
            while time.time() < t_end:
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=1.0 / hz)
        finally:
            msg.linear.x = 0.0
            for _ in range(10):
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--dur', type=float, default=10.0)
    ap.add_argument('--settle', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    node = M()

    print('等待 /odom /odom_laser /odom_fused ...')
    if not node.wait_all():
        print('有话题没数据，先检查节点')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    s0 = node.snap()
    print('\n起点')
    for k, v in s0.items():
        print('  %-6s x=%+8.3f  y=%+8.3f' % (k, v[0], v[1]))

    target = a.speed * a.dur
    print('\n>>> /cmd_vel %.2f m/s × %.1fs（理论 %.2f m）' % (a.speed, a.dur, target))
    node.drive(a.speed, a.dur)

    print('停车，等 %.1fs ...' % a.settle)
    t_end = time.time() + a.settle
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    s1 = node.snap()
    print('\n终点')
    for k, v in s1.items():
        print('  %-6s x=%+8.3f  y=%+8.3f' % (k, v[0], v[1]))

    d = {k: dist(s0[k], s1[k]) for k in s0}
    print('\n' + '=' * 58)
    print('累积位移')
    for k in ('odom', 'laser', 'fused'):
        print('  %-6s %7.3f m' % (k, d[k]))
    print('\n  轮式 / 激光 = %.4f' % (d['odom'] / d['laser'] if d['laser'] > 1e-6 else 0))
    print('  轮式 / 目标 = %.4f' % (d['odom'] / target if target else 0))
    print('  激光 / 目标 = %.4f' % (d['laser'] / target if target else 0))
    print('=' * 58)

    # 方向判断：位移是否沿 x 轴（前进），y 应接近 0
    dx = s1['laser'][0] - s0['laser'][0]
    dy = s1['laser'][1] - s0['laser'][1]
    print('\n  激光位移方向: dx=%+.3f dy=%+.3f  (dx>0 = 前进)' % (dx, dy))
    if dx < 0:
        print('  ⚠ 激光位移为负 → 方向仍反，检查 /scan 的 angle_offset_deg')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
