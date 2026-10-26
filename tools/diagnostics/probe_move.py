#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_move.py —— 不用任何 odom，纯用雷达几何判定"车实际往哪走"

为什么这么写：
  autorun 实测发现：融合系里前进 1.186m，但位移方向与航向差 101°。
  当时靠 odom 自己互相印证，全是循环论证 —— 三路 odom 可能一起错。
  唯一的外部裁判是雷达：车往哪边走，那个方向的墙面距离就减少多少。
  判据（2026-10-22 已验证过的纯几何关系）：
      Δr(θ) = −d·cos(θ − θ_move)
  即"沿 θ_move 走 d"，则 θ_move 方向的距离减少 d，反方向增加 d。

全程用**时间**控制速度，不读任何 odom 做反馈 —— 这样测出来的才是真值。
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


class Probe(Node):
    def __init__(self):
        super().__init__('probe_move')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None
        self.fl = None
        self.fw = None
        self.fr = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos)
        self.create_subscription(Odometry, '/odom_laser',
                                 lambda m: setattr(self, 'fl', m), qos)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, 'fw', m), qos)
        self.create_subscription(Odometry, '/odom_fused',
                                 lambda m: setattr(self, 'fr', m), qos)

    def wait(self, sec=0.4):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def snap(self):
        s = self.scan
        inc = s.angle_increment
        bins = {}
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min or r > s.range_max:
                continue
            ad = math.degrees(s.angle_min + i * inc) % 360.0
            k = int(round(ad / 30.0)) % 12
            bins.setdefault(k, []).append(r)
        out = {}
        for k, v in bins.items():
            v.sort()
            out[k * 30] = v[len(v) // 2]
        return out

    def pos(self, m):
        if m is None:
            return None
        p = m.pose.pose.position
        return (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def stop(self):
        t = Twist()
        for _ in range(8):
            self.pub.publish(t)
            time.sleep(0.03)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turn', type=float, default=90.0, help='先原地转多少度（朝开阔方向）')
    ap.add_argument('--w', type=float, default=0.45)
    ap.add_argument('--v', type=float, default=0.10)
    ap.add_argument('--dist', type=float, default=0.30)
    a = ap.parse_args()

    rclpy.init()
    n = Probe()
    n.wait(1.5)

    print('=== T0 起始 ===')
    b0 = n.snap()
    l0, w0, f0 = n.pos(n.fl), n.pos(n.fw), n.pos(n.fr)
    print('  laser=%s' % ('(%.3f, %.3f)' % l0[:2] if l0 else 'None'))
    print('  wheel=%s' % ('(%.3f, %.3f)' % w0[:2] if w0 else 'None'))
    print('  fused=%s' % ('(%.3f, %.3f)' % f0[:2] if f0 else 'None'))

    # ---- 原地转，朝开阔方向 ----
    if abs(a.turn) > 1.0:
        print('\n=== 原地转 %.0f° ===' % a.turn)
        dur = abs(math.radians(a.turn)) / a.w
        t = Twist()
        t.angular.z = a.w if a.turn > 0 else -a.w
        t0 = time.time()
        while time.time() - t0 < dur:
            n.pub.publish(t)
            n.wait(0.05)
        n.stop()
        n.wait(0.8)

    print('\n=== T1 转向后（即将前进的起点）===')
    b1 = n.snap()
    l1, w1, f1 = n.pos(n.fl), n.pos(n.fw), n.pos(n.fr)
    print('  laser=%s' % ('(%.3f, %.3f)' % l1[:2] if l1 else 'None'))
    print('  wheel=%s' % ('(%.3f, %.3f)' % w1[:2] if w1 else 'None'))
    print('  fused=%s' % ('(%.3f, %.3f)' % f1[:2] if f1 else 'None'))
    print('  扇区距离(角度:中位):')
    for k in range(0, 360, 30):
        if k in b1:
            tag = ''
            if k == 180:
                tag = ' <-车头'
            elif k == 0:
                tag = ' <-车尾'
            elif k == 90:
                tag = ' <-右'
            elif k == 270:
                tag = ' <-左'
            print('    %3d°: %.2f m%s' % (k, b1[k], tag))

    # ---- 纯时间控制前进 ----
    print('\n=== 前进 %.2f m（%.1f s @ %.2f m/s，不读 odom）==='
          % (a.dist, a.dist / a.v, a.v))
    t = Twist()
    t.linear.x = a.v
    t0 = time.time()
    while time.time() - t0 < a.dist / a.v:
        n.pub.publish(t)
        n.wait(0.05)
    n.stop()
    n.wait(1.0)

    print('\n=== T2 前进后 ===')
    b2 = n.snap()
    l2, w2, f2 = n.pos(n.fl), n.pos(n.fw), n.pos(n.fr)
    print('  laser=%s' % ('(%.3f, %.3f)' % l2[:2] if l2 else 'None'))
    print('  wheel=%s' % ('(%.3f, %.3f)' % w2[:2] if w2 else 'None'))
    print('  fused=%s' % ('(%.3f, %.3f)' % f2[:2] if f2 else 'None'))

    # ---- 判定 ----
    print('\n' + '=' * 72)
    print('扇区距离变化（T1 -> T2，负数=该方向变近=车往那边走了）')
    print('%-6s %-9s %-9s %-9s' % ('角度', 'T1', 'T2', 'Δ'))
    print('-' * 72)
    deltas = {}
    for k in range(0, 360, 30):
        if k in b1 and k in b2:
            d = b2[k] - b1[k]
            deltas[k] = d
            nm = {0: '车尾', 90: '右', 180: '车头', 270: '左'}.get(k, '')
            print('%-6s %-9.2f %-9.2f %+9.2f  %s' % ('%d°' % k, b1[k], b2[k], d, nm))

    # 用最大减少量判方向
    if deltas:
        best_k = min(deltas, key=lambda k: deltas[k])
        print('\n>>> 雷达裁判：距离减少最多的方向 = %d°（%+.2f m）' % (best_k, deltas[best_k]))
        print('    => 车实际朝 %d° 方向走了约 %.2f m' % (best_k, -deltas[best_k]))
        # 拟合：Δr(θ) = -d*cos(θ-θm) —— 取 Δr 最小与最大两点的中点更稳
        max_k = max(deltas, key=lambda k: deltas[k])
        print('    反向校验：%d° 增加 %+.2f m（应约等于 +d）' % (max_k, deltas[max_k]))

    def vec(p, q):
        if p is None or q is None:
            return None
        dx, dy = q[0] - p[0], q[1] - p[1]
        return dx, dy, math.hypot(dx, dy), math.degrees(math.atan2(dy, dx))

    print('\n' + '=' * 72)
    print('三路 odom 的位移向量（T1 -> T2）')
    print('-' * 72)
    for nm, p, q in (('/odom_laser', l1, l2), ('/odom(固件)', w1, w2), ('/odom_fused', f1, f2)):
        v = vec(p, q)
        if v:
            print('%-14s Δ=(%+.3f, %+.3f)  |Δ|=%.3f m  方向=%+.1f°'
                  % (nm, v[0], v[1], v[2], v[3]))
        else:
            print('%-14s 无数据' % nm)

    if deltas:
        best_k = min(deltas, key=lambda k: deltas[k])
        print('\n判读：')
        print('  雷达说车朝 %d° 走' % best_k)
        for nm, p, q in (('/odom_laser', l1, l2), ('/odom', w1, w2), ('/odom_fused', f1, f2)):
            v = vec(p, q)
            if v:
                print('  %s 说 %+.1f°  → 差 %+.1f°'
                      % (nm, v[3], (v[3] - best_k + 180) % 360 - 180))

    n.stop()
    n.destroy_node()
    rclpy.shutdown()


main()
