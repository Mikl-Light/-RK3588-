#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
融合动态精度验收（2026-10-20 磁标定之后）

目的：磁锚(v3.4.5-MAGCAL)已经可信，重新检验 /odom_fused 在动态下是否可信。
做法：静止基线 → 匀速原地转 N 秒 → 停稳，全程同时记录四路 yaw：
        wheel  = /odom        （固件 AHRS 航向 = 磁+陀螺，实测残差 0.93°）
        laser  = /odom_laser  （rf2o 激光里程计）
        fused  = /odom_fused  （odom_fusion 输出）
并把 wheel 当参考，看 laser/fused 的累计转角偏差。

用法：python3 fusion_dynamic_test.py [转速rad/s=0.3] [转动秒数=60]
注意：订阅全部用 best_effort —— 发布端若是 best_effort，reliable 订阅者收不到。
"""
import sys
import math
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

OMEGA = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
SPIN_SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
BASE_SEC = 4.0
STILL_SEC = 8.0


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(seq):
    if not seq:
        return []
    out = [seq[0]]
    for v in seq[1:]:
        d = v - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


class Tester(Node):
    def __init__(self):
        super().__init__('fusion_dyn_test')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.series = {'wheel': [], 'laser': [], 'fused': []}
        self.diag = None
        self.create_subscription(Odometry, '/odom', lambda m: self._cb('wheel', m), qos)
        self.create_subscription(Odometry, '/odom_laser', lambda m: self._cb('laser', m), qos)
        self.create_subscription(Odometry, '/odom_fused', lambda m: self._cb('fused', m), qos)
        self.create_subscription(String, '/fusion_diag', self._diag, qos)
        self.t0 = time.time()

    def _cb(self, key, msg):
        self.series[key].append((time.time() - self.t0,
                                 yaw_from_quat(msg.pose.pose.orientation)))

    def _diag(self, msg):
        try:
            self.diag = json.loads(msg.data)
        except Exception:
            pass

    def spin_for(self, sec):
        t_end = time.time() + sec
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, wz, sec):
        """持续发速度命令 sec 秒，期间每 50ms 重发"""
        t_end = time.time() + sec
        while rclpy.ok() and time.time() < t_end:
            m = Twist()
            m.angular.z = wz
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
        # 停车
        for _ in range(8):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def take(self, name):
        """取某一路当前累计序列，按窗口切出基线/旋转/停止段"""
        return self.series[name]


def main():
    rclpy.init()
    n = Tester()

    print('等待 DDS 发现（最多 12s，需 ≥20 帧/路）...')
    t_end = time.time() + 12.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if all(len(v) >= 20 for v in n.series.values()):
            break
    n.series = {k: [] for k in n.series}
    n.t0 = time.time()
    counts = {k: len(v) for k, v in n.series.items()}
    print('发现结果: %s  diag=%s' % (counts, 'YES' if n.diag else 'NO'))
    if not all(len(v) >= 20 for v in n.series.values()):
        # 再来一轮，打印每路帧数
        t_end = time.time() + 8.0
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
        print('重试后帧数: %s' % {k: len(v) for k, v in n.series.items()})
        if not all(len(v) >= 10 for v in n.series.values()):
            print('有话题收不到数据，放弃')
            n.destroy_node(); rclpy.shutdown(); return

    marks = {}

    print('\n[1] 静止基线 %.0fs ...' % BASE_SEC)
    n.spin_for(BASE_SEC)
    marks['base'] = {k: len(v) for k, v in n.series.items()}

    print('[2] 原地旋转 wz=%+.2f rad/s，持续 %.0fs ...' % (OMEGA, SPIN_SEC))
    n.drive(OMEGA, SPIN_SEC)
    marks['spin'] = {k: len(v) for k, v in n.series.items()}

    print('[3] 停稳观察 %.0fs ...' % STILL_SEC)
    n.drive(0.0, 0.5)
    n.spin_for(STILL_SEC)
    marks['end'] = {k: len(v) for k, v in n.series.items()}

    # ---- 分析 ----
    print('\n=== 各段帧数 ===')
    for seg in ('base', 'spin', 'end'):
        print('  %-5s %s' % (seg, marks[seg]))

    print('\n=== 四路航向（deg，unwrap 后相对基线起点）===')
    res = {}
    for key in ('wheel', 'laser', 'fused'):
        raw = [y for _, y in n.series[key]]
        un = unwrap(raw)
        if not un:
            continue
        base_n = marks['base'][key]
        spin_n = marks['spin'][key]
        hop = math.degrees(un[-1] - un[0])
        hspin = math.degrees(un[spin_n - 1] - un[base_n - 1]) if spin_n > base_n else 0.0
        hstill = math.degrees(un[-1] - un[spin_n - 1]) if len(un) > spin_n else 0.0
        res[key] = {'total': hop, 'spin': hspin, 'still': hstill,
                    'n': len(un)}
        print('  %-6s 总转角 %+8.2f°   旋转段 %+8.2f°   停后 %+7.2f°   (n=%d)'
              % (key, hop, hspin, hstill, len(un)))

    if 'wheel' in res and 'laser' in res:
        print('\n=== 相对 wheel(磁+陀螺，残差0.93°) 的偏差 ===')
        for key in ('laser', 'fused'):
            if key in res:
                print('  %-6s 总转角偏差 %+8.2f°  (%.2f%%)'
                      % (key, res[key]['total'] - res['wheel']['total'],
                         (res[key]['total'] - res['wheel']['total'])
                         / res['wheel']['total'] * 100 if res['wheel']['total'] else 0))

    if n.diag:
        print('\n=== /fusion_diag 终值 ===')
        for k in ('yaw_wheel_deg', 'yaw_laser_deg', 'yaw_fused_deg',
                  'trust_w', 'w_wheel', 'w_laser', 'still',
                  'scan_valid_ratio', 'mag_outlier', 'bias_b_deg_s'):
            if k in n.diag:
                print('  %-18s %s' % (k, n.diag[k]))

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
