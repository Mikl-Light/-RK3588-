#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""静止漂移测试：记录 /fusion_diag 的 yaw_fused / yaw_wheel / yaw_laser 在静止时的变化"""
import sys
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0


class D(Node):
    def __init__(self):
        super().__init__('drift_test')
        self.rows = []
        self.t0 = time.time()
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/fusion_diag', self.cb, qos)

    def cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self.rows.append((time.time() - self.t0, d.get('yaw_fused_deg'),
                          d.get('yaw_wheel_deg'), d.get('yaw_laser_deg'),
                          d.get('still')))


rclpy.init()
n = D()
t_end = time.time() + 12.0
while time.time() < t_end and len(n.rows) < 20:
    rclpy.spin_once(n, timeout_sec=0.05)
print('发现完成，帧数 %d，开始记录 %.0fs' % (len(n.rows), SEC))
base = len(n.rows)

t_end = time.time() + SEC
while rclpy.ok() and time.time() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)

seg = n.rows[base:]
if len(seg) < 20:
    print('采集不足: %d' % len(seg))
else:
    dur = seg[-1][0] - seg[0][0]
    print('\n=== 静止 %.1fs 漂移（%d 帧）===' % (dur, len(seg)))
    for idx, name in ((1, 'yaw_fused'), (2, 'yaw_wheel'), (3, 'yaw_laser')):
        v = [r[idx] for r in seg if r[idx] is not None]
        if not v:
            continue
        print('  %-10s 起始 %8.2f  终止 %8.2f  变化 %+7.2f°  峰峰 %6.2f°  段数 %d'
              % (name, v[0], v[-1], v[-1] - v[0], max(v) - min(v), len(v)))
    st = sum(1 for r in seg if r[4])
    print('  still=True 占比 %.0f%%' % (st / len(seg) * 100))
    # 分段看是否单调
    k = max(1, len(seg) // 6)
    print('  分段 yaw_fused: ' + ' | '.join(
        '%.2f' % seg[i][1] for i in range(0, len(seg), k)))

n.destroy_node()
rclpy.shutdown()
