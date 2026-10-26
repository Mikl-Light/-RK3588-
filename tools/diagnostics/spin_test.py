#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spin_test.py — 原地转一圈闭环测试（判圈用轮式磁航向 yaw_wheel_deg）

流程：持续发 /cmd_vel(angular.z)，用 /fusion_diag 的 yaw_wheel_deg 累计转过的角度，
      累计到目标角度 → 发 0 停止 → 打印 yaw_wheel 与 yaw_fused 的闭环差。

用法：python3 spin_test.py [目标角度=360] [角速度rad/s=0.5]
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class SpinTest(Node):
    def __init__(self, target_deg, wz):
        super().__init__('spin_test')
        self.target = target_deg
        self.wz = wz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/fusion_diag', self.cb, 10)
        self.accum = 0.0
        self.last_wheel = None
        self.last_fused = None
        self.start_wheel = None
        self.start_fused = None
        self.done = False

    def cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        yh = d['yaw_wheel_deg']
        yf = d['yaw_fused_deg']
        if self.last_wheel is None:
            self.start_wheel = yh
            self.start_fused = yf
        else:
            self.accum += wrap180(yh - self.last_wheel)
        self.last_wheel = yh
        self.last_fused = yf

    def spin(self):
        cmd = Twist()
        cmd.angular.z = self.wz
        t0 = time.time()
        rate_hz = 20.0
        while rclpy.ok() and time.time() - t0 < 60.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            # 还没收到第一帧 diag，先别动
            if self.last_wheel is None:
                continue
            if abs(self.accum) >= abs(self.target):
                break
            self.pub.publish(cmd)
            time.sleep(1.0 / rate_hz)
        # 停止：连发 5 帧零速
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.03)
        # 再收 1 秒数据，让 yaw_fused 收敛
        t_end = time.time() + 1.5
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        print()
        print('=== 原地旋转闭环结果 (目标 %.0f°, wz=%.2f rad/s) ===' % (self.target, self.wz))
        print('用时                : %.2f s' % (time.time() - t0))
        print('磁航向累计转过      : %+.2f °' % self.accum)
        if self.last_wheel is not None:
            print('yaw_wheel 起点→终点 : %+.2f → %+.2f，净变化 %+.2f °'
                  % (self.start_wheel, self.last_wheel, wrap180(self.last_wheel - self.start_wheel)))
            print('yaw_fused 起点→终点 : %+.2f → %+.2f，净变化 %+.2f °'
                  % (self.start_fused, self.last_fused, wrap180(self.last_fused - self.start_fused)))
            print('融合 - 磁航向 偏差  : %+.2f °' % wrap180(self.last_fused - self.last_wheel))


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
    wz = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    rclpy.init()
    node = SpinTest(target, wz)
    try:
        node.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
