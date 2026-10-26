#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sign_test.py — 定速短时旋转，逐帧记录三条链路的方向与响应

目的：一次看清
  1. 命令 wz 的符号 与 IMU gyro_z 符号 是否一致（固件坐标系约定）
  2. 命令 wz 与 磁航向(yaw_wheel) 变化方向是否一致、速率是否对得上
  3. 命令 wz 与 激光航向(yaw_laser) 变化方向是否一致

用法：python3 sign_test.py [角速度=0.3] [时长秒=3]
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


class SignTest(Node):
    def __init__(self):
        super().__init__('sign_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/fusion_diag', self.cb, 10)
        self.d = None
        self.first = None
        self.rows = []

    def cb(self, msg):
        try:
            self.d = json.loads(msg.data)
        except Exception:
            return

    def run(self, wz, dur):
        # 等 DDS 发现
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.d is not None:
                break
        if self.d is None:
            print('收不到 /fusion_diag')
            return
        # 倒计时，方便人在车旁观察实际转向
        for k in (3, 2, 1):
            print('%d 秒后开始%s转（wz=%+.2f）…' % (k, '左' if wz > 0 else '右', wz))
            time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.05)

        self.first = dict(self.d)
        print('起始：yaw_wheel=%.2f  yaw_laser=%s  gyro_z=%+.4f'
              % (self.first['yaw_wheel_deg'],
                 self.first.get('yaw_laser_deg'), self.first['gyro_z_rad_s']))
        print()
        print('  时刻   gyro_z(rad/s)  yaw_wheel  yaw_laser  融合  Δwheel  Δlaser')
        print('-' * 78)

        cmd = Twist()
        cmd.angular.z = wz
        t_start = time.time()
        next_log = 0.0
        while rclpy.ok() and time.time() - t_start < dur:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
            el = time.time() - t_start
            if el >= next_log and self.d is not None:
                next_log += 1.0
                dw = wrap180(self.d['yaw_wheel_deg'] - self.first['yaw_wheel_deg'])
                dl = (wrap180(self.d['yaw_laser_deg'] - self.first['yaw_laser_deg'])
                      if self.d.get('yaw_laser_deg') is not None else 0.0)
                print('%6.2f   %+9.4f      %8.2f  %8s  %6.2f  %+7.2f  %+7.2f'
                      % (el, self.d['gyro_z_rad_s'], self.d['yaw_wheel_deg'],
                         '%.2f' % self.d['yaw_laser_deg'] if self.d.get('yaw_laser_deg') is not None else 'NA',
                         self.d['yaw_fused_deg'], dw, dl))
            time.sleep(0.02)

        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.03)

        # 停止后 3s，看磁航向是否还在"追赶"
        print()
        print('--- 停止后 ---')
        t_end = time.time() + 3.0
        next_log = 0.0
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            el = 3.0 - (t_end - time.time())
            if el >= next_log and self.d is not None:
                next_log += 0.6
                dw = wrap180(self.d['yaw_wheel_deg'] - self.first['yaw_wheel_deg'])
                dl = (wrap180(self.d['yaw_laser_deg'] - self.first['yaw_laser_deg'])
                      if self.d.get('yaw_laser_deg') is not None else 0.0)
                print('%6.2f   %+9.4f      %8.2f  %8.2f  %6.2f  %+7.2f  %+7.2f'
                      % (el, self.d['gyro_z_rad_s'], self.d['yaw_wheel_deg'],
                         self.d['yaw_laser_deg'] if self.d.get('yaw_laser_deg') is not None else 0.0,
                         self.d['yaw_fused_deg'], dw, dl))


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    rclpy.init()
    node = SignTest()
    try:
        node.run(wz, dur)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
