#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spin_test2.py — 原地转一圈，同时记录 磁航向 / 激光航向 / 融合航向 三路累计

目的：判断"旋转过程中融合航向偏离磁航向"到底该归咎于谁——
      若激光累计明显大于磁累计，说明 rf2o 在旋转时高估转角，
      那么 w_laser=0.7 就是在把激光的旋转误差按 70% 放大。

用法：python3 spin_test2.py [目标角度=360] [角速度rad/s=0.5]
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
        super().__init__('spin_test2')
        self.target = target_deg
        self.wz = wz
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/fusion_diag', self.cb, 10)

        self.acc_wheel = 0.0
        self.acc_laser = 0.0
        self.acc_fused = 0.0
        self.last = {'w': None, 'l': None, 'f': None}
        self.start = {}
        self.samples = []
        self.frames = 0

    def cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception as e:
            print('解析失败:', e)
            return
        cur = {'w': d['yaw_wheel_deg'],
               'l': d.get('yaw_laser_deg'),
               'f': d['yaw_fused_deg']}
        for k in ('w', 'l', 'f'):
            if cur[k] is None:
                continue
            if self.last[k] is None:
                self.start[k] = cur[k]
                self.last[k] = cur[k]
                continue
            delta = wrap180(cur[k] - self.last[k])
            self.last[k] = cur[k]
            if k == 'w':
                self.acc_wheel += delta
            elif k == 'l':
                self.acc_laser += delta
            else:
                self.acc_fused += delta
        self.frames += 1
        self.samples.append((time.time(),
                             self.acc_wheel, self.acc_laser, self.acc_fused,
                             cur['w'], cur['l'], cur['f']))

    def run(self):
        cmd = Twist()
        cmd.angular.z = self.wz

        # 等 DDS 发现完成并抓静置基线（最多 10s：rmw 首次发现常要 1~3s，
        # 早先写死 1s 会误判"收不到"而直接放弃）
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last['w'] is not None and self.frames >= 20:
                break
        if self.last['w'] is None:
            print('收不到 /fusion_diag，放弃（frames=%d）' % self.frames)
            return
        # 再静置 0.5s 让三路基准稳定
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 0.5:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.samples = []   # 清掉静置期样本，只统计旋转段
        self.acc_wheel = self.acc_laser = self.acc_fused = 0.0

        t_start = time.time()
        while rclpy.ok() and time.time() - t_start < 60.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if abs(self.acc_wheel) >= abs(self.target):
                break
            self.pub.publish(cmd)
            time.sleep(0.05)

        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.03)
        t_stop = time.time()

        # 停后再收 3s，看静止锚收敛过程
        t_end = time.time() + 3.0
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        dur = t_stop - t_start
        print()
        print('=== 旋转段 %.2f s，目标 %.0f°，wz=%.2f rad/s ===' % (dur, self.target, self.wz))
        print('磁航向累计   : %+8.2f °' % self.acc_wheel)
        print('激光累计     : %+8.2f °  (相对磁: %+7.2f °)' % (self.acc_laser, self.acc_laser - self.acc_wheel))
        print('融合累计     : %+8.2f °  (相对磁: %+7.2f °)' % (self.acc_fused, self.acc_fused - self.acc_wheel))
        if abs(self.acc_wheel) > 1:
            print('激光/磁 比例 : %.4f' % (self.acc_laser / self.acc_wheel))

        # 每 2 秒打一行轨迹
        print()
        print('  时刻  磁累计   激光累计  融合累计  激光-磁  融合-磁')
        print('-' * 60)
        last_t = -99.0
        for (t, aw, al, af, _, _, _) in self.samples:
            dt = t - t_start
            if dt < 0 or dt - last_t < 2.0:
                continue
            last_t = dt
            print('%6.1f %8.2f %9.2f %9.2f %8.2f %8.2f'
                  % (dt, aw, al, af, al - aw, af - aw))

        # 停后收敛检查
        if self.samples:
            _, aw, al, af, yh, yl, yf = self.samples[-1]
            print()
            print('停 3s 后：yaw_wheel=%.2f  yaw_laser=%.2f  yaw_fused=%.2f  (融合-磁 %+.2f°)'
                  % (yh, yl, yf, wrap180(yf - yh)))


def main():
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
    wz = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    rclpy.init()
    node = SpinTest(target, wz)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
