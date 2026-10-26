#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mag_probe.py —— 在指定位置采集静置磁场（不动机器人）

为什么需要它：
  serial_bridge 起来后会独占 /dev/ttyS1，不能再直接开串口读 $T 帧。
  但它同时把磁力计原始值转发到 /imu/imu407/mag（未校准 µT），
  所以测磁场可以改走 ROS 话题，不用停串口桥。

用途（走廊验证的核心）：
  在 A 点采一次、把车推到 B 点再采一次，比较 |B| 与方向。
  - 如果 A/B 两点磁场矢量几乎相同 → 走廊磁场均匀，之前 21µT/m 是那个房间特有的
  - 如果两点差很多 → 环境梯度是普遍现象，磁航向在平移中一律不可信

用法：
  python3 mag_probe.py --dur 6 --label A
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagProbe(Node):
    def __init__(self):
        super().__init__('mag_probe')
        self.samples = []
        self.sub = self.create_subscription(
            MagneticField, '/imu/imu407/mag', self._cb, 10)

    def _cb(self, msg):
        self.samples.append((msg.magnetic_field.x,
                             msg.magnetic_field.y,
                             msg.magnetic_field.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dur', type=float, default=6.0, help='采集时长(s)')
    ap.add_argument('--label', default='', help='位置标签, 例如 A / B')
    a = ap.parse_args()

    rclpy.init()
    node = MagProbe()

    print("采集磁场 %.1fs（不要碰车、不要拿金属靠近）..." % a.dur)

    import time
    t_end = time.time() + a.dur
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    s = node.samples
    if len(s) < 20:
        print("只收到 %d 帧，话题可能没起来（检查 serial_bridge 是否在跑）" % len(s))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    n = len(s)
    mx = sum(v[0] for v in s) / n
    my = sum(v[1] for v in s) / n
    mz = sum(v[2] for v in s) / n

    # 串口桥单位修正（2026-10-22 实测确认）：
    #   serial_bridge.py:300 'mag_ut100' 本身已是 µT×100，但 :351 又按 µT 乘了
    #   MAG_LSB_PER_UT 再乘回来 —— 巧合地多乘了 10000 倍。
    #   判据：正常地磁 |B| ≈ 40~60 µT，读数若 > 500 就是被放大了 10000 倍。
    scale = 1.0
    if abs(mx) + abs(my) + abs(mz) > 500.0:
        scale = 1e-4
    mx, my, mz = mx * scale, my * scale, mz * scale
    s = [(v[0] * scale, v[1] * scale, v[2] * scale) for v in s]

    norm = math.sqrt(mx * mx + my * my + mz * mz)
    hmag = math.hypot(mx, my)
    hdir = math.degrees(math.atan2(my, mx)) % 360.0

    sx = math.sqrt(sum((v[0] - mx) ** 2 for v in s) / n)
    sy = math.sqrt(sum((v[1] - my) ** 2 for v in s) / n)
    sz = math.sqrt(sum((v[2] - mz) ** 2 for v in s) / n)

    tag = "[%s] " % a.label if a.label else ""
    print("\n" + "=" * 58)
    print("%s磁场静置平均值（%d 帧 / %.1fs）" % (tag, n, a.dur))
    print("=" * 58)
    print("  mx = %+8.3f uT   my = %+8.3f uT   mz = %+8.3f uT" % (mx, my, mz))
    print("  |B|  = %7.3f uT   水平分量 = %7.3f uT" % (norm, hmag))
    print("  水平方向 = %6.2f°（atan2(my,mx)，仅作相对比较）" % hdir)
    print("  噪声 std:  x %.3f  y %.3f  z %.3f uT" % (sx, sy, sz))
    print("\n  请把下面这行抄下来做对比：")
    print("  %s mx=%+.3f my=%+.3f mz=%+.3f |B|=%.3f H=%.3f dir=%.2f"
          % (a.label or '?', mx, my, mz, norm, hmag, hdir))
    print("=" * 58)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
