#!/usr/bin/env python3
"""决定性实验：固定时间转动，用雷达几何真值判定 wheel / gyro 谁错。

三路对比
--------
- 几何(雷达点云)  ：绝对真值，不经任何里程计
- gyro_only       ：陀螺原始角速度逐帧积分（无磁修正）
- yaw_wheel       ：AHRS 输出 = 陀螺 + 磁 2% 互补修正

判读
----
- 几何 ≈ gyro  → wheel 错，磁在污染航向
- 几何 ≈ wheel → gyro 错，陀螺积分/零偏不准
- 三者都不同   → 各有问题

用法：python3 transient_geo.py -0.3 5
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

BE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)


def wrap(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def geom_rotation(ra, rb, inc, search_deg=150.0):
    """角度互相关求 A→B 的旋转量（度），优先用远处点。"""
    spf = max(1, int(round(search_deg / math.degrees(inc))))
    scores = {}
    for s in range(-spf, spf + 1):
        rs = np.roll(rb, s)
        valid = (np.isfinite(ra) & np.isfinite(rs)
                 & (ra > 0.40) & (ra < 8.0) & (rs > 0.40) & (rs < 8.0))
        far = valid & (ra > 1.20)
        m = far if int(far.sum()) >= 80 else valid
        if int(m.sum()) < 60:
            continue
        scores[s] = float(np.median(np.abs(ra[m] - rs[m])))
    if not scores:
        return None, None, None
    s0 = min(scores, key=scores.get)
    guard = max(1, int(round(30.0 / math.degrees(inc))))
    others = [scores[k] for k in scores if abs(k - s0) > guard]
    second = min(others) if others else float('nan')
    d = 0.0
    if (s0 - 1) in scores and (s0 + 1) in scores:
        y0, y1, y2 = scores[s0 - 1], scores[s0], scores[s0 + 1]
        den = y0 - 2.0 * y1 + y2
        if abs(den) > 1e-12:
            d = max(-0.5, min(0.5, 0.5 * (y0 - y2) / den))
    margin = (second - scores[s0]) / scores[s0] * 100.0 if scores[s0] > 1e-9 else float('nan')
    return (s0 + d) * math.degrees(inc), scores[s0], margin


class G(Node):
    def __init__(self):
        super().__init__('transient_geo')
        self.buf = []
        self.d = None
        self.inc = None
        self.frames = []
        self.create_subscription(LaserScan, '/scan', self.scan_cb, BE)
        self.create_subscription(String, '/fusion_diag', self.diag_cb, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_cb(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        r[~np.isfinite(r)] = np.nan
        self.inc = m.angle_increment
        self.buf.append(r)
        if len(self.buf) > 60:
            self.buf.pop(0)

    def diag_cb(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        self.d = d
        self.frames.append(d)

    def spin_for(self, dur, wz=None):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < dur:
            if wz is not None:
                tw = Twist()
                tw.angular.z = float(wz)
                self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        for _ in range(8):
            tw = Twist()
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)

    def still_scan(self, dur=3.0):
        self.buf = []
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < dur + 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 >= dur and len(self.buf) >= 8:
                break
        if len(self.buf) < 8:
            return None
        return np.nanmedian(np.vstack(self.buf), axis=0)


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else -0.3
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    rclpy.init()
    g = G()
    try:
        t0 = time.time()
        while rclpy.ok() and g.d is None and time.time() - t0 < 20:
            rclpy.spin_once(g, timeout_sec=0.1)
        if g.d is None:
            print('收不到 /fusion_diag')
            return

        print('=== 固定时间转动：几何 / 陀螺 / wheel 三路判定 ===')
        print('wz=%+.2f rad/s，持续 %.1fs' % (wz, dur))
        print()

        A = g.still_scan(3.0)
        if A is None:
            print('采不到 /scan')
            return
        w0 = g.d['yaw_wheel_deg']
        f0 = len(g.frames)

        g.spin_for(dur, wz)
        g.spin_for(0.5)
        g.stop()
        g.spin_for(2.0)
        B = g.still_scan(3.0)
        if B is None:
            print('采不到停止后 /scan')
            return
        w1 = g.d['yaw_wheel_deg']

        # 陀螺积分（只用这批帧）
        seg = g.frames[f0:]
        gyro = 0.0
        for i in range(1, len(seg)):
            dt = seg[i].get('t', 0) - seg[i - 1].get('t', 0)
            if dt <= 0 or dt > 0.5:
                continue
            gyro += math.degrees(seg[i]['gyro_z_rad_s']) * dt

        dg, resid, margin = geom_rotation(A, B, g.inc)
        dw = wrap(w1 - w0)

        print('  几何(雷达点云，真值)  %+8.2f°   （残差 %.3f m，主峰裕度 %.0f%%）'
              % (dg, resid, margin))
        print('  gyro_only(纯陀螺积分) %+8.2f°   与真值差 %+7.2f°' % (gyro, gyro - dg))
        print('  yaw_wheel(AHRS+磁)    %+8.2f°   与真值差 %+7.2f°' % (dw, dw - dg))
        print()
        print('  磁力计累计修正量(wheel−gyro)  %+8.2f°' % (dw - gyro))
        print()
        # 判定
        eg = abs(gyro - dg)
        ew = abs(dw - dg)
        if eg < 1.5 and ew < 1.5:
            print('  → 三路一致，本次没有明显偏差')
        elif ew < eg:
            print('  → gyro 更偏（差 %.2f°），wheel 更准（差 %.2f°）⇒ 陀螺积分/零偏问题' % (eg, ew))
        else:
            print('  → **wheel 更偏（差 %.2f°），gyro 更准（差 %.2f°）⇒ 磁力计在污染航向**'
                  % (ew, eg))
        print()
        print('  终值 diag：yaw_wheel=%.2f  yaw_laser=%.2f  yaw_fused=%.2f'
              % (g.d['yaw_wheel_deg'], g.d['yaw_laser_deg'], g.d['yaw_fused_deg']))
    finally:
        try:
            g.stop()
        except Exception:
            pass
        g.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
