#!/usr/bin/env python3
"""闭环几何校验：用雷达原始点云当裁判，测「转 360° 后车身真实朝向偏了多少」。

为什么需要它
------------
磁航向(wheel)、rf2o(laser) 都是积分/传感器推出来的，它们自己可能一起偏。
人眼估角有 ±5° 主观误差。唯一不经过任何里程计的独立量是 ——
**同一物理场景在车体系扫描线上的角度平移量**。

原理
----
车身逆时针转 Δ，则一个世界固定点相对车体的角度 θ 会减小 Δ：
    ra(θ) = rb(θ - Δ)
在等角度网格上离散化，令 s = Δ / inc，则 rb[i + (-s)] ≈ ra[i]，
即 np.roll(rb, s)[i] = rb[i-s] 与 ra 对齐 → 取残差最小者，Δ = s·inc。

用法
----
    python3 loop_scan_test.py [转速rad/s] [圈数]

每轮流程：静止采 A → 用 yaw_wheel 判满 360° 停 → 停止稳定 2s → 静止采 B，
然后给出 Δ_geo、Δ_wheel、Δ_laser 三个数。
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

# 发布端 /scan 是 BEST_EFFORT，订阅必须匹配
BE = QoSProfile(depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST)


def wrap(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class LoopTester(Node):
    def __init__(self):
        super().__init__('loop_scan_tester')
        self.d = None
        self.buf = []
        self.frame = None
        self.create_subscription(LaserScan, '/scan', self.scan_cb, BE)
        self.create_subscription(String, '/fusion_diag', self.diag_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # ---------- 回调 ----------
    def scan_cb(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        # inf/nan 统一成 nan，便于后续 isfinite 过滤
        r[~np.isfinite(r)] = np.nan
        self.frame = (r, m.angle_min, m.angle_increment)
        self.buf.append(r)
        if len(self.buf) > 60:
            self.buf.pop(0)

    def diag_cb(self, m):
        try:
            self.d = json.loads(m.data)
        except Exception:
            pass

    # ---------- 工具 ----------
    def spin_for(self, dur):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_diag(self, timeout=20.0):
        t0 = time.time()
        while rclpy.ok() and self.d is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.d is not None

    def send(self, wz):
        t = Twist()
        t.angular.z = float(wz)
        self.pub.publish(t)

    def collect_still(self, dur=3.0, need=8):
        """静止采集若干帧，逐角度取中位数（抗瞬态遮挡/噪声）。"""
        self.buf = []
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < dur + 6.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 >= dur and len(self.buf) >= need:
                break
        if len(self.buf) < need:
            return None
        arr = np.vstack(self.buf)
        return np.nanmedian(arr, axis=0)

    # ---------- 核心：几何转角 ----------
    @staticmethod
    def geom_rotation(ra, rb, inc, search_deg=200.0):
        n = len(ra)
        spf = max(1, int(round(search_deg / math.degrees(inc))))
        scores = {}
        for s in range(-spf, spf + 1):
            rs = np.roll(rb, s)
            valid = (np.isfinite(ra) & np.isfinite(rs)
                     & (ra > 0.40) & (ra < 8.0)
                     & (rs > 0.40) & (rs < 8.0))
            # 优先用远处点：远处特征对车身平移不敏感，只对旋转敏感
            far = valid & (ra > 1.20)
            m = far if int(far.sum()) >= 80 else valid
            if int(m.sum()) < 60:
                continue
            scores[s] = float(np.median(np.abs(ra[m] - rs[m])))
        if not scores:
            return None, None, 0
        s0 = min(scores, key=scores.get)
        # 主峰显著性：排除 ±15° 邻域后的次优峰，两者差距越大越可信
        # （场地若近似旋转对称，会出现多个几乎等高的峰 → 结果不可信）
        guard = max(1, int(round(30.0 / math.degrees(inc))))
        others = [v for kk, v in scores.items() if abs(kk - s0) > guard]
        second = min(others) if others else float('nan')
        margin = (second - scores[s0]) / scores[s0] * 100.0 if scores[s0] > 1e-9 else float('nan')
        LoopTester.last_margin = margin
        # 抛物线插值把分辨率从 0.5° 细化到 ~0.1°
        if (s0 - 1) in scores and (s0 + 1) in scores:
            y0, y1, y2 = scores[s0 - 1], scores[s0], scores[s0 + 1]
            den = y0 - 2.0 * y1 + y2
            if abs(den) > 1e-12:
                d = 0.5 * (y0 - y2) / den
                d = max(-0.5, min(0.5, d))
            else:
                d = 0.0
        else:
            d = 0.0
        return (s0 + d) * math.degrees(inc), scores[s0], int(m.sum())

    # ---------- 转 N 圈 ----------
    def spin_turns(self, wz, laps, tol=2.0):
        """按 yaw_wheel 累计角判满 laps×360° 后停。中间不停车，只在最后停一次，
        这样停车惯性过冲只出现一次，不会污染每轮的圈数比例误差。
        返回 (wheel 累计转角, 耗时)。"""
        if self.d is None:
            return None, 0.0
        target = laps * 360.0
        last = self.d['yaw_wheel_deg']
        total = 0.0
        t0 = time.time()
        while rclpy.ok():
            self.send(wz)
            rclpy.spin_once(self, timeout_sec=0.03)
            if self.d is None:
                continue
            cur = self.d['yaw_wheel_deg']
            total += wrap(cur - last)
            last = cur
            if time.time() - t0 > 60.0 * laps + 40.0:
                break
            if abs(total) >= target - tol:
                break
        # 停 + 稳住
        for _ in range(6):
            self.send(0.0)
            self.spin_for(0.1)
        self.spin_for(1.8)
        return total, time.time() - t0


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    laps = int(sys.argv[2]) if len(sys.argv) > 2 else 1      # 每轮连续转几圈
    turns = int(sys.argv[3]) if len(sys.argv) > 3 else 1     # 跑几轮
    if wz < 0:
        wz = abs(wz)
    laps = max(1, laps)

    rclpy.init()
    t = LoopTester()
    try:
        if not t.wait_diag():
            print('收不到 /fusion_diag，放弃')
            return

        print('=== 闭环几何校验（雷达点云当裁判）===')
        print('转速 wz=+%.2f rad/s，每轮连转 %d 圈不停车，共 %d 轮' % (wz, laps, turns))
        print()

        A = t.collect_still(3.0)
        if A is None or t.frame is None:
            print('采不到 /scan，放弃')
            return
        inc = t.frame[2]
        base = dict(t.d)
        print('起始：yaw_wheel=%.2f  yaw_laser=%.2f  | 有效扫描点 %d/%d'
              % (base['yaw_wheel_deg'], base['yaw_laser_deg'],
                 int(np.isfinite(A).sum()), len(A)))
        print()
        print(' 轮  几何Δ°   wheelΔ°   laserΔ°   几何累计   wheel累计  laser累计  残差m  主峰裕度%')
        print('-' * 90)

        round_target = laps * 360.0
        cum_geo = cum_w = cum_l = cum_f = 0.0
        prev_w = base['yaw_wheel_deg']
        prev_l = base['yaw_laser_deg']
        prev_f = base['yaw_fused_deg']
        prev_scan = A

        for k in range(1, turns + 1):
            # 奇数轮正转、偶数轮反转 → 两轮构成一次"往返闭环"，直接复现
            # "转过去再转回来，车停的角度与最初差多少"这个现象
            wz_eff = wz if (k % 2 == 1) else -wz
            sign = 1.0 if (k % 2 == 1) else -1.0
            spin_total, dt = t.spin_turns(wz_eff, laps)
            if spin_total is None:
                print('  第 %d 轮判圈失败' % k)
                break
            B = t.collect_still(3.0)
            if B is None:
                print('  第 %d 轮采不到停止后扫描' % k)
                break

            dg, resid, npts = t.geom_rotation(prev_scan, B, inc)
            dw = wrap(t.d['yaw_wheel_deg'] - prev_w)
            dl = wrap(t.d['yaw_laser_deg'] - prev_l)
            df = wrap(t.d['yaw_fused_deg'] - prev_f)
            # 每轮转了整整 laps 圈 → 真值 = ±laps×360 + 残差(wrap 后的 ±小量)
            rnd = sign * round_target
            if dg is not None:
                dg = rnd + dg
                cum_geo += dg
            dw = rnd + dw
            dl = rnd + dl
            cum_w += dw
            cum_l += dl
            cum_f += rnd + df

            print('  %d%s %8.2f  %8.2f  %8.2f   %9.2f  %9.2f  %9.2f  %6.3f  %8.1f'
                  % (k, '+' if sign > 0 else '-',
                     dg if dg is not None else float('nan'), dw, dl,
                     cum_geo, cum_w, cum_l, resid if resid else 0.0,
                     getattr(LoopTester, 'last_margin', float('nan'))))

            prev_w = t.d['yaw_wheel_deg']
            prev_l = t.d['yaw_laser_deg']
            prev_f = t.d['yaw_fused_deg']
            prev_scan = B

        print()
        total_laps = turns * laps
        pingpong = (turns % 2 == 0)
        print('=== 净变化（%d 轮 × %d 圈，%s）==='
              % (turns, laps,
                 '正反往返闭环，理想应为 0°' if pingpong
                 else '单向，理想 %+.0f°' % round_target))
        print('  几何(雷达点云)  %+9.2f°   ← 车身真实朝向净变化（真值）' % cum_geo)
        print('  wheel(磁+陀螺)  %+9.2f°   与真值差 %+7.2f°' % (cum_w, cum_w - cum_geo))
        print('  laser(rf2o)     %+9.2f°   与真值差 %+7.2f°' % (cum_l, cum_l - cum_geo))
        print('  fused(/odom_fused) %+7.2f°   与真值差 %+7.2f°' % (cum_f, cum_f - cum_geo))
        if pingpong:
            print()
            print('  闭环残留（真值）：%+.2f°   ← 这就是"车停的角度与最初差多少"' % cum_geo)
    finally:
        try:
            t.send(0.0)
        except Exception:
            pass
        t.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
