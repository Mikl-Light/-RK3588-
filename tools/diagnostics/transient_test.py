#!/usr/bin/env python3
"""起动/停止瞬态取证：定位「顺时针段固定 7.5° 偏差」发生在哪一拍。

思路
----
`yaw_wheel` = AHRS 输出（陀螺积分 + 磁力计 2% 互补修正）。
`gyro_z` 是陀螺原始角速度，自己积分得到 gyro_only。
两者之差 (yaw_wheel - gyro_only) 就是「磁力计累计修正了多少度」。

如果偏差来自电机漏磁瞬态污染磁航向，那么在起动瞬间会看到：
    gyro_only 平滑上升（真实转动）
    yaw_wheel 却突跳一段（磁修正猛地吃进 7.5°）
即 wheel−gyro 的差值在 0.1~0.5s 内阶跃。

如果两者全程贴合、偏差只出现在停止减速段，那就是机械/控制问题，不是磁。

用法
----
    python3 transient_test.py -0.3     # 顺时针（问题方向）
    python3 transient_test.py +0.3     # 逆时针（对照）
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def wrap(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class Transient(Node):
    def __init__(self):
        super().__init__('transient_test')
        self.frames = []          # [(t, dict)]
        self.d = None
        self.create_subscription(String, '/fusion_diag', self.cb, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb(self, m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        self.d = d
        self.frames.append((d.get('t', time.time()), d))

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
            tw.angular.z = 0.0
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else -0.3

    rclpy.init()
    n = Transient()
    try:
        t0 = time.time()
        while rclpy.ok() and n.d is None and time.time() - t0 < 20:
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.d is None:
            print('收不到 /fusion_diag')
            return

        print('=== 起动瞬态取证  wz=%+.2f rad/s ===' % wz)
        print('静止 2.5s → 起动 5s → 停止 3s（全程 50Hz 记录）')
        print()

        n.frames = []
        n.spin_for(2.5)                    # 静止基线
        t_start = time.time()
        n.spin_for(5.0, wz)                # 起动并维持
        t_stop = time.time()
        n.spin_for(0.6)
        n.stop()
        n.spin_for(2.4)                    # 停止观察

        if len(n.frames) < 50:
            print('帧数不足（%d）' % len(n.frames))
            return

        ts = [f[0] for f in n.frames]
        base = ts[0]
        gyro_only = 0.0
        prev_t = ts[0]
        fwd = []
        for t, d in n.frames:
            dt = max(0.0, t - prev_t)
            prev_t = t
            gyro_only += math.degrees(d['gyro_z_rad_s']) * dt
            fwd.append([t - base, t - t_start, t - t_stop,
                        d['yaw_wheel_deg'], gyro_only, d])

        # yaw_wheel 逐帧 unwrap，换成「相对首帧的累计转角」
        acc = 0.0
        last = fwd[0][3]
        for idx, r in enumerate(fwd):
            if idx > 0:
                acc += wrap(r[3] - last)
            last = r[3]
            r[3] = acc

        print('  时间(s)   阶段   yaw_wheel°   gyro_only°   差(磁修正)°   gyro_z(rad/s)')
        print('-' * 82)
        step = 0.10
        next_t = -0.60
        for r in fwd:
            if r[0] < next_t:
                continue
            if r[0] > 1.30:
                break
            ph = '静止' if r[1] < 0 else '起动'
            print('  %+7.2f  %4s  %10.2f  %11.2f  %12.3f  %12.4f'
                  % (r[0], ph, r[3], r[4], r[3] - r[4], r[5]['gyro_z_rad_s']))
            next_t += step

        print()
        print('  停止前后：')
        next_t = -0.40
        for r in fwd:
            if r[2] < next_t:
                continue
            if r[2] > 1.00:
                break
            print('  %+7.2f  %4s  %10.2f  %11.2f  %12.3f  %12.4f'
                  % (r[2], '停车', r[3], r[4], r[3] - r[4], r[5]['gyro_z_rad_s']))
            next_t += 0.10

        last = fwd[-1]
        print()
        print('=== 全程汇总（%d 帧）===' % len(fwd))
        print('  yaw_wheel 净变化   %+8.2f°' % last[3])
        print('  gyro_only 净变化   %+8.2f°' % last[4])
        print('  磁力计累计修正量    %+8.2f°   ← 差值就是磁把航向拽走的总量'
              % (last[3] - last[4]))

        # 导出逐帧 CSV 供离线分析
        out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/transient.csv'
        with open(out, 'w') as fp:
            fp.write('t_rel,t_from_start,t_from_stop,yaw_wheel_acc,gyro_acc,'
                     'yaw_wheel_raw,gyro_z,dyaw_wheel_deg_s,dyaw_laser_deg_s,'
                     'laser_rate_deg_s,still,mag_outlier,bias_b_deg_s\n')
            for r in fwd:
                d = r[5]
                fp.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%.4f,%s,%s,%.5f\n'
                         % (r[0], r[1], r[2], r[3], r[4], d['yaw_wheel_deg'],
                            d['gyro_z_rad_s'], d['dyaw_wheel_deg_s'],
                            d['dyaw_laser_deg_s'], d.get('laser_rate_deg_s', 0.0),
                            int(bool(d.get('still'))), int(bool(d.get('mag_outlier'))),
                            d.get('bias_b_deg_s', 0.0)))
        print('  逐帧数据已写入 %s' % out)

        # 找磁修正量变化最剧烈的 0.25s 窗口
        best = None
        for i in range(len(fwd)):
            t_i = fwd[i][0]
            j = i
            while j < len(fwd) and fwd[j][0] - t_i <= 0.25:
                j += 1
            if j >= len(fwd):
                break
            d_corr = (fwd[j][3] - fwd[j][4]) - (fwd[i][3] - fwd[i][4])
            if best is None or abs(d_corr) > abs(best[0]):
                best = (d_corr, fwd[i][1], fwd[j][1])   # 用「相对起动」的时间
        if best:
            print()
            print('  磁修正最剧烈窗口：起动后 %+.2f~%+.2f s，修正量 %+.2f°（0.25s 内）'
                  % (best[1], best[2], best[0]))
            if best[1] < 0.6:
                print('  → 落在【起动瞬间】 ⇒ 支持「电机漏磁瞬态污染磁航向」')
            elif best[1] > 4.4:
                print('  → 落在【停止瞬间】 ⇒ 更可能是减速/停车过程')
            else:
                print('  → 落在【匀速转动中】 ⇒ 是持续性的磁速率误差，不是瞬态')
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
