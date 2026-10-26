#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光位移测试 —— 用 /scan 测车前障碍物距离变化，得到与里程计无关的真实位移。

原理：
  车前放一个固定参照物（盒子/墙）。车后退或前进 Δ 米后，
  激光到该参照物的距离会变化 Δ（前提：车沿"激光->参照物"连线运动）。
  这跟编码器、轮径、减速比统统无关，是唯一能定性"码盘系数对不对"的裁判。

用法：
  python3 laser_line.py --probe                    # 只看前方扇区距离，确认参照物在正前方
  python3 laser_line.py --cmd "BACK,0.1" --dur 3   # 测一次位移
"""
import argparse
import time
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial")


class ScanProbe(Node):
    def __init__(self):
        super().__init__("laser_line")
        # /scan 是 BEST_EFFORT，用默认 RELIABLE 会一帧都收不到
        qos = QoSProfile(depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos)
        self.last = None

    def cb(self, msg):
        self.last = msg


def wait_frame(node, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.08)
        if node.last is not None:
            m = node.last
            node.last = None
            return m
    return None


def sector_min(msg, center_deg, half_deg):
    """取以 center_deg 为中心、±half_deg 扇区内有效距离的最小值。"""
    if msg is None:
        return None
    vals = []
    inc = msg.angle_increment
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            continue
        a = msg.angle_min + i * inc
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        ad = math.degrees(a)
        # 角度差按 ±180 回绕，否则 center=180 时 ad=-180 会被判成差 360° 而漏掉
        dd = (ad - center_deg + 180.0) % 360.0 - 180.0
        if abs(dd) <= half_deg:
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 4]          # 取 25 分位，抗噪又不被最近点带偏


def measure(node, center_deg, half_deg, n=8):
    """连续取 n 帧，返回中位数。"""
    out = []
    for _ in range(n):
        m = wait_frame(node, timeout=3.0)
        v = sector_min(m, center_deg, half_deg)
        if v is not None:
            out.append(v)
    if not out:
        return None
    out.sort()
    return out[len(out) // 2]


def send(ser, cmd):
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--cmd", default=None, help="要发的底盘命令，如 BACK,0.1")
    ap.add_argument("--dur", type=float, default=3.0)
    ap.add_argument("--center", type=float, default=0.0, help="参照物所在角度(度)，0=车头正前")
    ap.add_argument("--half", type=float, default=6.0, help="扇区半宽(度)")
    ap.add_argument("--probe", action="store_true", help="只打印近处物体方向")
    ap.add_argument("--probe-max", type=float, default=1.5, help="probe 时只列小于该距离的方向(m)")
    ap.add_argument("--stable", type=int, default=0, help="稳定性测试：不动车，每个方向连续测 N 次待输入")
    ap.add_argument("--stable-n", type=int, default=0)
    a = ap.parse_args()

    rclpy.init()
    node = ScanProbe()

    if a.stable:
        print("稳定性测试：车不动，每个方向连续测 %d 次（极差越小越适合当参照）" % a.stable)
        print("  角度     最小     中位     最大     极差")
        best = []
        base = int(round(a.center))
        hw = int(round(a.half))
        for c in range(base - hw, base + hw + 1, 2):
            vals = []
            for _ in range(a.stable):
                v = measure(node, c, 1.0, n=1)
                if v is not None:
                    vals.append(v)
            if len(vals) >= 3:
                vals.sort()
                spread = vals[-1] - vals[0]
                flag = "  <== 可用" if spread < 0.03 else ""
                print("  %+4d°  %.3f   %.3f   %.3f   %.3f%s"
                      % (c, vals[0], vals[len(vals) // 2], vals[-1], spread, flag))
                best.append((spread, c))
        node.destroy_node()
        rclpy.shutdown()
        return

    if a.probe:
        print("近处物体方向（全向扫描，只列出 < %.1f m 的方向）：" % a.probe_max)
        groups = []
        for c in range(-180, 180, 3):
            d = measure(node, c, 1.5, n=3)
            if d is not None and d < a.probe_max:
                groups.append((c, d))
        if not groups:
            print("  %0.1f m 内没有任何回波" % a.probe_max)
        else:
            # 把相邻角度合并成簇，每簇只报最近的那个
            cur = [groups[0]]
            for g in groups[1:]:
                if g[0] - cur[-1][0] <= 3:
                    cur.append(g)
                else:
                    best = min(cur, key=lambda x: x[1])
                    print("  %+4d° ~ %+4d°  最近 %.3f m (在 %+d°)"
                          % (cur[0][0], cur[-1][0], best[1], best[0]))
                    cur = [g]
            best = min(cur, key=lambda x: x[1])
            print("  %+4d° ~ %+4d°  最近 %.3f m (在 %+d°)"
                  % (cur[0][0], cur[-1][0], best[1], best[0]))
        node.destroy_node()
        rclpy.shutdown()
        return

    if not a.cmd:
        print("需要 --cmd 或 --probe")
        return

    # --- 起始距离 ---
    d0 = measure(node, a.center, a.half, n=10)
    if d0 is None:
        print("!! 前方 %.0f°±%.0f° 没有回波，无法测量。先跑 --probe 看看参照物在哪。" % (a.center, a.half))
        node.destroy_node()
        rclpy.shutdown()
        return
    print("起始距离 d0 = %.4f m" % d0)

    # --- 发命令 ---
    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()
    send(ser, "TELEM ON")
    time.sleep(0.2)
    base = None
    print(">>> 发送 %s，持续 %.1fs" % (a.cmd, a.dur))
    send(ser, a.cmd)
    t0 = time.time()

    # 走动过程中顺便抓 odom，用来和激光真值对比
    buf = b""
    odom_first = odom_last = None
    while time.time() - t0 < a.dur:
        try:
            chunk = ser.read(4096)
        except Exception:
            break
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"$T,"):
                continue
            try:
                f = line.split(b",")
                if len(f) < 25:
                    continue
                rec = (int(f[15]) / 1000.0, int(f[16]) / 1000.0, int(f[17]) / 10.0,
                       int(f[18]) / 1000.0)
                if odom_first is None:
                    odom_first = rec
                odom_last = rec
            except (ValueError, IndexError):
                pass

    send(ser, "STOP")
    ser.close()

    # --- 等车彻底停稳再测 ---
    time.sleep(1.2)
    d1 = measure(node, a.center, a.half, n=10)
    if d1 is None:
        print("!! 终止距离测不到（可能车走偏、参照物离开扇区）")
        node.destroy_node()
        rclpy.shutdown()
        return

    delta_laser = d1 - d0
    print("\n终止距离 d1 = %.4f m" % d1)
    print("【激光位移】d1-d0 = %+.4f m  (%.1f cm)" % (delta_laser, delta_laser * 100))

    if odom_first and odom_last:
        od_x = odom_last[0] - odom_first[0]
        od_y = odom_last[1] - odom_first[1]
        od_d = math.hypot(od_x, od_y)
        print("【里程计】  Δx=%+.3f Δy=%+.3f |Δ|=%.4f m  (%.1f cm)"
              % (od_x, od_y, od_d, od_d * 100))
        if abs(delta_laser) > 1e-3:
            ratio = od_d / abs(delta_laser)
            print("\n里程计/激光 = %.3f  ->  %s"
                  % (ratio, "里程计偏小（码盘系数需调大）" if ratio < 0.95
                     else ("里程计偏大（码盘系数需调小）" if ratio > 1.05
                           else "两者一致，里程计距离可信")))
    print()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
