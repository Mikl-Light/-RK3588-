#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""点云平移匹配测位移 —— 用整帧 /scan 做 2D 互相关，算出车的真实平移量。

为什么不用单方向测距：
  小房间里四面都是东西，某个方向上可能是斜面、可能是几个物体边缘重叠。
  车一动，扇区里换了一批不同距离的点，读数就失去物理意义。
  而整帧点云匹配只看"整片点云整体平移了多少"，不挑方向，也不怕斜面。

方向约定：
  车前进 d 后，车上雷达看到的场景相对车体后退了 d。
  作用在点云上就是：帧 B 的点整体要向 +x 平移 d 才能和帧 A 重合。
  所以匹配结果 dx > 0 表示车前进，dx < 0 表示车后退。dy 为横向偏移。

用法：
  python3 cloud_shift.py --cmd "V,0.1,0,0" --dur 2.0 --search 0.4
  python3 cloud_shift.py --still-check            # 不动车，验证匹配算法本身的分辨率
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

try:
    from scipy.spatial import cKDTree
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial")


class ScanNode(Node):
    def __init__(self):
        super().__init__("cloud_shift")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
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


def to_xy(msg, rmin=0.12, rmax=3.0):
    inc = msg.angle_increment
    pts = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < rmin or r > rmax:
            continue
        a = msg.angle_min + i * inc
        pts.append((r * math.cos(a), r * math.sin(a)))
    return np.array(pts) if pts else np.zeros((0, 2))


def gather(node, n=6):
    """连续取 n 帧合并（车静止时用于增强点云密度）"""
    acc = []
    for _ in range(n):
        m = wait_frame(node)
        if m is not None:
            p = to_xy(m)
            if len(p):
                acc.append(p)
    return np.vstack(acc) if acc else None


def thin(P, n=280):
    if len(P) <= n:
        return P
    return P[:: max(1, len(P) // n)][:n]


def match(A, B, max_d, step, axis_only=True, trim=0.85):
    """搜索平移量使 B 平移到 A。返回 (cost, dx, dy)。cost 为 trimmed 平均最近邻距离(m)。"""
    A = thin(np.asarray(A, float))
    B = thin(np.asarray(B, float))
    rng = np.arange(-max_d, max_d + 1e-9, step)
    if axis_only:
        cands = [(float(d), 0.0) for d in rng]
    else:
        cands = [(float(dx), float(dy)) for dx in rng[::2] for dy in rng[::2]]

    best = (1e9, 0.0, 0.0)
    for dx, dy in cands:
        B2 = B + np.array([dx, dy])
        d2 = ((A[:, None, :] - B2[None, :, :]) ** 2).sum(axis=2)
        nn = np.min(d2, axis=1)                 # 每个 A 点到最近 B2 点的平方距离
        k = max(1, int(len(nn) * trim))
        part = np.partition(nn, k - 1)[:k]      # 只用最好的 85%，抗离群点
        cost = math.sqrt(float(part.mean()))
        if cost < best[0]:
            best = (cost, dx, dy)
    return best


def match_rt(A, B, max_d, max_yaw_deg, dx_step=0.01, yaw_step=1.0, trim=0.85):
    """联合搜索 (dx, dyaw)：把 B 先绕激光原点旋转 dyaw，再沿 x 平移 dx，去对 A。

    为什么必须搜旋转：
      `match()` 只搜平移，前提是"车走得笔直"。
      但车只要偏 3°，0.3m 行程里点云就会被拧歪 3°，
      280 个点上平均每个偏 3~8mm，cost 从 10mm 涨到 60mm 以上，
      于是"匹配不可靠"——可其实位移是好的，只是模型少了自由度。

    附带价值：dyaw 是**不经过 IMU 的独立航向裁判**，
      可以直接回答"车到底有没有走直线、偏了多少"。

    返回 (cost, dx, dyaw_deg)。
    """
    A = thin(np.asarray(A, float))
    B = thin(np.asarray(B, float))
    tree = cKDTree(A) if _HAS_SCIPY else None

    def cost_of(dx, dyaw_deg):
        th = math.radians(dyaw_deg)
        c, s = math.cos(th), math.sin(th)
        B2 = B @ np.array([[c, -s], [s, c]]).T   # 绕原点旋转
        B2[:, 0] += dx
        if tree is not None:
            d, _ = tree.query(B2)
        else:
            d2 = ((A[:, None, :] - B2[None, :, :]) ** 2).sum(axis=2)
            d = np.sqrt(np.min(d2, axis=1))
        k = max(1, int(len(d) * trim))
        return float(np.partition(d, k - 1)[:k].mean())

    best = (1e9, 0.0, 0.0)
    for dyaw in np.arange(-max_yaw_deg, max_yaw_deg + 1e-9, yaw_step):
        for dx in np.arange(-max_d, max_d + 1e-9, dx_step):
            co = cost_of(float(dx), float(dyaw))
            if co < best[0]:
                best = (co, float(dx), float(dyaw))
    # 细化一轮：在粗解附近用更细的步长再找
    c0, dx0, y0 = best
    for dyaw in np.arange(y0 - yaw_step, y0 + yaw_step + 1e-9, yaw_step / 4.0):
        for dx in np.arange(dx0 - dx_step, dx0 + dx_step + 1e-9, dx_step / 5.0):
            co = cost_of(float(dx), float(dyaw))
            if co < best[0]:
                best = (co, float(dx), float(dyaw))
    return best


def send(ser, cmd):
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyS1")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--cmd", default=None)
    ap.add_argument("--dur", type=float, default=2.0)
    ap.add_argument("--search", type=float, default=0.40, help="平移搜索范围(m)")
    ap.add_argument("--step", type=float, default=0.005, help="搜索步长(m)")
    ap.add_argument("--settle", type=float, default=1.6, help="停车后等待静止的秒数")
    ap.add_argument("--still-check", action="store_true", help="不动车，验证算法分辨率")
    ap.add_argument("--yaw-search", type=float, default=0.0,
                    help="同时搜索偏航，单位度；如 8 表示搜 ±8°。0=只搜平移")
    a = ap.parse_args()

    rclpy.init()
    node = ScanNode()

    print("采集第 1 帧点云 ...")
    A = gather(node, n=8)
    if A is None or len(A) < 30:
        print("!! 点云太少（%d 点），检查雷达" % (0 if A is None else len(A)))
        return
    print("  帧 A: %d 点" % len(A))

    if a.still_check:
        A2 = gather(node, n=8)
        cost, dx, dy = match(A, A2, a.search, a.step)
        print("车未移动，匹配结果 dx=%+.3f dy=%+.3f  cost=%.4f m" % (dx, dy, cost))
        print("  -> cost 越小、dx 越接近 0，说明算法分辨率够用")
        node.destroy_node()
        rclpy.shutdown()
        return

    if not a.cmd:
        print("需要 --cmd 或 --still-check")
        return

    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()
    send(ser, "TELEM ON")
    time.sleep(0.2)
    ser.reset_input_buffer()

    print(">>> 发送 %s，持续 %.1fs" % (a.cmd, a.dur))
    stopped = False
    try:
        send(ser, a.cmd)
        t0 = time.time()
        buf = b""
        of = ol = None
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
                    rec = (int(f[15]) / 1000.0, int(f[16]) / 1000.0, int(f[17]) / 10.0)
                    if of is None:
                        of = rec
                    ol = rec
                except (ValueError, IndexError):
                    pass
    finally:
        # 无论正常结束、异常还是被 Ctrl-C，都尽量把 STOP 送出去。
        # 固件没有命令超时保护（V 是持续指令），不发 STOP 车会一直走。
        try:
            send(ser, "STOP")
            send(ser, "STOP")
            stopped = True
        finally:
            ser.close()
    if not stopped:
        print("!! 警告：STOP 未能确认发出，立刻手动断电车")

    print("等待车停稳 %.1fs ..." % a.settle)
    time.sleep(a.settle)
    for _ in range(5):                         # 丢弃停车过程中残留的帧（固定次数，别写成 while）
        wait_frame(node, timeout=0.25)

    B = gather(node, n=8)
    if B is None or len(B) < 30:
        print("!! 第 2 帧点云太少")
        return
    print("  帧 B: %d 点" % len(B))

    if a.yaw_search > 0:
        cost, dx, dyaw = match_rt(A, B, a.search, a.yaw_search,
                                  dx_step=max(a.step * 2.0, 0.01), yaw_step=1.0)
        print("\n【点云匹配·含旋转搜索】")
        print("  dx=%+.4f m   dyaw=%+.2f°   cost=%.4f m" % (dx, dyaw, cost))
        print("  纵向真实位移 = %.1f cm" % (abs(dx) * 100))
        print("  点云几何偏航  = %.2f°（不经过 IMU 的独立裁判：0 表示车走直了）" % dyaw)
    else:
        cost, dx, dy = match(A, B, a.search, a.step, axis_only=True)
        print("\n【点云匹配】dx=%+.4f m  dy=%+.4f m  cost=%.4f m" % (dx, dy, cost))
        print("  纵向真实位移 = %+.1f cm" % (dx * 100))

    if of and ol:
        ox = ol[0] - of[0]
        oy = ol[1] - of[1]
        od = math.hypot(ox, oy)
        print("【里程计】  |Δ| = %.1f cm" % (od * 100))
        if abs(dx) > 0.01:
            print("  里程计/点云 = %.3f" % (od / abs(dx)))
            err = (od - abs(dx)) / abs(dx) * 100.0
            print("  相对误差 = %+.1f%%" % err)
            if abs(od - abs(dx)) > 0.03:
                print("  -> 两者差 %.1f cm，里程计距离有系统偏差" % ((od - abs(dx)) * 100))
            else:
                print("  -> 两者一致（3cm 内），本次里程计距离可信")
    print("  （cost 若 > 0.06m 说明匹配不可靠，先跑 --still-check 看基准）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
