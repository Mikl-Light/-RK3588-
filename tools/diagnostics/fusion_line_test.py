#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直线行驶时三路航向对比 —— 融合输出到底该不该信？

背景（2026-10-22）：
  固件 /odom 的 yaw 来自 AHRS（陀螺 + 磁互补滤波）。实测环境磁场梯度 21µT/m，
  车前进 30cm，磁矢量方向转 8°，导致 3s 内 HDG 假漂 −4.3°（陀螺积分只有 +0.21°）。
  但导航实际用的是 odom_fusion 的输出 /odom_fused，里面有 rf2o 激光里程计参与（不感知磁场）。
  → 本脚本回答：融合之后，直线行驶的航向还准不准。

三路对比（真值由激光点云几何互相关给出，不经过任何惯性传感器）：
  /odom        —— 固件：轮速 + AHRS(磁)          预期：会漂
  /odom_laser  —— rf2o：纯激光几何              预期：稳
  /odom_fused  —— 融合                           待测（本次主角）

关键：命令走 /cmd_vel（此时 serial_bridge 占着 /dev/ttyS1，不能直接开串口）。

用法：
  python3 fusion_line_test.py --speed 0.1 --dur 3
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from cloud_shift import to_xy, match_rt


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap180(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class Probe(Node):
    def __init__(self):
        super().__init__("fusion_line_test")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.scan = None
        self.odom = None
        self.laser = None
        self.fused = None
        self.create_subscription(LaserScan, "/scan", self._scan, q)
        self.create_subscription(Odometry, "/odom", self._odom, q)
        self.create_subscription(Odometry, "/odom_laser", self._laser, q)
        self.create_subscription(Odometry, "/odom_fused", self._fused, q)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _scan(self, m):
        self.scan = m

    def _odom(self, m):
        self.odom = m

    def _laser(self, m):
        self.laser = m

    def _fused(self, m):
        self.fused = m

    def spin_for(self, sec):
        t_end = time.time() + sec
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def gather_scan(self, n=8):
        acc = []
        for _ in range(n):
            self.spin_for(0.15)
            if self.scan is not None:
                p = to_xy(self.scan)
                if len(p):
                    acc.append(p)
        return np.vstack(acc) if acc else None

    def yaw_snapshot(self, sec=0.8):
        """静止采 sec 秒，返回三路 yaw 的均值（比单帧稳）"""
        buf = {"odom": [], "laser": [], "fused": []}
        t_end = time.time() + sec
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None:
                buf["odom"].append(yaw_of(self.odom))
            if self.laser is not None:
                buf["laser"].append(yaw_of(self.laser))
            if self.fused is not None:
                buf["fused"].append(yaw_of(self.fused))
        return {k: (sum(v) / len(v) if v else float("nan")) for k, v in buf.items()}


def yaw_triple(node):
    return (yaw_of(node.odom) if node.odom is not None else float("nan"),
            yaw_of(node.laser) if node.laser is not None else float("nan"),
            yaw_of(node.fused) if node.fused is not None else float("nan"))


def drive(node, speed, dur, hz=20):
    """通过 /cmd_vel 持续发速度，dur 秒后归零。返回运动过程中每步的 yaw 三元组。

    ⚠️ serial_bridge 没有 cmd_vel 超时停车（只有 /stop_chassis 服务），
    固件也没有命令超时。所以脚本一旦异常退出，车会保持最后速度一直走。
    必须用 try/finally 保证任何路径下都发一次 0。
    """
    msg = Twist()
    msg.linear.x = speed
    traj = []
    try:
        t_end = time.time() + dur
        while time.time() < t_end:
            node.pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            traj.append(yaw_triple(node))
            time.sleep(1.0 / hz)
    finally:
        msg.linear.x = 0.0
        for _ in range(10):          # 多发几次，防止单帧丢失
            node.pub.publish(msg)
            node.spin_for(0.05)
    return traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.1)
    ap.add_argument("--dur", type=float, default=3.0)
    ap.add_argument("--search", type=float, default=0.45)
    ap.add_argument("--yaw-search", type=float, default=10.0)
    a = ap.parse_args()

    rclpy.init()
    node = Probe()

    print("等待四路话题有数据（最多 25s）...")
    t_end = time.time() + 25
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(x is not None for x in (node.scan, node.odom, node.laser, node.fused)):
            break
    missing = [n for n, v in (("scan", node.scan), ("/odom", node.odom),
                              ("/odom_laser", node.laser),
                              ("/odom_fused", node.fused)) if v is None]
    if missing:
        print("!! 缺少话题：%s" % ", ".join(missing))
        node.destroy_node()
        rclpy.shutdown()
        return
    print("四路齐全。\n")

    print("采集运动前点云 A ...")
    A = node.gather_scan(8)
    y0 = node.yaw_snapshot(0.8)
    print("  帧 A: %d 点" % (len(A) if A is not None else 0))
    print("  起始 yaw: /odom %+.2f°  /odom_laser %+.2f°  /odom_fused %+.2f°"
          % (math.degrees(y0["odom"]), math.degrees(y0["laser"]),
             math.degrees(y0["fused"])))

    print("\n>>> /cmd_vel 前进 %.2f m/s，持续 %.1fs" % (a.speed, a.dur))
    traj = drive(node, a.speed, a.dur)

    print("停车，等 1.5s ...")
    node.spin_for(1.5)

    print("采集运动后点云 B ...")
    B = node.gather_scan(8)
    y1 = node.yaw_snapshot(0.8)
    print("  帧 B: %d 点" % (len(B) if B is not None else 0))

    if A is None or B is None or len(A) < 100 or len(B) < 100:
        print("!! 点云不足，无法给真值")
        node.destroy_node()
        rclpy.shutdown()
        return

    cost, dx, dyaw = match_rt(A, B, a.search, a.yaw_search,
                              dx_step=0.01, yaw_step=1.0)

    d_odom = wrap180(math.degrees(y1["odom"] - y0["odom"]))
    d_laser = wrap180(math.degrees(y1["laser"] - y0["laser"]))
    d_fused = wrap180(math.degrees(y1["fused"] - y0["fused"]))

    # 停车瞬间（最后一个 cmd_vel 周期）的三路航向：此时尚未被静止锚拉过
    if traj:
        ys = traj[-1]
        s_odom = wrap180(math.degrees(ys[0] - y0["odom"]))
        s_laser = wrap180(math.degrees(ys[1] - y0["laser"]))
        s_fused = wrap180(math.degrees(ys[2] - y0["fused"]))
    else:
        s_odom = s_laser = s_fused = float("nan")

    print("\n" + "=" * 70)
    print("【真值：激光点云几何互相关】")
    print("  纵向位移 dx = %.1f cm     偏航 dyaw = %+.2f°     cost = %.1f mm"
          % (abs(dx) * 100, dyaw, cost * 1000))
    print("  （cost > 60mm 说明匹配不可信）")

    print("\n【A. 行驶段：起步 → 刚停车那一刻】（这一段最能说明行驶中准不准）")
    print("  偏航(点云真值)         %+8.2f°" % dyaw)
    print("  /odom    (固件磁)      %+8.2f°   偏差 %+7.2f°" % (s_odom, s_odom - dyaw))
    print("  /odom_laser (rf2o)     %+8.2f°   偏差 %+7.2f°" % (s_laser, s_laser - dyaw))
    print("  /odom_fused (融合)     %+8.2f°   偏差 %+7.2f°" % (s_fused, s_fused - dyaw))

    print("\n【B. 含停车后 1.5s 静止收敛的总变化】（静止锚会把融合航向拉向磁航向）")
    print("  /odom    (固件磁)      %+8.2f°   偏差 %+7.2f°" % (d_odom, d_odom - dyaw))
    print("  /odom_laser (rf2o)     %+8.2f°   偏差 %+7.2f°" % (d_laser, d_laser - dyaw))
    print("  /odom_fused (融合)     %+8.2f°   偏差 %+7.2f°" % (d_fused, d_fused - dyaw))

    print("\n【C. 静止锚造成的额外漂移（B − A）】")
    print("  /odom_fused            %+8.2f°" % (d_fused - s_fused))
    print("=" * 70)

    print("\n【判读】")
    thr = 1.0
    if abs(s_fused - dyaw) < thr:
        print("  → 行驶中融合航向可信（A 表偏差 %.2f°）。" % (s_fused - dyaw))
        if abs(d_fused - dyaw) >= thr:
            print("     但停车后被静止锚拉歪了（B 表 %.2f°，额外 %.2f°）。"
                  % (d_fused - dyaw, d_fused - s_fused))
            print("     → 问题不在融合，在『静止时用磁航向做绝对锚』这个假设。")
            print("       环境磁场不均匀时，停下后的磁航向本身就是错的。")
    else:
        print("  → 行驶中融合航向就歪了（A 表偏差 %.2f°）。" % (s_fused - dyaw))
        print("     → rf2o 权重不够，融合仍大量采信固件磁航向。")
    if abs(s_laser - dyaw) > 2.0:
        print("  注意：rf2o 自身 A 表偏差 %.2f°，小房间内激光里程计也未必可靠。"
              % (s_laser - dyaw))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
