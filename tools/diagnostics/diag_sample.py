#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""连续采样 /fusion_diag，观察静止锚是否把融合航向拉向磁航向。

背景：fusion_node 在 still_active 时，yaw_fused 按 still_anchor_gain(0.02)@50Hz
（τ≈1s）向 yaw_wheel_corrected（磁绝对航向）收敛。若环境磁场不均匀，
车停下后的磁航向本身就是错的，这个"收敛"就是把对的航向拉歪。

用法：
  python3 diag_sample.py --dur 8 --every 0.5
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Diag(Node):
    def __init__(self):
        super().__init__("diag_sample")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, "/fusion_diag", self.cb, q)
        self.last = None

    def cb(self, msg):
        try:
            self.last = json.loads(msg.data)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=float, default=8.0)
    ap.add_argument("--every", type=float, default=0.5)
    a = ap.parse_args()

    rclpy.init()
    n = Diag()
    t0 = time.time()
    nxt = 0.0
    print("  t(s)   wheel     fused     laser   still   w_wheel  w_laser   wheel-fused")
    while time.time() - t0 < a.dur:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time() - t0
        if now >= nxt and n.last is not None:
            nxt = now + a.every
            d = n.last
            print("  %4.1f  %7.2f  %8.2f  %8.2f   %-5s    %.2f     %.2f     %+7.2f"
                  % (now, d["yaw_wheel_deg"], d["yaw_fused_deg"], d["yaw_laser_deg"],
                     str(d["still"]), d["w_wheel"], d["w_laser"],
                     d["yaw_wheel_deg"] - d["yaw_fused_deg"]))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
