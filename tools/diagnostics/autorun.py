#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""autorun.py —— 自动跑圈建图（带雷达安全刹车）

为什么这么写：
  1) 上次手动发速度撞了。这里每一帧（20Hz）都先查雷达，前方过近立刻停车退出。
  2) chassis_bridge 有 0.5s 看门狗，所以 cmd_vel 必须持续发，不能"发一次就等"。
  3) 退出路径必须发零速 —— 用 try/finally 兜住，包括 Ctrl-C 与异常。

角度约定（本车实测）：
  雷达 0°=车尾、180°=车头、90°=右侧。所以"前方安全距离"取 180°±30°。
  cmd_vel 沿用 ROS 口径：linear.x 前为正，angular.z 逆时针为正。

用法：
  python3 autorun.py --mode spin360     # 原地转一圈（只验证航向，不移动位置）
  python3 autorun.py --mode rect        # 矩形环（前进+转90°×4，回到起点）
  python3 autorun.py --mode zigzag      # 「几」字形开链
  python3 autorun.py --mode explore --hops 2 --step 1.2
                                        # 雷达自主探索：扫全→选最空→走一跳，循环
"""
import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def median(v):
    s = sorted(v)
    n = len(s)
    if n == 0:
        return 99.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class Runner(Node):
    def __init__(self):
        super().__init__('autorun')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_fused', self._odom_cb, qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Odometry, '/odom', self._raw_cb, qos)
        self.create_subscription(Odometry, '/odom_laser', self._laser_cb, qos)

        self.x = self.y = None
        self.yaw = None
        self.yaw_acc = 0.0
        self.prev_yaw = None
        self.start = None          # (x, y, yaw_acc)
        self.rx = self.ry = self.ryaw = None   # 固件 /odom 原始（对照用）
        self.lx = self.ly = self.lyaw = None   # rf2o /odom_laser（对照用）

        self.front = 99.0
        self.left = 99.0
        self.right = 99.0
        self.back = 99.0
        self.scan_n = 0
        # 12 扇区中位距离（雷达角 0=车尾, 90=右, 180=车头, 270=左）
        #   探索模式靠它选"最空方向"。用中位数而不是最小值，
        #   因为最小会被一根柱子/桌腿骗走，中位数才代表"这个方向整体开阔"。
        self.sec = [99.0] * 12

    def _odom_cb(self, m):
        p = m.pose.pose
        y = yaw_of(p.orientation)
        if self.prev_yaw is not None:
            self.yaw_acc += wrap(y - self.prev_yaw)
        self.prev_yaw = y
        self.x = p.position.x
        self.y = p.position.y
        self.yaw = y
        if self.start is None:
            self.start = (self.x, self.y, self.yaw_acc)

    def _raw_cb(self, m):
        p = m.pose.pose
        self.rx, self.ry, self.ryaw = p.position.x, p.position.y, yaw_of(p.orientation)

    def _laser_cb(self, m):
        p = m.pose.pose
        self.lx, self.ly, self.lyaw = p.position.x, p.position.y, yaw_of(p.orientation)

    def _scan_cb(self, m):
        inc = m.angle_increment
        f = l = r = b = 99.0
        buckets = [[] for _ in range(12)]
        for i, rv in enumerate(m.ranges):
            if not math.isfinite(rv) or rv < m.range_min or rv > m.range_max:
                continue
            ad = math.degrees(m.angle_min + i * inc) % 360.0
            # 前方 180±30
            if 150.0 <= ad <= 210.0:
                f = min(f, rv)
            # 左侧 270±25
            if 245.0 <= ad <= 295.0:
                l = min(l, rv)
            # 右侧 90±25
            if 65.0 <= ad <= 115.0:
                r = min(r, rv)
            # 后方 0±30
            if ad <= 30.0 or ad >= 330.0:
                b = min(b, rv)
            buckets[int(ad // 30.0) % 12].append(rv)
        self.front, self.left, self.right, self.back = f, l, r, b
        for k in range(12):
            self.sec[k] = median(buckets[k]) if buckets[k] else 99.0
        self.scan_n += 1

    def ready(self):
        return (self.x is not None) and (self.scan_n > 3)

    def stop(self):
        t = Twist()
        for _ in range(6):
            self.pub.publish(t)
            time.sleep(0.03)
        self.get_logger().info('已发零速')

    def spin(self, dt):
        rclpy.spin_once(self, timeout_sec=dt)


def drive(node, v, w, seconds=None, done=None, safe_dist=0.45, need_front=True,
          timeout=60.0, label=''):
    """以 20Hz 闭环发送速度。done() 返回 True 即结束。"""
    period = 0.05
    t0 = time.time()
    t = Twist()
    min_front = 99.0
    while True:
        node.spin(period)
        el = time.time() - t0

        if need_front:
            min_front = min(min_front, node.front)
        if need_front and node.front < safe_dist:
            node.pub.publish(Twist())
            node.stop()
            raise RuntimeError('[%s] 安全刹车：前方 %.2f m < %.2f m' % (label, node.front, safe_dist))
        if node.left < 0.30 or node.right < 0.30:
            node.pub.publish(Twist())
            node.stop()
            raise RuntimeError('[%s] 侧向过近：左 %.2f 右 %.2f' % (label, node.left, node.right))
        if el > timeout:
            node.pub.publish(Twist())
            node.stop()
            raise RuntimeError('[%s] 超时 %.0fs 未完成' % (label, el))

        t.linear.x = v
        t.angular.z = w
        node.pub.publish(t)

        if seconds is not None and el >= seconds:
            return el
        if done is not None and done():
            return el


def spin_by(node, delta, w_max=0.45, tol=0.035, safe=True, timeout=45.0, label=''):
    """原地转 delta 弧度（正=逆时针/左转）。返回实际转角。

    判据 `err*sign <= tol` 必须带 sign —— 否则负转角时 err 一开始就是负的，
    条件立刻成立，转向被瞬间"完成"（2026-10-22 在 zigzag 模式踩过这个坑）。
    """
    y0 = node.yaw_acc
    sign = 1.0 if delta > 0 else -1.0
    t0 = time.time()
    while True:
        node.spin(0.05)
        err = delta - (node.yaw_acc - y0)
        if err * sign <= tol:
            break
        if time.time() - t0 > timeout:
            node.pub.publish(Twist())
            node.stop()
            raise RuntimeError('[%s] 转向超时（已转 %.1f°）'
                               % (label, math.degrees(node.yaw_acc - y0)))
        if safe and (node.front < 0.35 or node.left < 0.25 or node.right < 0.25):
            node.pub.publish(Twist())
            node.stop()
            raise RuntimeError('[%s] 转向安全刹车 前 %.2f 左 %.2f 右 %.2f'
                               % (label, node.front, node.left, node.right))
        tw = Twist()
        tw.angular.z = max(-w_max, min(w_max, 1.2 * err))
        node.pub.publish(tw)
    node.pub.publish(Twist())
    time.sleep(0.3)
    node.spin(0.2)
    return node.yaw_acc - y0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='spin360', choices=['spin360', 'rect', 'zigzag', 'explore'])
    ap.add_argument('--v', type=float, default=0.10, help='线速度 m/s')
    ap.add_argument('--w', type=float, default=0.45, help='角速度 rad/s')
    ap.add_argument('--safe', type=float, default=0.45, help='前方安全距离 m')
    ap.add_argument('--hops', type=int, default=2, help='explore 模式走几跳')
    ap.add_argument('--step', type=float, default=1.2, help='explore 模式每跳前进距离 m')
    ap.add_argument('--log', default=os.path.expanduser('~/autorun_log.csv'))
    a = ap.parse_args()

    rclpy.init()
    node = Runner()
    while not node.ready():
        node.spin(0.1)

    x0, y0, ya0 = node.start
    node.get_logger().info('起点 fused=(%.3f, %.3f) yaw=%.1f°  前 %.2f 左 %.2f 右 %.2f 后 %.2f'
                           % (x0, y0, math.degrees(ya0),
                              node.front, node.left, node.right, node.back))

    f = open(a.log, 'w', newline='')
    cw = csv.writer(f)
    cw.writerow(['t', 'fx', 'fy', 'fyaw', 'yaw_acc', 'wx', 'wy', 'wyaw',
                 'lx', 'ly', 'lyaw', 'front', 'left', 'right', 'back'])

    t_start = time.time()
    last_log = 0.0

    def log_row():
        cw.writerow(['%.2f' % (time.time() - t_start),
                     '%.4f' % node.x, '%.4f' % node.y, '%.4f' % node.yaw,
                     '%.4f' % node.yaw_acc,
                     '%.4f' % (node.rx or 0), '%.4f' % (node.ry or 0), '%.4f' % (node.ryaw or 0),
                     '%.4f' % (node.lx or 0), '%.4f' % (node.ly or 0), '%.4f' % (node.lyaw or 0),
                     '%.3f' % node.front, '%.3f' % node.left,
                     '%.3f' % node.right, '%.3f' % node.back])

    try:
        if a.mode == 'spin360':
            target = 2.0 * math.pi
            node.get_logger().info('原地逆时针转 360°（目标 Δyaw=%.1f°）' % math.degrees(target))

            def done():
                return node.yaw_acc - ya0 >= target - 0.035   # 提前 2° 收手，惯性补上

            def keep_log():
                nonlocal last_log
                now = time.time()
                if now - last_log > 0.1:
                    log_row()
                    last_log = now

            t0 = time.time()
            while True:
                node.spin(0.05)
                err = target - (node.yaw_acc - ya0)
                if err <= 0.035:
                    break
                if node.front < 0.35 or node.left < 0.25 or node.right < 0.25:
                    raise RuntimeError('安全刹车（旋转中）前 %.2f 左 %.2f 右 %.2f'
                                       % (node.front, node.left, node.right))
                if time.time() - t0 > 60:
                    raise RuntimeError('旋转超时')
                w = max(-a.w, min(a.w, 1.2 * err))
                tw = Twist()
                tw.angular.z = w
                node.pub.publish(tw)
                keep_log()

            node.pub.publish(Twist())
            time.sleep(0.3)
            node.spin(0.2)
            log_row()
            d_yaw = math.degrees(node.yaw_acc - ya0)
            node.get_logger().info('旋转结束：累计 Δyaw=%.2f°（目标 360°）误差 %+.2f°'
                                   % (d_yaw, d_yaw - 360.0))
            node.get_logger().info('位置变化：Δx=%+.3f Δy=%+.3f  |Δ|=%+.3f m（原地旋转应≈0）'
                                   % (node.x - x0, node.y - y0,
                                      math.hypot(node.x - x0, node.y - y0)))

        elif a.mode in ('rect', 'zigzag'):
            if a.mode == 'rect':
                segs = [('前进1.2m', 'fwd', 1.2), ('左转90', 'turn', 90),
                        ('前进0.8m', 'fwd', 0.8), ('左转90', 'turn', 90),
                        ('前进1.2m', 'fwd', 1.2), ('左转90', 'turn', 90),
                        ('前进0.8m', 'fwd', 0.8), ('左转90', 'turn', 90)]
            else:
                # 「几」字形：前进 → 左转 → 前进 → 右转 → 前进
                # 选它而不是闭环矩形，是因为实测后方只有 1.27m，闭环第三段朝后必撞。
                segs = [('前进0.8m', 'fwd', 0.8), ('左转90', 'turn', 90),
                        ('前进1.0m', 'fwd', 1.0), ('右转90', 'turn', -90),
                        ('前进0.8m', 'fwd', 0.8)]
            cur_yaw = node.yaw_acc
            cx, cy = node.x, node.y
            for name, kind, amt in segs:
                node.get_logger().info('--- %s ---' % name)
                if kind == 'fwd':
                    tx, ty = cx, cy
                    def done_dir(tx=tx, ty=ty, amt=amt):
                        return math.hypot(node.x - tx, node.y - ty) >= amt - 0.02
                    # 方向用起始朝向锁定
                    drive(node, a.v, 0.0, done=done_dir, safe_dist=a.safe,
                          timeout=amt / a.v * 2.5 + 8, label=name)
                    cx, cy = node.x, node.y
                else:
                    tgt = cur_yaw + math.radians(amt)
                    # 双向转向：amt 为负即右转。
                    #   ⚠ 判据必须与转向方向同号，否则右转时 err 一开始就是负值，
                    #     写成 `err <= 0.035` 会立刻成立 → 转向被瞬间"完成"（2026-10-22 踩过）。
                    sign = 1.0 if amt > 0 else -1.0
                    t0 = time.time()
                    while True:
                        node.spin(0.05)
                        err = tgt - node.yaw_acc
                        if err * sign <= 0.035 or time.time() - t0 > 30:
                            break
                        if node.front < 0.35 or node.left < 0.25 or node.right < 0.25:
                            raise RuntimeError('转向安全刹车 前 %.2f 左 %.2f 右 %.2f'
                                               % (node.front, node.left, node.right))
                        tw = Twist()
                        tw.angular.z = max(-a.w, min(a.w, 1.2 * err))
                        node.pub.publish(tw)
                    node.pub.publish(Twist())
                    time.sleep(0.35)
                    node.spin(0.2)
                    cur_yaw = node.yaw_acc
                    node.get_logger().info('    yaw_acc=%.1f° 已到位' % math.degrees(cur_yaw))
                log_row()

            node.get_logger().info('收尾：回到起点附近，Δ=(%+.3f, %+.3f) m'
                                   % (node.x - x0, node.y - y0))

        elif a.mode == 'explore':
            # 雷达自主探索：转一圈扫全 → 选最空方向 → 转过去 → 走一跳 → 重复。
            #   目的不是"走遍全场"，而是从**不同视角**重复观测同一批结构：
            #   真墙在两次点云里位置一致且变密，重影会随视角变形。
            #   角度换算：雷达角 180°=车头=车体 0°，雷达角增大=逆时针，故 Δ=雷达角−180°。
            for hop in range(a.hops):
                node.get_logger().info('===== 第 %d/%d 跳：原地扫全 =====' % (hop + 1, a.hops))
                d_turn = spin_by(node, 2.0 * math.pi, a.w, label='scan%d' % hop)
                log_row()
                node.get_logger().info('  扫全完成，实际转 %.1f°' % math.degrees(d_turn))

                idx = max(range(12), key=lambda i: node.sec[i])
                d = node.sec[idx]
                node.get_logger().info(
                    '  最空扇区 %d° 中位 %.2f m | 12扇区= %s'
                    % (idx * 30, d, ' '.join('%.1f' % v for v in node.sec)))
                if d < 1.3:
                    node.get_logger().warn('  最空方向仅 %.2f m（<1.3），停止探索' % d)
                    break

                delta = wrap(math.radians(idx * 30 - 180))
                node.get_logger().info('  转向该方向：车体 %+.0f°' % math.degrees(delta))
                spin_by(node, delta, a.w, label='aim%d' % hop)
                log_row()

                step = min(a.step, d - a.safe - 0.15)
                node.get_logger().info('  前进 %.2f m（该方向 %.2f m，走完留 %.2f m）'
                                       % (step, d, d - step))
                tx, ty = node.x, node.y

                def done_dir(tx=tx, ty=ty, step=step):
                    return math.hypot(node.x - tx, node.y - ty) >= step - 0.02

                drive(node, a.v, 0.0, done=done_dir, safe_dist=a.safe,
                      timeout=step / a.v * 2.5 + 10, label='hop%d' % hop)
                log_row()
                node.get_logger().info('  到位 fused=(%.3f, %.3f)'
                                       % (node.x, node.y))

            node.get_logger().info('探索结束：总位移 Δ=(%+.3f, %+.3f) m'
                                   % (node.x - x0, node.y - y0))

    except RuntimeError as e:
        node.get_logger().error('中止：%s' % e)
    finally:
        node.stop()
        f.close()
        node.get_logger().info('日志已写 %s' % a.log)
        node.destroy_node()
        rclpy.shutdown()


main()
