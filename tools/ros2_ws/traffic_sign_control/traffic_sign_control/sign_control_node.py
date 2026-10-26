#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sign_control_node.py — 交通标志 AI 识别 → 麦轮底盘动作执行节点
================================================================

控制方式与 mecanum_bringup_hector/keyboard_teleop.py 一致：
发布 /cmd_vel (geometry_msgs/Twist)，由 serial_bridge 转发给 STM32 底盘。
本节点是独立控制包，不与其他包参杂。

工作流程：
  板端 AI 推理脚本(rknn_infer_camera.py) 检测到交通标志后，
  通过 UDP 发送一行  "类别名 置信度"  → 本节点 UDP 监听端口接收，
  按类别执行对应动作（发 /cmd_vel，持续一段时间后自动停止）。

动作表（速度与 keyboard_teleop.py 初始速度一致：linear=0.3/0.3 m/s, angular=0.8 rad/s；
direction 符号遵循 ROS 约定：angular.z 逆时针为正，linear.y 左移为正）：
  类别                  动作              轴        方向
  no-right            右转 90°          angular.z  负   @0.8 rad/s
  no-left             左转 90°          angular.z  正   @0.8 rad/s
  speed-limit-40      左转 180°         angular.z  正   @0.8 rad/s
  end-speed-limit-40  右转 180°         angular.z  负   @0.8 rad/s
  Car-no-p            前进 1m           linear.x   正   @0.3 m/s
  Car-p               右横移 1m         linear.y   负   @0.3 m/s

安全特性：
  - 动作执行中忽略新的检测（防抖动重复触发）
  - 动作结束后冷却 cooldown 秒
  - UDP 收到 "stop" 立即急停
  - Ctrl+C 退出时发布零速度

运行：
  ros2 run traffic_sign_control sign_control_node
"""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ---------------------------------------------------------------------------
# 动作表：类别 -> 执行参数
#   axis:  'az' = angular.z, 'lx' = linear.x, 'ly' = linear.y
#   direction: +1/-1（符合 ROS 符号约定）
#   speed_param / dist_param: 对应可调参数名
# ---------------------------------------------------------------------------
ACTIONS = {
    'no-right':            dict(axis='az', direction=-1.0, speed_param='rot_speed_90',
                                angle=math.pi / 2, desc='右转 90°'),
    'no-left':             dict(axis='az', direction=+1.0, speed_param='rot_speed_90',
                                angle=math.pi / 2, desc='左转 90°'),
    'speed-limit-40':      dict(axis='az', direction=+1.0, speed_param='rot_speed_180_low',
                                angle=math.pi, desc='左转 180°'),
    'end-speed-limit-40':  dict(axis='az', direction=-1.0, speed_param='rot_speed_180_high',
                                angle=math.pi, desc='右转 180°'),
    'Car-no-p':            dict(axis='lx', direction=+1.0, speed_param='fwd_speed',
                                dist_param='fwd_dist', desc='前进 1m'),
    'Car-p':               dict(axis='ly', direction=-1.0, speed_param='strafe_speed',
                                dist_param='strafe_dist', desc='右横移 1m'),
}


class SignControlNode(Node):

    def __init__(self):
        super().__init__('sign_control_node')

        # ---------------- 参数 ----------------
        # UDP 接收
        self.declare_parameter('udp_ip', '127.0.0.1')   # 与 AI 推理脚本同机
        self.declare_parameter('udp_port', 8877)
        # 执行控制
        self.declare_parameter('keepalive_hz', 20.0)    # 动作期间保活重发频率
        self.declare_parameter('cooldown', 1.0)         # 动作结束后的冷却秒数
        self.declare_parameter('conf_thr', 0.5)         # 置信度阈值（低于忽略）

        # 动作速度 / 距离
        # 速度默认值与 keyboard_teleop.py 初始速度完全一致：
        #   linear_vel=0.3 m/s (前进) / linear_y_vel=0.3 m/s (横移) / angular_vel=0.8 rad/s (旋转)
        self.declare_parameter('rot_speed_90', 0.8)     # 左/右转 90° 角速度 (rad/s)
        self.declare_parameter('rot_speed_180_low', 0.8)    # 限速40 左转180° (rad/s)
        self.declare_parameter('rot_speed_180_high', 0.8)   # 解除限速40 右转180° (rad/s)
        self.declare_parameter('fwd_speed', 0.3)        # 前进速度 (m/s)
        self.declare_parameter('fwd_dist', 1.0)         # 前进距离 (m)
        self.declare_parameter('strafe_speed', 0.3)     # 横移速度 (m/s)
        self.declare_parameter('strafe_dist', 1.0)      # 横移距离 (m)

        # ---------------- 发布 ----------------
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------- 状态 ----------------
        self._busy = False
        self._lock = threading.Lock()
        self._stop = threading.Event()     # 急停当前动作
        self._alive = True

        # ---------------- UDP 接收线程 ----------------
        threading.Thread(target=self.udp_loop, daemon=True).start()

        self.get_logger().info(
            'sign_control_node 就绪: UDP {}:{} → /cmd_vel'.format(
                self.get_parameter('udp_ip').value,
                self.get_parameter('udp_port').value))

    # ===================================================================
    # UDP 接收
    # ===================================================================
    def udp_loop(self):
        ip = self.get_parameter('udp_ip').value
        port = self.get_parameter('udp_port').value
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((ip, port))
        except OSError as e:
            self.get_logger().error('UDP 绑定 {}:{} 失败: {}'.format(ip, port, e))
            return
        sock.settimeout(0.2)
        self.get_logger().info('UDP 监听 {}:{}，等待 AI 检测结果...'.format(ip, port))
        while self._alive:
            try:
                data, _addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            msg = data.decode('utf-8', 'ignore').strip()
            if msg:
                self.handle_detection(msg)
        sock.close()

    # ===================================================================
    # 检测结果处理
    # ===================================================================
    def handle_detection(self, msg):
        """解析 "类别 置信度 [x1 y1 x2 y2]" 并触发动作"""
        parts = msg.split()
        if len(parts) < 2:
            return
        cls, conf_str = parts[0], parts[1]
        try:
            conf = float(conf_str)
        except ValueError:
            self.get_logger().warn('无法解析置信度: {}'.format(msg))
            return

        # 急停指令
        if cls == 'stop':
            self._stop.set()
            self.pub.publish(Twist())
            self.get_logger().warn('收到 stop，急停！')
            return

        if conf < self.get_parameter('conf_thr').value:
            self.get_logger().info(
                '{} 置信度 {:.2f} 低于阈值 {:.2f}，忽略'.format(
                    cls, conf, self.get_parameter('conf_thr').value))
            return

        action = ACTIONS.get(cls)
        if action is None:
            self.get_logger().warn('未知类别: {}'.format(cls))
            return

        with self._lock:
            if self._busy:
                self.get_logger().info('动作执行中，忽略新的 {}'.format(cls))
                return
            self._busy = True

        threading.Thread(target=self._run_action, args=(action,), daemon=True).start()

    # ===================================================================
    # 动作执行
    # ===================================================================
    def _run_action(self, action):
        speed = float(self.get_parameter(action['speed_param']).value)
        if 'angle' in action:
            duration = action['angle'] / speed
        else:
            dist = float(self.get_parameter(action['dist_param']).value)
            duration = dist / speed

        self._stop.clear()
        self.get_logger().info(
            '执行动作: {}  速度={:.2f}  时长={:.2f}s'.format(
                action['desc'], speed, duration))

        period = 1.0 / float(self.get_parameter('keepalive_hz').value)
        t_end = time.time() + duration
        while time.time() < t_end and not self._stop.is_set() and self._alive:
            twist = Twist()
            if action['axis'] == 'az':
                twist.angular.z = action['direction'] * speed
            elif action['axis'] == 'lx':
                twist.linear.x = action['direction'] * speed
            else:  # 'ly'
                twist.linear.y = action['direction'] * speed
            self.pub.publish(twist)
            time.sleep(period)

        # 停止
        self.pub.publish(Twist())
        self.get_logger().info('动作结束: {}'.format(action['desc']))

        # 冷却，避免同一标志连续触发
        if self._alive:
            time.sleep(float(self.get_parameter('cooldown').value))
        with self._lock:
            self._busy = False

    # ===================================================================
    def destroy_node(self):
        self._alive = False
        self._stop.set()
        try:
            self.pub.publish(Twist())   # 退出前确保零速度
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SignControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
