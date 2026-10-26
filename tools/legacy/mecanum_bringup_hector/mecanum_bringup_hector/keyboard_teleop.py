#!/usr/bin/env python3
"""
keyboard_teleop.py — 修复版键盘遥控节点（mecanum_bringup_hector 内）

相对原 mecanum_bringup/keyboard_teleop.py 的修改:
  1. 修复 +/- 调速崩溃:
     原代码调用 self.adjust_axis_speed(), 但方法实际叫 self.adjust_speed()
     导致单独按 + 或 - 直接 AttributeError。
  2. 保留原版“组合键同时生效”的计算逻辑:
     compute_twist() 本来就支持 W+D 同时前进+右转，
     只要两个键都被终端送进 stdin，就会合成 Twist。

SSH stdin 模式的限制:
  SSH 终端只能靠“按键自动重复字符流”来判断按住状态，
  不同终端对多键同时按下的重复流支持不一样。
  如果 W+D 组合不稳定，可尝试:
    - 调大 auto_stop_timeout: --ros-args -p auto_stop_timeout:=0.5
    - 或在 RK3588 本地物理键盘用 evdev 模式:
      --ros-args -p input_mode:=evdev
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import os
import select
import sys
import termios
import threading
import time
import tty

try:
    import evdev
    from evdev import ecodes
    HAVE_EVDEV = True
except ImportError:
    HAVE_EVDEV = False

# evdev 键码 -> 逻辑键
KEY_MAP = {
    ecodes.KEY_W: 'w', ecodes.KEY_S: 's', ecodes.KEY_A: 'a', ecodes.KEY_D: 'd',
    ecodes.KEY_Q: 'q', ecodes.KEY_E: 'e',
    ecodes.KEY_SPACE: 'space',
    ecodes.KEY_EQUAL: 'plus', ecodes.KEY_KPPLUS: 'plus',
    ecodes.KEY_MINUS: 'minus', ecodes.KEY_KPMINUS: 'minus',
} if HAVE_EVDEV else {}

# stdin 字符 -> 逻辑键
CHAR_MAP = {
    'w': 'w', 'W': 'w',
    's': 's', 'S': 's',
    'a': 'a', 'A': 'a',
    'd': 'd', 'D': 'd',
    'q': 'q', 'Q': 'q',
    'e': 'e', 'E': 'e',
    ' ': 'space',
    '+': 'plus', '=': 'plus',
    '-': 'minus', '_': 'minus',
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_hector')

        # ---------- 参数 ----------
        self.declare_parameter('linear_vel', 0.3)     # 前进速度初值 (m/s)
        self.declare_parameter('linear_y_vel', 0.3)   # 横移速度初值 (m/s)
        self.declare_parameter('angular_vel', 0.8)    # 旋转速度初值 (rad/s)
        self.declare_parameter('speed_step', 0.1)     # +/- 每次调节步进
        self.declare_parameter('device', '')          # evdev 设备 (空=自动查找)
        self.declare_parameter('input_mode', 'auto')  # auto/evdev/stdin
        self.declare_parameter('keepalive_hz', 10.0)  # 保活重发频率
        self.declare_parameter('auto_stop_timeout', 0.35)  # stdin 模式松开判定 (s)

        # 三轴速度独立: 前进 linear_x / 横移 linear_y / 旋转 angular_z
        self.linear_x_vel = self.get_parameter('linear_vel').value
        self.linear_y_vel = self.get_parameter('linear_y_vel').value
        self.angular_z_vel = self.get_parameter('angular_vel').value
        self.speed_step = self.get_parameter('speed_step').value
        keepalive_hz = self.get_parameter('keepalive_hz').value
        self.auto_stop_timeout = self.get_parameter('auto_stop_timeout').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 当前按住的键集合 (输入线程写, 发布线程读, 加锁)
        self.keys = set()
        self.lock = threading.Lock()
        self.running = True
        self.input_is_stdin = False
        self.last_input = 0.0   # stdin 模式: 最后一次收到字符的时间

        # 选择输入模式
        mode = self.get_parameter('input_mode').value
        if mode == 'stdin':
            self.input_is_stdin = True
        elif mode == 'evdev':
            if not HAVE_EVDEV:
                self.get_logger().error('未安装 python3-evdev, 回退 stdin 模式')
                self.input_is_stdin = True
            else:
                self.input_is_stdin = False
        else:  # auto
            if HAVE_EVDEV and self.find_device() is not None:
                self.input_is_stdin = False
            else:
                self.input_is_stdin = True

        # 保活定时器 (顺带做 stdin 模式的"松开超时停")
        if keepalive_hz > 0.0:
            self.create_timer(1.0 / keepalive_hz, self.timer_cb)

        # 输入线程
        if self.input_is_stdin:
            self.thread = threading.Thread(target=self.stdin_loop, daemon=True)
            self.get_logger().info(
                '输入模式: stdin (SSH 远程) — 按住 WASD 移动, 松开自动停')
        else:
            self.thread = threading.Thread(target=self.evdev_loop, daemon=True)
            self.get_logger().info('输入模式: evdev (本地物理键盘) — 按住移动, 松开即停')
        self.thread.start()

        self.get_logger().info(
            f'键盘遥控就绪: fwd={self.linear_x_vel:.2f} strf={self.linear_y_vel:.2f} m/s, '
            f'rot={self.angular_z_vel:.2f} rad/s')
        self.get_logger().info(
            'W/S 前后  Q/E 横移  A/D 旋转  Space 急停\n'
            '调速: 按住移动键再按 +/- 调对应轴 (如 A+ 调旋转速度), 单独 +/- 全调')

    # ================================================================ evdev
    def find_device(self):
        dev_path = self.get_parameter('device').value
        if dev_path:
            return evdev.InputDevice(dev_path)
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                keys = dev.capabilities(verbose=False).get(ecodes.EV_KEY, [])
                if ecodes.KEY_W in keys and ecodes.KEY_A in keys:
                    self.get_logger().info(f'使用键盘: {dev.name} ({path})')
                    return dev
            except Exception:
                continue
        return None

    def evdev_loop(self):
        self.dev = self.find_device()
        if self.dev is None:
            self.get_logger().error('未找到键盘设备! 用 device 参数指定, 如 /dev/input/event3')
            return
        try:
            for event in self.dev.read_loop():
                if event.type != ecodes.EV_KEY:
                    continue
                key = KEY_MAP.get(event.code)
                if key is None:
                    continue
                if event.value == 1:          # 按下
                    with self.lock:
                        self.keys.add(key)
                    if key == 'plus':
                        self.adjust_speed(+1.0)
                    elif key == 'minus':
                        self.adjust_speed(-1.0)
                    elif key == 'space':
                        with self.lock:
                            self.keys.clear()
                    self.publish_twist()
                elif event.value == 0:        # 松开 → 立即重算 (全松开=零速度)
                    with self.lock:
                        self.keys.discard(key)
                    self.publish_twist()
                # value==2 为长按自动重复, 忽略
        except OSError as e:
            self.get_logger().error(f'键盘读取失败: {e} (设备可能被拔出)')

    # ================================================================ stdin
    def stdin_loop(self):
        """SSH 终端字符流: 按住→重复字符→续命; 松开→超时→停"""
        fd = sys.stdin.fileno()
        try:
            old = termios.tcgetattr(fd)
        except termios.error:
            self.get_logger().error(
                'stdin 不是终端设备! 键盘节点必须用 ros2 run 在终端中直接启动\n'
                '        (不能通过 ros2 launch 启动 — 与 teleop_twist_keyboard 相同的限制,\n'
                '         因为 launch 子进程的 stdin 不是终端)')
            return
        try:
            tty.setcbreak(fd)   # 无回显、无行缓冲、立即读取
            self.get_logger().info('在 SSH 终端中按住 WASD 控制, 松开自动停 (Ctrl+C 退出)')
            while self.running:
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    data = os.read(fd, 64)
                except OSError:
                    break
                if not data:
                    break
                for ch in data.decode('utf-8', 'ignore'):
                    self.handle_char(ch)
        finally:
            # 恢复终端属性 (退出/断开时)
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except termios.error:
                pass

    def handle_char(self, ch):
        key = CHAR_MAP.get(ch)
        if key is None:
            return
        with self.lock:
            self.keys.add(key)
            self.last_input = time.time()
        if key == 'plus':
            self.adjust_speed(+1.0)
        elif key == 'minus':
            self.adjust_speed(-1.0)
        elif key == 'space':
            with self.lock:
                self.keys.clear()
        self.publish_twist()

    # ================================================================ 发布
    def compute_twist(self):
        with self.lock:
            k = set(self.keys)
        t = Twist()
        # 键映射 (用户习惯): W/S=前后, Q/E=横移, A/D=旋转
        if 'w' in k:
            t.linear.x += self.linear_x_vel
        if 's' in k:
            t.linear.x -= self.linear_x_vel
        if 'q' in k:
            t.linear.y += self.linear_y_vel    # ROS y 左为正
        if 'e' in k:
            t.linear.y -= self.linear_y_vel
        if 'a' in k:
            t.angular.z += self.angular_z_vel  # 左转 (逆时针) 为正
        if 'd' in k:
            t.angular.z -= self.angular_z_vel
        return t

    def publish_twist(self):
        """按键事件驱动: 即时发布 (松开即停的关键)"""
        self.pub.publish(self.compute_twist())

    def timer_cb(self):
        """保活重发 + stdin 模式"松开超时停" (auto_stop_timeout 无输入 → 清空按键)"""
        if self.input_is_stdin:
            with self.lock:
                if self.keys and (time.time() - self.last_input > self.auto_stop_timeout):
                    self.keys.clear()
        self.publish_twist()

    def adjust_speed(self, direction):
        # 组合键: 按住移动键再按 +/- 调对应轴; 无移动键按住时三轴同调(兼容旧版)
        with self.lock:
            held = set(self.keys)
        if 'w' in held or 's' in held:
            self.linear_x_vel = max(0.05, min(2.0, self.linear_x_vel + direction * self.speed_step))
            self.get_logger().info(f'前进速度: {self.linear_x_vel:.2f} m/s')
        elif 'q' in held or 'e' in held:
            self.linear_y_vel = max(0.05, min(2.0, self.linear_y_vel + direction * self.speed_step))
            self.get_logger().info(f'横移速度: {self.linear_y_vel:.2f} m/s')
        elif 'a' in held or 'd' in held:
            self.angular_z_vel = max(0.1, min(5.0, self.angular_z_vel + direction * self.speed_step))
            self.get_logger().info(f'旋转速度: {self.angular_z_vel:.2f} rad/s')
        else:
            self.linear_x_vel = max(0.05, min(2.0, self.linear_x_vel + direction * self.speed_step))
            self.linear_y_vel = max(0.05, min(2.0, self.linear_y_vel + direction * self.speed_step))
            self.angular_z_vel = max(0.1, min(5.0, self.angular_z_vel + direction * self.speed_step))
            self.get_logger().info(
                f'全调: fwd={self.linear_x_vel:.2f} strf={self.linear_y_vel:.2f} '
                f'rot={self.angular_z_vel:.2f}')

    def destroy_node(self):
        self.running = False
        # 退出前发零速度, 防止小车持续运动
        try:
            zero = Twist()
            self.pub.publish(zero)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
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
