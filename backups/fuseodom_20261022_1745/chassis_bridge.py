#!/usr/bin/env python3
"""底盘串口桥：/cmd_vel -> 串口 ASCII 指令 "V,<vx>,<vy>,<wz>\\r"

设计取舍（新实现，未复用仓库内既有串口桥代码）：
  1. 职责最小化：只做「速度指令 -> 底盘命令」单向翻译。不解析遥测帧、不发布里程计。
     本系统的运动估计来自激光里程计 rf2o，轮式里程计不参与定位。
  2. **带失控看门狗**：超过 watchdog_timeout 没有收到 /cmd_vel，就主动下发零速度。
     没有看门狗时，上层节点崩溃 / 卡死会让底盘保持最后一个速度指令继续行驶——
     这在无人巡检场景里是严重安全隐患。
  3. 串口断开自动重连；进程退出前必定下发零速度。
  4. 速度做硬限幅，防止上层发出超限指令。

坐标约定（与固件一致）：
  vx > 0 = 前进（车头方向）, vy > 0 = 左移, wz > 0 = 逆时针（左转）
"""

import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import run_node


class ChassisBridge(Node):

    def __init__(self):
        super().__init__('chassis_bridge')

        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_vx', 0.60)
        self.declare_parameter('max_vy', 0.60)
        self.declare_parameter('max_wz', 2.00)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('watchdog_rate', 20.0)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('dry_run', False)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self._ser = None
        self._io_lock = threading.Lock()
        self._last_cmd_time = None
        self._stopped_by_watchdog = False
        self._last_reconnect_try = 0.0
        self._running = True

        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd_vel, 10)
        self.create_service(Trigger, '~/stop', self._on_stop)

        rate = float(self.get_parameter('watchdog_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._on_watchdog)

        self._connect()
        self.get_logger().info(
            '底盘桥就绪: %s @ %d, 限幅 vx=%.2f vy=%.2f wz=%.2f, 看门狗 %.1fs%s' % (
                self.port, self.baudrate, self.max_vx, self.max_vy, self.max_wz,
                self.watchdog_timeout, '  [DRY-RUN 不真正发串口]' if self.dry_run else ''))

    # ------------------------------------------------------------------ 串口

    def _connect(self):
        if self.dry_run:
            return True
        try:
            with self._io_lock:
                if self._ser is not None and self._ser.is_open:
                    return True
                self._ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
                time.sleep(0.2)
            self.get_logger().info('串口已打开: %s' % self.port)
            return True
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn('串口打开失败 (%s): %s' % (self.port, exc),
                                   throttle_duration_sec=5.0)
            return False

    def _write(self, text):
        """向底盘写一条 ASCII 命令；失败就地重连。"""
        if self.dry_run:
            return True
        if self._ser is None or not self._ser.is_open:
            return False
        try:
            with self._io_lock:
                self._ser.write((text + '\r').encode('ascii'))
            return True
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error('串口写入失败，将重连: %s' % exc)
            with self._io_lock:
                try:
                    if self._ser is not None:
                        self._ser.close()
                except Exception:                     # noqa: BLE001
                    pass
                self._ser = None
            return False

    def _send_velocity(self, vx, vy, wz):
        ok = self._write('V,%.3f,%.3f,%.3f' % (vx, vy, wz))
        self._publish_status('ok' if ok else 'serial_down',
                             'V,%.3f,%.3f,%.3f' % (vx, vy, wz))
        return ok

    # ------------------------------------------------------------- /cmd_vel

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _on_cmd_vel(self, msg: Twist):
        vx = self._clamp(msg.linear.x, self.max_vx)
        vy = self._clamp(msg.linear.y, self.max_vy)
        wz = self._clamp(msg.angular.z, self.max_wz)

        self._last_cmd_time = self.get_clock().now()
        if self._stopped_by_watchdog:
            self.get_logger().info('收到新的速度指令，接管看门狗停车状态')
            self._stopped_by_watchdog = False

        if not self._send_velocity(vx, vy, wz):
            self._try_reconnect()

    # -------------------------------------------------------------- 看门狗

    def _on_watchdog(self):
        """超过 watchdog_timeout 未收到 /cmd_vel 就主动停车。"""
        if self._last_cmd_time is None:
            if self._ser is None and not self.dry_run:
                self._try_reconnect()
            return

        gap = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if gap > self.watchdog_timeout and not self._stopped_by_watchdog:
            self._stopped_by_watchdog = True
            self.get_logger().warn(
                '看门狗触发：%.2fs 未收到 /cmd_vel，下发零速度' % gap)
            self._send_velocity(0.0, 0.0, 0.0)

    def _try_reconnect(self):
        now = time.time()
        if now - self._last_reconnect_try < self.reconnect_interval:
            return
        self._last_reconnect_try = now
        self._connect()

    # ---------------------------------------------------------------- 服务

    def _on_stop(self, _request, response):
        self._stopped_by_watchdog = True
        ok = self._send_velocity(0.0, 0.0, 0.0)
        response.success = bool(ok)
        response.message = '已下发零速度' if ok else '串口不可用，下发失败'
        return response

    def _publish_status(self, state, last_cmd):
        msg = String()
        msg.data = 'state=%s serial=%s last_cmd=%s' % (
            state, 'open' if (self._ser is not None and self._ser.is_open) else 'closed',
            last_cmd)
        self.status_pub.publish(msg)

    # --------------------------------------------------------------- 退出

    def destroy_node(self):
        # 退出前必定下发零速度：避免底盘保持最后一条速度指令继续行驶
        for _ in range(3):
            if self._write('V,0.000,0.000,0.000'):
                break
            time.sleep(0.1)
        with self._io_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:                     # noqa: BLE001
                    pass
                self._ser = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    run_node(ChassisBridge())


if __name__ == '__main__':
    main()
