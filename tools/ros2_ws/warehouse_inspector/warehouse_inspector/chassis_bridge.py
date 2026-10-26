#!/usr/bin/env python3
"""底盘串口桥：/cmd_vel -> 串口 ASCII 指令 "V,<vx>,<vy>,<wz>\\r"
            串口 $T 遥测 -> /odom 与 /imu/imu407/data_raw

设计取舍：
  1. 主职责是「速度指令 -> 底盘命令」单向翻译，外加**带失控看门狗**：
     超过 watchdog_timeout 没有收到 /cmd_vel，就主动下发零速度。
     没有看门狗时，上层节点崩溃 / 卡死会让底盘保持最后一个速度指令继续行驶——
     这在无人巡检场景里是严重安全隐患。
  2. **2026-10-22 起补充遥测解析**（publish_telemetry，默认开）：
     融合节点 odom_fusion 需要固件的轮式 odom（磁绝对航向 / 线速度）与陀螺角速度，
     用来估计零偏、判静止、以及激光退化时兜底。串口只能被一个进程占用，
     所以在桥里就地补齐，避免再起第二个串口进程。
     ⚠ **本节点不发布任何 TF**：`odom_fused -> base_link` 由 fusion_node 统一发布，
       若这里也发 `odom_wheel -> base_link` 会与 rf2o / fusion 抢同一棵 TF 树的父节点。
  3. 串口断开自动重连；进程退出前必定下发零速度。
  4. 速度做硬限幅，防止上层发出超限指令。

坐标约定（与固件一致）：
  vx > 0 = 前进（车头方向）, vy > 0 = 左移, wz > 0 = 逆时针（左转）

⚠ /odom 的位置是**固件世界系**（X=北 / Y=西），与激光系（+x=车头）相差约 90°。
  该位姿只作为融合节点的辅助输入，**不可直接与 /odom_laser 按分量加权**
  （fusion_node 已改为只累加激光增量，位置不再引用本节点的 x/y）。
"""

import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
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
        # ── 遥测解析（2026-10-22 新增，供 odom_fusion 使用）──
        self.declare_parameter('publish_telemetry', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/imu407/data_raw')
        self.declare_parameter('odom_frame', 'odom_wheel')   # 仅作消息 header，**不发 TF**
        self.declare_parameter('base_frame', 'base_link')

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
        self._rx_thread = None
        self._telemetry_on_sent = False

        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.publish_telemetry = bool(self.get_parameter('publish_telemetry').value)
        if self.publish_telemetry:
            self.odom_pub = self.create_publisher(
                Odometry, self.get_parameter('odom_topic').value, 10)
            self.imu_pub = self.create_publisher(
                Imu, self.get_parameter('imu_topic').value, 10)

        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd_vel, 10)
        self.create_service(Trigger, '~/stop', self._on_stop)

        rate = float(self.get_parameter('watchdog_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._on_watchdog)

        self._connect()
        if self.publish_telemetry:
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
        self.get_logger().info(
            '底盘桥就绪: %s @ %d, 限幅 vx=%.2f vy=%.2f wz=%.2f, 看门狗 %.1fs%s%s' % (
                self.port, self.baudrate, self.max_vx, self.max_vy, self.max_wz,
                self.watchdog_timeout,
                '  [DRY-RUN 不真正发串口]' if self.dry_run else '',
                '  遥测->/odom+/imu' if self.publish_telemetry else '  遥测关闭'))

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
            if self.publish_telemetry:
                # 固件默认不发 $T，必须显式开启（与 mecanum_bringup/serial_bridge 同做法）
                self._telemetry_on_sent = self._write('TELEM ON')
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

    # -------------------------------------------------------------- 遥测解析
    #
    # $T 帧字段（Core/Src/telemetry.c，30 字段 ASCII，50Hz）：
    #   [1]=tick  [2..5]=编码器增量  [6..8]=加速度(mg)  [9..11]=陀螺(0.01°/s)
    #   [12..14]=磁场(0.01µT)  [15]=x(mm) [16]=y(mm) [17]=航向(0.1°)
    #   [18..20]=vx(mm/s) vy(mm/s) wz(mrad/s)  [21..24]=四轮 rpm(0.1)
    # 位置是**固件世界系**（X=北 / Y=西），航向是罗盘角（0=北，顺时针）。

    def _rx_loop(self):
        """后台线程：读串口 -> 解析 $T -> 发布 /odom 与 /imu。

        与 `_write` 共用同一个 Serial 对象。pyserial 的 read/write 内部各自加锁，
        可以并发；这里刻意不拿 `_io_lock`，否则读操作（timeout 0.05s）会阻塞写指令。
        """
        buf = b''
        while self._running:
            ser = self._ser
            if ser is None or not ser.is_open:
                time.sleep(0.2)
                self._try_reconnect()
                continue
            try:
                chunk = ser.read(4096)
            except Exception:                          # noqa: BLE001
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.strip().decode('ascii', 'replace')
                if line.startswith('$T,'):
                    try:
                        self._publish_telemetry(line)
                    except Exception as exc:           # noqa: BLE001
                        self.get_logger().warn('遥测解析异常: %s' % exc,
                                               throttle_duration_sec=5.0)
            if len(buf) > 16384:                       # 防止无换行垃圾撑爆内存
                buf = b''

    def _publish_telemetry(self, line):
        parts = line.split(',')
        if len(parts) < 25:
            return
        try:
            accel_mg = [int(parts[i]) for i in range(6, 9)]
            gyro_cdps = [int(parts[i]) for i in range(9, 12)]
            px, py = int(parts[15]) / 1000.0, int(parts[16]) / 1000.0
            heading_deg = int(parts[17]) / 10.0
            vx = int(parts[18]) / 1000.0
            vy = int(parts[19]) / 1000.0
            wz = int(parts[20]) / 1000.0
        except (ValueError, IndexError):
            return

        now = self.get_clock().now().to_msg()

        # --- /odom：固件 EKF 位姿（磁绝对航向 + 轮式）---
        # 🔴 航向符号：**不取负**（2026-10-22 19:18 修正，此前 `-heading_deg` 是错的）
        #   固件 `AHRS_GetHeading()` 返回的本来就是「逆时针为正」的角，`odometry.c` 中
        #   就是 `X[2] = +heading_deg`。旧文档里的「ROS yaw = −heading」是 10-11 已废弃
        #   的口径，这里照抄了它，导致 /odom 的 yaw 与真实旋转方向**相反**。
        #   实锤（原地逆时针转一圈，三路累计）：
        #     laser(rf2o) +397.9° / wheel(本行) −405.6° / fused +358.4°
        #     → 前两者方向应相同，wheel 却为负；由 0.95·397.9 + 0.05·(−405.6) = 357.7
        #       正好等于 fused 的 358.4°，证明融合一直在吃这个反号。
        #   ⚠ 之前「直线实测 +0.09° 验证过」对这个 bug **无效**：直线时 yaw 几乎不变，
        #     符号反不反都看不出来。只有整体旋转才能暴露。
        #   注意：下方 px/py 仍是**固件世界系**（X=北 / Y=西），未做 ROS 换算；
        #     融合节点只用本话题的 yaw 增量，不用其位置，所以暂不动位置（见 MEMORY 遗留项）。
        yaw_ros = math.radians(heading_deg)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.orientation.z = math.sin(yaw_ros / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw_ros / 2.0)
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 0.01
            odom.twist.covariance[i] = 0.001
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # --- /imu：融合节点只用 angular_velocity.z（零偏估计 / 静止判据）---
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu407_link'
        imu.linear_acceleration.x = accel_mg[0] * 9.80665 / 1000.0
        imu.linear_acceleration.y = accel_mg[1] * 9.80665 / 1000.0
        imu.linear_acceleration.z = accel_mg[2] * 9.80665 / 1000.0
        imu.angular_velocity.x = math.radians(gyro_cdps[0] / 100.0)
        imu.angular_velocity.y = math.radians(gyro_cdps[1] / 100.0)
        imu.angular_velocity.z = math.radians(gyro_cdps[2] / 100.0)
        for i in (0, 4, 8):                            # 对角元 0,7,14 -> 展平后 0,4,8
            imu.orientation_covariance[i] = 0.0
            imu.angular_velocity_covariance[i] = 0.0005
            imu.linear_acceleration_covariance[i] = 0.01
        imu.orientation_covariance[0] = -1.0           # 不提供姿态解算
        self.imu_pub.publish(imu)

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
        self._running = False
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
