#!/usr/bin/env python3
"""
serial_bridge.py — STM32F407 (serial) <-> ROS2 (humble) 桥

职责:
  1. 解析 407 串口数据 (自动识别两种格式):
     - 二进制帧: 0xAA 0x55 + type + payload_len + timestamp + enc_raw[4]i32
                 + accel/gyro/mag_raw[3]i16 + pose[3]f + twist[3]f
                 + wheel_speed[4]f + crc8  (共 84 字节, 见 telemetry.c)
     - ASCII 帧:  $T,... (23 字段) 与 $STAT,...  (调试用)
  2. 发布话题:
     /odom                    Odometry        融合位姿+速度 (坐标系可配)
     /imu/imu407/data_raw     Imu            原始 int16 -> SI (RK3588 对比用)
     /imu/imu407/attitude     Imu            407 融合航向 (yaw, roll/pitch=0)
     /imu/imu407/mag          MagneticField  磁力计 (估算 uT, 未校准)
     /wheel_encoders          Int32MultiArray 编码器原始 CNT
     /wheel_speeds            Float32MultiArray 轮速 RPM
     /stm32_tick              Int32          407 内部 tick(ms) — 离线对齐时间基准
     /chassis_status          String         $STAT 原始行
  3. 订阅:
     /cmd_vel (Twist) -> V,<Vx>,<Vy>,<Wz>   (固件已 ROS 化: Vx=前 Vy=左, 直通)
  4. 服务:
     /stop_chassis (Trigger) -> STOP

坐标系 (2026-08-12 固件已按 ROS 约定):
  固件机器人系: X=前, Y=左, Wz=逆时针正 — 与 ROS 完全一致, 命令/数据直通
  固件世界系:   X=北, Y=西, heading=罗盘角(0=北,顺时针) → ROS yaw = -heading

用法:
  ros2 run <pkg> serial_bridge.py --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=460800
依赖: geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
"""

import rclpy
from rclpy.node import Node

import serial
import threading
import time
import math
import struct

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# ---------------------------------------------------------------- 帧定义
BIN_HEADER = b'\xAA\x55'
BIN_TYPE = 0x01
BIN_PAYLOAD_LEN = 81      # type..wheel_speed 字节数
BIN_FRAME_SIZE = 84

# 407 传感器量程 (与 mpu9250.c MPU9250_Init 一致)
ACCEL_LSB_PER_G = 16384.0   # ±2g
GYRO_LSB_PER_DPS = 131.0    # ±250dps
MAG_LSB_PER_UT = 0.15       # AK8963 16bit 标称


def crc8(data: bytes) -> int:
    """CRC-8, 多项式 0x07, 初值 0, 无反射 (与 telemetry.c 一致)"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_binary(buf: bytes):
    """解析 84 字节二进制帧 -> dict; 校验失败返回 None"""
    if len(buf) < BIN_FRAME_SIZE:
        return None
    if buf[0:2] != BIN_HEADER or buf[2] != BIN_TYPE:
        return None
    (plen,) = struct.unpack_from('<H', buf, 3)
    if plen != BIN_PAYLOAD_LEN:
        return None
    if buf[-1] != crc8(buf[2:2 + BIN_PAYLOAD_LEN]):
        return None

    (tick,) = struct.unpack_from('<I', buf, 5)
    enc_raw = list(struct.unpack_from('<4i', buf, 9))
    accel_raw = list(struct.unpack_from('<3h', buf, 25))
    gyro_raw = list(struct.unpack_from('<3h', buf, 31))
    mag_raw = list(struct.unpack_from('<3h', buf, 37))
    pose = list(struct.unpack_from('<3f', buf, 43))     # x, y, yaw(rad)
    twist = list(struct.unpack_from('<3f', buf, 55))    # vx, vy, wz
    wheel = list(struct.unpack_from('<4f', buf, 67))    # RPM
    return dict(tick=tick, enc_raw=enc_raw, accel_raw=accel_raw,
                gyro_raw=gyro_raw, mag_raw=mag_raw,
                pose=pose, twist=twist, wheel=wheel)


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # ---------- 参数 ----------
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('log_status', True)      # 打印 $STAT/OK/ERR 行(调试)
        self.declare_parameter('auto_telem_on', True)   # 打开串口后自动发 TELEM,ON

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.pub_odom = self.get_parameter('publish_odom').value
        self.pub_imu = self.get_parameter('publish_imu').value
        self.log_status = self.get_parameter('log_status').value
        # ---------- 发布器 ----------
        if self.pub_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.tf_broadcaster = TransformBroadcaster(self)
        if self.pub_imu:
            self.imu_raw_pub = self.create_publisher(Imu, '/imu/imu407/data_raw', 10)
            self.imu_att_pub = self.create_publisher(Imu, '/imu/imu407/attitude', 10)
            self.mag_pub = self.create_publisher(MagneticField, '/imu/imu407/mag', 10)
        self.enc_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.tick_pub = self.create_publisher(Int32, '/stm32_tick', 10)
        self.status_pub = self.create_publisher(String, '/chassis_status', 10)

        # ---------- 订阅/服务 ----------
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.stop_srv = self.create_service(Trigger, '/stop_chassis', self.stop_cb)

        # ---------- 串口 ----------
        self.ser = None
        self.running = True
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(
            f'SerialBridge: {self.port}@{self.baudrate}, '
            f'odom={self.frame_id}->{self.child_frame_id}')

    # ================================================================ 串口
    def open_serial(self):
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.get_logger().info(f'Opened {self.port}@{self.baudrate}')
            # 自动开启 407 遥测 ($T 50Hz): 否则桥只收 $STAT, 无 /odom 与 TF
            if self.get_parameter('auto_telem_on').value:
                self.send('TELEM,ON')
            return True
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f'Open {self.port} failed: {e}')
            return False

    def send(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\r').encode())
                self.get_logger().debug(f'TX: {cmd}')
            except Exception as e:
                self.get_logger().warn(f'Write failed: {e}')
        else:
            self.get_logger().warn('Serial not open, TX dropped')

    def rx_loop(self):
        """接收循环: 二进制按帧同步(0xAA55+CRC), ASCII 按行; 自动重连"""
        bin_buf = bytearray()
        ascii_buf = bytearray()
        while self.running:
            if not self.open_serial():
                time.sleep(1.0)
                continue
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                for b in chunk:
                    # --- 二进制帧同步: 0xAA 0x55 ---
                    if len(bin_buf) > 0 or b == BIN_HEADER[0]:
                        bin_buf.append(b)
                        if len(bin_buf) >= BIN_FRAME_SIZE:
                            data = unpack_binary(bytes(bin_buf))
                            if data is not None:
                                self.publish_binary(data)
                            bin_buf.clear()   # 无论校验成败都丢弃(重新同步)
                        elif len(bin_buf) > BIN_FRAME_SIZE:
                            bin_buf.clear()
                    # --- ASCII 行 ---
                    elif b in (0x0D, 0x0A):
                        if ascii_buf:
                            self.process_line(bytes(ascii_buf).decode('ascii', 'ignore'))
                            ascii_buf.clear()
                    else:
                        if b < 0x20 or b > 0x7E:   # 非打印字符(二进制残留) -> 丢弃该行
                            ascii_buf.clear()
                        elif len(ascii_buf) < 512:
                            ascii_buf.append(b)
            except Exception as e:
                if not self.running:
                    break   # 正常退出中 (Ctrl+C), 不报错
                self.get_logger().error(f'Serial read error: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(0.5)
        self.get_logger().info('RX loop ended')

    # ================================================================ 解析
    def process_line(self, line: str):
        if not line:
            return
        if line.startswith(('OK', 'ERR', 'ERR ')):
            if self.log_status:
                self.get_logger().info(f'< {line}')
            return
        if line.startswith('$T'):
            self.parse_ascii_telemetry(line)
        elif line.startswith('$STAT'):
            msg = String()
            msg.data = line
            self.status_pub.publish(msg)
            if self.log_status:
                self.get_logger().info(f'< {line}')

            # 解析 $STAT 位姿并发布 odom/TF:
            # $STAT,<state>,HDG=<deg>,X=<m>,Y=<m>,DIST=<m>,REM=<m>,Vx=<mps>,Vy=<mps>,Wz=<radps>,M1..M4
            data = {}
            for part in line.split(',')[1:]:
                if '=' in part:
                    k, v = part.split('=', 1)
                    data[k] = v
            try:
                x = float(data.get('X', 0.0))
                y = float(data.get('Y', 0.0))
                hdg = float(data.get('HDG', 0.0))     # 度 (罗盘角)
                vx = float(data.get('Vx', 0.0))
                vy = float(data.get('Vy', 0.0))
                wz = float(data.get('Wz', 0.0))
            except ValueError:
                return
            self.publish_pose(x, y, math.radians(hdg), vx, vy, wz)
        else:
            self.get_logger().debug(f'Unknown: {line}')

    def parse_ascii_telemetry(self, line: str):
        """$T 帧 -> 与二进制帧相同的结构, 走同一套发布逻辑 (调试模式可用)"""
        parts = line.split(',')
        if len(parts) < 25:
            return
        try:
            tick = int(parts[1])
            enc_delta = [int(parts[i]) for i in range(2, 6)]
            accel_mg = [int(parts[i]) for i in range(6, 9)]
            gyro_dps100 = [int(parts[i]) for i in range(9, 12)]
            mag_ut100 = [int(parts[i]) for i in range(12, 15)]
            pose_mm = [int(parts[15]), int(parts[16])]
            heading_ddeg = int(parts[17])
            vx_mmps, vy_mmps, wz_mradps = int(parts[18]), int(parts[19]), int(parts[20])
            wheel_rpm10 = [int(parts[i]) for i in range(21, 25)]
        except ValueError:
            return

        data = {
            'tick': tick,
            'enc_raw': enc_delta,                    # ASCII 帧只有增量, 无原始 CNT
            'accel_raw': [int(round(mg * ACCEL_LSB_PER_G / 1000.0)) for mg in accel_mg],
            'gyro_raw': [int(round(d * GYRO_LSB_PER_DPS / 100.0)) for d in gyro_dps100],
            'mag_raw': [int(round(u * 100.0 / MAG_LSB_PER_UT)) for u in mag_ut100],
            'pose': [pose_mm[0] / 1000.0, pose_mm[1] / 1000.0,
                     math.radians(heading_ddeg / 10.0)],
            'twist': [vx_mmps / 1000.0, vy_mmps / 1000.0, wz_mradps / 1000.0],
            'wheel': [r / 10.0 for r in wheel_rpm10],
        }
        self.publish_binary(data)

    # ================================================================ 发布
    def publish_binary(self, d: dict):
        now = self.get_clock().now().to_msg()

        # 407 tick -> /stm32_tick (离线对齐时间基准)
        tick_msg = Int32()
        tick_msg.data = int(d['tick'])
        self.tick_pub.publish(tick_msg)

        # 编码器 + 轮速
        enc = Int32MultiArray()
        enc.data = d['enc_raw']
        self.enc_pub.publish(enc)
        spd = Float32MultiArray()
        spd.data = [float(x) for x in d['wheel']]
        self.speed_pub.publish(spd)

        if self.pub_imu:
            # --- /imu/imu407/data_raw: 原始 int16 -> SI ---
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu407_link'
            imu.linear_acceleration.x = d['accel_raw'][0] / ACCEL_LSB_PER_G * 9.80665
            imu.linear_acceleration.y = d['accel_raw'][1] / ACCEL_LSB_PER_G * 9.80665
            imu.linear_acceleration.z = d['accel_raw'][2] / ACCEL_LSB_PER_G * 9.80665
            imu.angular_velocity.x = d['gyro_raw'][0] / GYRO_LSB_PER_DPS * math.pi / 180.0
            imu.angular_velocity.y = d['gyro_raw'][1] / GYRO_LSB_PER_DPS * math.pi / 180.0
            imu.angular_velocity.z = d['gyro_raw'][2] / GYRO_LSB_PER_DPS * math.pi / 180.0
            self.imu_raw_pub.publish(imu)

            # --- /imu/imu407/attitude: 407 融合航向 (只有 yaw) ---
            att = Imu()
            att.header.stamp = now
            att.header.frame_id = 'imu407_link'
            yaw = self.to_ros_yaw(d['pose'][2])
            att.orientation = self.yaw_to_quat(yaw)
            self.imu_att_pub.publish(att)

            # --- /imu/imu407/mag: 磁力计估算 uT (未校准) ---
            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = 'imu407_link'
            mag.magnetic_field.x = d['mag_raw'][0] * MAG_LSB_PER_UT
            mag.magnetic_field.y = d['mag_raw'][1] * MAG_LSB_PER_UT
            mag.magnetic_field.z = d['mag_raw'][2] * MAG_LSB_PER_UT
            self.mag_pub.publish(mag)

        x, y, yaw_407 = d['pose']
        vx, vy, wz = d['twist']
        self.publish_pose(x, y, yaw_407, vx, vy, wz)

    def publish_pose(self, x, y, yaw_407, vx, vy, wz):
        """发布 /odom 消息 + odom->base_link TF (slam_toolbox/导航需要)
        yaw_407: 罗盘角 (弧度, 0=北 顺时针); ROS yaw = -yaw_407
        $T/$STAT/二进制帧 都会调用, 保证 TF 链始终存在"""
        if not self.pub_odom:
            return
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x       # 固件世界 X=北 (与 yaw=0 同向)
        odom.pose.pose.position.y = y       # 固件世界 Y=西
        odom.pose.pose.orientation = self.yaw_to_quat(self.to_ros_yaw(yaw_407))
        odom.pose.covariance = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[i] = 0.01

        odom.twist.twist.linear.x = vx       # 前 (固件已 ROS 化, 直通)
        odom.twist.twist.linear.y = vy       # 左
        odom.twist.twist.angular.z = wz      # 逆时针+
        odom.twist.covariance = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[i] = 0.001

        self.odom_pub.publish(odom)

        # odom -> base_link TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.frame_id        # odom
        tf_msg.child_frame_id = self.child_frame_id   # base_link
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def to_ros_yaw(self, yaw_407: float) -> float:
        """407 heading 为罗盘角 (0=北, 顺时针); ROS yaw 逆时针正 = -heading"""
        yaw = -yaw_407
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi
        return yaw

    @staticmethod
    def yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ================================================================ 控制
    def cmd_vel_cb(self, msg: Twist):
        """固件已按 ROS 约定 (Vx=前, Vy=左, Wz=逆时针正): 直接透传"""
        vx = max(-1.0, min(1.0, msg.linear.x))
        vy = max(-1.0, min(1.0, msg.linear.y))
        wz = max(-3.0, min(3.0, msg.angular.z))
        self.send(f'V,{vx:.3f},{vy:.3f},{wz:.3f}')

    def stop_cb(self, request, response):
        self.send('STOP')
        response.success = True
        response.message = 'STOP sent'
        return response

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
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
