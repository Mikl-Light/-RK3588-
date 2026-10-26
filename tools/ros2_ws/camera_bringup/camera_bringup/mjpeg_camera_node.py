#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mjpeg_camera_node.py — USB UVC MJPG 相机 ROS2 节点

专门针对 DH-200W-X1 (USB2.0, MJPG/YUYV, 1080p30) 设计：
  - 优先使用 MJPG 采集，避免 YUYV 在 USB2.0 下带宽过大导致帧率不稳
  - 默认只发布 /image_raw/compressed (sensor_msgs/CompressedImage)
  - 可选发布 /image_raw (sensor_msgs/Image) 用于调试，但会增大带宽
  - 默认 640x480@30，带宽小、延迟低，适合路标识别/车道保持
  - 后续需要高分辨率可调 1280x720 或 1920x1080

用法:
  ros2 run camera_bringup mjpeg_camera_node
  ros2 run camera_bringup mjpeg_camera_node --ros-args \
      -p video_device:=/dev/video1 \
      -p width:=1280 -p height:=720 -p fps:=30

话题:
  /image_raw/compressed : sensor_msgs/CompressedImage   (默认发布)
  /image_raw            : sensor_msgs/Image             (可选)
  /image_raw/camera_info: sensor_msgs/CameraInfo
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge


class MjpegCameraNode(Node):
    def __init__(self):
        super().__init__('mjpeg_camera_node')

        # ---------------- 参数 ----------------
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('pixel_format', 'mjpg')   # mjpg / yuyv
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('rotate', 0)              # 0/90/180/270

        self.video_device = self.get_parameter('video_device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.pixel_format = self.get_parameter('pixel_format').value.lower()
        self.image_topic = self.get_parameter('image_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_name = self.get_parameter('camera_name').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.publish_raw = self.get_parameter('publish_raw').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.rotate = self.get_parameter('rotate').value

        self.bridge = CvBridge()

        # ---------------- 发布器 ----------------
        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage,
                f'{self.image_topic}/compressed',
                qos_profile_sensor_data)
            self.get_logger().info(
                f'Publishing compressed: /{self.image_topic}/compressed')

        if self.publish_raw:
            # web_video_server / rqt_image_view 默认用 RELIABLE 订阅原始图，
            # 因此 raw 发布器不能再用 BEST_EFFORT，否则对方收不到。
            self.raw_pub = self.create_publisher(
                Image,
                self.image_topic,
                10)
            self.get_logger().info(
                f'Publishing raw: /{self.image_topic} (reliable QoS)')

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            f'{self.image_topic}/camera_info',
            qos_profile_sensor_data)

        # ---------------- 打开相机 ----------------
        self.cap = None
        self.open_camera()

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera {self.video_device}. '
                f'Check video_device and permissions.')
            return

        self.get_logger().info(
            f'Camera opened: {self.video_device} '
            f'{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x'
            f'{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ '
            f'{self.cap.get(cv2.CAP_PROP_FPS):.0f} FPS')

        # ---------------- 定时发布 ----------------
        self.timer = self.create_timer(1.0 / self.fps, self.timer_cb)

    # ================================================================
    def open_camera(self):
        """优先 MJPG，失败再退 YUYV"""
        if self.pixel_format in ('mjpg', 'mjpeg'):
            ok = self.try_open_fourcc('MJPG')
            if not ok:
                self.get_logger().warn(
                    'MJPG open failed, falling back to YUYV')
                self.try_open_fourcc('YUYV')
        else:
            self.try_open_fourcc('YUYV')

    def try_open_fourcc(self, fourcc: str) -> bool:
        try:
            cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
            if not cap.isOpened():
                return False

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            # 减少缓存，降低延迟
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # 确认一下最终格式
            actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
            actual_fourcc_str = ''.join([
                chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)
            ])
            self.get_logger().info(
                f'Requested {fourcc}, actual fourcc={actual_fourcc_str}')

            self.cap = cap
            return True
        except Exception as e:
            self.get_logger().error(f'open {fourcc} error: {e}')
            return False

    # ================================================================
    def timer_cb(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn('Camera not opened, skip frame')
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to read frame')
            return

        if self.rotate != 0:
            if self.rotate == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif self.rotate == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        stamp = self.get_clock().now().to_msg()

        if self.publish_compressed:
            ok, buf = cv2.imencode(
                '.jpg',
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.format = 'jpeg'
                msg.data = np.array(buf).tobytes()
                self.compressed_pub.publish(msg)

        if self.publish_raw:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                self.raw_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'raw publish failed: {e}')

        self.publish_camera_info(stamp)

    # ================================================================
    def publish_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = frame_h = self.height
        msg.width = frame_w = self.width
        if self.rotate in (90, 270):
            msg.height, msg.width = frame_w, frame_h
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # 未标定，给一个粗略单位矩阵内参，后续可替换为标定结果
        msg.k = [
            float(msg.width), 0.0, float(msg.width) / 2.0,
            0.0, float(msg.width), float(msg.height) / 2.0,
            0.0, 0.0, 1.0,
        ]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            float(msg.width), 0.0, float(msg.width) / 2.0, 0.0,
            0.0, float(msg.width), float(msg.height) / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.camera_info_pub.publish(msg)

    # ================================================================
    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MjpegCameraNode()
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
