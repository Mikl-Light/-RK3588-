#!/usr/bin/env python3
"""USB 相机图像源：V4L2 -> sensor_msgs/Image

用 OpenCV 直接开 V4L2 设备并发布 bgr8 图像。仅作为「自备图像源」使用——
如果现场已经跑着别的相机节点（本机是 /image_raw），
把 launch 的 start_camera 保持为 false 即可，本节点不会启动。

参数：
  device       /dev/videoX
  fourcc       MJPG（USB 相机上比 YUYV 更容易跑到 30fps）
  width/height/fps
  topic        发布话题
  frame_id     图像坐标系（仅用于标记，不发布 TF）
  read_timeout_ms  单次取帧超时（毫秒）。**不要设成 0**：设备卡死时
                   `VideoCapture.read()` 会永久阻塞，表现是"节点活着、话题有发布者、
                   但一帧都不来、日志里连错误都没有" —— 最难查的一种故障。
  stale_reopen_s   连续多久没拿到新帧就重开设备（0=关闭该保护）

为什么有这些保护（2026-09-13 实锤踩过）：
  1. 相机被上一个进程非正常释放（比如被 kill）后，新进程 `isOpened()` 为真、
     `read()` 却永久阻塞 —— 不打日志、不发帧、不报错。
     ⇒ 必须给 read 设超时，并加"多久没新帧就重开"的看门狗。
  2. `fps` 谈不下来时驱动会给个别的值（本机要 15 给 30），
     ⇒ 回读并打印**实际**协商结果，别让用户以为生效了。
  3. 打不开的最常见原因是**设备被别的进程占用**（另一个相机节点、或上次没退干净的
     自己）⇒ 报错里直接给出 `fuser -v <dev>` 的自查方法。
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from . import run_node

SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class CameraSource(Node):

    def __init__(self):
        super().__init__('camera_source')

        self.declare_parameter('device', '/dev/video1')
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('read_timeout_ms', 2000)
        self.declare_parameter('stale_reopen_s', 5.0)

        self.device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value
        self._fourcc = str(self.get_parameter('fourcc').value)[:4]
        self.read_timeout_ms = int(self.get_parameter('read_timeout_ms').value)
        self.stale_reopen_s = float(self.get_parameter('stale_reopen_s').value)

        self._cap = None
        self._fail_count = 0
        self._last_frame_time = 0.0
        self._frame_count = 0
        self._rate_mark = time.time()
        self._opened_once = False

        self.pub = self.create_publisher(Image, self.get_parameter('topic').value, SENSOR_QOS)
        self.create_timer(1.0 / max(self.fps, 1.0), self._on_timer)

        self._open()
        if self._cap is not None:
            self.get_logger().info('相机源就绪: %s %dx%d @%.0ffps -> %s' % (
                self.device, self.width, self.height, self.fps,
                self.get_parameter('topic').value))
        else:
            self.get_logger().error(
                '相机 %s 当前打不开，会每约 5 秒重试一次。'
                '最常见原因：**设备被别的进程占着** —— 本机跑着另一个相机节点，'
                '或上一次的 camera_source 没退干净。'
                '用 `fuser -v %s` 看是谁占着；'
                '另一个可能是设备号不对（USB 相机常在 /dev/video1，'
                '/dev/video0 往往是板载 HDMI 输入）。' % (self.device, self.device))

    # ------------------------------------------------------------------ 开关

    def _open(self):
        import cv2
        try:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError('无法打开设备（可能被占用 / 设备号不对 / 掉线）')

            # ★ 先设超时：设备卡死时 read() 会永久阻塞，那是"静默死亡"的根源。
            if self.read_timeout_ms > 0 and hasattr(cv2, 'CAP_PROP_READ_TIMEOUT_MSEC'):
                cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.read_timeout_ms)
            if hasattr(cv2, 'CAP_PROP_OPEN_TIMEOUT_MSEC'):
                cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            self._cap = cap
            self._fail_count = 0
            self._last_frame_time = time.time()
            self._rate_mark = time.time()
            self._frame_count = 0
            self._opened_once = True
            self._report_format()
            return True
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error('相机打开失败 (%s): %s' % (self.device, exc))
            self._cap = None
            return False

    def _report_format(self):
        """回读驱动**实际协商**到的格式 —— 很多"能开但不出帧 / 很慢"都是格式没谈成。"""
        import cv2
        cap = self._cap
        if cap is None:
            return
        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = float(cap.get(cv2.CAP_PROP_FPS))
        raw = int(cap.get(cv2.CAP_PROP_FOURCC))
        got_fcc = ''.join(chr((raw >> (8 * i)) & 0xFF) for i in range(4))
        self.get_logger().info(
            '相机实际格式: %dx%d @%.1ffps %s（请求 %dx%d @%.0ffps %s）'
            % (got_w, got_h, got_fps, got_fcc, self.width, self.height, self.fps, self._fourcc))
        if (got_w, got_h) != (self.width, self.height):
            self.get_logger().warn(
                '分辨率没谈成，驱动给的是 %dx%d。用 '
                '`v4l2-ctl -d %s --list-formats-ext` 看相机支持哪些组合。'
                % (got_w, got_h, self.device))
        if self.fps > 0 and got_fps > 0 and abs(got_fps - self.fps) / self.fps > 0.2:
            self.get_logger().warn(
                '帧率没谈成，驱动给的是 %.1ffps（请求 %.0f）。发布节奏仍按请求的 %.0ffps 走，'
                '不会因此丢照片。' % (got_fps, self.fps, self.fps))

    def _release(self, reason):
        self.get_logger().error('%s，重开 %s' % (reason, self.device))
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:                         # noqa: BLE001
                pass
        self._cap = None
        self._fail_count = 0

    # ------------------------------------------------------------------ 主循环

    def _on_timer(self):
        import cv2

        if self._cap is None:
            self._fail_count += 1
            # 约每 5 秒重试一次（1/75 tick），别刷屏
            if self._fail_count % 75 == 1:
                self.get_logger().warn(
                    '相机未就绪，重试打开 %s（`fuser -v %s` 可查是否被占用）'
                    % (self.device, self.device))
                self._open()
            return

        # ★ 看门狗：read() 有超时保护，但驱动也可能"能读、只是永远不返回新帧"。
        if (self.stale_reopen_s > 0 and self._frame_count > 0
                and time.time() - self._last_frame_time > self.stale_reopen_s):
            self._release('已 %.1f 秒没有新帧（疑似相机卡死）'
                          % (time.time() - self._last_frame_time))
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._fail_count += 1
            if self._fail_count >= 5:
                self._release('连续 %d 次取帧失败（read 超时）' % self._fail_count)
            return

        self._fail_count = 0
        self._last_frame_time = time.time()
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        frame = np.ascontiguousarray(frame)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[0], frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

        # 每 10 秒汇报一次真实出帧率，方便发现"能开但很慢"的情况
        self._frame_count += 1
        span = time.time() - self._rate_mark
        if span >= 10.0:
            self.get_logger().info('出帧率 %.1f fps（最近 %.0f 秒 %d 帧）'
                                   % (self._frame_count / span, span, self._frame_count))
            self._frame_count = 0
            self._rate_mark = time.time()

    def destroy_node(self):
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:                         # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    run_node(CameraSource())


if __name__ == '__main__':
    main()
