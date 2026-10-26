# camera_bringup — DH-200W-X1 USB 相机 ROS2 节点（MJPG 优化版）

> 适用：RK3588 + Ubuntu 24.04 + ROS 2 Jazzy（2024.04）  
> 摄像头：DH-200W-X1（USB2.0 UVC，MJPG / YUYV，1080p30）  
> 目标：在 USB2.0 带宽限制下稳定出图，为路标识别 / 车道保持提供低带宽图像流

---

## 1. 为什么不用官方 v4l2_camera

- 官方 `v4l2_camera_node` 对 MJPG 支持不完整，直接输出 MJPG 会崩溃；
- 用 YUYV 时 USB2.0 带宽压力大，1080p 很难稳定 30fps；
- 本节点直接以 **MJPG 采集 + JPEG 压缩发布**，只发布 `CompressedImage`，带宽远小于 YUYV 原始图像。

---

## 2. 安装依赖

```bash
sudo apt update
sudo apt install python3-opencv ros-jazzy-cv-bridge
```

---

## 3. 编译

把 `tools/ros2_ws/camera_bringup` 整个目录拷到 `~/ros2_ws/src/`：

```bash
cd ~/ros2_ws
colcon build --packages-select camera_bringup
source install/setup.bash
```

---

## 4. 启动相机

### 4.1 默认参数（640x480 @30，MJPG）

```bash
ros2 run camera_bringup mjpeg_camera_node
```

### 4.2 指定设备（你的摄像头可能是 /dev/video1）

```bash
ros2 run camera_bringup mjpeg_camera_node --ros-args \
    -p video_device:=/dev/video1 \
    -p width:=640 -p height:=480 -p fps:=30
```

### 4.3 用 launch 启动

```bash
ros2 launch camera_bringup camera.launch.py \
    video_device:=/dev/video1 \
    width:=1280 height:=720 fps:=30
```

### 4.4 如果需要原始图像话题

默认只发压缩图，如果调试时需要 `/image_raw`：

```bash
ros2 run camera_bringup mjpeg_camera_node --ros-args \
    -p video_device:=/dev/video1 \
    -p publish_raw:=true
```

> 注意：`publish_raw:=true` 会增加带宽，正式跑路标识别时建议关闭。

---

## 5. 查看图像

### 5.1 网页查看（推荐，电脑不用装 ROS2）

先启动 web_video_server：

```bash
sudo apt install ros-jazzy-web-video-server
ros2 run web_video_server web_video_server
```

然后浏览器打开：

```text
http://<RK3588_IP>:8080/stream?topic=/image_raw
```

`<RK3588_IP>` 用下面命令查：

```bash
hostname -I
```

### 5.2 用 rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

### 5.3 用 rviz2

```bash
rviz2
```

Add → By topic → `/image_raw` → Camera。

---

## 6. 话题说明

| 话题 | 类型 | 默认 |
|---|---|---|
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | ✅ 发布 |
| `/image_raw` | `sensor_msgs/Image` | ❌ 默认关闭 |
| `/image_raw/camera_info` | `sensor_msgs/CameraInfo` | ✅ 发布 |

---

## 7. 后续路标识别 / 车道保持怎么接

在 ROS2 里订阅压缩图并解码即可，例如 Python：

```python
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_cb(msg):
    frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # 在这里做路标识别 / 车道线检测
    cv2.imshow('camera', frame)
```

或者用 `image_transport` 直接订阅：

```bash
ros2 run image_transport republish compressed in/raw \
    --ros-args -r in/compressed:=/image_raw/compressed -r out/raw:=/image_raw
```

---

## 8. 参数说明

| 参数 | 默认 | 说明 |
|---|---|---|
| `video_device` | `/dev/video0` | 摄像头设备 |
| `width` | 640 | 采集宽度 |
| `height` | 480 | 采集高度 |
| `fps` | 30 | 目标帧率 |
| `pixel_format` | `mjpg` | 优先 MJPG，失败自动退 YUYV |
| `image_topic` | `image_raw` | 图像话题名前缀 |
| `frame_id` | `camera_link` | TF frame |
| `publish_compressed` | true | 发布压缩图 |
| `publish_raw` | false | 是否发布原始图 |
| `jpeg_quality` | 85 | JPEG 质量 1-100 |
| `rotate` | 0 | 旋转 0/90/180/270 |

---

## 9. 推荐分辨率

| 场景 | 分辨率 | 帧率 |
|---|---|---|
| 路标识别 / 车道保持 | 640x480 | 30 |
| 需要更远识别 | 1280x720 | 30 |
| 大范围建图/记录 | 1920x1080 | 30（MJPG 下带宽允许） |

USB2.0 下建议优先使用 **640x480 或 1280x720**。
