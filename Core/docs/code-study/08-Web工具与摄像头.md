# 08 Web 工具与摄像头

## 1. odom_compare_web

位置：

```text
tools/legacy/odom_compare_web/            （已停用，当前手机端出口是 inspector_web）
```

### 1.1 作用

```text
对比轮式里程计 /odom 和激光里程计 /odom_laser
Web 远程控制小车
Web 发送固件串口命令
显示磁力计原始值
```

### 1.2 核心节点

```text
odom_compare_web/odom_compare_web/odom_compare_bridge.py
```

它继承：

```python
from mecanum_bringup.serial_bridge import SerialBridge
```

所以同一个节点：

```text
既能串口收发
又能 Web 命令
又能里程计对比
```

### 1.3 HTTP 接口

```text
/          -> 网页
/data      -> JSON 数据
/send?cmd= -> 发送串口命令
/baseline  -> 设置基准点
/clear     -> 清空轨迹
```

### 1.4 前端

- Canvas 画轨迹
- 红色 = 轮式
- 蓝色 = 激光
- 显示 ΔX / ΔY / ΔYaw / 磁力计

---

## 2. camera_bringup

位置：

```text
tools/ros2_ws/camera_bringup/
```

### 2.1 作用

```text
USB UVC 摄像头
MJPG 采集
发布压缩图 /image_raw/compressed
降低 USB2.0 带宽
```

### 2.2 核心节点

```text
camera_bringup/camera_bringup/mjpeg_camera_node.py
```

关键参数：

```yaml
video_device: /dev/video1
width: 640
height: 480
fps: 30
pixel_format: mjpg
publish_raw: true
```

### 2.3 发布话题

```text
/image_raw/compressed  CompressedImage
/image_raw             Image
/image_raw/camera_info CameraInfo
```

---

## 3. Web 查看

```bash
ros2 run web_video_server web_video_server
```

浏览器：

```text
http://<ip>:8080/stream?topic=/image_raw
```

---

## 4. 常见问题

### 4.1 黑屏

原因：

```text
原始图 QoS 不匹配
```

解决：

```python
self.raw_pub = self.create_publisher(Image, self.image_topic, 10)
```

### 4.2 轨迹消失

原因：

```text
baseline 结构是嵌套的，画图时代码用错
```

解决：

```javascript
baseline.wheel.x
baseline.laser.x
```
