# 项目代码学习指南

> 目标：从零读懂 STM32F407 固件、Bootloader、ROS2 上位机全部核心代码
> 路径：`C:\dosk\STM32\V1`

---

## 学习顺序

```text
第 1 步：系统架构与启动流程
第 2 步：STM32 外设驱动
第 3 步：控制与算法（PID、麦轮、里程计、AHRS）
第 4 步：通信与遥测、OTA
第 5 步：Bootloader
第 6 步：ROS2 串口桥
第 7 步：ROS2 建图与导航
第 8 步：Web 工具与摄像头
```

---

## 文档索引

| 编号 | 文档 | 内容 |
|---|---|---|
| 01 | `01-系统架构与启动流程.md` | 系统框图、STM32 启动流程、Bootloader 跳转 |
| 02 | `02-STM32外设驱动.md` | 定时器、编码器、PWM、串口、I2C |
| 03 | `03-控制与算法.md` | PID、麦轮运动学、里程计、AHRS |
| 04 | `04-通信与遥测OTA.md` | 串口协议、遥测帧、OTA |
| 05 | `05-Bootloader.md` | Bootloader 启动、升级协议、跳转 |
| 06 | `06-ROS2串口桥.md` | serial_bridge 数据流、话题、QoS |
| 07 | `07-ROS2建图与导航.md` | 建图、rf2o、slam_toolbox、Nav2 |
| 08 | `08-Web工具与摄像头.md` | odom_compare_web、camera_bringup |

---

## 核心代码文件

### STM32 App

```text
Core/Src/main.c           入口、主循环、串口命令解析
Core/Src/motor.c          电机驱动：编码器、PWM、PID
Core/Src/chassis.c        麦轮运动学、底盘状态机
Core/Src/odometry.c       里程计：轮速 -> 位姿
Core/Src/ahrs.c           航向：陀螺仪/磁力计融合
Core/Src/mpu9250.c        IMU 驱动：MPU9250 + AK8963
Core/Src/uart_io.c        串口收发、环形缓冲区
Core/Src/telemetry.c      遥测帧：ASCII + 二进制
Core/Src/ota.c            OTA 升级命令
Core/Src/pid.c            PID 控制器
```

### Bootloader

```text
tools/Bootloader/Core/Src/main.c          Bootloader 入口
tools/Bootloader/Core/Src/bootloader.c    Flash 擦写、升级协议、跳转 App
```

### ROS2

当前在用（`tools/ros2_ws/`）：

```text
tools/ros2_ws/warehouse_inspector/warehouse_inspector/chassis_bridge.py   串口桥（当前主桥）
tools/ros2_ws/warehouse_inspector/warehouse_inspector/scan_filter.py      雷达过滤
tools/ros2_ws/warehouse_inspector/warehouse_inspector/map_despeckle.py    地图除噪
tools/ros2_ws/warehouse_inspector/launch/inspection.launch.py             主 launch
tools/ros2_ws/odom_fusion/odom_fusion/fusion_node.py                      融合 odom
tools/ros2_ws/camera_bringup/camera_bringup/mjpeg_camera_node.py          USB 相机
```

旧方案（`tools/legacy/`，已停用，仅供对照）：

```text
tools/legacy/mecanum_bringup/mecanum_bringup/serial_bridge.py
tools/legacy/mecanum_bringup/mecanum_bringup/keyboard_teleop.py
tools/legacy/mecanum_bringup_hector/mecanum_bringup_hector/map_trigger.py
tools/legacy/mecanum_bringup_hector/mecanum_bringup_hector/goal_pose_bridge.py
tools/legacy/odom_compare_web/odom_compare_web/odom_compare_bridge.py
```

---

## 建议

1. 先读 `01-系统架构与启动流程.md`。
2. 再按主循环调用顺序读固件代码。
3. ROS2 部分先读 `serial_bridge.py`，再读 launch。
4. 每个模块都要能回答：**这个文件负责什么？数据从哪里来？到哪里去？**
