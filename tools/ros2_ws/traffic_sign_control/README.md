# traffic_sign_control — 交通标志 AI 识别 → 麦轮底盘动作执行

独立 ROS2 控制包（ROS2 Jazzy / Ubuntu 24.04 / RK3588 / STM32F407 麦轮底盘）。
控制方式与 `tools/legacy/mecanum_bringup_hector/keyboard_teleop.py` 完全一致：发布 `/cmd_vel`，
由串口桥转发给 STM32 底盘（**现在是 `warehouse_inspector/chassis_bridge`**，
旧的 `serial_bridge` 已停用）。本包不修改、不依赖其他任何包。

## 工作原理

```
手机摄像头 → scrcpy → /dev/video3 → rknn_infer_camera.py（AI 识别，板端）
                                            │  UDP 发送 "类别 置信度"
                                            ▼
                              sign_control_node（本包）
                                            │  按类别执行动作
                                            ▼
                                     /cmd_vel (Twist)
                                            │  串口桥转发
                                            ▼
                                   STM32F407 麦轮底盘
```

## 动作表（速度与 keyboard_teleop.py 初始速度完全一致）

键盘控制节点初始速度：`linear_vel=0.3 m/s`（前进）/ `linear_y_vel=0.3 m/s`（横移）/ `angular_vel=0.8 rad/s`（旋转）。

| 识别类别 | 动作 | 轴 | 方向 | 速度 | 时长 |
|---|---|---|---|---|---|
| `no-right` | 右转 90° | angular.z | 负（顺时针） | 0.8 rad/s | ≈1.96s |
| `no-left` | 左转 90° | angular.z | 正（逆时针） | 0.8 rad/s | ≈1.96s |
| `speed-limit-40` | 左转 180° | angular.z | 正 | 0.8 rad/s | ≈3.93s |
| `end-speed-limit-40` | 右转 180° | angular.z | 负 | 0.8 rad/s | ≈3.93s |
| `Car-no-p` | 前进 1m | linear.x | 正 | 0.3 m/s | ≈3.33s |
| `Car-p` | 右横移 1m | linear.y | 负（右） | 0.3 m/s | ≈3.33s |

### 与键盘控制节点指令一致性对照

本节点发布的每条 Twist，字段与符号均与键盘节点 `compute_twist()` 输出的**同键位指令完全一致**：

| 键盘键 | 键盘节点输出 | 本节点对应动作 |
|---|---|---|
| `w` 前进 | `linear.x = +0.3` | `Car-no-p` 前进 1m → `linear.x = +0.3` |
| `e` 右横移 | `linear.y = -0.3` | `Car-p` 右横移 1m → `linear.y = -0.3` |
| `a` 左转 | `angular.z = +0.8` | `no-left` 左转 90°、`speed-limit-40` 左转 180° → `angular.z = +0.8` |
| `d` 右转 | `angular.z = -0.8` | `no-right` 右转 90°、`end-speed-limit-40` 右转 180° → `angular.z = -0.8` |

唯一区别：键盘节点是"按住键持续发"，本节点是"检测到标志后发固定时长自动停"（时长 = 角度/速度 或 距离/速度）。链路完全借用键盘节点：`/cmd_vel → serial_bridge → STM32`。

> 动作时长为开环估算，受地面摩擦、电压影响；需要更准可后续加里程计闭环。

## 板端 AI 脚本接入（rknn_infer_camera.py 加 3 行）

在板端 `/home/mikl/AI/YOLO/rknn_infer_camera.py` 顶部加：

```python
import socket
_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
```

在 `detect()` 返回后、画框循环里，检测到目标时发送（注意节流，同一标志别刷太密）：

```python
        # 检测到目标 → UDP 发给 sign_control_node（127.0.0.1 同机；跨机改成板子 IP）
        for cls_id, conf, _ in results:
            _udp.sendto(f"{NAMES[cls_id]} {conf:.2f}".encode(), ("127.0.0.1", 8877))
```

> 建议节流：记录上次发送时间，间隔 ≥0.3s 才发，避免同一标志每帧狂发
> （本节点有 busy 锁和冷却，但节流更省资源）。

## 构建与运行（RK3588 上，ROS2 Jazzy）

```bash
# 把本目录拷到工作空间（或直接从 Windows 拷贝 tools/traffic_sign_control）
cd ~/ros2_ws
colcon build --packages-select traffic_sign_control
source install/setup.bash
```

### 方式 A：一条命令同时启动「串口桥 + 控制节点」（推荐）

```bash
ros2 launch traffic_sign_control traffic_sign_control.launch.py
# 串口不是 /dev/ttyS1 时:
# ros2 launch traffic_sign_control traffic_sign_control.launch.py port:=/dev/ttyACM0
```

launch 内部包含：
1. `serial_bridge`（来自 **`tools/legacy/mecanum_bringup`，已停用**）：底盘串口 <-> ROS2，
   订阅 `/cmd_vel` 转发给 STM32 —— 单独拎出来，不跑建图/雷达
2. `sign_control_node`（本包）：UDP 8877 收 AI 检测 → 执行动作 → `/cmd_vel`

> ⚠️ 本包是 2026-09 独立写的，串口桥复用了当时的 `serial_bridge`。
> 当前主链路的桥是 `warehouse_inspector/chassis_bridge`（带 0.5 s 失控看门狗）。
> 如果与主链路同时用，**不要起本 launch 里的 serial_bridge**，改用方式 B 单独起 `sign_control_node`。

### 方式 B：分开启动（不依赖本 launch）

```bash
# 终端1: 只跑串口桥（mecanum_bringup 自带，不碰建图）
ros2 launch mecanum_bringup mecanum_control.launch.py

# 终端2: 控制节点
ros2 run traffic_sign_control sign_control_node
```

## 参数（全部可 --ros-args -p 覆盖）

| 参数 | 默认 | 说明 |
|---|---|---|
| `udp_ip` | 127.0.0.1 | UDP 监听地址（AI 识别与 ROS2 同机用默认） |
| `udp_port` | 8877 | UDP 端口 |
| `keepalive_hz` | 20.0 | 动作期间保活重发频率（底盘超时保护） |
| `cooldown` | 1.0 | 动作结束后冷却秒数 |
| `conf_thr` | 0.5 | 置信度阈值，低于忽略 |
| `rot_speed_90` | 0.8 | 左右转 90° 角速度 (rad/s) |
| `rot_speed_180_low` | 0.8 | 限速40 左转 180° 角速度 |
| `rot_speed_180_high` | 0.8 | 解除限速40 右转 180° 角速度 |
| `fwd_speed` / `fwd_dist` | 0.3 / 1.0 | 前进速度 / 距离 |
| `strafe_speed` / `strafe_dist` | 0.3 / 1.0 | 横移速度 / 距离 |

> 所有默认速度 = 键盘控制节点初始速度（0.3 / 0.3 / 0.8）。如需改变速，直接 `--ros-args -p` 覆盖。

示例（调快 90° 转弯）：

```bash
ros2 run traffic_sign_control sign_control_node --ros-args -p rot_speed_90:=1.0
```

## 安全说明

- 动作执行中忽略新的检测（busy 锁），结束后冷却 1s，防同一标志反复触发。
- UDP 发一行 `stop` 立即急停（清空当前动作并发布零速度）。
- Ctrl+C 退出时自动发布零速度。
- 纯开环控制：首次实车测试请先架空轮子验证动作方向，再落地跑。
