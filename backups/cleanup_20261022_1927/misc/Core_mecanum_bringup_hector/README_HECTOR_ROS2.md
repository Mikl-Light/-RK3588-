# mecanum_bringup_hector — Hector / 激光里程计建图对比包（ROS2 Jazzy / 2024.04）

> 本目录是**新增对比包**，不改动原 `tools/mecanum_bringup/` 任何文件。
> 适用：RK3588 + Ubuntu 24.04 + **ROS 2 Jazzy（2024.04 对应发行版）** + STM32F407 麦轮底盘。

---

## 0. 先说结论

### 407 底盘固件需要改吗？
**不需要。** 无论用真正的 Hector SLAM 还是下面的 ROS2 激光里程计替代方案：

- 都用 `/scan`（雷达）和 `base_link -> laser` 静态 TF；
- 都不依赖底盘 odom 作为主定位来源；
- `serial_bridge` 原封不动继续用，`/cmd_vel -> V` 照常下发。

所以**不用重新编译/烧录 407 固件**，也不用改串口协议。

### 但有一个必须知道的坑
**ROS2 Jazzy 官方 apt 仓库没有 hector_slam / hector_mapping 包。**
Hector SLAM 是 ROS1 时代的经典算法；ROS2 没有官方移植。

因此我给了两个 launch：

| launch | 说明 | 能否直接在 ROS2 Jazzy 跑 |
| --- | --- | --- |
| `hector_mapping.launch.py` | 真·Hector 版（需要你自行安装可用的 ROS2 hector_mapping 移植版） | 需要额外移植版 |
| `laser_odom_mapping.launch.py` | **ROS2 原生替代版**：rf2o 激光里程计 + slam_toolbox，思路和 Hector 一样不信任底盘 odom | ✅ 按下面装 rf2o 即可 |

**建议先直接跑 `laser_odom_mapping.launch.py` 对比**，它就是为了解决“odom 有问题导致建图差”的 ROS2 原生方案。

---

## 1. 包结构

```
tools/mecanum_bringup_hector/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/mecanum_bringup_hector
├── mecanum_bringup_hector/
│   └── __init__.py
├── launch/
│   ├── hector_mapping.launch.py       # 真·Hector 版（需移植版）
│   └── laser_odom_mapping.launch.py   # ROS2 原生替代版（推荐先跑这个）
└── README_HECTOR_ROS2.md
```

---

## 2. 准备工作（一次性）

如果你的 RK3588 上还没有 ROS2 Jazzy，先按官方装好：

```bash
# Ubuntu 24.04 / ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-ros-base
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

建工作空间：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --packages-select mecanum_bringup
source install/setup.bash
```

把本目录 `tools/mecanum_bringup_hector` 整个拷到：

```bash
# 在 RK3588 上，假设你拷到了 ~/ros2_ws/src/
cd ~/ros2_ws/src
ls mecanum_bringup        # 原包
ls mecanum_bringup_hector # 新对比包
```

---

## 3. 方案 A（推荐）：ROS2 原生替代 — 激光里程计 + slam_toolbox

### 3.1 安装 rf2o_laser_odometry（ROS2 移植版）

任选一个 ROS2 移植源：

```bash
cd ~/ros2_ws/src

# 源 1：wahello 的 ros2 分支
git clone -b ros2 https://github.com/wahello/rf2o_laser_odometry.git

# 源 2：linuxsen 的 ROS2 版（如果源 1 编译有问题再试）
# git clone https://github.com/linuxsen/rf2o_laser_odometry_ros2.git
```

安装依赖并编译：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rf2o_laser_odometry mecanum_bringup mecanum_bringup_hector
source install/setup.bash
```

> 如果 `rf2o_laser_odometry` 的包名/可执行名不是这个，请以你 clone 的 README 为准，
> 并把 `laser_odom_mapping.launch.py` 里的 `package` / `executable` 改一下。

### 3.2 启动对比建图

```bash
# 终端 1：一键建图（桥 + 雷达 + 激光里程计 + slam_toolbox 自动激活）
ros2 launch mecanum_bringup_hector laser_odom_mapping.launch.py

# 终端 2：键盘遥控（必须 ros2 run，不能 ros2 launch）
# 推荐用本包的修复版（原版单独按 +/- 会崩溃）
ros2 run mecanum_bringup_hector keyboard_teleop_hector
# SSH 远程时建议:
# ros2 run mecanum_bringup_hector keyboard_teleop_hector --ros-args -p input_mode:=stdin
```

> SSH stdin 模式下组合键（如 W+D）受终端自动重复流影响，可能不稳定。
> 如果组合键不灵，可先调大超时再试：
> ```bash
> ros2 run mecanum_bringup_hector keyboard_teleop_hector \
>     --ros-args -p input_mode:=stdin -p auto_stop_timeout:=0.5
> ```
> 想要最稳的组合键，建议在 RK3588 本地物理键盘上用 evdev 模式：
> ```bash
> ros2 run mecanum_bringup_hector keyboard_teleop_hector \
>     --ros-args -p input_mode:=evdev
> ```

可选参数：

```bash
# 指定雷达串口 / 打开 rviz
ros2 launch mecanum_bringup_hector laser_odom_mapping.launch.py \
    lidar_port:=/dev/ttyUSB0 rviz:=true
```

### 3.3 诊断命令

```bash
# 关键话题是否有数据
ros2 topic hz /scan /odom_laser /map

# TF 链是否完整
ros2 run tf2_ros tf2_echo map odom_laser       # slam_toolbox 在发
ros2 run tf2_ros tf2_echo odom_laser base_link # rf2o 激光里程计在发
ros2 run tf2_ros tf2_echo base_link laser      # 静态 TF，应 x=0.13 z=0.03

# slam_toolbox 是否 active（必须 active 才建图）
ros2 lifecycle get /slam_toolbox

# 看一帧激光里程计
ros2 topic echo /odom_laser --once
```

### 3.4 保存地图

```bash
ros2 run slam_toolbox slam_toolbox_save_map -f ~/map/hector_compare
# 生成 ~/map/hector_compare.pgm + ~/map/hector_compare.yaml
```

---

## 4. 方案 B：真·Hector SLAM（仅当你有 ROS2 移植版时）

如果你的 ROS2 环境里已经能 `ros2 run hector_mapping hector_mapping`，直接用：

```bash
# 编译本包
cd ~/ros2_ws
colcon build --packages-select mecanum_bringup_hector
source install/setup.bash

# 启动真 Hector 建图
ros2 launch mecanum_bringup_hector hector_mapping.launch.py

# 另一个终端键盘遥控
ros2 run mecanum_bringup_hector keyboard_teleop_hector
```

注意：

- Hector 不是 lifecycle 节点，不需要 `ros2 lifecycle set`。
- 本 launch 里 `use_tf_pose_start_estimate=false`，即**不让底盘 odom 做匹配初值**，
  专门针对你觉得 odom 不准的问题。
- 如果 ROS2 移植版的参数名不同，以该移植版 README 为准。

### 如果你只能跑 ROS1 Hector（Docker 方式，离线对比）

ROS2 Jazzy 没有官方 Hector，如果你坚持要原版 Hector，可以用 ROS1 Noetic Docker
录包/对比，但这不是 ROS2 原生方案：

```bash
# 在 RK3588 上装 Docker（仅示例，需要联网）
# sudo apt install docker.io
# 用 ROS1 Noetic + hector_slam 镜像，把 /scan 和 TF 录成 bag 再离线跑
```

---

## 5. 和原方案对比

| 项目 | 原 `mecanum_bringup/mapping.launch.py` | 新 `laser_odom_mapping.launch.py` |
| --- | --- | --- |
| SLAM | slam_toolbox | slam_toolbox |
| 里程计来源 | 407 轮式 odom（`/odom` + `odom->base_link`） | 雷达激光里程计（`/odom_laser` + `odom_laser->base_link`） |
| odom 不准时 | 会把错误先验带进建图 | 基本不依赖底盘 odom |
| 麦轮横移 | 不友好（slam_toolbox 差速模型） | 仍不友好，建图建议少横移 |
| 407 固件 | 不用改 | 不用改 |

**操作建议（和之前一样）**：
- 启动时车静止，等 `slam_toolbox` active 后再动；
- 建图慢速，少横移，转弯 ≤0.5 rad/s；
- 分别跑原方案和新方案各存一张图，再对比 `~/map/*.pgm`。

---

## 5.1 Nav2 自主导航（新增）

导航配置文件：

```text
config/nav2_params.yaml
launch/navigation.launch.py
```

安装 Nav2：

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

编译并启动导航：

```bash
cd ~/ros2_ws
colcon build --packages-select mecanum_bringup_hector
source install/setup.bash

ros2 launch mecanum_bringup_hector navigation.launch.py rviz:=true
```

Jazzy 的 map_server 启动后不会主动发 `/map`，需要手动触发一次：

```bash
ros2 service call /map_server/get_map nav_msgs/srv/GetMap "{}"
```

然后确认：

```bash
ros2 topic hz /map
```

指定地图：

```bash
ros2 launch mecanum_bringup_hector navigation.launch.py \
    map_file:=/home/mikl/map/hector_compare.yaml rviz:=true
```

在 rviz2 中：

1. Fixed Frame 设为 `map`
2. Add → By topic → 添加 `/map`、`/scan`
3. 先用 `2D Pose Estimate` 设置初始位置
4. 再用 `2D Goal Pose` 下发目标点

---

## 6. 常用命令速查（ROS2 Jazzy / 2024.04）

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select mecanum_bringup mecanum_bringup_hector
source install/setup.bash

# 新方案建图
ros2 launch mecanum_bringup_hector laser_odom_mapping.launch.py

# 键盘（修复版）
ros2 run mecanum_bringup_hector keyboard_teleop_hector

# 真 Hector 版（需移植版）
ros2 launch mecanum_bringup_hector hector_mapping.launch.py

# 诊断
ros2 topic hz /scan /odom_laser /map
ros2 run tf2_ros tf2_echo map odom_laser
ros2 run tf2_ros tf2_echo odom_laser base_link
ros2 lifecycle get /slam_toolbox

# 保存地图
ros2 run slam_toolbox slam_toolbox_save_map -f ~/map/hector_compare
```

---

## 7. 常见问题

| 问题 | 处理 |
| --- | --- |
| `rf2o_laser_odometry` 找不到 | 没编译/没 source；或 clone 的包名不同，改 launch 的 package/executable |
| `slam_toolbox` 没地图 | `ros2 lifecycle get /slam_toolbox` 应为 active；未 active 就手动 `ros2 lifecycle set /slam_toolbox configure` 再 `activate` |
| `map` frame 不存在 | 等 slam active；检查 `/scan` 有数据、`odom_laser->base_link` TF 存在 |
| 地图还是糊 | 麦轮横移/快转导致；按第 5 节操作规范重试 |
| 想同时看轮式 odom 和新 odom | 新 launch 为避 TF 冲突关闭了 serial_bridge 的 odom；可另外开原 `mecanum_control.launch.py` 在另一个 ROS 域或录包对比 |
| 真 Hector 版启动报找不到 `hector_mapping` | 说明 ROS2 移植版没装；先用方案 A |
