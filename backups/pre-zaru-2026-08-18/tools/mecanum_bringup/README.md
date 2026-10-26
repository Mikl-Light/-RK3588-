# RK3588 上位机部署教程 — mecanum_bringup

> 适用：RK3588 + Ubuntu 24.04 + **ROS 2 Jazzy**（交接.md 指定版本）
> 底盘：STM32F407 麦轮小车（协议见 `Core/docs/serial-commands.md`）
> 本包 = **serial_bridge**（串口桥，现实代码）+ **keyboard_teleop**（键盘遥控，按住动/松开停）

---

## 一、包结构总览

```
mecanum_bringup/
├── package.xml                     # 包描述 (ament_python)
├── setup.py                        # 安装脚本 (入口点: serial_bridge / keyboard_teleop)
├── setup.cfg
├── resource/mecanum_bringup
├── launch/
│   └── mecanum_control.launch.py   # 一键启动两个节点
└── mecanum_bringup/
    ├── __init__.py
    ├── serial_bridge.py            # 串口桥: 407 数据 -> ROS2 话题, /cmd_vel -> V 命令
    └── keyboard_teleop.py          # 键盘: WASD 按住移动, 松开停止
```

## 二、环境准备（一次性）

```bash
# 1. 系统依赖
sudo apt update
sudo apt install python3-serial python3-evdev

# 2. 串口权限 — RK3588 原生 UART /dev/ttyS1 (默认仅 root 可访问)
#    方案 A (推荐, 永久): udev 规则
echo 'KERNEL=="ttyS1", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ttyS1.rules
sudo udevadm control --reload
sudo udevadm trigger
#    方案 B (临时): sudo chmod 666 /dev/ttyS1  (重启失效)

# 3. 键盘权限 (evdev 读 /dev/input/eventX)
sudo usermod -a -G input $USER
#    重新登录（或重启）使组生效

# 4. 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 5. 把本项目 tools/mecanum_bringup 整个目录拷进来：
#    (scp / U盘 / 或直接 git) 拷到 ~/ros2_ws/src/ 下
```

## 三、编译与运行

```bash
cd ~/ros2_ws
colcon build --packages-select mecanum_bringup
source install/setup.bash
```

**两个节点分开启动（重要）：**

```bash
# 终端 1 — 串口桥（headless，可以 launch 或后台）
ros2 launch mecanum_bringup mecanum_control.launch.py
#   或: ros2 run mecanum_bringup serial_bridge

# 终端 2 — 键盘遥控（必须 ros2 run 在终端里跑!）
ros2 run mecanum_bringup keyboard_teleop
```

> ⚠️ **键盘节点不能用 ros2 launch 启动**——launch 子进程的 stdin 不是终端，
> stdin 模式会报"不是终端设备"。这与 teleop_twist_keyboard 是同样的限制：
> 键盘节点必须在**你按键盘的那个 SSH 终端**里直接 `ros2 run`。
> （launch 文件里只放串口桥，就是这么设计的。）

```bash
# 换串口时:
ros2 launch mecanum_bringup mecanum_control.launch.py port:=/dev/ttyUSB0
```

## 四、键盘操作（按住动 / 松开停）

| 按键 | 动作 | 说明 |
|------|------|------|
| `W` / `S` | 前进 / 后退 | 按住移动，**松开立即停** |
| `A` / `D` | 左移 / 右移（横移） | 麦轮横移 |
| `Q` / `E` | 原地左转 / 右转 | |
| 组合键 | W+A 斜向移动 | 同时按多个键自动合成矢量 |
| `Space` | 急停 | 清空所有按键，发零速度 |
| `+` / `-` | 速度倍率 ±0.1 | 实时调整 |

**两种输入模式**（`input_mode` 参数，默认 auto 自动选择）：

| 模式 | 适用场景 | 松开停的实现 | 依赖 |
|------|---------|-------------|------|
| `evdev` | RK3588 上接了**本地物理键盘** | 读内核按下/松开事件，松开**立即**停 | python3-evdev |
| `stdin` | **SSH 远程控制**（无物理键盘） | 读终端字符流：按住→键盘重复产生字符→续命；松开→无字符 0.25s 后自动停（≈松开即停） | 仅标准库 |

> SSH 用法（和小乌龟同原理）：直接 ssh 进去，在终端里跑节点即可——
> 你的按键经 SSH 变成终端字符流送到 RK3588，节点读 stdin 就能收到。
> **要求：SSH 终端开启键盘重复**（绝大多数终端默认开启；Windows Terminal /
> PuTTY / VS Code 远程都支持）。若按住后车只动一下，说明终端重复没开。
>
> ```bash
> # SSH 里启动串口桥（任意方式）
> ros2 launch mecanum_bringup mecanum_control.launch.py
> # 然后在当前 SSH 终端启动键盘（必须 ros2 run!）
> ros2 run mecanum_bringup keyboard_teleop --ros-args -p input_mode:=stdin
> # 本地物理键盘（RK3588 上操作）：
> ros2 run mecanum_bringup keyboard_teleop --ros-args -p input_mode:=evdev
> ```

**实现原理**（和"按一下动、再按一下停"的 toggle 完全不同）：
- 节点维护"当前按住的键集合"——**任何一次按下/松开/字符输入都立即重算并发布 Twist**，
  松开最后一个键的瞬间发布的是零速度（stdin 模式为 0.25s 内）；
- 10Hz 定时保活重发当前状态：即使串口丢一帧，下位机也会在 100ms 内收到最新指令；
- 组合键（W+A）自动矢量合成，互斥键（W+S）自动抵消。

## 五、话题与坐标系（serial_bridge）

| 话题 | 类型 | 内容 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 407 融合位姿+速度（ROS 坐标系，参数可调） |
| `/imu/imu407/data_raw` | `sensor_msgs/Imu` | **原始** IMU 转 SI（RK3588 对比用） |
| `/imu/imu407/attitude` | `sensor_msgs/Imu` | 407 融合航向（yaw） |
| `/imu/imu407/mag` | `sensor_msgs/MagneticField` | 磁力计估算 µT |
| `/wheel_encoders` | `std_msgs/Int32MultiArray` | 编码器原始值 |
| `/wheel_speeds` | `std_msgs/Float32MultiArray` | 四轮 RPM |
| `/stm32_tick` | `std_msgs/Int32` | 407 tick(ms)，离线对齐时间基准 |
| `/chassis_status` | `std_msgs/String` | `$STAT` 原始行（调试） |

- **订阅**：`/cmd_vel`（`geometry_msgs/Twist`）→ 转 `V,<Vx>,<Vy>,<Wz>`
  - ROS：x=前 y=左 z=逆时针；底盘：Vx=右 Vy=前 Wz=逆时针
  - 转换：`Vx=-linear.y`，`Vy=linear.x`，`Wz=angular.z`
- **服务**：`/stop_chassis`（`std_srvs/Trigger`）→ 发 `STOP`
- **帧格式**：自动识别二进制帧（`TELEM,BIN`，0xAA55+CRC8，含原始 int16）与
  ASCII 帧（`$T`/`$STAT`）——下位机默认发 ASCII，联调时可直接用；
  上位机正式接入建议 `TELEM,BIN`（定长+CRC，更可靠，带宽省 2/3）

## 六、测试流程（建议顺序）

```bash
# 1. 先手动验证串口（不启动 ROS）— 与 pyserial 参数完全一致:
#    460800 8N1, 无校验, 无停止位扩展, 关回显
sudo stty -F /dev/ttyS1 460800 cs8 -cstopb -parenb -echo
sudo cat /dev/ttyS1
#    发送测试: 另开终端 → sudo stty 同配置后:
#      echo "WALL,60" | sudo tee /dev/ttyS1   → 四轮应同向 60 RPM
#      echo "STOP"   | sudo tee /dev/ttyS1
#    Ctrl+C 退出 cat
#    注: 若下位机开了 TELEM,BIN, cat 看到的会是 0xAA 0x55 开头乱码 (正常)

# 2. 只启动桥，看数据
ros2 run mecanum_bringup serial_bridge
ros2 topic echo /odom --once
ros2 topic echo /imu/imu407/data_raw --once   # 静止时: 加速度≈9.8, 陀螺≈0

# 3. 启动键盘，车轮悬空测试
ros2 run mecanum_bringup keyboard_teleop
#    按住 W → 四轮同向转; 松开 → 立即停
#    ros2 topic echo /cmd_vel   (另一个终端, 观察指令)

# 4. 落地测试
ros2 launch mecanum_bringup mecanum_control.launch.py
#    W 前进 / A 左移 / Q 左转 观察实际方向
```

## 七、常见问题

| 问题 | 解决 |
|------|------|
| `Permission denied: /dev/ttyS1` | udev 规则（见二.2），或临时 `sudo chmod 666 /dev/ttyS1` |
| `Permission denied: /dev/input/eventX` | input 组（见二.3），重登 |
| 键盘节点报"未找到键盘设备" | 说明走 stdin 模式了；若要 evdev 用 `--ros-args -p input_mode:=evdev -p device:=/dev/input/eventX` |
| 键盘节点报"stdin 不是终端设备" | 用了 ros2 launch 启动键盘 → 必须 `ros2 run mecanum_bringup keyboard_teleop` 在终端里直接跑 |
| SSH 里按住键只动一下 | 终端键盘重复没开（stdin 模式依赖重复字符流续命），检查终端设置 |
| 无遥测数据 | 下位机先 `TELEM,ON`；确认波特率 460800；`ls /dev/ttyS*` 确认设备存在 |
| 数据乱码 | 检查 460800 与 8N1 是否一致（stty 与桥的 baudrate 参数）；若开了 TELEM,BIN 属正常二进制 |
| 方向反了 | 桥的 `pose_invert_yaw` / `pose_swap_xy` 参数；或固件 motor.c 方向表 |
| 键盘失灵（拔插后） | evdev 设备路径会变，重启键盘节点或指定 device 参数 |

## 八、与导航对接（下一步）

- **一键建图**（雷达 C1 + 底盘 + slam_toolbox + rviz2）：
  ```bash
  sudo apt install ros-jazzy-slam-toolbox
  ros2 launch mecanum_bringup mapping.launch.py
  # launch 会自动: 启动桥/雷达/TF → 4s 后 configure slam → 6s 后 activate
  # rviz2 里 Fixed Frame=map, 应看到地图实时生长; 键盘(W/A/S/D+Q/E)缓慢推车
  # 建图完成后保存:
  ros2 run slam_toolbox slam_toolbox_save_map -f ~/map/mymap
  # 注意: 麦轮横移对 slam_toolbox 不友好, 建图时尽量前进+原地旋转
  # 若 launch 启动 8s 后仍无 /map: 手动执行下面两条补救
  #   ros2 lifecycle set /slam_toolbox configure
  #   ros2 lifecycle set /slam_toolbox activate
  ```
- **rviz 兜底**（若 launch 里的 rviz 配置加载报错，直接手动开）：
  ```bash
  # 建图照常跑, 另开终端:
  rviz2
  # 1. Global Options → Fixed Frame 改成 map
  # 2. Add → By topic → /map (Map) 和 /scan (LaserScan)
  # 3. 完成, 应看到雷达点云 + 地图生长
  ```
- TF 树：`map → odom (slam) → base_link (桥) → laser (静态: yaw=π, 与车头相反, z=0.03)`；
- **自主导航**：`nav2` 发布 `/cmd_vel`，桥自动转发成 `V` 命令——**不需要改任何代码**；
- **IMU 对比**：407 原始数据走 `/imu/imu407/data_raw`，RK3588 自带 IMU 走
  `/imu/imu_rk3588/data_raw`，同一套算法（如 Madgwick 相同增益）分别解算，
  按 `/stm32_tick` + 时间戳离线对齐后比较。

---
*2026-08-12 · 与 `Core/docs/project-report.md` 配套*
