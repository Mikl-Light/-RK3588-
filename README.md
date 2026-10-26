# V1 — STM32F407 麦轮巡检小车

麦轮全向底盘 + STM32F407 固件 + RK3588/ROS2 上位机 + 微信小程序。

```
微信小程序 / Web ──HTTP──> RK3588 (ROS2 Jazzy) ──USART2 460800──> STM32F407ZGT6 (裸机 168MHz)
```

---

## 一、当前版本（改代码前先看这里）

| 项 | 值 |
|---|---|
| 固件版本 | **v3.4.8-WHEEL80**（97572 B，版本宏在 `Core/Inc/ota.h`） |
| 车上固件 | `firmware/V1_v3.4.8-WHEEL80.bin` |
| 车上主包 | `tools/ros2_ws/warehouse_inspector` |
| 主 launch | `inspection.launch.py`（nav2 / inspection / circle 三个开关默认见文件头） |
| 全车唯一 odom | `/odom_fused`（`odom_fusion` 融合节点发布） |
| 遥控端 | `tools/ros2_ws/robot_gui_ng` |

**硬参数（都不许凭记忆改，改前实测）**

- 轮径 **80 mm**（R = 0.040 m）。代码里有两处独立定义，**必须同步改**：
  `Core/Src/odometry.c` 的 `DEFAULT_WHEEL_RADIUS`、`Core/Src/chassis.c` 的 `DEFAULT_WHEEL_DIAMETER`。
  只改一处 → 速度指令按旧轮径、里程计按新轮径，两者互相打架。
- 麦轮几何 Lx = Ly = 75 mm，L = Lx + Ly = 0.15 m
- 电机 MC520P30_12V：13 线编码器 / 30:1 减速，输出轴 52 cnt/rev
- 轮序 M1 = FL / M2 = FR / M3 = RL / M4 = RR
- 坐标系（已 ROS 化）：**Vx = 前，Vy = 左，Wz = 逆时针为正**
- **雷达 0° 在车尾，180° 在车头**。补偿由 `base_link→laser` 的 TF `yaw = π` 完成，
  **`scan_filter` 的 `angle_offset_deg` 必须为 0**，两条路只能选一条，叠加就转 360° 等于没补。

---

## 二、目录结构

```
ODOM - 副本/
├── Core/                 # 固件源码（本工程 = App，Bootloader 在 tools/Bootloader）
│   ├── Src/ Inc/         # 各模块 .c / .h
│   └── docs/             # 项目文档（见第五节）
├── Drivers/              # CMSIS + HAL
├── cmake/                # 工具链文件
├── build-chassisfix/     # 当前构建目录（唯一的，别用别的）
├── firmware/             # 固件产物，只留当前车上版本
├── tools/
│   ├── ros2_ws/          # 车上在跑的上位机包（与 ~/ros2_ws/src/ 对应）
│   ├── legacy/           # 旧方案，已停用，仅供查历史
│   ├── diagnostics/      # 调试脚本（跑车、测里程计、判方向）
│   ├── Bootloader/       # 独立 Bootloader 工程
│   ├── magcal/           # 磁力计标定
│   └── ota.py            # OTA 烧录工具
├── client/               # 微信小程序
├── backups/              # 带时间戳的改动前备份
└── .workbuddy/memory/    # 工程记忆（踩坑记录都在这里，比文档新）
```

### tools/ros2_ws/ — 车上在用的包

| 包 | 职责 |
|---|---|
| `warehouse_inspector` | **主包**。`chassis_bridge`（串口桥 + 0.5s 看门狗）、`scan_filter`、`map_despeckle`、nav2/slam 参数、`inspection.launch.py` |
| `odom_fusion` | 融合 odom：位置用激光绝对位姿、航向用激光增量 + 磁增量加权，发布 `/odom_fused` 与 TF |
| `rf2o_laser_odometry` | 激光里程计 → `/odom_laser`（**不发 TF**，由 fusion_node 统一广播） |
| `rplidar_ros` | RPLIDAR C1 驱动，**必须 `-p topic_name:=scan_raw`** |
| `robot_gui_ng` | Web 可视化控制中心（单端口，浏览器直开） |
| `camera_bringup` | DH-200W-X1 USB 相机节点 |
| `traffic_sign_control` | 交通标志识别 → 底盘动作 |

### tools/legacy/ — 已停用

| 包 | 为什么停用 |
|---|---|
| `mecanum_bringup` | 旧串口桥 `serial_bridge`，已由 `warehouse_inspector/chassis_bridge` 取代 |
| `mecanum_bringup_hector` | 旧 SLAM 方案，**与 warehouse_inspector 冲突，不能同时起** |
| `robot_gui` | 被 `robot_gui_ng` 取代（前端已合并进 ng） |
| `odom_compare_web` | 旧方案下的里程计对比工具 |

> 停用 ≠ 删除。这些包结构完整、能编译，只是不该再用于当前链路。

### tools/diagnostics/ — 常用脚本

| 脚本 | 用途 |
|---|---|
| `probe_dir.py --dist 0.30` | **位姿自洽性裁判**：位移方向 vs 自身航向。动过融合/坐标系必跑，偏差进 ±3° 才算过 |
| `autorun.py --mode {spin360,rect,zigzag,explore}` | 自动跑圈，全程雷达安全刹车。`explore` 为雷达自主探索 |
| `cloud_shift.py --yaw-search` | 点云几何 dyaw，**不经过 IMU 的独立航向裁判** |
| `survey.py --n 12` | 12 扇区环境勘察 |
| `line_test.py` / `laser_line.py` / `measure_line.py` | 直线行走与测距 |
| `yaw3_analyze.py`（在 `out/`） | 三路航向对比（laser / wheel / fused） |
| `out/pgm2png.py`、`out/pgmcrop.py` | PGM 地图转 PNG / 局部放大 |

---

## 三、构建与烧录

**编译固件**
```bash
ninja -C build-chassisfix
arm-none-eabi-objcopy -O binary build-chassisfix/V1.elf firmware/V1_<版本>.bin
```
> `build/` 自 2026-10-11 起就停更了，**只认 `build-chassisfix/`**。

**烧录**（需先释放 `/dev/ttyS1`，即停掉 chassis_bridge）
```bash
python3 ~/Desktop/OTA/ota.py /dev/ttyS1 firmware/V1_<版本>.bin
```

**部署上位机包**
```bash
# 本地 tools/ros2_ws/<包> → 车上 ~/ros2_ws/src/<包>
scp -r tools/ros2_ws/warehouse_inspector mikl@<车IP>:~/ros2_ws/src/
# 车上编译
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select warehouse_inspector
```

**起建图**
```bash
ros2 launch warehouse_inspector inspection.launch.py nav2:=false inspection:=false circle:=false
```

**存地图**（不要用 `slam_toolbox/save_map`，它订阅 `/map` 而该话题是事件驱动的，常失败）
```bash
ros2 run nav2_map_server map_saver_cli -t /map_raw -f ~/map_xxx
```

---

## 四、排查顺序（出问题时照这个来）

1. **先看地图。** 墙有重影 / 房间内部有条斜线 → 是喂给 slam 的 odom 不自洽，不是建图参数问题。
2. **跑 `probe_dir.py --dist 0.30`。** 位移方向和自身航向偏差 > 5° 就别往下查了，先修 odom。
3. **跑原地转圈 + `yaw3_analyze.py`。** 三路航向（laser / wheel / fused）必须同号同量级。
   只测直线和只测整圈都可能漏掉符号反的问题。
4. **确认 `/scan` 只有一个发布者。** 多个 `rplidar_node` 抢串口会让 `/scan` 直接消失。
   起 launch 前先 `ps -ef | grep -c '[r]plidar_node'`。
5. **后台起 launch 用 `setsid`。** 否则 SSH 断开时会连带杀掉。

详细踩坑记录见 `.workbuddy/memory/MEMORY.md`，比 `Core/docs` 更新。

---

## 五、文档

`Core/docs/` 现存这些（历次清理已删掉过时与互相矛盾的）：

| 文件 | 说明 |
|---|---|
| `整体项目介绍.md` | 系统全景 + 客户端接入 |
| `communication-protocol.md` | 串口协议参考（**版本标注 v3.0，命令以源码为准**） |
| `serial-commands.md` | 串口命令手册（同上） |
| `磁力计干扰实测与结论-2026-10-18.md` | 磁力计干扰的权威结论 |
| `STM32上电卡死问题记录.md` | 未解决的偶发上电卡死 |
| `code-study/01~10` | 固件教学资料（09 串口驱动、10 阶段复习最完整） |
| `求职/` | 简历与面试材料，与技术内容无关 |

> 文档时间戳若与代码冲突，**以代码为准**；代码与实车实测冲突，**以实测为准**。
