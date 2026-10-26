# tools/ — 上位机与工具

分成四类，**只有 `ros2_ws/` 是当前在用的**。

```
tools/
├── ros2_ws/        车上在跑的 ROS 2 包（部署时上传到 ~/ros2_ws/src/）
├── legacy/         已停用的旧方案，结构完整可编译，但不要用于当前链路
├── diagnostics/    调试脚本（跑车、测里程计、判方向、看地图）
├── Bootloader/     独立 Bootloader 工程（Boot @0x08000000 / App @0x08010000）
├── magcal/         磁力计标定采集与分析
└── ota.py          OTA 烧录工具
```

---

## 1. ros2_ws/ — 当前链路

| 包 | 状态 | 说明 |
|---|---|---|
| `warehouse_inspector` | ✅ 主包 | 串口桥 `chassis_bridge`、`scan_filter`、`map_despeckle`、nav2/slam 参数、主 launch |
| `odom_fusion` | ✅ | 融合 odom，发布 `/odom_fused` + TF（**全车唯一 odom 广播者**） |
| `rf2o_laser_odometry` | ✅ | 激光里程计 → `/odom_laser`（**`publish_tf: False`**） |
| `rplidar_ros` | ✅ | RPLIDAR C1 驱动（**必须 `-p topic_name:=scan_raw`**） |
| `robot_gui_ng` | ✅ | Web 可视化控制中心 |
| `camera_bringup` | ✅ 独立 | USB 相机，不依赖主链路 |
| `traffic_sign_control` | ✅ 独立 | 交通标志识别 → 底盘动作 |

### 部署映射

目录结构与车上 `~/ros2_ws/src/` 一一对应（包名一致，可以整目录传）：

```bash
scp -r tools/ros2_ws/warehouse_inspector mikl@<车IP>:~/ros2_ws/src/
# 然后车上编译
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select warehouse_inspector
```

> 本地的 `rplidar_ros` 对应车上 `~/ros2_ws/src/rplidar_ros`，
> 本地 `rf2o_laser_odometry` 对应车上 `~/ros2_ws/src/rf2o_laser_odometry-ros2`（仅目录名不同，包名一致）。

---

## 2. legacy/ — 已停用

| 包 | 为什么停用 |
|---|---|
| `mecanum_bringup` | 旧串口桥 `serial_bridge`。已被 `warehouse_inspector/chassis_bridge` 取代（新桥带 0.5s 失控看门狗，旧的没有） |
| `mecanum_bringup_hector` | 旧 SLAM 方案（Hector/slam_toolbox + nav2 launch）。**与 warehouse_inspector 冲突，绝对不能同时起** |
| `robot_gui` | 被 `robot_gui_ng` 取代，前端已合并进 ng |
| `odom_compare_web` | 旧方案下的里程计对比 + Web 控制（端口 8765）。当前手机端出口是 `inspector_web` |

> 「停用」≠「删除」。留着是为了查历史设计、对照旧代码。**新代码不要 import 或 launch 它们。**

---

## 3. diagnostics/ — 调试脚本

脚本在车上跑（需 `source ~/ros2_ws/install/setup.bash`），上传到 `~/` 即可。

### 判据类（结论靠它们，不靠肉眼）

| 脚本 | 用途 | 通过标准 |
|---|---|---|
| `probe_dir.py --dist 0.30` | **位姿自洽性裁判**：位移方向 vs 自身航向 | 偏差 **±3°** 内。动过融合/坐标系必跑 |
| `cloud_shift.py --yaw-search` | **点云几何 dyaw**，不经过 IMU 的独立航向裁判 | 直线跑完 dyaw ≈ 0° |
| `yaw3_analyze.py <log.csv>` | 三路航向对比（laser / wheel / fused） | 三者同号、同量级 |
| `survey.py --n 12` | 12 扇区环境勘察（注意本车雷达 0° 在车尾） | — |

### 跑车类

| 脚本 | 用途 |
|---|---|
| `autorun.py --mode spin360` | 原地转一圈（只验航向，不移动） |
| `autorun.py --mode rect / zigzag` | 矩形 / 几字形闭环 |
| `autorun.py --mode explore --hops N` | 雷达自主探索：扫全 → 选最空方向 → 走一跳 |
| `line_test.py` / `laser_line.py` | 直线行走与测距 |
| `measure_line.py` | 位移精测 |

> `autorun.py` 全程带雷达安全刹车（前方 < 0.45 m 停），2026-10-22 实测救过一次。

### 看数据类

| 脚本 | 用途 |
|---|---|
| `mag_probe.py` | 磁力计三轴实时读数（判环境磁场梯度） |
| `transient_test.py` / `transient_geo.py` | 航向瞬态响应（逐帧 wheel vs gyro） |
| `loop_scan_test.py` | 点云几何闭环验证 |
| `out/pgm2png.py` | PGM 地图 → PNG（纯标准库，带网格） |
| `out/pgmcrop.py` | PGM 局部放大（查重影） |

---

## 4. 安全提示

- **起 launch 前先数雷达进程**：`ps -ef | grep -c '[r]plidar_node'` 必须为 0。
  多个实例抢 `/dev/ttyUSB0` 会让 `/scan` 直接消失，现场看起来毫无报错。
- **后台起 launch 用 `setsid`**，否则 SSH 断开会连带杀掉整条链。
- **跑车前确认没有任何东西在发 `/cmd_vel`**（比如另一个 autorun 残留）。
