# ROS2 上位机问题总结（RK3588 + STM32F407 麦轮小车）

> 日期：2026-08-16
> 范围：RK3588 ROS2 Jazzy 侧的 serial_bridge / keyboard_teleop / rplidar / slam_toolbox 建图 全链路排障记录
> 相关：固件侧问题见 `OTA-交接文档.md`；主进度见 `进度记录-2026-08-12.md`

---

## 一、系统链路总览

```
┌─ STM32F407（固件 V2.3）──────────────────────────────────────┐
│  AHRS(yaw) ──┐                                               │
│  编码器×4 ──┼──→ odometry(pose.heading=AHRS每周期同步)       │
│  麦轮运动学 ──┘        → $T(50Hz) / $STAT(500ms)             │
└──────────────┬───────────────────────────────────────────────┘
               │ /dev/ttyS1 @460800
┌──────────────▼───────────────────────────────────────────────┐
│ serial_bridge.py（直通 + TF + 自动TELEM,ON + $STAT兜底）       │
│   /odom + odom→base_link TF + /imu/imu407/* + /cmd_vel→V      │
├───────────────────────────────────────────────────────────────┤
│ rplidar_node (C1) → /scan (frame: laser)                     │
│ static TF: base_link→laser (x=0.13 y=0 z=0.03 yaw=0)         │
│ slam_toolbox (lifecycle 自动激活) → /map + map→odom TF        │
│ keyboard_teleop（三轴独立调速, AD=旋转 QE=横移）               │
└───────────────────────────────────────────────────────────────┘
```

---

## 二、问题清单（现象 → 根因 → 解决）

| # | 现象 | 根因 | 解决 | 状态 |
| --- | --- | --- | --- | --- |
| 1 | 键盘节点用 `ros2 launch` 启动后没反应 | launch 子进程 stdin 不是终端 | 必须 `ros2 run`（终端直启） | ✅ |
| 2 | `/dev/ttyS1` 打不开 | 串口权限 | udev 规则 `KERNEL=="ttyS*", MODE="0666"` | ✅ |
| 3 | 手动遥控 OK 但 rviz/建图方向不对 | 桥的 cmd_vel/odom 坐标系转换（90° 错位） | 固件运动学 ROS 化后**桥直通零转换**，yaw=-heading | ✅ |
| 4 | 雷达 `/scan` 无数据 | udev 权限 / 波特率 | C1 需 460800 + `create_udev_rules.sh` | ✅ |
| 5 | slam_toolbox 启动后无 /map（只有一行日志） | **Jazzy slam_toolbox 是 lifecycle 节点**，停在 unconfigured | 手动 `ros2 lifecycle set configure/activate`；launch 内 TimerAction 4s/6s 自动激活 | ✅ |
| 6 | `odom` frame 不存在（tf2_echo 报错） | **407 TELEM 默认关闭** → 桥只收 $STAT 从不走发布路径 | 桥打开串口自动发 `TELEM,ON`（50Hz $T）；另加 $STAT 解析兜底发布 | ✅ |
| 7 | static TF 报 `Old-style arguments deprecated` + rotation=(1,0,0,0) | 旧式位置参数被 Jazzy 弃用 | 改新式 `--x/--y/--z/--yaw` | ✅ |
| 8 | static TF 报 `Extra unparsed arguments`（进程死） | 参数名用了下划线 `--frame_id` | 按 `--help` 核对改**连字符 `--frame-id`** | ✅ |
| 9 | rviz 报 `rviz_default_plugins/Displays` 面板加载失败 | 自定义 .rviz 的 Panels 段兼容问题 | 去掉 Panels 段；或手动 rviz2 + Add 面板兜底 | ✅ |
| 10 | rviz 报 GLSL sampler 错误 | **RK3588 Mali GPU 驱动兼容性**（显示层） | 无害，忽略；地图数据正常 | ✅ |
| 11 | 建图重影/撕裂/轮廓重复 | odom 位姿不准：① yaw 冻结（固件侧 odometry 不同步 AHRS）② 磁力计把航向钉死 ③ 打滑/快转 | 固件 V2.3 修复 yaw 同步 + AHRSMAG 默认 OFF；建图操作：慢速、少横移 | 🔄 待重测 |
| 12 | 键盘只能统一调速，横移速度不可单独调 | 设计缺陷 | 三轴独立速度 + 组合键（按住移动键+/-调对应轴） | ✅ |
| 13 | AD/QE 键位与设计相反 | 固件运动学未 ROS 化时行为错乱 | 键盘节点直接按用户习惯映射（A/D=旋转 Q/E=横移），与固件无关 | ✅ |
| 14 | 磁力计导致 yaw 被钉住/漂移（"加磁力计反而不稳"） | 电机磁场干扰 + AK8963 读失败 + 轴交换未验证 | `AHRSMAG,ON/OFF` 开关（默认 OFF 纯陀螺，slam 激光兜底） | ✅ |
| 15 | 雷达改朝前后建图方向错 | mapping.launch.py 的 TF 未更新 | static TF 改 `x=0.13, yaw=0`（朝前，前移 13cm） | ✅ |
| 16 | 桥跑着但 odom→base_link 一直不存在（已拷新版后） | 旧进程/旧 install（拷目录后未 colcon build） | 整目录覆盖 + `colcon build --packages-select mecanum_bringup` | ✅ |

---

## 三、重点问题详解

### 3.1 slam_toolbox 是 lifecycle 节点（#5）—— 最坑的一个

**现象**：launch 启动后 slam_toolbox 只有一行日志 `Node using stack size 40000000`，无 /map、无 scan 订阅。

**排查链**：
```
ros2 node info /slam_toolbox → 只有 lifecycle 服务（change_state/get_state...）
                                 没有 /scan 订阅、没有 /map 发布
→ 结论: 节点活着但停在 unconfigured 状态
```

**解决**（两处）：
```bash
# 手动（验证用）
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

# launch 自动（mapping.launch.py 内）
TimerAction(period=4.0, actions=[ExecuteProcess(cmd=['ros2','lifecycle','set','/slam_toolbox','configure'])])
TimerAction(period=6.0, actions=[ExecuteProcess(cmd=['ros2','lifecycle','set','/slam_toolbox','activate'])])
```

**教训**：Jazzy 的 slam_toolbox 节点是 LifecycleNode，`ros2 launch` 直接起不会自动激活。以后遇到"节点活着但啥也不干"先查 `ros2 lifecycle get /<node>`。

### 3.2 TELEM 默认关闭导致桥"半死"（#6）

**现象**：桥正常收 $STAT、串口正常，但 /odom 话题存在、`odom→base_link` TF 永远不存在 → slam 拿不到 TF 链不建图。

**根因**：407 默认 TELEM 关闭 → 串口只有 $STAT（500ms）→ 桥的发布逻辑（publish_binary）只在收到 $T 帧时执行 → TF 从未发布。

**解决**（双保险，serial_bridge.py）：
1. 打开串口后自动发 `TELEM,ON`（参数 `auto_telem_on`，默认开）→ 50Hz $T
2. 解析 `$STAT` 的 X/Y/HDG/Vx/Vy/Wz → 也调用 `publish_pose()` 发 odom+TF（500ms 保底）

**教训**：桥的"话题存在但 TF 缺失"要想到**数据源根本没触发**，而不是发布代码没写。

### 3.3 static TF 参数名（#7 #8）

```
旧式位置参数:     0 0 0.03 0 0 3.14159 base_link laser   ← deprecated, 会被忽略成零旋转
新式命名参数:     --x 0 --y 0 --z 0.03 --roll 0 --pitch 0 --yaw 3.14159 \
                  --frame-id base_link --child-frame-id laser   ← 连字符!
下划线 --frame_id ← 报 "Extra unparsed arguments" 进程退出
```
**教训**：`ros2 run tf2_ros static_transform_publisher --help` 先看，参数名是**连字符**（--frame-id），不是 ROS 参数惯例的下划线。

### 3.4 建图质量（#11）—— 多层原因，固件侧为主

| 层面 | 原因 | 修复 |
| --- | --- | --- |
| 固件 odometry | `pose.heading` 从不与 AHRS 同步 → 世界系投影方向冻结 | v2.3 每周期同步 |
| 固件 AHRS | 磁力计读失败/电机干扰 → 修正把 yaw 钉住 | AHRSMAG 默认 OFF（纯陀螺，slam 兜底） |
| 固件 I2C | SWRST 恢复后外设配置丢失 → 陀螺读失败 → yaw 冻结 | v2.2 I2C_Recover 重配 |
| 操作 | 麦轮横移/急转/打滑 | 建图用 W/S+Q/E 慢速，少横移 |
| 参数（可选） | slam_toolbox 对 odom 信任度 | 可调 position_covariance / search window（未实施，先重测） |

**建图操作建议**：
```
旋转速度调到 ≤0.5 rad/s（键盘 A+ 调速）
转弯后停 1 秒再走（让 scan matching 收敛）
避免横移（Q/E）——slam_toolbox 运动模型是差速的
启动 launch 时车静止，等 slam active 后再动
地图保存: ros2 run slam_toolbox slam_toolbox_save_map -f ~/map/mymap
```

### 3.5 键盘三轴独立调速（#12）

```text
W/S 前后   Q/E 横移   A/D 旋转   Space 急停
组合键调速（按住移动键再按 +/-）:
  W+ / W-  → 前进速度      Q+ / Q- → 横移速度
  A+ / A-  → 旋转速度      单独 +/-  → 三轴同调（兼容旧习惯）
参数: linear_vel / linear_y_vel / angular_vel / speed_step
```

---

## 四、slam_toolbox 专项问题

### 4.1 lifecycle 激活（最重要，详见 3.1）

Jazzy slam_toolbox = LifecycleNode，启动停在 `unconfigured`。**判定**：

```bash
ros2 lifecycle get /slam_toolbox        # active[3] 才对
ros2 node info /slam_toolbox            # 应有 /scan 订阅 + /map 发布
```
未激活的特征：节点只有 lifecycle 服务、无业务话题、只打一行 `Node using stack size` 日志。

### 4.2 地图重影/撕裂/轮廓重复

**slam 侧机理**：scan matching 把新帧激光与地图匹配，误差超出搜索窗口就匹配错位 → 地图叠加错乱。

| 诱因 | 说明 | 对策 |
| --- | --- | --- |
| odom 位姿不准 | yaw 冻结/被磁力计钉住/投影方向错 → 运动先验错 | 固件侧修复（v2.2/v2.3）后重测 |
| 转弯过头/打滑 | 麦轮转向时编码器打滑，odom 旋转误差大 | 旋转速度 ≤0.5 rad/s；转弯后停 1s |
| **麦轮横移** | slam_toolbox 运动模型是**差速**的，横移（linear.y）不符合模型 → 匹配崩溃 | 建图时**禁用横移**（Q/E） |
| 启动时车在动 | slam 初始化需要静止环境建立初始位姿 | launch 启动后等 active 再动 |
| 搜索窗口太小 | odom 误差 > 搜索窗时无法修正 | 调大 angle/linear search window |

### 4.3 参数调优候选（地图仍差时启用，mapping.launch.py 的 parameters 里加）

| 参数 | 默认 | 调优建议 | 理由 |
| --- | --- | --- | --- |
| `position_covariance` | 0.3 | 0.5 | 更信任激光、更不信 odom |
| `angle_search_window` | 0.12 | 0.3 | 容忍转向漂移 |
| `linear_search_window` | 0.2 | 0.5 | 容忍平移漂移 |
| `minimum_travel_heading` | 0.25 | 0.1 | 小角度也触发匹配 |
| `transform_publish_period` | 0.05 | 0.05 | TF 平滑（默认即可） |
| `scan_buffer_size` | 10 | 10~20 | 匹配参考帧数 |
| `resolution` | 0.05 | 0.05 | 地图分辨率（不轻易改） |

### 4.4 保存/复用地图

```bash
# 保存（生成 mymap.pgm + mymap.yaml）
ros2 run slam_toolbox slam_toolbox_save_map -f ~/map/mymap

# 定位模式（加载地图，导航前）
ros2 run slam_toolbox localization_slam_toolbox_node \
  --ros-args -p map_file_name:=/home/mikl/map/mymap \
  -p mode:=localization \
  -p scan_topic:=/scan -p odom_frame:=odom -p map_frame:=map -p base_frame:=base_link
```

### 4.5 常见现象速查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `/map` 话题不存在 | slam 未激活 | lifecycle configure/activate |
| `map` frame 不存在（rviz 报错） | 同上 | 同上 |
| rviz Map 显示 "No map received" | 同上 | 同上 |
| 有 /map 但地图一直空/黑 | 没推车 / scan 没进 | 推车；`ros2 topic hz /scan` 确认 |
| 地图方向与房间不对 | odom 初始 yaw 错 / 雷达 TF 错 | `tf2_echo base_link laser` 检查 TF；重启 slam |
| 地图越建越糊 | 横移 / 打滑 / 快转 | 按 4.2 操作规范 |
| `slam_toolbox_save_map` 报服务不存在 | slam 未激活 | 先 lifecycle activate |
| 保存的地图是全黑 | 建图期间没动/没数据 | 重建 |

### 4.6 教训

1. **slam_toolbox 与"地图质量"是两码事**：节点能出 /map ≠ 地图准。地图准 = odom 先验 + 操作规范 + 参数匹配，三者缺一不可。
2. **麦轮横移是 slam_toolbox 的克星**（差速模型）——建图禁用，导航时 nav2 自己处理横移（nav2 不依赖 slam 的运动模型）。
3. **先修数据链再调参数**：odom/yaw 不对时调 slam 参数是浪费——4.2 的固件修复优先于 4.3 的参数调优。

---

## 五、关键命令速查
```bash
# 一键建图（桥+雷达+TF+slam 自动激活；rviz 手动开）
ros2 launch mecanum_bringup mapping.launch.py

# 手动 rviz（launch 不带）
rviz2   → Fixed Frame=map → Add → /map + /scan

# 键盘遥控（必须 ros2 run，不能 launch）
ros2 run mecanum_bringup keyboard_teleop

# 诊断
ros2 topic hz /scan /map /odom
ros2 run tf2_ros tf2_echo base_link laser      # 应: x=0.13 y=0 z=0.03 yaw=0
ros2 run tf2_ros tf2_echo odom base_link       # 桥在发
ros2 run tf2_ros tf2_echo map odom             # slam 在发（active 后）
ros2 lifecycle get /slam_toolbox               # 应为 active; 否则 configure/activate
ros2 topic echo /map --once                    # 有数据 = 建图在跑

# 保存地图
ros2 run slam_toolbox slam_toolbox_save_map -f ~/map/mymap
```

---

## 六、踩坑经验总结（通用性）

1. **Jazzy slam_toolbox 是 lifecycle 节点**——启动后必须 configure + activate（launch 里用 TimerAction 自动做，别信"节点起来了就行"）。
2. **TF 参数名用连字符**（`--frame-id`）——先 `--help` 再写，下划线会静默/直接报错。
3. **桥的发布链路由数据源触发**——上游没数据（TELEM 关）时下游全静默；诊断先确认"数据真的在流"（`ros2 topic hz`），再查发布代码。
4. **rk3588 rviz GLSL 报错是 Mali GPU 驱动问题**，显示层无害，别浪费时间。
5. **坐标系改动的"一次对齐"原则**：固件 ROS 化（Vx=前 Vy=左）后，桥零转换直通，yaw=-heading 唯一保留的变换——所有层（cmd_vel/odom/TF）共用一套约定，避免多层转换叠加错误。
6. **odom 位姿质量决定建图质量**——yaw 冻结/被钉住都会让地图撕裂；固件侧问题优先排（用 $STAT HDG + Heartbeat 对比定位"显示层 vs 数据源"）。

---

## 七、待办

```text
□ 烧固件 V2.3 → 验证 $STAT HDG 跟随旋转（yaw 同步修复）
□ 重新建图（干净环境）→ 地图质量对比
□ 若仍撕裂: slam_toolbox 参数调优（position_covariance / search window）
□ nav2 导航（localization 模式 + 初始位姿 + 2D Goal）
□ IMU 对比（407 vs RK3588）脚本
□ （可选）AK8963 磁力计读失败排查（ST1 DRDY）——AHRSMAG,ON 时才需要
```
