# warehouse_inspector — 仓库巡检小车

在线建图（无需事先选起点）+ 狭窄空间导航 + 多航点自动巡航与到点拍照。

**本包所有节点均为独立实现**，未复用仓库内任何既有包的代码。参考的只是  
`robot_gui_ng / robot_gui / odom_fusion / odom_compare_web / mecanum_bringup_hector /
mecanum_bringup / camera_bringup` 的**设计思路与踩坑记录**；代码依赖全部是外部  
ROS 2 官方包（nav2 / slam_toolbox / rf2o_laser_odometry / rplidar_ros）。

---

## 1. 数据流

```
rplidar_ros ──/scan_raw──> scan_filter ──/scan──┬─> rf2o_laser_odometry ─> /odom_laser
                                                ├─> slam_toolbox ────────> /map_raw
                                                └─> nav2 局部/全局代价地图
slam_toolbox /map_raw ──> map_despeckle ──/map──> nav2 global_costmap.static_layer
相机 /image_raw ────────> inspection_route（到航点后拍照 + 写巡检报告）
nav2 /cmd_vel ──────────> chassis_bridge ──串口──> STM32 底盘

odom_fusion/fusion_node：
    /odom_laser（位置）+ /odom（航向增量）+ /imu  ──>  /odom_fused  +  TF
```

TF 树（**由 `fusion_node` 统一广播，rf2o 不再发 TF**）：

```
map ──slam_toolbox──> odom_fused ──fusion_node──> base_link ──static──> laser
```

> ⚠ **本包原设计里没有 `odom` 这个坐标系，直接用 `odom_laser`**。2026-10-22 起改为
> 由 `odom_fusion` 发布融合 odom，坐标系名为 **`odom_fused`**，slam 与 nav2 全部指向它。
>
> Nav2 里 `behavior_server.local_frame`、`local_costmap.global_frame` 等参数的
> **默认值就是 `odom`**，漏改一个就坏一块功能（见 §5）。本包在 `config/nav2_params.yaml`
> 里把每一个 frame 参数都显式写死。

---

## 2. 包内节点

| 节点                 | 作用                                       | 为什么需要它                                                                 |
| ------------------ | ---------------------------------------- | ---------------------------------------------------------------------- |
| `chassis_bridge`   | `/cmd_vel` → 串口 ASCII `V,<vx>,<vy>,<wz>` | 替代旧桥。**带失控看门狗**：0.5s 收不到 `/cmd_vel` 就主动下发零速度。旧桥没有看门狗，上层崩溃会让底盘保持最后速度继续跑 |
| `scan_filter`      | `/scan_raw` → `/scan`：距离下限 + 角度扇区屏蔽      | 挡掉雷达自回波。这是地图幽灵像素的**源头**                                                |
| `map_despeckle`    | `/map_raw` → `/map`：去掉孤立占据像素、可选填洞        | 实测孤立占据像素占全部占据像素 17.6%，膨胀后被切碎自由空间                                       |
| `camera_source`    | V4L2 → `sensor_msgs/Image`（cv2）          | 现场已有 `/image_raw` 时不用启动（`start_camera:=false`）                         |
| `inspection_route` | 航点巡航 + 到点拍照 + JSON 巡检报告                  | nav2 自带 waypoint_follower 的失败策略颗粒度不够，照片也无法与航点关联归档                      |

---

## 3. 快速开始

```
[rf2o_laser_odometry_node-4] [WARN] [1789289916.228442013] [rf2o_laser_odometry]: Waiting for laser_scans....                                [scan_filter-2] Traceback (most recent call last):                                                                                           [scan_filter-2]   File "/home/mikl/ros2_ws/install/warehouse_inspector/lib/warehouse_inspector/scan_filter", line 33, in             [scan_filter-2]     sys.exit(load_entry_point('warehouse-inspector==0.1.0', 'console_scripts', 'scan_filter')())                             [scan_filter-2]              ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^                              [scan_filter-2]   File "/home/mikl/ros2_ws/install/warehouse_inspector/lib/python3.12/site-packages/warehouse_inspector/scan_filter.py", line 199, in main                                                                                                                                [scan_filter-2]     run_node(ScanFilter())                                                                                                   [scan_filter-2]              ^^^^^^^^^^^^                                                                                                    [scan_filter-2]   File "/home/mikl/ros2_ws/install/warehouse_inspector/lib/python3.12/site-packages/warehouse_inspector/scan_filter.py", line 65, in __init__                                                                                                                             [scan_filter-2]     self.declare_parameter('invalid_value', 'inf')                                                                           [scan_filter-2]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/node.py", line 389, in declare_parameter                           [scan_filter-2]     return self.declare_parameters('', [args], ignore_override)[0]                                                           [scan_filter-2]            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^                                                              [scan_filter-2]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/node.py", line 528, in declare_parameters                          [scan_filter-2]     self._declare_parameter_common(                                                                                          [scan_filter-2]   File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/node.py", line 584, in _declare_parameter_common                   [scan_filter-2]     raise InvalidParameterTypeException(                                                                                     [scan_filter-2] rclpy.exceptions.InvalidParameterTypeException: Trying to set parameter 'invalid_value' to 'inf' of type 'DOUBLE', expecting type 'STRING': invalid_value                                                                                                                 [rf2o_laser_odometry_node-4] [WARN] [1789289916.328436280] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289916.428427047] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rplidar_node-1] [INFO] [1789289916.449283699] [rplidar_node]: current scan mode: Standard, sample rate: 5 Khz, max_distance: 16.0 m, scan frequency:10.0 Hz,                                                                                                                             [ERROR] [scan_filter-2]: process has died [pid 18986, exit code 1, cmd '/home/mikl/ros2_ws/install/warehouse_inspector/lib/warehouse_inspector/scan_filter --ros-args -r __node:=scan_filter --params-file /tmp/launch_params__pjm3xo_'].                                                 [rf2o_laser_odometry_node-4] [WARN] [1789289916.528529232] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289916.628514458] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289916.728492683] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289916.828541493] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289916.928532843] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.028533236] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.128438254] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.228508940] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.328506709] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.428495728] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.528497580] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.628490973] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.728485242] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.828476011] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289917.928476988] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.028469799] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.128451235] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.228470006] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.328469818] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.428466421] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.528489275] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.628554420] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.728540232] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.828463335] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289918.928479480] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.028478417] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.128509731] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.228558253] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.328530649] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.428468920] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.528482442] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.628548172] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.728541569] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.828472840] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289919.928467403] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.028438341] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.128530907] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.228523721] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.328529661] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.428532392] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.528456081] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.628521521] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.728462127] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.828453483] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289920.928436673] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.028450779] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.128431929] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.228485412] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.328514978] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.428515377] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.528528901] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.628535134] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.728527367] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.828522224] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289921.928552374] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.028548398] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.128551132] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.228571075] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.328560101] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.428732586] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.528638194] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.628609720] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.728596120] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.828590396] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289922.928596630] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.028593823] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.128590141] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.228641294] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.328608154] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.428585222] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.528590874] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.628584276] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.728580012] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.828579831] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289923.928574108] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289924.028586468] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289924.128585997] [rf2o_laser_odometry]: Waiting for laser_scans....                                [rf2o_laser_odometry_node-4] [WARN] [1789289924.228581150] [rf2o_laser_odometry]: Waiting for laser_scans..
```

照片与报告输出到 `~/inspection_shots/`（`save_dir` 可改）：

```
001_aisle_north_20260913_164512.jpg
002_shelf_east_20260913_164553.jpg
report_20260913_164610.json
```

报告结构：每个航点的名称、坐标、尝试次数、终态、耗时、剩余距离、照片绝对路径。

---

## 4. 航点配置

**两种来源，都是「一次性给出全部点、然后一次触发跑完」**：A 写在文件里；B 在 RViz 里连点。
两者同时存在时**以 B（刚收集的）为准**，`~/clear` 清空后回落到 A。

### 4.A 写文件（适合固定路线、要版本管理）

编辑 `config/waypoints.yaml`（或在 launch 里用 `route_file:=` 指向你自己的文件）：

```yaml
loop: false
waypoints:
  - name: shelf_a
    x: 1.20
    y: -0.35
    yaw: 1.5708     # 可选；拍照航点建议写，否则每次构图不同
    settle: 1.5     # 可选；到点后等几秒再拍（停稳 + 自动曝光收敛）
    note: "货架A正面"
```

取值方法：把车开到目标处，读 `ros2 run tf2_ros tf2_echo map base_link`，  
把 `Translation` 的 x/y 和 `RPY` 的 yaw 填进去。

### 4.B RViz 连点（不用抄坐标，边看边点）

```bash
ros2 launch warehouse_inspector inspection.launch.py collect_mode:=true
```

然后在 RViz 里：

1. **Fixed Frame 必须是 `map`**（不是 map 的点会被明确拒收并打一条 ERROR 日志）；
2. 用工具栏的 **「2D Goal Pose」** 工具在地图上点一下 → 拖出朝向 → 松手；
   每点一下，终端里出现一行 `已收集第 N 个航点: pNN (x, y, yaw=…)`；
   **车不会动**，放心连着点多下；
3. 点完调一次服务，它才按点击顺序依次跑完：

```bash
ros2 service call /inspection_route/start std_srvs/srv/Trigger
```

- 顺序 = 点击顺序；朝向 = 你拖出来的朝向（会参与 `yaw_goal_tolerance` 判定）；
- 每个点仍会**停稳 `settle_time` 秒再拍照**，语义和 4.A 完全一样；
- 收集结果实时写入 `~/inspection_shots/collected_waypoints.yaml`，
  **可直接当 `route_file` 复用**（改个名就是一份固定路线）；
- 点错了重来：`ros2 service call /inspection_route/clear std_srvs/srv/Trigger`
  （只清收集列表，不动 `route_file`）。若已开始跑，先 `~/stop`。

⚠ **两个必看的前提**：
- 收集期间车不动，是因为**本节点是这个话题唯一的消费者**。如果你同时跑着旧包的
  `goal_pose_bridge`，它会立刻把 `/goal_pose` 转成导航目标 → 点一下车就跑一次。先停掉它。
- 「2D Goal Pose」工具发的是 `/goal_pose`；RViz 的**「2D Pose Estimate」发的是
  `/initialpose`**（那是给 AMCL 定初始位的），**不要用它点航点**。

---

## 5. 已固化的经验教训

以下每一条都来自 `mecanum_bringup_hector` 方案的实测失败，本包在设计上直接规避：

| 教训（实测现象）                                                                                                                                  | 本包的处理                                                                                                           |
| ----------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `behavior_server.local_frame` 默认 `odom`，本车无此坐标系 → `Spin`/`BackUp` 全部 `Initial robot pose is not available` → **一旦卡住永远卡住**，因为唯一能脱困的后退行为是坏的 | `nav2_params.yaml` 中 `behavior_server` 显式写 `global_frame: map` + **`local_frame: odom_laser`**；文件头列出全部 frame 约定 |
| 雷达自回波写进 SLAM，生成**落在车体足迹内部的幽灵占据像素**（实测距车体中心 0.05m）→ 机器人自身栅格被判 INSCRIBED → NavFn 从起点一步都出不去（`Failed to create plan`）                         | 新增 `scan_filter`，在进 SLAM **之前**用距离下限 + 角度扇区挡掉自回波                                                                |
| 地图孤立占据像素占 17.6%，每个都长出一个膨胀圆盘，窄处被切碎                                                                                                         | 新增 `map_despeckle`                                                                                              |
| `transform_tolerance` 太小 → `Lookup would require extrapolation into the future` → `follow_path` 中止 → `Goal failed`                        | 三处 `transform_tolerance` 统一 **0.5**。里程计来自扫描匹配，**天生滞后一个扫描周期**                                                    |
| `failure_tolerance: 0.3` → 控制环 6.6Hz 时两三次瞬时碰撞判断就放弃目标                                                                                      | 改 **1.0**                                                                                                       |
| RViz2 跑在机器人上吃 **396% CPU / 8 核**，控制环 20Hz → 6.6Hz                                                                                         | launch 的 `rviz` 默认 `false` 并打印警告；迁移教程另见项目文档                                                                     |
| `robot_radius` 与 `footprint` 并存产生误导（实际生效的是 footprint）                                                                                     | 只保留 footprint 为主，`robot_radius` 注释说明它只是回退值，并把 `footprint_padding` 归零                                            |
| 航点巡航缺逐点失败策略、照片无归档                                                                                                                         | 自写 `inspection_route`：逐航点重试 N 次 → `skip`/`abort`，输出 JSON 报告                                                     |
| 旧底盘桥无失控看门狗                                                                                                                                | 自写 `chassis_bridge`，带 0.5s 看门狗 + 断线重连 + 退出前必发零速度                                                                |

---

## 6. 上电调试清单（按顺序做）

1. **量 footprint**：用卡尺量车体**含轮最宽处**，改 `nav2_params.yaml` 里两处 `footprint`。  
   `footprint_padding` 保持 0。注释里的 0.32×0.20m 只是默认值。
2. **确认雷达 yaw**：按住 `W` 让车直线走 20cm，看 RViz 里机器人箭头是「沿自身 +x 前进」  
   还是「沿 +x 倒退」。倒退就把 `inspection.launch.py` 里 `--yaw` 加减 π。  
   （「雷达装在车体最前方」说的是**位置**，不代表 0° **朝向**车头。）
3. **校准自回波过滤**：把 `scan_filter` 的 `report_period` 设为 3.0，车停空旷处跑一次，  
   看日志里「方位最小回波」是否在某个扇区异常近——那就是自身结构所在方位。
4. **干跑**：`dry_run:=true` 起来，确认 `/scan`、`/odom_laser`、`/map`、`/map_raw`、  
   `/cmd_vel` 都有数据，`map_despeckle` 日志显示去掉了多少噪点。
5. **低速建图**：手动推车或键盘遥控走一圈，确认 RViz 里地图干净（白区里没有成片黑点）。
6. **改航点**，然后跑一轮巡检，检查报告与照片。

---

## 7. 已知限制

- **不做地图持久化**：纯在线建图，重启即重来。若场地固定下来，建议改为  
  「建图 → 保存 → 清图 → 静态图 + 定位」模式，稳定性和重复精度都会明显更好。
- **无硬件安全逻辑**：没有电量、急停、碰撞传感器接入。`chassis_bridge` 的看门狗只  
  解决"指令断流"，不解决"避障失效"。
- **未发布 `camera_info`**：如需图像去畸变或视觉测量，请补一个标定文件发布节点。
- **长距离依赖激光里程计会漂**：`rf2o` 是纯扫描匹配，没有回环之外的绝对参考；  
  场地大时建议加轮式里程计融合（可参考 `odom_fusion` 的思路）。
- **单机单进程**：没有多机协同、没有任务调度中心。

---

## 8. 许可与出处

Apache-2.0。本包为独立实现；`package.xml` 里的 `<exec_depend>` 列出了全部外部依赖。
