# robot_gui_ng

新一代麦轮小车 Web 可视化控制中心。

现已把旧 `robot_gui` 的前端界面/交互合并进来，但底层改用标准库 `ThreadingHTTPServer` + HTTP 轮询，因此只占用一个端口，局域网浏览器可直接打开。

## 实现说明

- 使用 Python 标准库 `http.server.ThreadingHTTPServer` 提供网页，和 `odom_compare_web` 同样方式，保证局域网可直接打开。
- 不再依赖 `websockets` 兼作 HTTP 服务。
- 前端通过 HTTP API 轮询获取地图/激光/位姿/路径。

## 功能

- 合并旧 `robot_gui` 的界面：模式按钮、虚拟摇杆、底部控制、导航浮层
- 实时地图显示（OccupancyGrid）
- 激光扫描点叠加
- 机器人位姿箭头
- Nav2 规划路径显示
- 虚拟摇杆遥控 `/cmd_vel`
- 地图点击发送导航目标（NavigateToPose action）
- 横向移动 / STOP
- 实时数据面板
- 标准 ThreadingHTTPServer + HTTP 轮询，不依赖 websockets HTTP 握手

## 使用

```bash
# 放到 ROS2 工作区后构建
cd ~/ros2_ws
colcon build --packages-select robot_gui_ng
source install/setup.bash
ros2 launch robot_gui_ng robot_gui_ng.launch.py
```

浏览器打开：

```text
http://<机器人IP>:8766/
```

如果旧 `robot_gui` 仍占用 8766，可用参数修改端口：

```bash
ros2 run robot_gui_ng robot_gui_ng --ros-args -p ws_port:=8767
```

## 数据来源

GUI 会主动兼容你当前 ROS2 系统里的几套数据源：

| 数据 | 话题 / 来源 |
|---|---|
| 地图 | `/map`（slam_toolbox / map_server / map_trigger） |
| 激光 | `/scan`（rplidar） |
| 激光里程计 | `/odom_laser`（rf2o_laser_odometry） |
| 轮式里程计 | `/odom`（chassis_bridge，**位置是固件世界系，与 ROS 系差 90°**）|
| 融合里程计 | `/odom_fused`（odom_fusion，**当前 nav/slam 唯一使用的 odom**）|
| 地图坐标位姿 | `/amcl_pose`、`/slam_out_pose`、TF `map -> base_link` |
| 规划路径 | `/plan` |
| 遥控 | `/cmd_vel` |

GUI 会优先用 `map -> base_link` 的 TF 把机器人画在地图上；没有地图 TF 时则回退显示里程计位姿。

## 可配置参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ws_host` | `0.0.0.0` | WebSocket/HTTP 监听地址 |
| `ws_port` | `8766` | 监听端口 |
| `map_topic` | `/map` | 地图话题 |
| `scan_topic` | `/scan` | 激光话题 |
| `odom_topic` | `/odom_laser` | 位姿/里程计话题 |
| `path_topic` | `/plan` | 规划路径话题 |
| `cmd_vel_topic` | `/cmd_vel` | 速度指令话题 |
| `nav_action_name` | `navigate_to_pose` | Nav2 action 名称 |
| `cmd_timeout` | `0.5` | 遥控断连自动停车秒数 |

## 注意

- 默认端口 `8766`，与旧 `robot_gui` 相同；不要同时启动两个占用同一端口的节点。
- 需要局域网访问时，确保 Linux 防火墙放行：

```bash
sudo ufw allow 8766/tcp
```

## 新增：画路径 / 航点巡航

页面顶栏新增：

- `✏️ 画路径`
  - 按住鼠标左键拖拽画曲线
  - 点“执行路径”后，GUI 会发到 `/route/follow_path`
  - 由 `route_follower` 转给 Nav2 `FollowPath` action
  - 路径会先经过代价地图检查，穿障碍直接拒绝
- `📍 航点导航`
  - 依次点击地图添加航点
  - 点“开始导航”后，GUI 会发到 `/route/waypoints`
  - 由 `route_follower` 逐个调用 Nav2 `NavigateToPose`
  - 航点落在障碍里会被跳过或终止，航点间由 Nav2 全局规划绕障

依赖 `mecanum_bringup_hector` 中的 `route_follower` 节点。

## 修复

- 地图缩放改为“像素/米”模型，滚轮缩放更直观
- 摇杆控制改为 HTTP `/api/cmd_vel`，按住可控制前后/旋转，松开自动停车
- 横向移动按钮和 STOP 也走 HTTP
- 地图点击导航目标会发到 `/api/nav_goal`
