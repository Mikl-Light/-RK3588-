"""warehouse_inspector — 仓库巡检系统一键启动

数据流
------
  rplidar(/scan_raw) ─> scan_filter ─> /scan ─┬─> rf2o_laser_odometry ─> /odom_laser
                                              ├─> slam_toolbox ─> /map_raw + TF
                                              ├─> fusion_node（退化检测） 
                                              └─> nav2 局部/全局代价地图
  chassis_bridge ─> /odom（固件磁绝对航向+轮式）+ /imu（陀螺）
  rf2o /odom_laser + 固件 /odom + /imu ─> fusion_node ─> /odom_fused + TF（唯一 odom 源）
  slam_toolbox /map_raw ─> map_despeckle ─> /map ─> nav2 global_costmap.static_layer
  相机 /image_raw ─> inspection_route（到航点后拍照并写巡检报告）
  nav2 /cmd_vel ─> chassis_bridge ─> 底盘串口

TF 树
-----
  map ──slam_toolbox──> odom_fused ──fusion_node──> base_link ──static──> laser
  ⚠ 本车没有 odom 这个坐标系，全部 frame 参数已显式写死在 config/nav2_params.yaml。
  ⚠ 2026-10-22 起 rf2o 不再发 TF（publish_tf:=False）：base_link 只能有一个父节点。
    融合的收益实测：直线 yaw 偏差 −0.37°（含停车 +0.09°），未融合的 /odom 单独 −7.99°。

常用启动方式
------------
  1) 完整系统（含底盘与雷达）
       ros2 launch warehouse_inspector inspection.launch.py

  2) 只起感知与建图，先看地图和 TF 对不对（不动底盘）
       ros2 launch warehouse_inspector inspection.launch.py nav2:=false

  3) 起步即自动巡检
       ros2 launch warehouse_inspector inspection.launch.py autostart:=true

  4) 手动触发一轮巡检（相机、导航都在跑时）
       ros2 service call /inspection_route/start std_srvs/srv/Trigger
       ros2 service call /inspection_route/stop  std_srvs/srv/Trigger

注意：不要把 RViz2 跑在机器人本机上。实测单个 rviz2 会吃掉约 4 个核（8 核机器），
     直接把控制环从 20Hz 拖到 6~9Hz，引发 TF 时间戳外推失败与恢复行为超时。
     请在上位机远程订阅（见项目里的《RViz迁移到WSL2上位机教程》）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    """把 launch 参数的字符串结果显式转成 bool。

    launch 的字符串替换结果不会自动变成 bool：Substitution 会被原样传给节点。
    数字样式的 'false' 恰好能被 YAML 还原成 false，但依赖这种隐式行为太脆弱，
    而且一旦真的按字符串进来，bool('false') 会是 **True**（危险）。这里显式转换。
    """
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def _as_int(text):
    return int(float(str(text).strip()))


def _nodes(context):
    pkg = get_package_share_directory('warehouse_inspector')
    slam_share = get_package_share_directory('slam_toolbox')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_override = os.path.join(pkg, 'config', 'slam_params.yaml')
    slam_defaults = os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml')
    default_route = os.path.join(pkg, 'config', 'waypoints.yaml')
    scan_filter_params = os.path.join(pkg, 'config', 'scan_filter_params.yaml')

    lidar_port = LaunchConfiguration('lidar_port')
    chassis_port = LaunchConfiguration('chassis_port')
    baudrate = _as_int(LaunchConfiguration('baudrate').perform(context))
    dry_run = _as_bool(LaunchConfiguration('dry_run').perform(context))
    autostart = _as_bool(LaunchConfiguration('autostart').perform(context))
    collect_mode = _as_bool(LaunchConfiguration('collect_mode').perform(context))
    route_file = LaunchConfiguration('route_file')
    on_failure = LaunchConfiguration('on_failure')

    perception = [
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            # 原始激光先落到 /scan_raw，由 scan_filter 清洗后再变成 /scan，
            # 这样下游（rf2o / slam_toolbox / 代价地图）拿到的都是干净数据。
            remappings=[('scan', 'scan_raw')],
        ),
        Node(
            package='warehouse_inspector',
            executable='scan_filter',
            name='scan_filter',
            output='screen',
            # 参数一律来自 config/scan_filter_params.yaml，不要在本文件里写 dict：
            #   ① 字符串 'inf' 会被 yaml.dump() 写成不带引号的裸标量 inf，接收端
            #      按 YAML 1.1 解析成 DOUBLE，与节点里 STRING 的声明冲突 ——
            #      节点会在 __init__ 抛 InvalidParameterTypeException 直接崩溃；
            #   ② 数组会被 dump 成 !!python/tuple（PyYAML 专用标签）。
            # 独立文件里能显式加引号，类型不再漂移。
            parameters=[scan_filter_params],
        ),
        # base_link -> laser 静态 TF
        #   x = 0.13  雷达光心在车体几何中心正前方 13cm（车长 32cm，贴近车头）
        #   z = 0.03  高出底盘上表面 3cm（底盘上表面距地约 9cm => 雷达离地约 12cm）
        #   yaw = π   雷达 0° 方向与车体前进方向**相反**（反装）
        #
        #   这个 yaw 的判据只有一个：RViz 里机器人箭头的 +x 必须等于车体实际前进方向。
        #   判据做法：按住 W 让车直线走 20cm，看箭头是「沿自身 +x 前进」还是「沿 +x 倒退」。
        #     前进 -> yaw 正确；倒退 -> 把 yaw 加减 π。
        #   注意「雷达装在车体最前方」说的是**位置**，不代表 0° **朝向**车头。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['--x', '0.13', '--y', '0', '--z', '0.03',
                       '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_laser',
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom_laser',
                # 2026-10-22：TF 交给 fusion_node 统一发布（odom_fused -> base_link）。
                # 若这里也发 odom_laser -> base_link，base_link 会有两个父节点，
                # TF 树直接报 "TF_MULTIPLE_PARENT"，slam_toolbox / nav2 都起不来。
                # rf2o 只作为 /odom_laser **话题**的提供者存在。
                'publish_tf': False,
                'init_pose_from_topic': '',
                # 必须 **低于** 雷达扫描频率。rf2o 的定时器回调里，只要这一拍
                # 没拿到新扫描就打一条 WARN "Waiting for laser_scans...."
                # （源码 CLaserOdometry2DNode.cpp:179）。设成 20 时（旧配置）
                # 有一半迭代空转，日志 47 分钟涨到 10.9 MB；设成 10 与雷达同频
                # 会因拍频周期性错过，实测稳定后仍有约 1 条/秒；取 8 让定时器
                # 始终慢于扫描（扫描周期约 0.10s，节拍 0.125s），可彻底消除。
                # 若将来换用更低频的雷达，这里要跟着一起降（也可直接降到 5）。
                'freq': 8.0,
            }],
        ),
        # ── 融合里程计（2026-10-22 接入）──────────────────────────────────
        # 全车唯一的 odom 提供者：/odom_fused + TF `odom_fused -> base_link`。
        # 输入：
        #   /odom_laser  rf2o 纯激光（行驶段 yaw 偏差实测仅 0.03~0.53°，是最可靠的一路）
        #   /odom        chassis_bridge 从 $T 解出的固件 EKF（磁绝对航向 + 轮式）
        #   /imu         chassis_bridge 从 $T 解出的陀螺（零偏估计 / 静止判据）
        # 已验证（2026-10-22 直线实测，点云几何当真值）：
        #   yaw 行驶段偏差 −0.37°、含停车段 +0.09°（未融合的 /odom 单独是 −7.99°，差 90 倍）
        #   ⚠ 位置只累加激光增量，不做跨坐标系加权（详见 fusion_node 第 6 步注释）
        Node(
            package='odom_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                'wheel_topic': '/odom',
                'laser_topic': '/odom_laser',
                'imu_topic': '/imu/imu407/data_raw',
                'scan_topic': '/scan',
                'out_topic': '/odom_fused',
                'out_frame': 'odom_fused',
                'base_frame': 'base_link',
                'publish_tf': True,          # 唯一的 odom->base_link 发布者
                'wheel_weight_floor': 0.05,  # 磁权重上限 5%（环境磁场梯度 21µT/m，实测结论）
                'still_anchor_freeze': True, # 静止冻结航向，不向磁航向收敛
            }],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_defaults, slam_override],
            # 原始地图走 /map_raw，经 map_despeckle 清洗后才发布为 /map
            remappings=[('map', 'map_raw')],
        ),
        Node(
            package='warehouse_inspector',
            executable='map_despeckle',
            name='map_despeckle',
            output='screen',
            parameters=[{
                'map_in': '/map_raw',
                'map_out': '/map',
                'occupied_thresh': 65,
                'min_neighbors': 2,   # 8 邻域里占据邻居少于 2 个的判为噪点
                'passes': 1,
                'fill_holes': False,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['slam_toolbox'],
                # bond_timeout 必须为 0：**slam_toolbox 不实现 nav2 的 bond 心跳**
                # （二进制里没有任何 bond 符号；slam_toolbox 官方 launch 的
                #  use_lifecycle_manager 默认也是 false，其注释称之为
                #  "Enable bond connection during node activation"）。
                # 只要 >0，nav2_lifecycle_manager 就会等一个永远不来的心跳，
                # 20s 后打两条 ERROR 并 "Aborting bringup"：
                #   Server slam_toolbox was unable to be reached after 20.00s by bond.
                #   Failed to bring up all requested nodes. Aborting bringup.
                # 节点本身其实已 active 且工作正常，这两条是纯粹的误报。
                # 0 = 不建立 bond（节点仍由本管理器做 configure/activate）。
                'bond_timeout': 0.0,
            }],
        ),
    ]

    navigation = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_params]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True, 'bond_timeout': 20.0,
                         'node_names': ['planner_server', 'controller_server',
                                        'behavior_server', 'bt_navigator']}],
        ),
    ]

    chassis = [
        Node(
            package='warehouse_inspector',
            executable='chassis_bridge',
            name='chassis_bridge',
            output='screen',
            parameters=[{
                'port': chassis_port,
                'baudrate': baudrate,
                'max_vx': 0.60,
                'max_vy': 0.60,
                'max_wz': 2.00,
                'watchdog_timeout': 0.5,   # 断流 0.5s 主动停车
                # 2026-10-22：解析 $T 遥测 -> /odom + /imu，供 fusion_node 使用。
                # 串口只能被一个进程占用，所以必须由桥就地补齐，不能再起第二个串口节点。
                'publish_telemetry': True,
                'odom_topic': '/odom',
                'imu_topic': '/imu/imu407/data_raw',
                'dry_run': dry_run,
            }],
        ),
    ]

    inspection = [
        Node(
            package='warehouse_inspector',
            executable='inspection_route',
            name='inspection_route',
            output='screen',
            parameters=[{
                'route_file': route_file,
                'frame_id': 'map',
                'nav_action': 'navigate_to_pose',
                'image_topic': LaunchConfiguration('image_topic'),
                'save_dir': LaunchConfiguration('save_dir'),
                'settle_time': 1.5,
                'max_retries': 1,
                'navigate_timeout': 120.0,
                'on_failure': on_failure,
                'autostart': autostart,
                'start_delay': 8.0,
                # RViz 连点收集：collect_mode:=true 后，在 RViz 用「2D Goal Pose」
                # 连点多个点（每点停留 settle_time 秒拍照），点完调 ~/start 一次跑完。
                # 收集到的点优先于 route_file；~/clear 清空后回落到文件。
                'collect_mode': collect_mode,
                'collect_topic': LaunchConfiguration('collect_topic'),
                'collect_file': LaunchConfiguration('collect_file'),
                # ★ 车头自动对齐行驶方向：开跑前按"我怎么到这里的"算每点朝向，
                #   第 1 个点用"车当前位姿 → 该点"。于是可以直接用「Publish Point」
                #   连点（只发点、不带朝向，点起来最快），也可继续用「2D Goal Pose」。
                #   想改回"用你拖出的朝向"：ros2 param set /inspection_route auto_heading false
                'auto_heading': True,
                # ★ 与 path_follower 的 click_topic(/clicked_point) **必须分开**：
                #   两者都订阅同一话题时，点直线会让 path_follower 也画出一个圆。
                'point_topic': '/waypoint_point',
                # ★ 逐点之间怎么走：
                #   navigate = nav2 全局规划（会绕障，适合多点巡航）
                #   straight = 严格沿你连的直线（/follow_path，不过全局规划器；遇障不绕）
                'segment_mode': LaunchConfiguration('segment_mode'),
                'straight_step': 0.05,
                # 严格直线遇障怎么办（只作用于 straight）：
                #   wait = 停车等障碍消失，清空后自动继续这一段（默认）
                #   stop = 停车 + 中止整轮巡检；skip = 跳过该点继续
                'on_obstacle': LaunchConfiguration('on_obstacle'),
                'scan_topic': '/scan',
                'laser_frame': 'laser',
                'obstacle_fov_deg': 60.0,     # 报告用：看车头 ±多少度
                'wait_fov_deg': 25.0,         # 判"真障碍 vs 残留"用：只看车头 ±多少度
                'clear_distance': 0.55,       # 判"激光是否已空"的门槛（> 触发点，形成迟滞）
                'obstacle_poll_period': 0.3,  # 等待时多久看一次
                'obstacle_log_period': 10.0,  # 等待进度日志节流
                'obstacle_wait_timeout': 0.0,  # 0 = 一直等（~/stop 可中止）
                # ★ 判"能不能走了"的主判据用**代价地图**（与控制器同源），
                #   只看车前方一条窄走廊 ⇒ 窄通道里不会被两侧的东西卡死。
                #   激光只用来区分"真障碍"和"障碍移开后残留的像素"。
                'use_costmap': True,
                'costmap_topic': '/local_costmap/costmap',
                'clear_costmap_service': '/local_costmap/clear_entirely_local_costmap',
                'corridor_len': 0.60,
                'corridor_half_width': 0.16,
                'max_futile_probes': 6,
            }],
        ),
    ]

    # 圆形路径：车头始终指向圆心的**连续**跟随（自写控制律，直接发 /cmd_vel）。
    # 用 RViz 的「点圆心/半径(圆形)」工具点 1 下圆心、再点 1 下圆周上一点 →
    # 几何画到 /path_markers；确认后 ~/start 走一圈。
    # ⚠ 它绕开了 nav2 的安全层，自带 /scan 切向扇区急停 + /cmd_vel 来源互斥。
    # 只想用直线/巡检时可以 circle:=false 把它关掉。
    circle = [
        Node(
            package='warehouse_inspector',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                # ★ 与 inspection_route 的 point_topic(/waypoint_point) **必须分开**
                'click_topic': '/clicked_point',
                'marker_topic': 'path_markers',
                'base_frame': 'base_link',
                'laser_frame': 'laser',
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                # 速度与增益
                'linear_vel': 0.20,          # 切向线速度（用户定的）
                'max_lateral_vel': 0.60,     # 与 chassis_bridge 限幅一致
                'max_forward_vel': 0.15,     # 径向修正故意压小
                'max_angular_vel': 2.00,
                'control_rate': 20.0,        # 必须稳定喂住 chassis_bridge 的 0.5s 看门狗
                'k_theta': 2.0,
                'k_radial': 1.0,
                'counter_clockwise': True,
                'min_radius': 0.15,          # 比这更小的圆拒绝执行
                # 安全
                'stop_distance': 0.35,
                'slow_distance': 0.80,
                'sector_inner_deg': 35.0,    # 只看切向扇区：车头方向是圆心（绕柱时就是柱子）
                'sector_outer_deg': 145.0,
                'idle_check_sec': 1.5,       # 启动前听 /cmd_vel，有人开车就拒绝
                'dry_run': dry_run,
            }],
        ),
    ]

    camera = [
        Node(
            package='warehouse_inspector',
            executable='camera_source',
            name='camera_source',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'width': 640,
                'height': 480,
                'fps': 15.0,
                'topic': LaunchConfiguration('image_topic'),
                'frame_id': 'camera_link',
            }],
        ),
    ]

    # 用本包自带的专用配置启动：里面只有「2D Goal Pose」（只发 /goal_pose 话题），
    # 不含 nav2_rviz_plugins 的 GoalTool / Navigation 2 面板 —— 那两个会在你点击的
    # 瞬间把目标直接发给 nav2 action，导致"标点期间车自己跑、且只去最后一个点"。
    # 顺带把 /scan 的订阅设成 Best Effort（scan_filter 是 Best Effort 发的）。
    rviz = [
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'inspection_waypoints.rviz')]),
    ]

    return [
        # ★ 关键：把 Fast DDS 的共享内存段放大到 4 MB。
        #   Fast DDS 内置 SHM 的默认单消息上限是 512 KB，640x480 bgr8 的 Image 有 900 KB，
        #   超过后就退化成 UDP 分片 —— 实测订阅端 12 秒收到 **0 帧**（发布端自报 14.6 fps），
        #   表现就是"车跑到了、照片却一张都没有"。放大后实测 15.1 fps。
        #   SetEnvironmentVariable 对本 launch 后续启动的所有节点生效，
        #   于是「发图」与「收图」的节点共享同一个大段，SHM 才谈得起来。
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(pkg, 'config', 'fastdds_large_shm.xml')),
        GroupAction(condition=IfCondition(LaunchConfiguration('perception')),
                    actions=perception),
        GroupAction(condition=IfCondition(LaunchConfiguration('nav2')),
                    actions=navigation),
        GroupAction(condition=IfCondition(LaunchConfiguration('chassis')),
                    actions=chassis),
        GroupAction(condition=IfCondition(LaunchConfiguration('inspection')),
                    actions=inspection),
        GroupAction(condition=IfCondition(LaunchConfiguration('circle')),
                    actions=circle),
        GroupAction(condition=IfCondition(LaunchConfiguration('start_camera')),
                    actions=camera),
        GroupAction(condition=IfCondition(LaunchConfiguration('rviz')),
                    actions=rviz),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_inspector')

    return LaunchDescription([
        DeclareLaunchArgument('perception', default_value='true',
                              description='雷达/过滤/激光里程计/SLAM/地图清洗'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='Nav2 规划与控制'),
        DeclareLaunchArgument('chassis', default_value='true',
                              description='底盘串口桥'),
        DeclareLaunchArgument('inspection', default_value='true',
                              description='航点巡航 + 到点拍照（连点走直线用这个）'),
        DeclareLaunchArgument('circle', default_value='true',
                              description='圆形连续跟随（path_follower）。只想用直线时设 false'),
        DeclareLaunchArgument('segment_mode', default_value='navigate',
                              description='逐点之间怎么走：navigate=nav2 全局规划（多点巡航，会绕障）'
                                          '；straight=严格沿直线（/follow_path，不过全局规划器，遇障不绕）'),
        DeclareLaunchArgument('on_obstacle', default_value='wait',
                              description='straight 模式遇障怎么办：wait=停车等障碍消失后自动继续（默认）；'
                                          'stop=停车并中止整轮巡检；skip=重试后跳过该点继续'),
        DeclareLaunchArgument('start_camera', default_value='false',
                              description='是否启动自带相机源（现场已有 /image_raw 时保持 false）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='⚠ 只在上位机设 true；机器人本机开会吃满 CPU'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='启动后自动开始一轮巡检'),
        DeclareLaunchArgument('collect_mode', default_value='false',
                              description='RViz 连点收集航点：连点多个后调 ~/start 一次依次跑完。'
                                          '也可运行中切换：ros2 param set /inspection_route collect_mode true'),
        DeclareLaunchArgument('collect_topic', default_value='/goal_pose',
                              description='收集航点用的话题（RViz 的 2D Goal Pose 工具发这个）'),
        DeclareLaunchArgument('collect_file',
                              default_value=os.path.expanduser(
                                  '~/inspection_shots/collected_waypoints.yaml'),
                              description='收集到的航点存这里，可直接当 route_file 复用'),
        DeclareLaunchArgument('on_failure', default_value='skip',
                              choices=['skip', 'abort'],
                              description='单个航点重试仍失败时的策略'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('chassis_port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video1'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('save_dir',
                              default_value=os.path.expanduser('~/inspection_shots')),
        DeclareLaunchArgument('route_file',
                              default_value=os.path.join(pkg, 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='底盘桥只打印不发串口，用于干跑验证'),

        LogInfo(condition=IfCondition(LaunchConfiguration('rviz')),
                msg='警告：RViz2 跑在本机会占用约 4 个 CPU 核，请确认这是上位机而不是机器人'),

        OpaqueFunction(function=_nodes),
    ])
