"""navigation.launch.py — 麦轮小车 Nav2 自主导航一键启动

启动内容:
  serial_bridge + rplidar + rf2o 激光里程计 + map_server + AMCL
  + planner/controller/recoveries/bt_navigator + lifecycle_manager + 可选 rviz

用法:
  ros2 launch mecanum_bringup_hector navigation.launch.py
  ros2 launch mecanum_bringup_hector navigation.launch.py \
      map_file:=/home/mikl/map/hector_compare.yaml rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mecanum_bringup_hector')
    default_params = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    default_map = os.path.expanduser('~/map/hector_compare.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('rviz', default_value='false'),

        # 1. 底盘桥：只保留 cmd_vel -> V 和 IMU/编码器，不发布轮式 odom TF
        Node(
            package='mecanum_bringup',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'publish_odom': False,
            }],
        ),

        # 2. 雷达
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # 3. 静态 TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.13', '--y', '0', '--z', '0.03',
                       '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. 激光里程计：odom_laser -> base_link
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
                'publish_tf': True,
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
        ),

        # 5. 地图服务器：加载已保存地图
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'yaml_filename': LaunchConfiguration('map_file')},
            ],
        ),

        # 6. AMCL 定位（麦轮全向模型）
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # 7. 全局规划
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # 8. 局部控制（DWB，支持麦轮横移）
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # 9. 行为服务器（Jazzy 中由 nav2_behaviors 提供，节点名 behavior_server）
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # 10. 行为树导航
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # 11. Nav2 生命周期管理：自动激活所有 lifecycle 节点
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),

        # 12. 自动触发 map_server 发布 /map（Jazzy 需要 get_map 服务触发一次）
        Node(
            package='mecanum_bringup_hector',
            executable='map_trigger',
            name='map_trigger',
            output='screen',
        ),

        # 13. rviz2（可选）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
