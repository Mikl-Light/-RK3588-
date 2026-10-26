"""online_slam_navigation.launch.py — 在线 SLAM + 导航（不用 AMCL）

思路:
  不加载静态地图，不用 AMCL 初始定位。
  slam_toolbox 边建图边发布:
    /map 地图
    map -> odom_laser TF
  Nav2 直接使用 slam_toolbox 的 map / TF 做规划和局部控制。

启动:
  ros2 launch mecanum_bringup_hector online_slam_navigation.launch.py
  ros2 launch mecanum_bringup_hector online_slam_navigation.launch.py rviz:=true
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
    default_nav2_params = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # 1. 底盘桥
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

        # 4. 激光里程计
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

        # 5. slam_toolbox 在线 SLAM：发布 /map + map->odom_laser TF
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                default_slam_params,
                {
                    'odom_frame': 'odom_laser',
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                    'scan_topic': '/scan',
                    'mode': 'mapping',
                    'map_update_interval': 1.0,
                },
            ],
        ),

        # 6. Nav2 规划/控制/行为（不使用 map_server 和 AMCL）
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[default_nav2_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[default_nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[default_nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[default_nav2_params],
        ),

        # 8. Nav2 lifecycle 管理
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'slam_toolbox',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),

        # 9. rviz2（可选）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
