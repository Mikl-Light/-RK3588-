"""hector_mapping.launch.py — Hector SLAM 建图对比版（不修改原 mecanum_bringup）

用法:
  ros2 launch mecanum_bringup_hector hector_mapping.launch.py
  ros2 launch mecanum_bringup_hector hector_mapping.launch.py lidar_port:=/dev/ttyUSB0
  ros2 launch mecanum_bringup_hector hector_mapping.launch.py rviz:=true

TF 树:
  map -> odom (hector_mapping 根据激光匹配发布)
        -> base_link (serial_bridge 发布 407 轮式里程计/航向)
              -> laser (static TF)

说明:
  - 原包用 slam_toolbox；这里改成 hector_mapping。
  - hector_mapping 的特点是不依赖轮式 odom 做运动先验，因此 odom 不准时通常比
    slam_toolbox 更稳。这里仍保留 serial_bridge 发布 /odom 和 odom->base_link，
    是为了方便和原方案对比；Hector 的 map->odom 会通过激光匹配修正底盘 odom 的漂移。
  - 注意：ROS2 Jazzy 官方 apt 没有 hector_slam/hector_mapping 包。
    本 launch 需要在你有可用的 ROS2 hector_mapping 移植版时才能直接运行；
    若暂时没有，请先使用同目录 laser_odom_mapping.launch.py（ROS2 原生替代方案）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1',
                              description='底盘串口 (RK3588 原生 UART)'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='雷达串口'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否由 launch 启动 rviz2'),

        # 1. 底盘桥：/odom + /imu + odom->base_link TF + cmd_vel -> V
        #    保留是为了和原方案对比；Hector 本身不依赖 odom。
        Node(
            package='mecanum_bringup',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
            }],
        ),

        # 2. 雷达 RPLIDAR C1 (460800, frame: laser)
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

        # 3. static TF: base_link -> laser
        #    与现有 mapping.launch.py 保持一致：雷达物理朝前，但 rplidar 扫描零角相对车头反 180°：x=0.13, z=0.03, yaw=pi
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.13', '--y', '0', '--z', '0.03',
                       '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. Hector Mapping
        #    - use_tf_pose_start_estimate=false：不让 odom 做匹配初值，纯激光匹配
        #    - pub_map_odom_transform=true：发布 map->odom，便于 rviz 显示
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'pub_map_odom_transform': True,
                'pub_map_scanmatch_transform': False,
                'use_tf_scan_transformation': True,
                'use_tf_pose_start_estimate': False,
                'map_resolution': 0.05,
                'map_size': 1024,
                'map_start_x': 0.5,
                'map_start_y': 0.5,
                'map_multi_res_levels': 3,
                'map_pub_period': 2.0,
                'laser_min_dist': 0.2,
                'laser_max_dist': 12.0,
                'laser_z_min_value': -1.0,
                'laser_z_max_value': 1.0,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'map_update_distance_thresh': 0.2,
                'map_update_angle_thresh': 0.06,
                'scan_subscriber_queue_size': 5,
                'advertise_map_service': True,
            }],
        ),

        # 5. rviz2 (可选)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('mecanum_bringup'),
                                          'rviz', 'mapping.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
