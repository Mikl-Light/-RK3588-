"""laser_odom_mapping.launch.py — ROS2 原生“Hector 思路”对比版

为什么有这个文件:
  ROS2 Jazzy 官方 apt 仓库没有 hector_slam/hector_mapping。
  如果你不想为此装 ROS1/Docker，这个 launch 用激光里程计替代轮式里程计，
  思路和 Hector 一致：尽量不信任底盘 odom，用激光扫描匹配估算运动。

对比:
  - 原 mapping.launch.py:  slam_toolbox + 407 轮式 odom (odom -> base_link)
  - 本文件:               rf2o_laser_odometry 生成 /odom_laser
                          + slam_toolbox 使用 odom_laser -> base_link
  - 为了不出现两个节点同时抢 base_link 的 TF，本 launch 关闭 serial_bridge 的
    odom/TF 发布 (publish_odom:=false)，只保留 cmd_vel/IMU/编码器等。

用法:
  ros2 launch mecanum_bringup_hector laser_odom_mapping.launch.py
  ros2 launch mecanum_bringup_hector laser_odom_mapping.launch.py lidar_port:=/dev/ttyUSB0
  ros2 launch mecanum_bringup_hector laser_odom_mapping.launch.py rviz:=true

依赖（按 README_HECTOR_ROS2.md 安装）:
  - rf2o_laser_odometry (ROS2 移植版)
  - slam_toolbox
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

        # 1. 底盘桥：只做 cmd_vel -> V / IMU / 编码器，不发布 odom TF
        #    （避免 odom_laser 和 odom 同时抢 base_link 子节点）
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

        # 2. 雷达 RPLIDAR C1
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.13', '--y', '0', '--z', '0.03',
                       '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. 激光里程计：用 /scan 生成 /odom_laser + odom_laser -> base_link TF
        #    注意：这个 rf2o 移植版实际参数名是 base_frame_id / odom_frame_id，
        #    不是 base_frame / odom_frame。publish_covariance 这个参数不存在。
        #    最关键：init_pose_from_topic 必须设为空字符串 ''，
        #    否则它会一直等 /base_pose_ground_truth，永远不处理 /scan。
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

        # 5. slam_toolbox 建图：使用激光里程计 odom_laser
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('slam_toolbox'),
                             'config', 'mapper_params_online_async.yaml'),
                {
                    'odom_frame': 'odom_laser',
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                    'scan_topic': '/scan',
                    'mode': 'mapping',
                },
            ],
        ),

        # slam_toolbox lifecycle 自动激活（复用原包脚本）
        Node(
            package='mecanum_bringup',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
        ),

        # 6. rviz2 (可选)
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
