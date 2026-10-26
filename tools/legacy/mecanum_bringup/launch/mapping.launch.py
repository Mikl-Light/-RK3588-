"""mapping.launch.py — 一键建图: 底盘桥 + 雷达 + TF + slam_toolbox + rviz2

用法:
  ros2 launch mecanum_bringup mapping.launch.py
  ros2 launch mecanum_bringup mapping.launch.py lidar_port:=/dev/ttyUSB0
  ros2 launch mecanum_bringup mapping.launch.py rviz:=false

TF 树: map -> odom (slam_toolbox) -> base_link (serial_bridge) -> laser (static)
雷达安装: 与车头方向相反 (yaw=pi), 水平居中, 高出底盘 3cm
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
                              description='是否由 launch 启动 rviz2 (建议 false, 手动开 rviz 更稳)'),

        # 1. 底盘桥: /odom + /imu + odom->base_link TF + cmd_vel -> V
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
        #    2026-08-12 硬件变更: 雷达朝前(与车头同向 yaw=0), 位于中心前方 13cm (x=0.13),
        #    高出底盘 3cm (z=0.03)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.13', '--y', '0', '--z', '0.03',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 4. slam_toolbox 建图 (吃 /scan + odom->base_link TF, 出 /map)
        #    Jazzy 的 slam_toolbox 是 lifecycle 节点, 启动后停在 unconfigured,
        #    必须 configure + activate 才会订阅 /scan 并发布 /map → 延迟自动激活
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('slam_toolbox'),
                             'config', 'mapper_params_online_async.yaml'),
                {
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                    'scan_topic': '/scan',
                    'mode': 'mapping',
                },
            ],
        ),

        # slam_toolbox lifecycle 自动激活 (等待就绪 + 重试, 替换固定延时 CLI)
        # RK3588 上 DDS 发现慢, 原 TimerAction(4s/6s)+CLI 会竞态失败:
        #   configure 报 Node not found, activate 报 Unknown transition
        # lifecycle_manager: 轮询等节点就绪 → configure → activate → 校验
        Node(
            package='mecanum_bringup',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
        ),

        # 5. rviz2 (可选, 用打包的建图配置)
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