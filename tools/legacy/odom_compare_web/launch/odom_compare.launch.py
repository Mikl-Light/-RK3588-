"""odom_compare.launch.py — 轮式里程计 vs 激光里程计 Web 对比（独立包）

启动:
  ros2 launch odom_compare_web odom_compare.launch.py

Windows 浏览器打开:
  http://<RK3588_IP>:8765/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('baudrate', default_value='460800'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

        # 串口桥 + 里程计对比 + 串口命令 Web（同一个串口进程）
        Node(
            package='odom_compare_web',
            executable='odom_compare_bridge',
            name='odom_compare_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'publish_odom': True,
            }],
        ),

        # 雷达
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

        # 静态 TF：base_link -> laser（当前实测需 yaw=pi）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.13', '--y', '0', '--z', '0.03',
                       '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # 激光里程计：/odom_laser + odom_laser -> base_link TF
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

    ])
