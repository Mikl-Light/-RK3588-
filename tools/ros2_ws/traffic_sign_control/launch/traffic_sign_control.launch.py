"""traffic_sign_control.launch.py — 一键启动: 底盘串口桥 + 交通标志控制

用法:
  ros2 launch traffic_sign_control traffic_sign_control.launch.py
  ros2 launch traffic_sign_control traffic_sign_control.launch.py port:=/dev/ttyACM0
  ros2 launch traffic_sign_control traffic_sign_control.launch.py port:=/dev/ttyS1 baudrate:=460800

包含:
  1. serial_bridge (来自 mecanum_bringup): 底盘串口 <-> ROS2, 订阅 /cmd_vel -> V 命令
  2. sign_control_node (本包): 接收 AI 识别结果 (UDP 8877) -> 执行动作 -> /cmd_vel

不包含建图 / 雷达 / 键盘遥控 — 单独把串口桥与控制结合，链路最短。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1',
                              description='底盘串口设备路径 (RK3588 原生 UART)'),
        DeclareLaunchArgument('baudrate', default_value='460800',
                              description='串口波特率'),

        # 1. 串口桥: 解析 407 数据 -> /odom /imu, 订阅 /cmd_vel -> V 命令
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

        # 2. 交通标志控制: UDP 8877 收 AI 检测 -> 执行动作 -> /cmd_vel
        Node(
            package='traffic_sign_control',
            executable='sign_control_node',
            name='sign_control_node',
            output='screen',
        ),
    ])
