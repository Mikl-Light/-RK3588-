"""mecanum_control.launch.py — 一键启动 串口桥 + 键盘遥控

用法:
  ros2 launch mecanum_bringup mecanum_control.launch.py
  ros2 launch mecanum_bringup mecanum_control.launch.py port:=/dev/ttyACM0 linear_vel:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1',
                              description='串口设备路径 (RK3588 原生 UART)'),
        DeclareLaunchArgument('baudrate', default_value='460800',
                              description='串口波特率'),

        # 串口桥: 解析 407 数据 -> /odom, /imu/imu407/*, 订阅 /cmd_vel -> V 命令
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
        # 注意: 键盘节点不在 launch 中启动!
        # launch 子进程的 stdin 不是终端, stdin 模式会报"不是终端设备"。
        # 键盘控制必须在 SSH 终端里单独运行 (与 teleop_twist_keyboard 相同限制):
        #   ros2 run mecanum_bringup keyboard_teleop
    ])
