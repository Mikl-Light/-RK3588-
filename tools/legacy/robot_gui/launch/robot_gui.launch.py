#!/usr/bin/env python3
"""
robot_gui.launch.py — 启动麦轮小车可视化控制中心
HTTP 页面端口 8766，WebSocket 端口 8767；浏览器访问 http://<RK3588_IP>:8766/

注意：需要先启动建图导航全套（laser_odom_mapping.launch.py），
本 launch 只启动 GUI 节点，不启动雷达/里程计/SLAM/nav2。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_gui',
            executable='robot_gui',
            name='robot_gui',
            output='screen',
            parameters=[{
                'ws_host': '0.0.0.0',
                'http_port': 8766,   # 浏览器访问页面
                'ws_port': 8767,     # WebSocket 实时数据/控制
                'map_topic': '/map',
                'scan_topic': '/scan',
                'odom_topic': '/odom_laser',
                'path_topic': '/plan',
                'cmd_vel_topic': '/cmd_vel',
                'nav_action_name': 'navigate_to_pose',
            }],
        ),
    ])
