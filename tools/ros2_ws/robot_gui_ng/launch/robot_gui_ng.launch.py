#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_gui_ng',
            executable='robot_gui_ng',
            name='robot_gui_ng',
            output='screen',
            parameters=[{
                'ws_host': '0.0.0.0',
                'ws_port': 8766,
                'map_topic': '/map',
                'scan_topic': '/scan',
                'odom_topic': '/odom_laser',
                'path_topic': '/plan',
                'cmd_vel_topic': '/cmd_vel',
                'nav_action_name': 'navigate_to_pose',
            }],
        ),
    ])
