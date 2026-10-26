"""rf2o 定制参数启动（odom_topic=/odom_laser, frame=odom_laser, 8Hz）

⚠️ publish_tf 必须为 False：odom→base_link 的 TF 由 odom_fusion/fusion_node 独占总有，
   两路都发会让建图抖动。2026-10-22 定案。
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_laser',
                'publish_tf': False,          # ← 由 fusion_node 统一广播
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom_laser',
                'init_pose_from_topic': '',
                'freq': 8.0,
            }],
        ),
    ])
