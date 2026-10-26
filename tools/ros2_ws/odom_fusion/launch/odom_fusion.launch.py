"""odom_fusion.launch.py — 单独启动里程计融合节点（调试用）

用法:
  ros2 launch odom_fusion odom_fusion.launch.py
  ros2 launch odom_fusion odom_fusion.launch.py publish_tf:=false

正常跑车不需要这个文件 —— 主 launch 已经带起融合节点：
  ros2 launch warehouse_inspector inspection.launch.py nav2:=false

前提（这些节点由调用方提供）：
  - chassis_bridge 发布 /odom（轮式）+ /imu/imu407/data_raw
  - rf2o_laser_odometry 发布 /odom_laser
  - rplidar 发布 /scan

输出：
  - /odom_fused（融合后里程计，50Hz）
  - TF: odom_fused -> base_link
  - /fusion_diag（诊断 JSON）

⚠️ TF 所有权（2026-10-22 定案）：
  本节点是 odom→base_link 的**唯一**发布者。rf2o 必须 `publish_tf: False`，
  否则两个节点抢同一个 TF，建图会抖。旧版注释里"rf2o 可以保持 True，不冲突"是**错的**。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='是否发布 odom_fused->base_link TF'),
        DeclareLaunchArgument('out_frame', default_value='odom_fused',
                              description='输出里程计 frame_id'),

        Node(
            package='odom_fusion',
            executable='fusion_node',
            name='odom_fusion',
            output='screen',
            parameters=[{
                'publish_tf': LaunchConfiguration('publish_tf'),
                'out_frame': LaunchConfiguration('out_frame'),
                # 退化检测：scan 有效点占比低于 25% 视为激光退化
                'scan_valid_ratio_threshold': 0.25,
                # 零偏估计窗口 5 秒，低通系数 0.05（平滑）
                'bias_window_sec': 5.0,
                'bias_lowpass_alpha': 0.05,
                # 跳变剔除阈值 0.3rad(17°)：50Hz轮式+10Hz激光异步，激光更新帧diff会跳变，阈值太小会频繁清空窗口
                'bias_jump_reject_rad': 0.3,
                # 特征区激光 yaw 修正增益 0.1（慢速修正，避免跳变）
                'laser_correction_gain': 0.1,
            }],
        ),
    ])
