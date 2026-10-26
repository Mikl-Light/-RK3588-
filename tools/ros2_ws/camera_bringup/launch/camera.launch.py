"""camera.launch.py — 启动 DH-200W-X1 MJPG 相机节点

用法:
  ros2 launch camera_bringup camera.launch.py
  ros2 launch camera_bringup camera.launch.py video_device:=/dev/video1
  ros2 launch camera_bringup camera.launch.py width:=1280 height:=720 fps:=30
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('pixel_format', default_value='mjpg'),
        DeclareLaunchArgument('image_topic', default_value='image_raw'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('publish_compressed', default_value='true'),
        DeclareLaunchArgument('publish_raw', default_value='false'),
        DeclareLaunchArgument('jpeg_quality', default_value='85'),

        Node(
            package='camera_bringup',
            executable='mjpeg_camera_node',
            name='mjpeg_camera_node',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'image_topic': LaunchConfiguration('image_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'publish_raw': LaunchConfiguration('publish_raw'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }],
        ),
    ])
