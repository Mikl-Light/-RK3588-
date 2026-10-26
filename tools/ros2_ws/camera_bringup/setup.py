from setuptools import setup
import os
from glob import glob

package_name = 'camera_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='USB UVC MJPG camera ROS2 node optimized for DH-200W-X1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mjpeg_camera_node = camera_bringup.mjpeg_camera_node:main',
        ],
    },
)
