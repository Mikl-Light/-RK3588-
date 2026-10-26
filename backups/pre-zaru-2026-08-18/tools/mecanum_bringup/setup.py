from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='STM32F407 麦轮小车 ROS2 上位机: 串口桥 + 键盘遥控',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = mecanum_bringup.serial_bridge:main',
            'keyboard_teleop = mecanum_bringup.keyboard_teleop:main',
        ],
    },
)
