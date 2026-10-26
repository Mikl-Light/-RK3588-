from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_bringup_hector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Hector / laser-odometry mapping comparison bringup for STM32F407 mecanum robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop_hector = mecanum_bringup_hector.keyboard_teleop:main',
            'map_trigger = mecanum_bringup_hector.map_trigger:main',
            'goal_pose_bridge = mecanum_bringup_hector.goal_pose_bridge:main',
            'route_follower = mecanum_bringup_hector.route_follower:main',
        ],
    },
)
