from setuptools import setup
import os
from glob import glob

package_name = 'robot_gui_ng'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='麦轮小车新一代 Web 可视化控制中心',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_gui_ng = robot_gui_ng.robot_gui_ng:main',
        ],
    },
)
