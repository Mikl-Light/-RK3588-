from setuptools import setup
import os
from glob import glob

package_name = 'traffic_sign_control'

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
    description='交通标志 AI 识别 → 麦轮底盘动作执行（独立控制包）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sign_control_node = traffic_sign_control.sign_control_node:main',
        ],
    },
)
