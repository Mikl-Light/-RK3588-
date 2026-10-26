from setuptools import setup
import os
from glob import glob

package_name = 'odom_compare_web'

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
    description='Standalone web tool to compare wheel odometry and laser odometry',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_compare_bridge = odom_compare_web.odom_compare_bridge:main',
        ],
    },
)
