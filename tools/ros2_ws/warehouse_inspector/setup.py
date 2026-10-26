from glob import glob
import os

from setuptools import setup

package_name = 'warehouse_inspector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikl',
    maintainer_email='mikl@todo.tld',
    description='仓库巡检小车：在线 SLAM + 狭窄空间导航 + 航点巡航拍照',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_bridge    = warehouse_inspector.chassis_bridge:main',
            'scan_filter       = warehouse_inspector.scan_filter:main',
            'map_despeckle     = warehouse_inspector.map_despeckle:main',
            'camera_source     = warehouse_inspector.camera_source:main',
            'inspection_route  = warehouse_inspector.inspection_route:main',
            'path_follower     = warehouse_inspector.path_follower:main',
        ],
    },
)
