import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'realsense_dust_cloth'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamagata',
    maintainer_email='kamagata@example.com',
    description='RealSense Depth-only cloth trajectory tracking and expectation modeling package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_cloth_tracker_node = realsense_dust_cloth.pointcloud_cloth_tracker_node:main',
            'depth_trajectory_recorder_node = realsense_dust_cloth.depth_trajectory_recorder_node:main',
            'trajectory_analyzer = realsense_dust_cloth.trajectory_analyzer:main',
        ],
    },
)

