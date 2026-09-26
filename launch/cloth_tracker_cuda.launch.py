import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')

    declare_use_direct_rs = DeclareLaunchArgument(
        'use_direct_realsense',
        default_value='false',
        description='Whether to stream directly from RealSense C++ API instead of ROS 2 topic'
    )

    declare_publish_debug_cloud = DeclareLaunchArgument(
        'publish_debug_cloud',
        default_value='false',
        description='Whether to publish extracted cloth cluster pointcloud (for debug/RViz)'
    )

    cuda_tracker_node = Node(
        package='realsense_dust_cloth',
        executable='cloth_tracker_cuda_node',
        name='cloth_tracker_cuda_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_direct_realsense': LaunchConfiguration('use_direct_realsense'),
                'publish_debug_cloud': LaunchConfiguration('publish_debug_cloud'),
            }
        ]
    )

    return LaunchDescription([
        declare_use_direct_rs,
        declare_publish_debug_cloud,
        cuda_tracker_node,
    ])

