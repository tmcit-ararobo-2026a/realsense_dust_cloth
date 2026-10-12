import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cloth_tracker.rviz')

    # 1. CUDA Cloth Tracker Node
    tracker_node = Node(
        package='realsense_dust_cloth',
        executable='cloth_tracker_cuda_node',
        name='cloth_tracker_cuda_node',
        output='screen',
        parameters=[
            config_file,
            {
                'publish_debug_cloud': True,
                'use_direct_realsense': False,
                'use_sim_time': True,
            }
        ]
    )

    # 2. Static Transform for world -> base_link
    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # 3. Static mounting transform. RealSense publishes the optical frames.
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_mount_tf',
        arguments=[
            '0.15', '0', '0.35', '0',
            '0.3490658504', '1.5707963268',
            'base_link', 'camera_link'
        ]
    )

    # 4. RViz2 GUI
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        tracker_node,
        world_tf_node,
        camera_tf_node,
        rviz_node
    ])
