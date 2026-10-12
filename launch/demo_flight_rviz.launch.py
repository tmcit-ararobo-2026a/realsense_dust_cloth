import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cloth_tracker.rviz')

    # 1. Simulated Cloth Flight Depth Streamer
    sim_node = Node(
        package='realsense_dust_cloth',
        executable='sim_cloth_flight.py',
        name='cloth_flight_simulator',
        output='screen'
    )

    # 2. CUDA Cloth Tracker Node (with debug cloud enabled for RViz)
    tracker_node = Node(
        package='realsense_dust_cloth',
        executable='cloth_tracker_cuda_node',
        name='cloth_tracker_cuda_node',
        output='screen',
        parameters=[
            config_file,
            {
                'publish_debug_cloud': True,
                'use_direct_realsense': False
            }
        ]
    )

    # 3. Static Transform for world -> base_link
    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # 4. Static mounting transform (matches real-robot tracker_params.yaml)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_mount_tf',
        arguments=[
            '--x', '0.50',
            '--y', '0.0',
            '--z', '1.00',
            '--yaw', '0.0',
            '--pitch', '0.3490658504',
            '--roll', '1.5707963268',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # 5. RViz2 GUI
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        sim_node,
        tracker_node,
        world_tf_node,
        camera_tf_node,
        rviz_node
    ])
