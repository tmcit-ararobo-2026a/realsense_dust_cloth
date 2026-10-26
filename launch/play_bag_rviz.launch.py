import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    default_config = os.path.join(pkg_share, 'config', 'test_bag_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cloth_tracker.rviz')
    default_bag = '/home/akeru/rosbag/cloth_test_11_camera_only'

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Path to ROS 2 parameter file'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag,
        description='Path to rosbag directory to play (empty string to disable bag play)'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier for rosbag'
    )

    # 1. CUDA Cloth Tracker Node
    tracker_node = Node(
        package='realsense_dust_cloth',
        executable='cloth_tracker_cuda_node',
        name='cloth_tracker_cuda_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
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

    # 3. RViz2 GUI
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Automatic Looping Bag Playback (delayed 3.0s so CUDA context, RViz & subscriptions initialize before Frame 0)
    bag_play_proc = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--clock', '100', '-l', '-r', LaunchConfiguration('rate')],
                name='rosbag_play',
                output='screen',
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('bag_path'), "' != ''"]))
            )
        ]
    )

    return LaunchDescription([
        params_file_arg,
        bag_path_arg,
        rate_arg,
        tracker_node,
        world_tf_node,
        rviz_node,
        bag_play_proc
    ])
