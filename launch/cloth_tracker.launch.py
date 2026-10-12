import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    default_config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cloth_tracker.rviz')

    # =========================================================================
    # 実行モード切り替えスイッチ (Runtime Switches)
    # 調整用パラメータ (TF含む) は config/tracker_params.yaml に一本化されています
    # =========================================================================
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_file,
        description='Path to tracker parameter YAML file (tracker_params.yaml or test_bag_params.yaml)'
    )

    declare_use_direct_rs = DeclareLaunchArgument(
        'use_direct_realsense',
        default_value='true',
        description='Stream directly via librealsense2 C++ API (true) or ROS 2 topics (false)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (Isaac Sim / rosbag)'
    )

    declare_use_bg_subtraction = DeclareLaunchArgument(
        'use_bg_subtraction',
        default_value='true',
        description='Enable background subtraction (set false for moving robot)'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 GUI window'
    )

    declare_record = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Automatically record TF, cloth trajectory, and video to rosbag'
    )

    declare_enable_analyzer = DeclareLaunchArgument(
        'enable_analyzer',
        default_value='true',
        description='Run trajectory probability analyzer & dataset logger'
    )

    # 1. Main CUDA Cloth Tracker Node
    # (Static TF for base_link -> camera_link is published internally from params_file)
    cuda_tracker_node = Node(
        package='realsense_dust_cloth',
        executable='cloth_tracker_cuda_node',
        name='cloth_tracker_cuda_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_direct_realsense': ParameterValue(
                    LaunchConfiguration('use_direct_realsense'), value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'use_bg_subtraction': ParameterValue(
                    LaunchConfiguration('use_bg_subtraction'), value_type=bool),
            }
        ]
    )

    # 2. Trajectory Probability Analyzer Node (Optional: enable_analyzer:=true)
    analyzer_node = Node(
        package='realsense_dust_cloth',
        executable='trajectory_probability_analyzer.py',
        name='cloth_trajectory_analyzer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_analyzer')),
        parameters=[
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }
        ]
    )

    # 3. RViz2 GUI Node (Optional: rviz:=true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }
        ]
    )

    # 4. Automatic Rosbag Recording (Optional: record:=true)
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '/tf', '/tf_static',
             '/cloth/position',
             '/cloth/trajectory_marker',
             '/cloth/measured_path',
             '/camera/camera/color/image_raw/compressed'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_direct_rs,
        declare_use_sim_time,
        declare_use_bg_subtraction,
        declare_rviz,
        declare_record,
        declare_enable_analyzer,
        cuda_tracker_node,
        analyzer_node,
        rviz_node,
        record_process,
    ])
