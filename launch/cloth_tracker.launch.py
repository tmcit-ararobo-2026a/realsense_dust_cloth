import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _create_tracker_node(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    if not os.path.isabs(params_file) and not os.path.exists(params_file):
        pkg_share = get_package_share_directory('realsense_dust_cloth')
        candidate = os.path.join(pkg_share, 'config', params_file)
        if os.path.exists(candidate):
            params_file = candidate

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_direct_rs_str = LaunchConfiguration('use_direct_realsense').perform(context)

    with open(params_file, encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    values = dict(config.get('/**', {}).get('ros__parameters', {}))
    values.update(config.get('cloth_tracker_cuda_node', {}).get('ros__parameters', {}))
    effective_sim_time = values.get('use_sim_time', False) if use_sim_time_str == '' else use_sim_time_str.lower() == 'true'
    context.launch_configurations['resolved_use_sim_time'] = str(effective_sim_time).lower()
    overrides = {}
    if use_sim_time_str != '':
        overrides['use_sim_time'] = use_sim_time_str.lower() == 'true'
    if use_direct_rs_str != '':
        overrides['use_direct_realsense'] = use_direct_rs_str.lower() == 'true'

    return [
        Node(
            package='realsense_dust_cloth',
            executable='cloth_tracker_cuda_node',
            name='cloth_tracker_cuda_node',
            output='screen',
            parameters=[params_file, overrides],
        )
    ]


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
        default_value='',
        description='Optional override: true (direct librealsense2) or false (ROS 2 topics). Empty uses YAML.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='',
        description='Use simulation clock (Isaac Sim / rosbag)'
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
    cuda_tracker_node = OpaqueFunction(function=_create_tracker_node)

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
                    LaunchConfiguration('resolved_use_sim_time'), value_type=bool),
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
                    LaunchConfiguration('resolved_use_sim_time'), value_type=bool),
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
             '/cloth/most_probable_path',
             '/cloth/probability_markers',
             '/cloth/predicted_landing_point',
             '/cloth/debug/roi_markers',
             '/cloth/debug/cluster_points',
             '/cloth/debug/filtered_points',
             '/camera/camera/depth/image_rect_raw',
             '/camera/camera/depth/camera_info',
             '/camera/camera/color/image_raw/compressed',
             '/camera/camera/color/camera_info'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_direct_rs,
        declare_use_sim_time,
        declare_rviz,
        declare_record,
        declare_enable_analyzer,
        cuda_tracker_node,
        analyzer_node,
        rviz_node,
        record_process,
    ])
