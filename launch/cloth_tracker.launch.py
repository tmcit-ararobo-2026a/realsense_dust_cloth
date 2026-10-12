import os
import math
import yaml
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

    # Read base parameters from tracker_params.yaml as the single source of truth
    yaml_params = {}
    if os.path.exists(default_config_file):
        try:
            with open(default_config_file, 'r') as f:
                loaded = yaml.safe_load(f)
                yaml_params = loaded.get('/**', {}).get('ros__parameters', {})
        except Exception:
            pass

    cam_x = yaml_params.get('camera_x_offset_m', 0.50)
    cam_z = yaml_params.get('camera_height_offset_m', 1.00)
    cam_pitch_deg = yaml_params.get('camera_pitch_deg', 20.0)
    cam_roll_deg = yaml_params.get('camera_roll_deg', 90.0)
    cam_parent = yaml_params.get('camera_parent_frame', 'base_link')
    cam_frame = yaml_params.get('camera_frame', 'camera_link')

    cam_pitch_rad = math.radians(cam_pitch_deg)
    cam_roll_rad = math.radians(cam_roll_deg)

    # =========================================================================
    # 実行モード切り替えスイッチ (Runtime Switches)
    # 調整用パラメータは config/tracker_params.yaml に一本化されています
    # =========================================================================
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_file,
        description='Path to tracker parameter YAML file'
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

    # 2. Camera Mounting Transform (base_link -> camera_link)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_mount_tf',
        output='screen',
        arguments=[
            '--x', str(cam_x),
            '--y', '0.0',
            '--z', str(cam_z),
            '--yaw', '0.0',
            '--pitch', str(cam_pitch_rad),
            '--roll', str(cam_roll_rad),
            '--frame-id', cam_parent,
            '--child-frame-id', cam_frame,
        ],
    )

    # 3. Direct Mode Optical Frame Transform (camera_link -> camera_color_optical_frame)
    # Required for RViz Camera view when using Direct RealSense API
    direct_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_direct_optical_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_direct_realsense')),
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '-1.57079632679',
            '--pitch', '0.0',
            '--roll', '-1.57079632679',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )

    # 4. Trajectory Probability Analyzer Node (Optional: enable_analyzer:=true)
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

    # 5. RViz2 GUI Node (Optional: rviz:=true)
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

    # 6. Automatic Rosbag Recording (Optional: record:=true)
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
        camera_tf_node,
        direct_optical_tf_node,
        analyzer_node,
        rviz_node,
        record_process,
    ])
