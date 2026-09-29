import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cloth_tracker.rviz')

    declare_use_direct_rs = DeclareLaunchArgument(
        'use_direct_realsense',
        default_value='false',
        description='Whether to stream directly from RealSense C++ API instead of ROS 2 topic'
    )

    declare_publish_debug_cloud = DeclareLaunchArgument(
        'publish_debug_cloud',
        default_value='true',
        description='Whether to publish extracted cloth cluster pointcloud (for debug/RViz)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Isaac Sim / bag) clock if true'
    )

    declare_use_bg_subtraction = DeclareLaunchArgument(
        'use_bg_subtraction',
        default_value='false',
        description='Whether to use static background subtraction (set to false if moving while shooting)'
    )

    declare_enable_analyzer = DeclareLaunchArgument(
        'enable_analyzer',
        default_value='true',
        description='Whether to run trajectory probability analyzer and master path publisher'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to open RViz2 visualization window'
    )

    declare_camera_roll_deg = DeclareLaunchArgument(
        'camera_roll_deg',
        default_value='90.0',
        description='Camera roll angle in degrees (+90 or -90 for vertical)'
    )

    declare_color_rotation_deg = DeclareLaunchArgument(
        'color_rotation_deg',
        default_value='180',
        description='Color image rotation in degrees (0, 90, 180, 270) to make display upright'
    )

    declare_camera_pitch_deg = DeclareLaunchArgument(
        'camera_pitch_deg',
        default_value='20.0',
        description='Camera pitch angle in degrees (upward tilt)'
    )

    declare_camera_height_offset_m = DeclareLaunchArgument(
        'camera_height_offset_m',
        default_value='0.35',
        description='Camera height offset from base_link (m)'
    )

    declare_camera_x_offset_m = DeclareLaunchArgument(
        'camera_x_offset_m',
        default_value='0.15',
        description='Camera forward offset from base_link (m)'
    )

    declare_min_y_m = DeclareLaunchArgument(
        'min_y_m',
        default_value='-0.35',
        description='PassThrough ROI Min Y (m)'
    )

    declare_max_y_m = DeclareLaunchArgument(
        'max_y_m',
        default_value='1.00',
        description='PassThrough ROI Max Y (m)'
    )

    declare_min_z_m = DeclareLaunchArgument(
        'min_z_m',
        default_value='0.0',
        description='PassThrough ROI Min Z (m)'
    )

    declare_max_z_m = DeclareLaunchArgument(
        'max_z_m',
        default_value='2.20',
        description='PassThrough ROI Max Z (m)'
    )

    declare_launch_min_y = DeclareLaunchArgument(
        'launch_min_y',
        default_value='-0.60',
        description='Ballistic launch gate Min Y (m)'
    )

    declare_launch_max_y = DeclareLaunchArgument(
        'launch_max_y',
        default_value='0.60',
        description='Ballistic launch gate Max Y (m)'
    )

    declare_launch_min_z = DeclareLaunchArgument(
        'launch_min_z',
        default_value='0.50',
        description='Ballistic launch gate Min Z (m)'
    )

    declare_launch_max_z = DeclareLaunchArgument(
        'launch_max_z',
        default_value='1.80',
        description='Ballistic launch gate Max Z (m)'
    )

    # 1. CUDA Cloth Tracker Node
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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_bg_subtraction': LaunchConfiguration('use_bg_subtraction'),
                'camera_roll_deg': LaunchConfiguration('camera_roll_deg'),
                'camera_pitch_deg': LaunchConfiguration('camera_pitch_deg'),
                'camera_height_offset_m': LaunchConfiguration('camera_height_offset_m'),
                'camera_x_offset_m': LaunchConfiguration('camera_x_offset_m'),
                'min_y_m': LaunchConfiguration('min_y_m'),
                'max_y_m': LaunchConfiguration('max_y_m'),
                'min_z_m': LaunchConfiguration('min_z_m'),
                'max_z_m': LaunchConfiguration('max_z_m'),
                'launch_min_y': LaunchConfiguration('launch_min_y'),
                'launch_max_y': LaunchConfiguration('launch_max_y'),
                'launch_min_z': LaunchConfiguration('launch_min_z'),
                'launch_max_z': LaunchConfiguration('launch_max_z'),
                'color_rotation_deg': LaunchConfiguration('color_rotation_deg'),
            }
        ]
    )

    # 2. Trajectory Probability Analyzer & Master Path Publisher
    analyzer_node = Node(
        package='realsense_dust_cloth',
        executable='trajectory_probability_analyzer.py',
        name='cloth_trajectory_analyzer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_analyzer')),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    # 3. RViz2 GUI (Optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    return LaunchDescription([
        declare_use_direct_rs,
        declare_publish_debug_cloud,
        declare_use_sim_time,
        declare_use_bg_subtraction,
        declare_enable_analyzer,
        declare_rviz,
        declare_camera_roll_deg,
        declare_color_rotation_deg,
        declare_camera_pitch_deg,
        declare_camera_height_offset_m,
        declare_camera_x_offset_m,
        declare_min_y_m,
        declare_max_y_m,
        declare_min_z_m,
        declare_max_z_m,
        declare_launch_min_y,
        declare_launch_max_y,
        declare_launch_min_z,
        declare_launch_max_z,
        cuda_tracker_node,
        analyzer_node,
        rviz_node,
    ])
