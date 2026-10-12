import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_dust_cloth')
    config_file = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cloth_tracker.rviz')

    declare_use_direct_rs = DeclareLaunchArgument(
        'use_direct_realsense',
        default_value='true',
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
        default_value='true',
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

    declare_record = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Record TF, cloth position, and trajectory markers to rosbag automatically'
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
        default_value='1.0',
        description='Camera height offset from base_link (m)'
    )

    declare_camera_x_offset_m = DeclareLaunchArgument(
        'camera_x_offset_m',
        default_value='0.5',
        description='Camera forward offset from base_link (m)'
    )

    declare_camera_parent_frame = DeclareLaunchArgument(
        'camera_parent_frame',
        default_value='base_link',
        description='Parent frame of the RealSense mounting transform'
    )

    declare_camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Child frame of the RealSense mounting transform'
    )

    declare_min_y_m = DeclareLaunchArgument(
        'min_y_m',
        default_value='-1.20',
        description='PassThrough ROI Min Y (m)'
    )

    declare_max_y_m = DeclareLaunchArgument(
        'max_y_m',
        default_value='1.20',
        description='PassThrough ROI Max Y (m)'
    )

    declare_min_z_m = DeclareLaunchArgument(
        'min_z_m',
        default_value='0.0',
        description='PassThrough ROI Min Z (m)'
    )

    declare_max_z_m = DeclareLaunchArgument(
        'max_z_m',
        default_value='3.30',
        description='PassThrough ROI Max Z (m)'
    )

    declare_launch_min_y = DeclareLaunchArgument(
        'launch_min_y',
        default_value='-0.40',
        description='Ballistic launch gate Min Y (m)'
    )

    declare_launch_max_y = DeclareLaunchArgument(
        'launch_max_y',
        default_value='0.40',
        description='Ballistic launch gate Max Y (m)'
    )

    declare_launch_min_z = DeclareLaunchArgument(
        'launch_min_z',
        default_value='1.00',
        description='Ballistic launch gate Min Z (m)'
    )

    declare_launch_max_z = DeclareLaunchArgument(
        'launch_max_z',
        default_value='2.00',
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
                'use_direct_realsense': ParameterValue(
                    LaunchConfiguration('use_direct_realsense'), value_type=bool),
                'publish_debug_cloud': ParameterValue(
                    LaunchConfiguration('publish_debug_cloud'), value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'use_bg_subtraction': ParameterValue(
                    LaunchConfiguration('use_bg_subtraction'), value_type=bool),
                'camera_roll_deg': ParameterValue(
                    LaunchConfiguration('camera_roll_deg'), value_type=float),
                'camera_pitch_deg': ParameterValue(
                    LaunchConfiguration('camera_pitch_deg'), value_type=float),
                'camera_height_offset_m': ParameterValue(
                    LaunchConfiguration('camera_height_offset_m'), value_type=float),
                'camera_x_offset_m': ParameterValue(
                    LaunchConfiguration('camera_x_offset_m'), value_type=float),
                'min_y_m': ParameterValue(
                    LaunchConfiguration('min_y_m'), value_type=float),
                'max_y_m': ParameterValue(
                    LaunchConfiguration('max_y_m'), value_type=float),
                'min_z_m': ParameterValue(
                    LaunchConfiguration('min_z_m'), value_type=float),
                'max_z_m': ParameterValue(
                    LaunchConfiguration('max_z_m'), value_type=float),
                'launch_min_y': ParameterValue(
                    LaunchConfiguration('launch_min_y'), value_type=float),
                'launch_max_y': ParameterValue(
                    LaunchConfiguration('launch_max_y'), value_type=float),
                'launch_min_z': ParameterValue(
                    LaunchConfiguration('launch_min_z'), value_type=float),
                'launch_max_z': ParameterValue(
                    LaunchConfiguration('launch_max_z'), value_type=float),
                'color_rotation_deg': ParameterValue(
                    LaunchConfiguration('color_rotation_deg'), value_type=int),
            }
        ]
    )

    # The RealSense driver owns camera_link -> *_optical_frame. This transform
    # is the only mounting transform and is intentionally configurable here.
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_mount_tf',
        output='screen',
        arguments=[
            LaunchConfiguration('camera_x_offset_m'),
            '0',
            LaunchConfiguration('camera_height_offset_m'),
            '0',
            PythonExpression([
                'float(', LaunchConfiguration('camera_pitch_deg'),
                ') * 3.141592653589793 / 180.0'
            ]),
            PythonExpression([
                'float(', LaunchConfiguration('camera_roll_deg'),
                ') * 3.141592653589793 / 180.0'
            ]),
            LaunchConfiguration('camera_parent_frame'),
            LaunchConfiguration('camera_frame'),
        ],
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
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
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
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }
        ]
    )

    # 4. Automatic rosbag recording (Optional: record:=true)
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
        declare_use_direct_rs,
        declare_publish_debug_cloud,
        declare_use_sim_time,
        declare_use_bg_subtraction,
        declare_enable_analyzer,
        declare_rviz,
        declare_record,
        declare_camera_roll_deg,
        declare_color_rotation_deg,
        declare_camera_pitch_deg,
        declare_camera_height_offset_m,
        declare_camera_x_offset_m,
        declare_camera_parent_frame,
        declare_camera_frame,
        declare_min_y_m,
        declare_max_y_m,
        declare_min_z_m,
        declare_max_z_m,
        declare_launch_min_y,
        declare_launch_max_y,
        declare_launch_min_z,
        declare_launch_max_z,
        cuda_tracker_node,
        camera_tf_node,
        analyzer_node,
        rviz_node,
        record_process,
    ])
