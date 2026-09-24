import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.conditions import IfCondition

def generate_launch_description():
    pkg_dir = get_package_share_directory('realsense_dust_cloth')
    config_file = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'cloth_tracker.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (rosbag) clock if true'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for 3D visualization'
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/depth/image_rect_raw',
        description='Depth image topic from RealSense D435'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/depth/camera_info',
        description='CameraInfo topic from RealSense D435'
    )

    # ベルト先端・縦置きRealSenseの静的TF (base_link -> camera_link)
    # 縦置き (ロール90度回転) を適用
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '0.25', '0.0', '0.35',        # x, y, z [m] (ベルト直動先端位置)
            '0.7071', '0.0', '0.0', '0.7071', # qx, qy, qz, qw (ロール90度縦置き)
            'base_link', 'camera_link'
        ]
    )

    recorder_node = Node(
        package='realsense_dust_cloth',
        executable='depth_trajectory_recorder_node',
        name='depth_trajectory_recorder_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        static_tf_node,
        recorder_node,
        rviz_node,
    ])

