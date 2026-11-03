"""Select one YAML; camera mode, clock and optional helpers all follow that file."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_nodes(context):
    package_dir = get_package_share_directory('realsense_dust_cloth')
    if 'cloth_tracker_params_file' in context.launch_configurations:
        requested = LaunchConfiguration('cloth_tracker_params_file').perform(context)
    elif 'params_file' in context.launch_configurations:
        requested = LaunchConfiguration('params_file').perform(context)
    else:
        requested = 'tracker_params.yaml'
    params_file = os.path.expanduser(requested)
    if not os.path.isabs(params_file) and not os.path.isfile(params_file):
        params_file = os.path.join(package_dir, 'config', params_file)
    if not os.path.isfile(params_file):
        raise FileNotFoundError(f'Parameter YAML not found: {requested}')
    with open(params_file, encoding='utf-8') as stream:
        values = yaml.safe_load(stream)['/**']['ros__parameters']
    for key in ('use_sim_time', 'use_direct_realsense', 'start_rviz',
                'start_analyzer', 'record_inputs'):
        if type(values.get(key)) is not bool:
            raise ValueError(f'{key} must be true or false in {params_file}')
    clock = {'use_sim_time': values['use_sim_time']}
    nodes = [Node(
        package='realsense_dust_cloth', executable='cloth_tracker_cuda_node',
        name='cloth_tracker_cuda_node', output='screen', parameters=[params_file],
    )]
    if values['start_analyzer']:
        nodes.append(Node(
            package='realsense_dust_cloth', executable='trajectory_probability_analyzer.py',
            name='cloth_trajectory_analyzer', output='screen', parameters=[clock],
        ))
    if values['start_rviz']:
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'cloth_tracker.rviz')],
            parameters=[clock],
        ))
    if values['record_inputs']:
        topics = values.get('record_topics')
        if not isinstance(topics, list) or not topics or not all(isinstance(t, str) for t in topics):
            raise ValueError('record_topics must be a nonempty topic list when recording')
        record_cmd = ['ros2', 'bag', 'record']
        if values['use_sim_time']:
            record_cmd.append('--use-sim-time')
        nodes.append(ExecuteProcess(cmd=[*record_cmd, *topics], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cloth_tracker_params_file', default_value='tracker_params.yaml',
                              description='tracker_params.yaml / unity_params.yaml / rosbag_params.yaml'),
        OpaqueFunction(function=_create_nodes),
    ])
