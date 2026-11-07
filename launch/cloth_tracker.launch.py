"""Select one YAML; camera mode, clock and optional helpers all follow that file."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch_ros.actions import Node

TRACKER_BOOL_KEYS = (
    'use_sim_time',
    'use_direct_realsense',
    'start_rviz',
    'start_analyzer',
    'record_inputs',
)
TRACKER_ONLY_KEYS = TRACKER_BOOL_KEYS[1:]


def _resolve_params_path(requested: str, package_dir: str) -> str:
    # unity_sim.launch.py 側の旧指定名をパッケージリネーム後の正本へ透過解決する。
    if requested.strip() == 'unity_map_params.yaml':
        requested = 'unity_params.yaml'
    path = os.path.expanduser(requested)
    if not os.path.isabs(path) and not os.path.isfile(path):
        path = os.path.join(package_dir, 'config', path)
    return path


def _select_params_file(context, package_dir: str) -> str:
    cloth_tracker_requested = context.launch_configurations.get(
        'cloth_tracker_params_file', '').strip()
    tracker_requested = context.launch_configurations.get(
        'tracker_params_file', '').strip()
    legacy_requested = context.launch_configurations.get('params_file', '').strip()

    if cloth_tracker_requested and cloth_tracker_requested != 'tracker_params.yaml':
        return _resolve_params_path(cloth_tracker_requested, package_dir)

    if tracker_requested and tracker_requested != 'tracker_params.yaml':
        return _resolve_params_path(tracker_requested, package_dir)

    if legacy_requested and legacy_requested != 'tracker_params.yaml':
        candidate = _resolve_params_path(legacy_requested, package_dir)
        if os.path.isfile(candidate):
            with open(candidate, encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
            values = data.get('/**', {}).get('ros__parameters', {})
            in_this_pkg = os.path.abspath(candidate).startswith(
                os.path.abspath(package_dir) + os.sep)
            has_tracker_keys = isinstance(values, dict) and any(
                k in values for k in TRACKER_ONLY_KEYS)
            if not in_this_pkg and not has_tracker_keys:
                fallback = cloth_tracker_requested or tracker_requested or 'tracker_params.yaml'
                return _resolve_params_path(fallback, package_dir)
        return candidate

    fallback = cloth_tracker_requested or tracker_requested or 'tracker_params.yaml'
    return _resolve_params_path(fallback, package_dir)


def _create_nodes(context):
    package_dir = get_package_share_directory('realsense_dust_cloth')
    params_file = _select_params_file(context, package_dir)
    if not os.path.isfile(params_file):
        raise FileNotFoundError(f'Parameter YAML not found: {params_file}')
    with open(params_file, encoding='utf-8') as stream:
        values = yaml.safe_load(stream)['/**']['ros__parameters']
    for key in TRACKER_BOOL_KEYS:
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
        GroupAction([
            DeclareLaunchArgument(
                'cloth_tracker_params_file', default_value='tracker_params.yaml',
                description='tracker_params.yaml / unity_params.yaml / rosbag_params.yaml'),
            DeclareLaunchArgument(
                'tracker_params_file', default_value='tracker_params.yaml',
                description='tracker_params.yaml / unity_params.yaml / rosbag_params.yaml'),
            DeclareLaunchArgument(
                'params_file', default_value='tracker_params.yaml',
                description='tracker_params.yaml / unity_params.yaml / rosbag_params.yaml'),
            OpaqueFunction(function=_create_nodes),
        ], scoped=True),
    ])
