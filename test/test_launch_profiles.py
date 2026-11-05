"""The selected YAML alone controls input mode and clocks; no node is started."""
import os
from pathlib import Path
import runpy
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

os.environ.setdefault('ROS_LOG_DIR', '/tmp/cloth_launch_profile_tests')
ROOT = Path(__file__).resolve().parents[1]


class Profiles(unittest.TestCase):
    def test_selected_file_and_clock(self):
        module = runpy.run_path(str(ROOT / 'launch' / 'cloth_tracker.launch.py'))
        create = module['_create_nodes']
        create.__globals__['get_package_share_directory'] = lambda _: str(ROOT)
        create.__globals__['Node'] = lambda **kwargs: kwargs
        create.__globals__['ExecuteProcess'] = lambda **kwargs: kwargs
        files = sorted((ROOT / 'config').glob('*.yaml'))
        self.assertEqual({p.name for p in files}, {'tracker_params.yaml', 'unity_params.yaml', 'rosbag_params.yaml'})
        for profile in files:
            # Test both cloth_tracker_params_file and legacy params_file
            for key in ('cloth_tracker_params_file', 'params_file'):
                context = LaunchContext()
                context.launch_configurations[key] = profile.name
                nodes = create(context)
                self.assertEqual(nodes[0]['parameters'], [str(profile)])
            # Enable helpers in a separate copy and verify they use the same clock.
            data = yaml.safe_load(profile.read_text())
            values = data['/**']['ros__parameters']
            values['start_rviz'] = values['start_analyzer'] = values['record_inputs'] = True
            with tempfile.TemporaryDirectory() as directory:
                modified = Path(directory) / profile.name
                modified.write_text(yaml.safe_dump(data))
                context = LaunchContext()
                context.launch_configurations['cloth_tracker_params_file'] = str(modified)
                nodes = create(context)
                self.assertEqual(nodes[0]['parameters'], [str(modified)])
                for helper in nodes[1:3]:
                    self.assertEqual(helper['parameters'], [{'use_sim_time': values['use_sim_time']}])
                self.assertEqual('--use-sim-time' in nodes[3]['cmd'], values['use_sim_time'])

    def test_coexistence_with_localization_params_file(self):
        module = runpy.run_path(str(ROOT / 'launch' / 'cloth_tracker.launch.py'))
        create = module['_create_nodes']
        create.__globals__['get_package_share_directory'] = lambda _: str(ROOT)
        create.__globals__['Node'] = lambda **kwargs: kwargs
        create.__globals__['ExecuteProcess'] = lambda **kwargs: kwargs

        with tempfile.TemporaryDirectory() as directory:
            foreign_yaml = Path(directory) / 'blue_params.yaml'
            foreign_yaml.write_text(yaml.safe_dump({
                '/**': {'ros__parameters': {'initial_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}}}
            }))

            # 1. Parent context polluted by gn10's params_file -> falls back to tracker_params.yaml
            context = LaunchContext()
            context.launch_configurations['params_file'] = str(foreign_yaml)
            nodes = create(context)
            self.assertEqual(nodes[0]['parameters'], [str(ROOT / 'config' / 'tracker_params.yaml')])

            # 2. Explicit tracker_params_file / cloth_tracker_params_file overrides polluted params_file
            context.launch_configurations['tracker_params_file'] = 'unity_params.yaml'
            nodes = create(context)
            self.assertEqual(nodes[0]['parameters'], [str(ROOT / 'config' / 'unity_params.yaml')])
            context.launch_configurations['tracker_params_file'] = ''
            context.launch_configurations['cloth_tracker_params_file'] = 'unity_params.yaml'
            nodes = create(context)
            self.assertEqual(nodes[0]['parameters'], [str(ROOT / 'config' / 'unity_params.yaml')])

            # 3. Tracker YAML missing required boolean still raises ValueError
            broken_tracker_yaml = Path(directory) / 'broken_tracker.yaml'
            broken_tracker_yaml.write_text(yaml.safe_dump({
                '/**': {'ros__parameters': {'use_direct_realsense': True}}
            }))
            broken_ctx = LaunchContext()
            broken_ctx.launch_configurations['params_file'] = str(broken_tracker_yaml)
            with self.assertRaises(ValueError):
                create(broken_ctx)

        # 4. Scoped GroupAction prevents cloth_tracker.launch.py from leaking params_file to gn10
        parent_ctx = LaunchContext()
        ld = module['generate_launch_description']()
        for entity in ld.entities:
            entity.visit(parent_ctx)
        DeclareLaunchArgument('params_file', default_value='/gn10/blue_params.yaml').visit(parent_ctx)
        self.assertEqual(parent_ctx.launch_configurations['params_file'], '/gn10/blue_params.yaml')

    def test_yaml_comment_preserving_update(self):
        gui_mod = runpy.run_path(str(ROOT / 'realsense_dust_cloth' / 'param_tuner_gui.py'))
        update_fn = gui_mod['update_yaml_file_in_place']
        orig_text = (ROOT / 'config' / 'tracker_params.yaml').read_text(encoding='utf-8')
        with tempfile.TemporaryDirectory() as directory:
            temp_yaml = Path(directory) / 'tracker_params.yaml'
            temp_yaml.write_text(orig_text, encoding='utf-8')
            changed = update_fn(temp_yaml, {
                'launch_max_x': (2.35, 'double'),
                'min_cluster_points': (42, 'int'),
                'publish_debug_cloud': (False, 'bool'),
            })
            self.assertTrue(changed)
            updated_text = temp_yaml.read_text(encoding='utf-8')
            # Ensure Japanese comments are preserved
            self.assertIn('# --- 4. 追跡する空間ROI・深度ノイズ除去 ---', updated_text)
            self.assertIn('launch_max_x: 2.35', updated_text)
            self.assertIn('min_cluster_points: 42', updated_text)
            self.assertIn('publish_debug_cloud: false', updated_text)
            parsed = yaml.safe_load(updated_text)['/**']['ros__parameters']
            self.assertAlmostEqual(parsed['launch_max_x'], 2.35)
            self.assertEqual(parsed['min_cluster_points'], 42)
            self.assertFalse(parsed['publish_debug_cloud'])


if __name__ == '__main__':
    unittest.main()
