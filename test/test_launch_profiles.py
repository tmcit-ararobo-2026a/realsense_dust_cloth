"""The selected YAML alone controls input mode and clocks; no node is started."""
import os
from pathlib import Path
import runpy
import tempfile
import unittest

import yaml
from launch import LaunchContext

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
            context = LaunchContext()
            context.launch_configurations['params_file'] = profile.name
            nodes = create(context)
            self.assertEqual(nodes[0]['parameters'], [str(profile)])
            # Enable helpers in a separate copy and verify they use the same clock.
            data = yaml.safe_load(profile.read_text())
            values = data['/**']['ros__parameters']
            values['start_rviz'] = values['start_analyzer'] = values['record_inputs'] = True
            with tempfile.TemporaryDirectory() as directory:
                modified = Path(directory) / profile.name
                modified.write_text(yaml.safe_dump(data))
                context.launch_configurations['params_file'] = str(modified)
                nodes = create(context)
                self.assertEqual(nodes[0]['parameters'], [str(modified)])
                for helper in nodes[1:3]:
                    self.assertEqual(helper['parameters'], [{'use_sim_time': values['use_sim_time']}])
                self.assertEqual('--use-sim-time' in nodes[3]['cmd'], values['use_sim_time'])


if __name__ == '__main__':
    unittest.main()
