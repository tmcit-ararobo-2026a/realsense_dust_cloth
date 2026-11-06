"""Exercise the actual Qt widgets and ROS parameter services against temporary YAML files."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import traceback
import yaml
import ctypes
import ctypes.util

count = ctypes.c_int()
runtime = ctypes.CDLL(ctypes.util.find_library("cudart"))
if runtime.cudaGetDeviceCount(ctypes.byref(count)) != 0 or count.value == 0:
    sys.exit(77)
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cloth_tuner_test', ROOT/'realsense_dust_cloth/param_tuner_gui.py')
gui = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = gui
spec.loader.exec_module(gui)
app = QApplication.instance() or QApplication([])
failures = []

with tempfile.TemporaryDirectory() as directory:
    config = Path(directory)
    for source in (ROOT/'config').glob('*.yaml'):
        (config/source.name).write_bytes(source.read_bytes())
    original = {p.name: p.read_text() for p in config.glob('*.yaml')}
    gui.discover_config_dirs = lambda: [config]
    # Offline restore: the rosbag profile must not default to all-profile synchronization.
    class OfflineBridge:
        def __init__(self, **kwargs): self.target_node='/not_running'
        def fetch_remote_params(self, names): return None
        def is_remote_available(self): return False
        def destroy_node(self): pass
    real_bridge = gui.RosParamBridge
    gui.RosParamBridge = OfflineBridge
    original_spin = gui.rclpy.spin_once
    gui.rclpy.spin_once = lambda *args, **kwargs: None
    def offline():
        try:
            win = next(w for w in app.topLevelWidgets() if hasattr(w, '_widgets'))
            assert win.profile_combo.currentIndex() == 3
            assert win._widgets['camera_roll_deg'][1].value() == 90
            win._widgets['launch_min_x'][1].setValue(1.0)
            assert yaml.safe_load((config/'rosbag_params.yaml').read_text())['/**']['ros__parameters']['launch_min_x'] == 1.0
            assert (config/'tracker_params.yaml').read_text() == original['tracker_params.yaml']
            assert (config/'unity_params.yaml').read_text() == original['unity_params.yaml']
            # Reload must not replace the adjustment-before snapshot.
            win.reload_parameters()
            win.reset_to_initial()
            assert all((config/name).read_text() == text for name, text in original.items())
            win.timer.stop(); win.close()
        except BaseException:
            failures.append(traceback.format_exc())
        finally: app.quit()
    QTimer.singleShot(0, offline)
    gui.run_gui('/not_running', 'rosbag_params.yaml')
    gui.RosParamBridge = real_bridge
    gui.rclpy.spin_once = original_spin

    # A real GPU node is used for online apply/reject/restore; no sensor/robot commands.
    process = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(config/'unity_params.yaml'), '-p', 'use_sim_time:=false'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    def online():
        probe = None
        try:
            win = next(w for w in app.topLevelWidgets() if hasattr(w, '_widgets') and w.isVisible())
            probe = real_bridge(target_node='/cloth_tracker_cuda_node')
            for _ in range(20):
                before = probe.fetch_remote_params(['launch_min_x'])
                if before: break
                time.sleep(.1)
            assert before and before['launch_min_x'] == .8
            # Open may have raced node discovery; the GUI captures pre-edit values on first edit too.
            win._widgets['launch_min_x'][1].setValue(1.0)
            assert probe.fetch_remote_params(['launch_min_x'])['launch_min_x'] == 1.0
            assert yaml.safe_load((config/'unity_params.yaml').read_text())['/**']['ros__parameters']['launch_min_x'] == 1.0
            bad = next(s for _, specs in gui.TAB_SPECS for s in specs if s.name == 'debug_publish_hz')
            unchanged = (config/'unity_params.yaml').read_text()
            win.on_value_changed(bad, 0.0)
            assert (config/'unity_params.yaml').read_text() == unchanged
            win.reset_to_initial()
            assert probe.fetch_remote_params(['launch_min_x'])['launch_min_x'] == .8
            assert all((config/name).read_text() == text for name, text in original.items())
            win.timer.stop(); win.close()
        except BaseException:
            failures.append(traceback.format_exc())
        finally:
            if probe: probe.destroy_node()
            app.quit()
    QTimer.singleShot(0, online)
    try: gui.run_gui('/cloth_tracker_cuda_node', 'unity_params.yaml')
    finally:
        process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill();process.wait()
    if failures:
        print('\n'.join(failures));sys.exit(1)
print('Qt offline profile isolation, wildcard YAML, 90-degree roll, apply/reject and session restore passed')
