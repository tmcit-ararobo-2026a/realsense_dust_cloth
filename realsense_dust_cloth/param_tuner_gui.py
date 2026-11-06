#!/usr/bin/env python3
"""Real-time parameter tuner GUI for cloth_tracker_cuda_node (Local Unity & Remote Jetson)."""

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys
from typing import Dict, List, Optional, Tuple, Union

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_msgs.msg import Empty
import yaml

ParamScalar = Union[float, int, bool]


@dataclass(frozen=True)
class ParamSpec:
    name: str
    label: str
    param_type: str  # "double", "int", "bool"
    min_val: float = 0.0
    max_val: float = 1.0
    step: float = 0.01
    decimals: int = 2


TAB_SPECS: List[Tuple[str, List[ParamSpec]]] = [
    (
        "1. 射出判定 & カメラ取付",
        [
            ParamSpec("launch_min_x", "射出枠 X最小 (前方) [m]", "double", 0.0, 5.0, 0.05, 2),
            ParamSpec("launch_max_x", "射出枠 X最大 (前方) [m]", "double", 0.2, 6.0, 0.05, 2),
            ParamSpec("launch_min_y", "射出枠 Y最小 (右) [m]", "double", -3.0, 3.0, 0.05, 2),
            ParamSpec("launch_max_y", "射出枠 Y最大 (左) [m]", "double", -3.0, 3.0, 0.05, 2),
            ParamSpec("launch_min_z", "射出枠 Z最小 (下限高さ) [m]", "double", 0.0, 4.0, 0.05, 2),
            ParamSpec("launch_max_z", "射出枠 Z最大 (上限高さ) [m]", "double", 0.5, 5.0, 0.05, 2),
            ParamSpec("min_launch_vx", "射出最小前方速度 vx [m/s]", "double", 0.1, 10.0, 0.1, 2),
            ParamSpec("min_launch_move_m", "射出最小移動距離 [m]", "double", 0.01, 1.0, 0.01, 2),
            ParamSpec("pending_reset_sec", "射出候補リセット時間 [s]", "double", 0.05, 1.0, 0.01, 2),
            ParamSpec("pending_max_drop_m", "射出候補最大落下許容 [m]", "double", 0.01, 1.0, 0.01, 2),
            ParamSpec("camera_pitch_deg", "カメラ仰角 Pitch [deg]", "double", -45.0, 45.0, 0.5, 2),
            ParamSpec("camera_roll_deg", "カメラ傾き Roll [deg]", "double", -180.0, 180.0, 0.5, 2),
            ParamSpec("camera_x_offset_m", "カメラ前後オフセット X [m]", "double", -1.0, 1.0, 0.01, 3),
            ParamSpec("camera_height_offset_m", "カメラ高さオフセット Z [m]", "double", 0.0, 2.0, 0.01, 3),
        ],
    ),
    (
        "2. クラスタ判定 & 空間ROI",
        [
            ParamSpec("min_cluster_points", "1フレーム目最小点数 [点]", "int", 1, 200, 1, 0),
            ParamSpec("min_tracking_points", "2フレーム目以降最小点数 [点]", "int", 1, 100, 1, 0),
            ParamSpec("cluster_eps_m", "クラスタ結合距離 eps [m]", "double", 0.01, 0.50, 0.01, 3),
            ParamSpec("voxel_size_m", "ボクセル間引きサイズ [m]", "double", 0.005, 0.10, 0.005, 3),
            ParamSpec("min_cloth_size_m", "雑巾の最小サイズ [m]", "double", 0.01, 0.50, 0.01, 2),
            ParamSpec("max_cloth_size_m", "雑巾の最大サイズ (構造物除外) [m]", "double", 0.10, 2.00, 0.05, 2),
            ParamSpec("prediction_distance_weight", "予測位置への近さ重み", "double", 0.0, 20.0, 0.5, 2),
            ParamSpec("cluster_size_weight", "正規化した点数の重み", "double", 0.0, 2.0, 0.01, 3),
            ParamSpec("launch_forward_weight", "射出時の前方優先重み", "double", 0.0, 10.0, 0.1, 2),
            ParamSpec("external_tracking_keep_radius_m", "gn10追従保護半径 [m]", "double", 0.0, 3.0, 0.05, 2),
            ParamSpec("min_x_m", "ROI X最小 (前方) [m]", "double", 0.0, 5.0, 0.1, 2),
            ParamSpec("max_x_m", "ROI X最大 (前方) [m]", "double", 1.0, 15.0, 0.5, 2),
            ParamSpec("min_y_m", "ROI Y最小 (右) [m]", "double", -10.0, 0.0, 0.5, 2),
            ParamSpec("max_y_m", "ROI Y最大 (左) [m]", "double", 0.0, 10.0, 0.5, 2),
            ParamSpec("min_z_m", "ROI Z最小 (高さ) [m]", "double", -2.0, 3.0, 0.1, 2),
            ParamSpec("max_z_m", "ROI Z最大 (高さ) [m]", "double", 0.5, 8.0, 0.1, 2),
        ],
    ),
    (
        "3. 飛翔予測 & 物理ガード",
        [
            ParamSpec("tube_radius_m", "予測チューブ半径 [m]", "double", 0.10, 2.50, 0.05, 2),
            ParamSpec("vel_smoothing_beta", "速度平滑化係数 beta", "double", 0.05, 1.0, 0.05, 2),
            ParamSpec("drag_coeff", "空気抵抗係数 drag", "double", 0.0, 1.0, 0.01, 3),
            ParamSpec("centroid_position_tolerance_m", "重心ジッター許容幅 [m]", "double", 0.0, 0.30, 0.01, 3),
            ParamSpec("vertical_jitter_tolerance_m", "上下ジッター許容幅 [m]", "double", 0.0, 0.30, 0.01, 3),
            ParamSpec("backward_jitter_tolerance_m", "前後ジッター許容幅 [m]", "double", 0.0, 0.30, 0.01, 3),
            ParamSpec("max_total_accel_m_s2", "最大合成加速度 [m/s²]", "double", 5.0, 100.0, 1.0, 1),
            ParamSpec("min_vertical_accel_m_s2", "最小鉛直加速度 [m/s²]", "double", -100.0, 0.0, 1.0, 1),
            ParamSpec("max_lateral_accel_m_s2", "最大横方向加速度 [m/s²]", "double", 5.0, 100.0, 1.0, 1),
            ParamSpec("max_heading_deg", "最大進行方向変化角 [deg]", "double", 5.0, 180.0, 5.0, 1),
            ParamSpec("min_flight_vx_m_s", "飛翔中最小前方速度 vx [m/s]", "double", -5.0, 5.0, 0.1, 2),
            ParamSpec("max_vx_m_s", "最大前方速度 vx [m/s]", "double", 2.0, 30.0, 0.5, 1),
            ParamSpec("max_abs_vy_m_s", "最大横速度 |vy| [m/s]", "double", 1.0, 20.0, 0.5, 1),
            ParamSpec("min_vz_m_s", "最小鉛直速度 vz [m/s]", "double", -30.0, 0.0, 0.5, 1),
            ParamSpec("max_vz_m_s", "最大鉛直速度 vz [m/s]", "double", 0.0, 30.0, 0.5, 1),
            ParamSpec("max_lost_frames", "ロスト許容フレーム数 [frame]", "int", 0, 30, 1, 0),
            ParamSpec("max_anomaly_frames", "異常許容フレーム数 [frame]", "int", 1, 20, 1, 0),
            ParamSpec("landing_min_z", "着地判定高さ Z [m]", "double", -1.0, 2.0, 0.05, 2),
            ParamSpec("reject_ascent_after_apex", "頂点到達後の再上昇を棄却", "bool"),
            ParamSpec("abort_on_anomaly", "連続異常時に即座に追跡終了", "bool"),
            ParamSpec("publish_debug_cloud", "RVizデバッグ点群・マーカー配信", "bool"),
        ],
    ),
    (
        "4. 背景差分・表示・射出連携",
        [
            ParamSpec("debug_publish_hz", "表示点群の最大配信頻度 [Hz]", "double", 1.0, 30.0, 1.0, 1),
            ParamSpec("static_marker_interval_sec", "軌跡表示の最小間隔 [s]", "double", 0.02, 1.0, 0.01, 2),
            ParamSpec("publish_raw_depth", "USB生深度配信（帯域大・bag記録用）", "bool"),
            ParamSpec("bg_update_interval_sec", "待機中の背景更新間隔 [s]", "double", 0.01, 1.0, 0.01, 2),
            ParamSpec("use_bg_subtraction", "GPU深度背景差分を有効化", "bool"),
            ParamSpec("bg_diff_thresh_m", "背景判定手前しきい値 [m]", "double", 0.02, 0.50, 0.01, 2),
            ParamSpec("bg_pre_shot_exclude_sec", "射出前除外時間 [s]", "double", 0.0, 0.30, 0.01, 2),
            ParamSpec("shot_window_sec", "背景凍結ウィンドウ時間 [s]", "double", 0.5, 10.0, 0.1, 1),
        ],
    ),
]


def format_yaml_scalar(value: ParamScalar, param_type: str) -> str:
    if param_type == "bool":
        return "true" if bool(value) else "false"
    if param_type == "int":
        return str(int(round(float(value))))
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    if "." not in text:
        text += ".0"
    return text


def update_yaml_file_in_place(
    yaml_path: Path, updates: Dict[str, Tuple[ParamScalar, str]]
) -> bool:
    """Update parameter values in a YAML file while preserving all comments and layout."""
    if not yaml_path.is_file():
        return False
    content = yaml_path.read_text(encoding="utf-8")
    lines = content.splitlines(keepends=True)
    modified = False

    for idx, line in enumerate(lines):
        for name, (val, param_type) in updates.items():
            pattern = re.compile(
                rf"^(\s*{re.escape(name)}\s*:\s*)([^\s#]+)(.*)$"
            )
            match = pattern.match(line.rstrip("\r\n"))
            if match:
                newline = "\r\n" if line.endswith("\r\n") else ("\n" if line.endswith("\n") else "")
                formatted = format_yaml_scalar(val, param_type)
                new_line = f"{match.group(1)}{formatted}{match.group(3)}{newline}"
                if new_line != line:
                    lines[idx] = new_line
                    modified = True

    if modified:
        yaml_path.write_text("".join(lines), encoding="utf-8")
    return modified


def restore_yaml_values(path: Path, before: str, names: set) -> None:
    """Restore only GUI-edited scalar tokens; retain comments and other edits."""
    text = path.read_text(encoding="utf-8")
    for name in names:
        pattern = rf"(?m)^([ \t]*{re.escape(name)}[ \t]*:[ \t]*)([^\s#]+)"
        original = re.search(pattern, before)
        if original:
            text = re.sub(pattern, lambda m: m[1] + original[2], text, count=1)
    path.write_text(text, encoding="utf-8")


def discover_config_dirs() -> List[Path]:
    dirs: List[Path] = []
    script_path = Path(__file__).resolve()
    # Check if running from source tree
    src_cfg = script_path.parent.parent / "config"
    if src_cfg.is_dir():
        dirs.append(src_cfg)

    try:
        share_dir = Path(get_package_share_directory("realsense_dust_cloth"))
        share_cfg = share_dir / "config"
        if share_cfg.is_dir() and share_cfg not in dirs:
            dirs.append(share_cfg)
        parent = share_dir
        for _ in range(5):
            parent = parent.parent
            candidate = parent / "src" / "realsense_dust_cloth" / "config"
            if candidate.is_dir() and candidate not in dirs:
                dirs.append(candidate)
                break
    except Exception:
        pass
    return dirs


def read_yaml_parameters(path: Path) -> Dict[str, ParamScalar]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    # ROSのワイルドカードと明示ノード名の両方に対応。明示設定を優先する。
    values = dict(data.get("/**", {}).get("ros__parameters", {}) or {})
    for key in ("cloth_tracker_cuda_node", "/cloth_tracker_cuda_node"):
        values.update(data.get(key, {}).get("ros__parameters", {}) or {})
    return values


def load_yaml_defaults(profile_name: str = "tracker_params.yaml") -> Dict[str, ParamScalar]:
    for cfg_dir in discover_config_dirs():
        yaml_file = cfg_dir / profile_name
        if yaml_file.is_file():
            return read_yaml_parameters(yaml_file)
    return {}


class RosParamBridge(Node):
    def __init__(self, target_node: str = "/cloth_tracker_cuda_node") -> None:
        super().__init__("cloth_tracker_param_tuner_gui")
        self.target_node = target_node.rstrip("/")
        self.get_cli = self.create_client(
            GetParameters, f"{self.target_node}/get_parameters"
        )
        self.set_cli = self.create_client(
            SetParametersAtomically, f"{self.target_node}/set_parameters_atomically"
        )
        self.reset_pub = self.create_publisher(Empty, "/cloth/reset", 10)

    def publish_reset(self) -> None:
        self.reset_pub.publish(Empty())

    def is_remote_available(self) -> bool:
        return self.set_cli.service_is_ready()

    def fetch_remote_params(self, names: List[str]) -> Optional[Dict[str, ParamScalar]]:
        if not self.get_cli.wait_for_service(timeout_sec=0.3):
            return None
        req = GetParameters.Request()
        req.names = names
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)
        if not future.done() or future.result() is None:
            return None
        values: Dict[str, ParamScalar] = {}
        for name, pval in zip(names, future.result().values):
            if pval.type == ParameterType.PARAMETER_DOUBLE:
                values[name] = float(pval.double_value)
            elif pval.type == ParameterType.PARAMETER_INTEGER:
                values[name] = int(pval.integer_value)
            elif pval.type == ParameterType.PARAMETER_BOOL:
                values[name] = bool(pval.bool_value)
        return values

    def push_remote_param(
        self, name: str, value: ParamScalar, param_type: str
    ) -> bool:
        return self.push_remote_params({name: (value, param_type)})

    def push_remote_params(self, updates: Dict[str, Tuple[ParamScalar, str]]) -> bool:
        if not updates:
            return True
        if not self.set_cli.wait_for_service(timeout_sec=0.15):
            return False
        req = SetParametersAtomically.Request()
        for name, (value, param_type) in updates.items():
            param = Parameter(name=name)
            if param_type == "bool":
                param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value))
            elif param_type == "int":
                param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(round(value)))
            else:
                param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            req.parameters.append(param)
        future = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)
        return bool(future.done() and future.result() and future.result().result.successful)


def run_gui(target_node: str, initial_profile: str) -> int:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QPushButton,
        QScrollArea,
        QSlider,
        QSpinBox,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )

    rclpy.init()
    bridge = RosParamBridge(target_node=target_node)
    app = QApplication.instance() or QApplication(sys.argv)

    all_specs: Dict[str, ParamSpec] = {
        spec.name: spec for _, specs in TAB_SPECS for spec in specs
    }

    class TunerWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle(
                f"RealSense 雑巾トラッカー パラメータ調整 GUI ({target_node})"
            )
            self.resize(820, 760)
            self._updating_ui = False
            self._widgets: Dict[str, Tuple[Optional[QSlider], QWidget]] = {}
            self._remote_before: Dict[str, ParamScalar] = {}
            self._remote_changed = set()
            self._file_before: Dict[Path, str] = {}
            self._file_changed: Dict[Path, set] = {}
            for directory in discover_config_dirs():
                for name in ("tracker_params.yaml", "unity_params.yaml", "rosbag_params.yaml"):
                    path = (directory / name).resolve()
                    if path.is_file():
                        self._file_before[path] = path.read_text(encoding="utf-8")

            central = QWidget()
            self.setCentralWidget(central)
            root_layout = QVBoxLayout(central)

            # Header bar
            header_layout = QHBoxLayout()
            self.status_label = QLabel("状態: 確認中...")
            self.status_label.setStyleSheet("font-weight: bold; padding: 4px;")
            header_layout.addWidget(self.status_label, stretch=1)

            header_layout.addWidget(QLabel("保存先YAML:"))
            self.profile_combo = QComboBox()
            self.profile_combo.addItems(
                [
                    "全プロファイル同期 (実機 + Unity + Rosbag)",
                    "tracker_params.yaml (実機 RealSense)",
                    "unity_params.yaml (Unity シミュレーション)",
                    "rosbag_params.yaml (Rosbag 再生)",
                ]
            )
            self.profile_combo.setCurrentIndex(
                {"tracker_params.yaml": 1, "unity_params.yaml": 2, "rosbag_params.yaml": 3}[initial_profile]
            )
            header_layout.addWidget(self.profile_combo)

            button_layout = QHBoxLayout()
            reload_btn = QPushButton("ノード/YAMLから再読込")
            reload_btn.clicked.connect(self.reload_parameters)
            button_layout.addWidget(reload_btn)

            restore_btn = QPushButton("↺ 調整前へ戻す")
            restore_btn.setStyleSheet(
                "background-color: #e0f2fe; color: #0369a1; font-weight: bold; padding: 4px 8px;"
            )
            restore_btn.clicked.connect(self.reset_to_initial)
            restore_btn.setToolTip("このGUIを開いた後に変更した項目を、変更前のノード値・YAML値へ戻す")
            button_layout.addWidget(restore_btn)

            reset_tracker_btn = QPushButton("⚡ 追跡&背景リセット")
            reset_tracker_btn.setStyleSheet(
                "background-color: #fef08a; color: #854d0e; font-weight: bold; padding: 4px 8px;"
            )
            reset_tracker_btn.clicked.connect(self.trigger_tracking_reset)
            button_layout.addWidget(reset_tracker_btn)

            root_layout.addLayout(header_layout)
            root_layout.addLayout(button_layout)

            # Tabs
            tabs = QTabWidget()
            root_layout.addWidget(tabs, stretch=1)

            defaults = load_yaml_defaults(initial_profile)

            for tab_title, specs in TAB_SPECS:
                scroll = QScrollArea()
                scroll.setWidgetResizable(True)
                container = QWidget()
                form = QFormLayout(container)
                form.setLabelAlignment(Qt.AlignLeft)

                for spec in specs:
                    default_val = defaults.get(spec.name, spec.min_val)
                    row_widget = QWidget()
                    row_layout = QHBoxLayout(row_widget)
                    row_layout.setContentsMargins(0, 2, 0, 2)

                    if spec.param_type == "bool":
                        chk = QCheckBox(f"({spec.name})")
                        chk.setChecked(bool(default_val))
                        chk.toggled.connect(
                            lambda checked, s=spec: self.on_value_changed(
                                s, bool(checked)
                            )
                        )
                        row_layout.addWidget(chk)
                        self._widgets[spec.name] = (None, chk)
                    elif spec.param_type == "int":
                        slider = QSlider(Qt.Horizontal)
                        slider.setRange(int(spec.min_val), int(spec.max_val))
                        slider.setSingleStep(max(1, int(spec.step)))
                        spin = QSpinBox()
                        spin.setRange(int(spec.min_val), int(spec.max_val))
                        spin.setSingleStep(max(1, int(spec.step)))
                        init_v = int(round(float(default_val)))
                        slider.setValue(init_v)
                        spin.setValue(init_v)

                        slider.valueChanged.connect(spin.setValue)
                        spin.valueChanged.connect(slider.setValue)
                        spin.valueChanged.connect(
                            lambda val, s=spec: self.on_value_changed(s, int(val))
                        )
                        row_layout.addWidget(slider, stretch=1)
                        row_layout.addWidget(spin)
                        self._widgets[spec.name] = (slider, spin)
                    else:
                        scale = round(1.0 / spec.step)
                        slider = QSlider(Qt.Horizontal)
                        slider.setRange(
                            int(round(spec.min_val * scale)),
                            int(round(spec.max_val * scale)),
                        )
                        spin = QDoubleSpinBox()
                        spin.setDecimals(spec.decimals)
                        spin.setRange(spec.min_val, spec.max_val)
                        spin.setSingleStep(spec.step)
                        init_v = float(default_val)
                        slider.setValue(int(round(init_v * scale)))
                        spin.setValue(init_v)

                        def make_slider_cb(sp: QDoubleSpinBox, sc: int):
                            return lambda sval: sp.setValue(sval / float(sc))

                        def make_spin_cb(sl: QSlider, sc: int, s: ParamSpec):
                            def _cb(fval: float) -> None:
                                sl.blockSignals(True)
                                sl.setValue(int(round(fval * sc)))
                                sl.blockSignals(False)
                                self.on_value_changed(s, float(fval))

                            return _cb

                        slider.valueChanged.connect(make_slider_cb(spin, scale))
                        spin.valueChanged.connect(make_spin_cb(slider, scale, spec))
                        row_layout.addWidget(slider, stretch=1)
                        row_layout.addWidget(spin)
                        self._widgets[spec.name] = (slider, spin)

                    form.addRow(f"{spec.label}\n[{spec.name}]", row_widget)

                scroll.setWidget(container)
                tabs.addTab(scroll, tab_title)

            # Status timer
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.check_connection)
            self.timer.start(1000)
            self.reload_parameters()
            self._remote_before = bridge.fetch_remote_params(list(all_specs)) or {}
            self.profile_combo.currentIndexChanged.connect(self.reload_parameters)

        def selected_profiles(self) -> List[str]:
            profiles = ["tracker_params.yaml", "unity_params.yaml", "rosbag_params.yaml"]
            index = self.profile_combo.currentIndex()
            return profiles if index == 0 else [profiles[index - 1]]

        def check_connection(self) -> None:
            rclpy.spin_once(bridge, timeout_sec=0.01)
            if bridge.is_remote_available():
                self.status_label.setText(
                    f"🟢 接続中: {bridge.target_node} (リアルタイム反映 + YAML自動保存)"
                )
                self.status_label.setStyleSheet(
                    "color: #1b5e20; font-weight: bold; padding: 4px;"
                )
            else:
                self.status_label.setText(
                    f"🟠 オフライン: {bridge.target_node} 未検出 (ローカルYAML直接保存モード)"
                )
                self.status_label.setStyleSheet(
                    "color: #e65100; font-weight: bold; padding: 4px;"
                )

        def reload_parameters(self) -> None:
            self._updating_ui = True
            try:
                values = bridge.fetch_remote_params(list(all_specs.keys()))
                if not values:
                    profiles = self.selected_profiles()
                    values = load_yaml_defaults(profiles[0] if len(profiles) == 1 else initial_profile)
                for name, val in values.items():
                    if name not in self._widgets:
                        continue
                    spec = all_specs[name]
                    slider, widget = self._widgets[name]
                    if spec.param_type == "bool":
                        widget.setChecked(bool(val))
                    elif spec.param_type == "int":
                        widget.setValue(int(round(float(val))))
                        if slider is not None:
                            slider.setValue(int(round(float(val))))
                    else:
                        scale = round(1.0 / spec.step)
                        widget.setValue(float(val))
                        if slider is not None:
                            slider.setValue(int(round(float(val) * scale)))
            finally:
                self._updating_ui = False
            self.check_connection()

        def on_value_changed(self, spec: ParamSpec, value: ParamScalar) -> None:
            if self._updating_ui:
                return
            if bridge.is_remote_available():
                if spec.name not in self._remote_before:
                    before = bridge.fetch_remote_params([spec.name]) or {}
                    if spec.name not in before:
                        self.status_label.setText("変更前のノード値を取得できません。再読込してから操作してください")
                        return
                    self._remote_before[spec.name] = before[spec.name]
                if not bridge.push_remote_param(spec.name, value, spec.param_type):
                    self.reload_parameters()
                    self.status_label.setText(f"ノードが変更を拒否しました: {spec.name}（YAMLには保存していません）")
                    return
                self._remote_changed.add(spec.name)

            for directory in discover_config_dirs():
                for name in self.selected_profiles():
                    path = (directory / name).resolve()
                    if not path.is_file():
                        continue
                    if path not in self._file_before:
                        self._file_before[path] = path.read_text(encoding="utf-8")
                    self._file_changed.setdefault(path, set()).add(spec.name)
                    update_yaml_file_in_place(path, {spec.name: (value, spec.param_type)})

        def trigger_tracking_reset(self) -> None:
            bridge.publish_reset()
            self.status_label.setText("⚡ /cloth/reset を送信しました (軌跡クリア & 背景再初期化)")
            self.status_label.setStyleSheet("color: #b45309; font-weight: bold; padding: 4px;")

        def reset_to_initial(self) -> None:
            # ファイルごとの元値を保持するので、異なる環境の設定を混ぜない。
            updates = {
                name: (self._remote_before[name], all_specs[name].param_type)
                for name in self._remote_changed
            }
            if updates and not bridge.push_remote_params(updates):
                self.status_label.setText("ノードを調整前へ戻せませんでした。接続を確認して再実行してください")
                return
            for path, names in self._file_changed.items():
                restore_yaml_values(path, self._file_before[path], names)
            self._remote_changed.clear()
            self._file_changed.clear()
            self.reload_parameters()
            self.status_label.setText("↺ このGUIで変更したノード値・YAML値を調整前へ戻しました")

    win = TunerWindow()
    win.show()
    exit_code = app.exec_()
    win.timer.stop()
    win.close()
    bridge.destroy_node()
    rclpy.shutdown()
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Real-time parameter tuner GUI for cloth_tracker_cuda_node"
    )
    parser.add_argument(
        "--node",
        default="/cloth_tracker_cuda_node",
        help="Target ROS 2 node name (default: /cloth_tracker_cuda_node)",
    )
    parser.add_argument(
        "--profile",
        default="tracker_params.yaml",
        choices=["tracker_params.yaml", "unity_params.yaml", "rosbag_params.yaml"],
        help="Initial YAML profile to load defaults from",
    )
    args, _ = parser.parse_known_args()
    return run_gui(target_node=args.node, initial_profile=args.profile)


if __name__ == "__main__":
    sys.exit(main())
