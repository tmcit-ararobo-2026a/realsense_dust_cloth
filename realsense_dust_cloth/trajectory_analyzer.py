"""
Analyzer for multiple cloth trajectory trials.
Computes expected value (mean & variance) for each output level (1~10),
and exports LUT configuration for shooter_calculator.
"""

import argparse
import glob
import json
import os
import numpy as np


def analyze_directory(data_dir: str):
    json_files = glob.glob(os.path.join(data_dir, 'trial_level_*.json'))
    if not json_files:
        print(f"No trial data files found in {data_dir}")
        return

    # Group by output level
    trials_by_level = {}
    for fpath in json_files:
        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                d = json.load(f)
            lvl = d.get('output_level', 1)
            params = d.get('fitted_params', {})
            if params and 'v0_mps' in params:
                trials_by_level.setdefault(lvl, []).append(params)
        except Exception as e:
            print(f"Error reading {fpath}: {e}")

    print("\n" + "="*80)
    print("      REAL-SENSE CLOTH TRAJECTORY EXPECTATION ANALYSIS REPORT      ")
    print("="*80)
    print(f"{'Level':^7}|{'Trials':^8}|{'Mean v0 [m/s]':^15}|{'Std v0':^10}|{'Distance [m]':^14}|{'Angle [deg]':^13}|{'Drag k':^10}")
    print("-"*80)

    lut_results = {}

    for lvl in sorted(trials_by_level.keys()):
        trials = trials_by_level[lvl]
        v0_list = [t['v0_mps'] for t in trials]
        dist_list = [t.get('flight_distance_m', 0.0) for t in trials]
        ang_list = [t.get('angle_deg', 50.0) for t in trials]
        k_list = [t.get('drag_k', 0.08) for t in trials]

        # Outlier rejection (if >= 4 samples)
        if len(v0_list) >= 4:
            med = np.median(v0_list)
            std = np.std(v0_list)
            valid_idx = [i for i, v in enumerate(v0_list) if abs(v - med) <= 2.0 * max(std, 0.2)]
            v0_list = [v0_list[i] for i in valid_idx]
            dist_list = [dist_list[i] for i in valid_idx]
            ang_list = [ang_list[i] for i in valid_idx]
            k_list = [k_list[i] for i in valid_idx]

        mean_v0 = np.mean(v0_list)
        std_v0 = np.std(v0_list)
        mean_dist = np.mean(dist_list)
        mean_ang = np.mean(ang_list)
        mean_k = np.mean(k_list)

        lut_results[lvl] = {
            "mean_v0_mps": round(float(mean_v0), 3),
            "std_v0_mps": round(float(std_v0), 3),
            "mean_flight_distance_m": round(float(mean_dist), 3),
            "mean_launch_angle_deg": round(float(mean_ang), 1),
            "mean_drag_k": round(float(mean_k), 4),
            "valid_samples": len(v0_list)
        }

        print(f"{lvl:^7}|{len(v0_list):^8}|{mean_v0:^15.3f}|{std_v0:^10.3f}|{mean_dist:^14.3f}|{mean_ang:^13.1f}|{mean_k:^10.4f}")

    print("="*80)

    # Export C++ LUT snippet for shooter_calculator
    cpp_snippet = "\n// Generated initializeLUT() for shooter_calculator_node.cpp:\nvoid initializeLUT() {\n"
    for lvl in sorted(lut_results.keys()):
        cpp_snippet += f"  setReleaseVelocityForOutputLevel({lvl}, {lut_results[lvl]['mean_v0_mps']:.3f}); // mean distance: {lut_results[lvl]['mean_flight_distance_m']}m\n"
    cpp_snippet += "}\n"

    print(cpp_snippet)

    # Save to JSON
    export_path = os.path.join(data_dir, 'expected_trajectory_model.json')
    with open(export_path, 'w', encoding='utf-8') as f:
        json.dump(lut_results, f, indent=2, ensure_ascii=False)
    print(f"Expectation model exported to: {export_path}\n")


def main():
    parser = argparse.ArgumentParser(description="Analyze recorded cloth trajectories and output expectation model.")
    parser.add_argument('--data-dir', type=str, default=os.path.expanduser('~/ros2_ws/trajectory_data'),
                        help="Directory containing trial_level_*.json files")
    args = parser.parse_args()
    analyze_directory(args.data_dir)


if __name__ == '__main__':
    main()

