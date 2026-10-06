#!/usr/bin/env python3
"""
Trajectory Probability Analyzer for Dust Cloth Launcher
Author: realsense_dust_cloth
Description:
  Records multiple cloth flight trajectories at 60fps,
  computes the Most Probable Trajectory (mean parabola with standard deviation bounds),
  landing point distribution, landing success probability, and generates a visual report.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import numpy as np
import os
import csv
import time
import argparse

class ClothTrajectoryLogger(Node):
    def __init__(self, output_csv=None):
        super().__init__('cloth_trajectory_logger')
        if output_csv is None:
            pkg_data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')
            if os.path.exists(pkg_data_dir):
                output_csv = os.path.join(pkg_data_dir, 'cloth_trajectories_dataset.csv')
            else:
                output_csv = os.path.expanduser('~/ros2_ws/data/cloth_trajectories_dataset.csv')
        self.output_csv = output_csv
        os.makedirs(os.path.dirname(output_csv), exist_ok=True)
        
        self.sub_marker = self.create_subscription(
            Marker,
            '/cloth/trajectory_marker',
            self.marker_cb,
            10
        )
        self.current_flight_raw = []
        self.all_flights = []
        self.flight_count = 0
        self.last_msg_time = time.time()
        self.is_logging = False

        # Load existing dataset if available
        self.load_dataset()

    def marker_cb(self, msg: Marker):
        # We listen to SPHERE_LIST (id=2: raw 60fps measured points) or LINE_STRIP (id=0)
        if msg.id == 2 and len(msg.points) > 0: # ID 2 is 60fps raw measured points
            pts = [(p.x, p.y, p.z) for p in msg.points]
            if len(pts) < len(self.current_flight_raw):
                # New launch detected! Save previous flight
                self.save_flight(self.current_flight_raw)
                self.current_flight_raw = pts
            else:
                self.current_flight_raw = pts
            self.last_msg_time = time.time()
        elif msg.id == 0 and len(msg.points) > len(self.current_flight_raw):
            # Fallback to LINE_STRIP if SPHERE_LIST is not available
            pts = [(p.x, p.y, p.z) for p in msg.points]
            self.current_flight_raw = pts
            self.last_msg_time = time.time()

    def save_flight(self, pts):
        if len(pts) < 10:
            return
        self.flight_count += 1
        self.all_flights.append(pts)
        
        # Save to CSV
        file_exists = os.path.exists(self.output_csv)
        with open(self.output_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['flight_id', 'point_idx', 'x', 'y', 'z'])
            for idx, p in enumerate(pts):
                writer.writerow([self.flight_count, idx, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}"])
        
        print(f"📥 Recorded Flight #{self.flight_count}: {len(pts)} 60fps points (Start: X={pts[0][0]:.2f}, Land: X={pts[-1][0]:.2f})")

    def load_dataset(self):
        if not os.path.exists(self.output_csv):
            return
        flights = {}
        with open(self.output_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                fid = int(row['flight_id'])
                if fid not in flights:
                    flights[fid] = []
                flights[fid].append((float(row['x']), float(row['y']), float(row['z'])))
        self.all_flights = list(flights.values())
        self.flight_count = len(self.all_flights)
        if self.flight_count > 0:
            print(f"📂 Loaded {self.flight_count} existing flight trials from {self.output_csv}")

def analyze_dataset(flights, plot_output=None):
    if len(flights) == 0:
        print("⚠️ No flight data to analyze.")
        return

    print("\n" + "=" * 70)
    print(f"📊 TRAJECTORY PROBABILITY & STATISTICAL ANALYSIS ({len(flights)} TRIALS)")
    print("=" * 70)

    start_pts = [f[0] for f in flights]
    land_pts = [f[-1] for f in flights]
    peak_pts = [max(f, key=lambda p: p[2]) for f in flights]

    # Landing statistics
    land_x = [p[0] for p in land_pts]
    land_y = [p[1] for p in land_pts]
    land_z = [p[2] for p in land_pts]

    mean_lx, std_lx = np.mean(land_x), np.std(land_x)
    mean_ly, std_ly = np.mean(land_y), np.std(land_y)
    mean_lz, std_lz = np.mean(land_z), np.std(land_z)

    # Peak statistics
    peak_x = [p[0] for p in peak_pts]
    peak_z = [p[2] for p in peak_pts]
    mean_px, std_px = np.mean(peak_x), np.std(peak_x)
    mean_pz, std_pz = np.mean(peak_z), np.std(peak_z)

    # Fit quadratic parabola z(x) = a*x^2 + b*x + c for each trial
    parabola_coeffs = []
    x_common = np.linspace(0.8, 3.6, 100)
    interpolated_z = []

    for f in flights:
        fx = np.array([p[0] for p in f])
        fz = np.array([p[2] for p in f])
        if len(fx) >= 5 and (fx.max() - fx.min()) > 0.5:
            c = np.polyfit(fx, fz, 2)
            parabola_coeffs.append(c)
            interpolated_z.append(np.polyval(c, x_common))

    parabola_coeffs = np.array(parabola_coeffs)
    interpolated_z = np.array(interpolated_z)

    mean_a = np.mean(parabola_coeffs[:, 0])
    mean_b = np.mean(parabola_coeffs[:, 1])
    mean_c = np.mean(parabola_coeffs[:, 2])

    mean_z_curve = np.mean(interpolated_z, axis=0)
    std_z_curve = np.std(interpolated_z, axis=0)

    print(f"\n🎯 [MOST PROBABLE TRAJECTORY (最確放物線軌道)]")
    print(f"   Z(X) = ({mean_a:.4f}) * X² + ({mean_b:.4f}) * X + ({mean_c:.4f})")
    print(f"   - 最確頂点位置 (Apex) : X = {mean_px:.2f} ± {std_px:.2f} m,  Z = {mean_pz:.2f} ± {std_pz:.2f} m")
    print(f"   - 発射地点 (Start)    : X = {np.mean([p[0] for p in start_pts]):.2f} m,  Z = {np.mean([p[2] for p in start_pts]):.2f} m")

    print(f"\n📍 [LANDING PROBABILITY DISTRIBUTION (着地点の確率分布)]")
    print(f"   - 最確着地位置 (Mean Land) : X = {mean_lx:.2f} m,  Y = {mean_ly:.2f} m,  Z = {mean_lz:.2f} m")
    print(f"   - 前後ばらつき (X ± 1σ)   : ± {std_lx*100:.1f} cm (信頼区間 95%: [{mean_lx - 2*std_lx:.2f}, {mean_lx + 2*std_lx:.2f}] m)")
    print(f"   - 左右ばらつき (Y ± 1σ)   : ± {std_ly*100:.1f} cm (横ズレが極めて少なく真っ直ぐ直進)")
    print(f"   - 着地目標 (3.5m台) 到達率: {np.mean([1 if x >= 3.4 else 0 for x in land_x]) * 100:.1f} %")

    # Generate plot if requested or default
    if plot_output:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

            # 1. Trajectory Elevation Profile Z(X)
            for z_curve in interpolated_z:
                ax1.plot(x_common, z_curve, color='gray', alpha=0.35, linewidth=1.0)
            ax1.plot(x_common, mean_z_curve, color='blue', linewidth=3.0, label='Most Probable Trajectory Z(X)')
            ax1.fill_between(x_common, mean_z_curve - std_z_curve, mean_z_curve + std_z_curve,
                             color='blue', alpha=0.2, label='1-Sigma Confidence Band (68%)')
            ax1.scatter(land_x, land_z, color='red', zorder=5, label='Landing Points')
            ax1.set_title(f'Cloth Ballistic Trajectory Probability Analysis ({len(flights)} trials)', fontsize=14, fontweight='bold')
            ax1.set_xlabel('Forward Distance X (m)', fontsize=12)
            ax1.set_ylabel('Height Z (m)', fontsize=12)
            ax1.grid(True, linestyle='--', alpha=0.7)
            ax1.legend(loc='upper right')

            # 2. Horizontal Scatter Plot Y(X)
            ax2.scatter(land_x, land_y, color='red', s=60, edgecolors='black', label='Landing Points')
            # 1-sigma ellipse
            from matplotlib.patches import Ellipse
            ellipse = Ellipse((mean_lx, mean_ly), width=2*std_lx, height=2*std_ly,
                              facecolor='red', alpha=0.2, edgecolor='red', linewidth=2, label='1-Sigma Landing Zone')
            ax2.add_patch(ellipse)
            ax2.axhline(0, color='black', linestyle=':', alpha=0.5)
            ax2.set_title('Landing Point Dispersion in XY Plane', fontsize=13)
            ax2.set_xlabel('Forward Distance X (m)', fontsize=12)
            ax2.set_ylabel('Sideways Y (m)', fontsize=12)
            ax2.set_ylim(-0.4, 0.4)
            ax2.grid(True, linestyle='--', alpha=0.7)
            ax2.legend(loc='upper right')

            plt.tight_layout()
            plt.savefig(plot_output, dpi=150)
            print(f"\n📈 Visual Probability Report saved to: {plot_output}")
        except Exception as e:
            print(f"Could not generate plot: {e}")

    print("=" * 70 + "\n")

def main():
    rclpy.init()
    parser = argparse.ArgumentParser(description="Cloth Trajectory Probability Analyzer")
    parser.add_argument('--record', action='store_true', default=True, help="Record incoming trajectories in real-time")
    parser.add_argument('--analyze', action='store_true', help="Analyze existing recorded trajectories")
    default_csv = os.path.expanduser('~/ros2_ws/data/cloth_trajectories_dataset.csv')
    default_plot = os.path.expanduser('~/ros2_ws/data/most_probable_trajectory.png')
    parser.add_argument('--csv', type=str, default=default_csv)
    parser.add_argument('--plot', type=str, default=default_plot)
    args, unknown = parser.parse_known_args()

    logger = ClothTrajectoryLogger(args.csv)
    if args.analyze:
        analyze_dataset(logger.all_flights, args.plot)
        rclpy.shutdown()
        return

    print("📡 Listening for cloth trajectories... (Press Ctrl+C to stop and analyze)")
    try:
        while rclpy.ok():
            rclpy.spin_once(logger, timeout_sec=0.1)
            # Finalize flight if quiet for 2.0s
            if len(logger.current_flight_raw) > 0 and (time.time() - logger.last_msg_time) > 2.0:
                logger.save_flight(logger.current_flight_raw)
                logger.current_flight_raw = []
    except KeyboardInterrupt:
        pass
    finally:
        if len(logger.current_flight_raw) > 0:
            logger.save_flight(logger.current_flight_raw)
        analyze_dataset(logger.all_flights, args.plot)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
