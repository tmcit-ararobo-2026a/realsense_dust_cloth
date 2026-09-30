#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
import csv
import os
import time
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # Declare output CSV path
        home_dir = os.path.expanduser('~')
        data_dir = os.path.join(home_dir, 'ros2_ws', 'data')
        os.makedirs(data_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_csv = os.path.join(data_dir, f"trajectories_{timestamp_str}.csv")
        self.declare_parameter('csv_path', default_csv)
        self.csv_path = self.get_parameter('csv_path').value

        self.sub_path = self.create_subscription(
            Path,
            '/cloth/measured_path',
            self.path_callback,
            10
        )

        self.current_flight_id = 0
        self.last_path_size = 0
        self.last_flight_points = []
        self.last_active_time = 0.0
        self.is_recording = False
        self.saved_flights = []

        # Periodic check timer to detect end of flight (10Hz)
        self.timer = self.create_timer(0.1, self.check_flight_completion)

        # Initialize CSV header
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['flight_id', 'timestamp_iso', 'point_idx', 'x_m', 'y_m', 'z_m'])

        print("=" * 65)
        print("🎯 REAL-TIME CLOTH TRAJECTORY RECORDER STARTED")
        print(f"📁 Saving all flight parabolas to: {self.csv_path}")
        print("=" * 65)
        print("Ready! Launch the cloth anytime. It will automatically record each flight.")
        print("Press Ctrl+C to finish recording and generate analysis report.\n")

    def path_callback(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        if not pts:
            return

        now = time.time()
        # If new path is shorter or starts over, it's a new flight!
        if len(pts) < self.last_path_size or (now - self.last_active_time) > 2.0:
            if len(self.last_flight_points) >= 15:
                self.save_flight(self.last_flight_points)
            self.last_flight_points = pts
            self.is_recording = True
        else:
            self.last_flight_points = pts

        self.last_path_size = len(pts)
        self.last_active_time = now

    def check_flight_completion(self):
        # If no new points for > 1.2s and we have a flight, save it!
        now = time.time()
        if self.is_recording and (now - self.last_active_time) > 1.2:
            if len(self.last_flight_points) >= 15:
                self.save_flight(self.last_flight_points)
            self.is_recording = False
            self.last_flight_points = []
            self.last_path_size = 0

    def is_duplicate(self, pts):
        if not self.saved_flights:
            return False, -1

        new_start = pts[0]
        new_peak = max(pts, key=lambda p: p[2])
        new_land = pts[-1]

        for old_id, old_pts in self.saved_flights:
            old_start = old_pts[0]
            old_peak = max(old_pts, key=lambda p: p[2])
            old_land = old_pts[-1]

            # Check distance between start, apex and landing
            d_start = sum((a - b) ** 2 for a, b in zip(new_start, old_start)) ** 0.5
            d_peak = sum((a - b) ** 2 for a, b in zip(new_peak, old_peak)) ** 0.5
            d_land = sum((a - b) ** 2 for a, b in zip(new_land, old_land)) ** 0.5

            # If all 3 key ballistic points match within 3.5cm, it's an identical repeated dataset!
            if d_start < 0.035 and d_peak < 0.035 and d_land < 0.040:
                return True, old_id
        return False, -1

    def save_flight(self, pts):
        is_dup, match_id = self.is_duplicate(pts)
        if is_dup:
            print(f"⚠️  [SKIPPED: DUPLICATE DETECTED!]")
            print(f"   Matches Flight #{match_id:02d} (Same rosbag replay or identical flight within 3cm).")
            print(f"   Skipping to keep dataset pure and meaningful!")
            print("-" * 65)
            return

        self.current_flight_id += 1
        self.saved_flights.append((self.current_flight_id, pts))
        iso_now = datetime.now().isoformat()

        # Write to CSV
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            for idx, p in enumerate(pts):
                writer.writerow([self.current_flight_id, iso_now, idx, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}"])

        start_p = pts[0]
        peak_p = max(pts, key=lambda p: p[2])
        land_p = pts[-1]
        print(f"✅ [Flight #{self.current_flight_id:02d} SAVED! (New Unique Data)]")
        print(f"   Points: {len(pts)} true frames")
        print(f"   Start : X={start_p[0]:.2f}m, Z={start_p[2]:.2f}m")
        print(f"   Apex  : X={peak_p[0]:.2f}m, Z={peak_p[2]:.2f}m (Height)")
        print(f"   Land  : X={land_p[0]:.2f}m, Y={land_p[1]:.2f}m, Z={land_p[2]:.2f}m")
        print("-" * 65)

def main():
    rclpy.init()
    recorder = TrajectoryRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        total = recorder.current_flight_id
        csv_file = recorder.csv_path
        recorder.destroy_node()
        rclpy.shutdown()

        print("\n" + "=" * 65)
        print(f"🎉 RECORDING FINISHED! Total Flights Recorded: {total}")
        print(f"📄 CSV File: {csv_file}")
        print("=" * 65)

        # Run statistical probability analyzer if flights exist
        if total > 0 and os.path.exists(csv_file):
            import sys
            curr_dir = os.path.dirname(os.path.abspath(__file__))
            if curr_dir not in sys.path:
                sys.path.append(curr_dir)
            try:
                from trajectory_probability_analyzer import analyze_dataset
                import pandas as pd
                df = pd.read_csv(csv_file)
                all_flights = []
                for fid in df['flight_id'].unique():
                    fdf = df[df['flight_id'] == fid]
                    pts = list(zip(fdf.x_m, fdf.y_m, fdf.z_m))
                    all_flights.append(pts)
                plot_png = csv_file.replace('.csv', '.png')
                analyze_dataset(all_flights, plot_output=plot_png)
                print(f"📊 Probability Graph Saved to: {plot_png}")
            except Exception as e:
                print(f"Note: Analysis skipped ({e})")

if __name__ == '__main__':
    main()

