#!/usr/bin/env python3
"""Live integration test: Unity simulation and gn10 localization tracking verification.

The tracker consumes localization TF (map -> base_link) and depth images from Unity.
Verifies stationary throws, travel-then-shoot, landing detection, and parameter compliance.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


def parse_arguments():
    parser = argparse.ArgumentParser(description="Verify cloth tracking with Unity simulation.")
    parser.add_argument("--params-file", required=True, help="Path to tracker YAML configuration")
    parser.add_argument("--output", required=True, help="Directory to store test results and logs")
    parser.add_argument("--require-observed-landing", action="store_true",
                        help="Require all throws to be observed down to landing height")
    parser.add_argument("--allow-default-domain", action="store_true",
                        help="Allow running on default ROS_DOMAIN_ID (0)")
    return parser.parse_args()


def main():
    args = parse_arguments()
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    if (not domain_id or domain_id == "0") and not args.allow_default_domain:
        raise RuntimeError("Set ROS_DOMAIN_ID or pass --allow-default-domain for this live test")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("map_cloth_verification")

    positions = []
    landings = []
    states = []
    poses = []
    sim_time = [0.0]
    is_ready = [False]
    tracking_state = ["IDLE"]

    def record_point(msg, target_list):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        target_list.append({
            "stamp": stamp_sec,
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z,
        })

    def record_pose(msg):
        pos = msg.pose.pose.position
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        poses.append({"stamp": stamp_sec, "x": pos.x, "y": pos.y})

    # Subscriptions
    node.create_subscription(
        PointStamped, "/cloth/position",
        lambda m: record_point(m, positions), 100)
    node.create_subscription(
        PointStamped, "/cloth/landing_position",
        lambda m: record_point(m, landings), 100)
    node.create_subscription(
        String, "/cloth/background_status",
        lambda m: states.append({"stamp": sim_time[0], "state": m.data}), 100)
    node.create_subscription(
        Bool, "/cloth/ready",
        lambda m: is_ready.__setitem__(0, m.data), 10)
    node.create_subscription(
        String, "/cloth/tracking_state",
        lambda m: tracking_state.__setitem__(0, m.data), 10)
    node.create_subscription(
        PoseWithCovarianceStamped, "/platform_constraint",
        record_pose, 100)
    node.create_subscription(
        Clock, "/clock",
        lambda m: sim_time.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    # Publishers
    pub_cmd_vel = node.create_publisher(Twist, "/robot/operation/cmd_vel", 10)
    pub_throw = node.create_publisher(Bool, "/belt/throw", 10)
    pub_speed = node.create_publisher(Float32, "/belt/speed_ratio", 10)

    def advance_time(duration_sec, velocity=0.0):
        end_time = time.monotonic() + duration_sec
        next_pub_time = 0.0
        while time.monotonic() < end_time:
            if time.monotonic() >= next_pub_time:
                twist = Twist()
                twist.linear.x = velocity
                pub_cmd_vel.publish(twist)
                next_pub_time = time.monotonic() + 0.02
            rclpy.spin_once(node, timeout_sec=0.005)

    def wait_until(predicate, timeout_sec):
        end_time = time.monotonic() + timeout_sec
        while time.monotonic() < end_time:
            advance_time(0.03)
            if predicate():
                return True
        return False

    trials = []
    result = {}
    log_path = output_dir / "tracker.log"

    with log_path.open("w") as log_file:
        launch_cmd = [
            "ros2", "launch", "realsense_dust_cloth", "cloth_tracker.launch.py",
            "params_file:=" + str(Path(args.params_file).resolve()),
            "enable_analyzer:=false",
            "rviz:=false",
        ]
        process = subprocess.Popen(
            launch_cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        try:
            pub_throw.publish(Bool(data=False))
            assert wait_until(lambda: is_ready[0], 15.0), f"Background never ready; last states: {states[-5:]}"
            assert len(poses) > 3, "Localization poses (/platform_constraint) are missing"

            # Check parameter compliance
            client = node.create_client(GetParameters, "/cloth_tracker_cuda_node/get_parameters")
            assert client.wait_for_service(timeout_sec=2.0)
            req = GetParameters.Request()
            req.names = ["use_sim_time", "use_direct_realsense"]
            future = client.call_async(req)
            assert wait_until(future.done, 3.0)
            values = future.result().values
            assert values[0].bool_value and not values[1].bool_value

            advance_time(0.5)
            assert len(positions) == 0, "False cloth detection occurred before any throw"

            # Execute 3 throw trials (stationary, stationary, moving-then-stop)
            speed_ratios = (0.4, 0.7, 0.4)
            for trial_index, ratio in enumerate(speed_ratios):
                movement_info = None

                # Trial 3: Robot drives, stops, settles, then shoots
                if trial_index == 2:
                    pose_before = poses[-1] if poses else None
                    state_idx = len(states)

                    advance_time(1.0, velocity=0.4)
                    stop_sim_time = sim_time[0]
                    wall_start = time.monotonic()

                    assert wait_until(lambda: is_ready[0], 5.0), "Not ready after robot motion"
                    assert is_ready[0] and states[-1] == "MAP_READY", f"Unexpected state after motion: {states[-5:]}"

                    pose_after = poses[-1] if poses else None
                    travel_distance = None
                    if pose_before and pose_after:
                        dx = pose_after["x"] - pose_before["x"]
                        dy = pose_after["y"] - pose_before["y"]
                        travel_distance = (dx * dx + dy * dy) ** 0.5
                    assert travel_distance is not None and travel_distance > 0.2, "Robot travel not observed"

                    movement_info = {
                        "sim_wait_sec": sim_time[0] - stop_sim_time,
                        "wall_wait_sec": time.monotonic() - wall_start,
                        "travel_distance_m": travel_distance,
                    }

                assert wait_until(lambda: is_ready[0], 3.0)

                point_start_idx = len(positions)
                landing_start_idx = len(landings)

                pub_speed.publish(Float32(data=ratio))
                advance_time(0.1)
                pub_throw.publish(Bool(data=True))
                advance_time(0.08)
                pub_throw.publish(Bool(data=False))

                detected = wait_until(lambda: len(positions) > point_start_idx, 3.0)
                ended = wait_until(
                    lambda: tracking_state[0] in ("LANDED", "LOST", "RESTING"), 5.0
                ) if detected else False

                advance_time(0.25)

                observed_pts = positions[point_start_idx:]
                published_landings = landings[landing_start_idx:]
                observed_landing = any(p["z"] <= 0.15 for p in observed_pts)

                trial_result = {
                    "trial": trial_index + 1,
                    "speed_ratio": ratio,
                    "detected": detected,
                    "points_count": len(observed_pts),
                    "landing_message_received": bool(published_landings),
                    "terminal_state": tracking_state[0] if ended else "TIMEOUT",
                    "first_point": observed_pts[0] if observed_pts else None,
                    "last_point": observed_pts[-1] if observed_pts else None,
                    "observed_landing_height": observed_landing,
                    "movement_info": movement_info,
                }
                trials.append(trial_result)
                print(json.dumps(trial_result), flush=True)
                advance_time(0.5)

            # Post-run assertion checks
            assert all(t["detected"] and t["points_count"] >= 5 for t in trials), "A throw was not tracked"
            if args.require_observed_landing:
                assert all(t["observed_landing_height"] for t in trials), "Not all throws reached landing height"
            assert not any(s["state"] == "MAP_PROCESSING_ERROR" for s in states)

            result = {
                "passed": True,
                "mode": "map",
                "trials": trials,
                "all_landings_observed": all(t["observed_landing_height"] for t in trials),
                "total_localization_messages": len(poses),
            }

        except Exception as error:
            result = {
                "passed": False,
                "mode": "map",
                "trials": trials,
                "error": str(error),
                "last_states": states[-10:],
            }
            raise

        finally:
            pub_cmd_vel.publish(Twist())
            pub_throw.publish(Bool(data=False))
            advance_time(0.1)

            (output_dir / "result.json").write_text(json.dumps(result, indent=2))
            (output_dir / "observations.json").write_text(
                json.dumps({
                    "positions": positions,
                    "landings": landings,
                    "states": states,
                    "poses": poses,
                }, indent=2)
            )

            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
