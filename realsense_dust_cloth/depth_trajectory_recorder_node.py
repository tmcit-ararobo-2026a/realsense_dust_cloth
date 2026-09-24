"""
ROS 2 Node for recording 3D cloth trajectory using RealSense Depth stream only.
Mounting: Vertical portrait orientation at belt launcher tip.
"""

import json
import os
import time
from datetime import datetime

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from realsense_dust_cloth.physics_model import ClothPhysicsModel


class DepthTrajectoryRecorderNode(Node):
    def __init__(self):
        super().__init__('depth_trajectory_recorder_node')

        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/ros2_ws/trajectory_data'))
        self.declare_parameter('current_output_level', 1)
        self.declare_parameter('depth_diff_threshold_m', 0.12)
        self.declare_parameter('min_depth_m', 0.20)  # Ignore robot belt body
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('min_cluster_pixels', 30)
        self.declare_parameter('max_cluster_pixels', 8000)
        self.declare_parameter('idle_timeout_sec', 0.4)
        self.declare_parameter('camera_height_offset_m', 0.35)
        self.declare_parameter('camera_pitch_deg', 45.0)  # Tilt angle relative to horizon

        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.camera_info = None
        self.fx = self.fy = self.cx = self.cy = None

        # Background Depth Map (Median Filtered)
        self.bg_depth_map = None
        self.bg_collect_frames = []
        self.is_collecting_bg = True
        self.bg_target_frames = 25

        # Recording State
        self.is_recording = False
        self.recorded_points = []  # list of (t, x, y, z)
        self.record_start_time = None
        self.last_detected_time = None

        # QoS for Depth
        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.sub_depth = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            depth_qos
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.info_callback,
            10
        )
        self.sub_level = self.create_subscription(
            Int32,
            '/cloth_tracker/set_output_level',
            self.level_callback,
            10
        )
        self.sub_recalib = self.create_subscription(
            Bool,
            '/cloth_tracker/recalibrate_background',
            self.recalibrate_callback,
            10
        )

        # Publishers
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/cloth_tracker/trajectory_markers',
            10
        )
        self.pub_status = self.create_publisher(
            String,
            '/cloth_tracker/status',
            10
        )

        self.get_logger().info('DepthTrajectoryRecorderNode started. Waiting for Depth stream...')

    def info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'Camera Info received: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def level_callback(self, msg: Int32):
        self.set_parameters([rclpy.parameter.Parameter('current_output_level', rclpy.Parameter.Type.INTEGER, msg.data)])
        self.get_logger().info(f'Set current output level to: {msg.data}')

    def recalibrate_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Recalibrating background depth...')
            self.bg_depth_map = None
            self.bg_collect_frames = []
            self.is_collecting_bg = True

    def depth_callback(self, msg: Image):
        if self.camera_info is None:
            return

        try:
            # 16-bit unsigned depth in millimeters
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        depth_m = depth_image.astype(np.float32) / 1000.0  # Convert to meters

        # Phase 1: Background Learning
        if self.is_collecting_bg:
            valid_mask = (depth_m > 0.1) & (depth_m < self.get_parameter('max_depth_m').value)
            depth_clean = np.where(valid_mask, depth_m, np.nan)
            self.bg_collect_frames.append(depth_clean)

            if len(self.bg_collect_frames) >= self.bg_target_frames:
                stack = np.stack(self.bg_collect_frames, axis=0)
                self.bg_depth_map = np.nanmedian(stack, axis=0)
                self.bg_depth_map = np.nan_to_num(self.bg_depth_map, nan=10.0)
                self.is_collecting_bg = False
                self.get_logger().info('Background depth map calibrated successfully! Ready to record trajectories.')
                self.publish_status('READY')
            return

        # Phase 2: Foreground Extraction (Depth Subtraction)
        diff_thresh = self.get_parameter('depth_diff_threshold_m').value
        min_depth = self.get_parameter('min_depth_m').value
        max_depth = self.get_parameter('max_depth_m').value

        # Foreground is objects significantly CLOSER than background
        fg_mask = (depth_m > min_depth) & (depth_m < (self.bg_depth_map - diff_thresh)) & (depth_m < max_depth)
        fg_mask_u8 = (fg_mask * 255).astype(np.uint8)

        # Morphological filtering to remove small noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_cleaned = cv2.morphologyEx(fg_mask_u8, cv2.MORPH_OPEN, kernel)

        # Find largest connected component (the flying cloth)
        contours, _ = cv2.findContours(fg_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_pix = self.get_parameter('min_cluster_pixels').value
        max_pix = self.get_parameter('max_cluster_pixels').value

        best_contour = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_pix <= area <= max_pix:
                if area > best_area:
                    best_area = area
                    best_contour = cnt

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if best_contour is not None:
            # Mask depth for this cluster
            c_mask = np.zeros_like(fg_cleaned)
            cv2.drawContours(c_mask, [best_contour], -1, 255, -1)
            valid_cluster_depths = depth_m[c_mask == 255]

            if len(valid_cluster_depths) > 0:
                M = cv2.moments(best_contour)
                if M['m00'] > 0:
                    u_center = M['m10'] / M['m00']
                    v_center = M['m01'] / M['m00']
                    z_cam = float(np.median(valid_cluster_depths))

                    # 3D De-projection in Camera Frame
                    x_cam = (u_center - self.cx) * z_cam / self.fx
                    y_cam = (v_center - self.cy) * z_cam / self.fy

                    # Transform to Robot base_link Frame (Vertical Portrait mount + Pitch tilt)
                    # For vertical camera: camera +Y is robot -X or +X depending on 90deg roll
                    pitch_rad = np.deg2rad(self.get_parameter('camera_pitch_deg').value)
                    h_offset = self.get_parameter('camera_height_offset_m').value

                    # Transformed coordinates (Robot base_link: X forward, Y left, Z up)
                    # Camera is rotated 90 deg roll + tilted pitch
                    # In vertical mount, camera u maps to vertical/horizontal
                    x_robot = z_cam * np.cos(pitch_rad) - y_cam * np.sin(pitch_rad)
                    y_robot = -x_cam
                    z_robot = h_offset + z_cam * np.sin(pitch_rad) + y_cam * np.cos(pitch_rad)

                    # State machine
                    if not self.is_recording:
                        self.is_recording = True
                        self.record_start_time = now_sec
                        self.recorded_points = []
                        self.get_logger().info('>>> Cloth trajectory recording STARTED')
                        self.publish_status('RECORDING')

                    t_rel = now_sec - self.record_start_time
                    self.recorded_points.append((t_rel, x_robot, y_robot, z_robot))
                    self.last_detected_time = now_sec

        # Check for trajectory completion (idle timeout)
        if self.is_recording:
            idle_timeout = self.get_parameter('idle_timeout_sec').value
            if (now_sec - self.last_detected_time) > idle_timeout:
                self.finish_recording()

        # Publish RViz Markers
        self.publish_markers()

    def finish_recording(self):
        self.is_recording = False
        if len(self.recorded_points) >= 6:
            self.save_and_fit_trajectory()
        else:
            self.get_logger().warn(f'Recorded only {len(self.recorded_points)} points. Discarded as noise.')
            self.publish_status('DISCARDED')
        self.recorded_points = []

    def save_and_fit_trajectory(self):
        pts_arr = np.array([[p[1], p[2], p[3]] for p in self.recorded_points])
        t_arr = np.array([p[0] for p in self.recorded_points])

        level = self.get_parameter('current_output_level').value
        fit_res = ClothPhysicsModel.fit_trajectory(t_arr, pts_arr)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'trial_level_{level:02d}_{timestamp_str}.json'
        filepath = os.path.join(self.output_dir, filename)

        data = {
            "output_level": level,
            "timestamp": timestamp_str,
            "fitted_params": fit_res,
            "raw_trajectory": [
                {"t": round(p[0], 4), "x": round(p[1], 4), "y": round(p[2], 4), "z": round(p[3], 4)}
                for p in self.recorded_points
            ]
        }

        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        self.get_logger().info(
            f'=== Saved Trial Level {level} to {filename} ===\n'
            f'  Fitted v0: {fit_res.get("v0_mps", 0):.2f} m/s | Angle: {fit_res.get("angle_deg", 0):.1f} deg | '
            f'Distance: {fit_res.get("flight_distance_m", 0):.2f} m'
        )
        self.publish_status('SAVED')

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def publish_markers(self):
        if not self.recorded_points:
            return

        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'cloth_trajectory'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r = 1.0
        m.color.g = 0.8
        m.color.b = 0.0
        m.color.a = 1.0

        for pt in self.recorded_points:
            p = Point()
            p.x = pt[1]
            p.y = pt[2]
            p.z = pt[3]
            m.points.append(p)

        ma.markers.append(m)
        self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DepthTrajectoryRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

