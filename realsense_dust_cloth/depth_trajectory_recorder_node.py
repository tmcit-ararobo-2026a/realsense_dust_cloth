#!/usr/bin/env python3
"""
Ultra-lightweight ROS 2 Node for broadcasting flying cloth 3D position as TF.
- Input: RealSense Depth only (no RGB)
- Output: TF (base_link -> flying_cloth)
- Recording: Simply record /tf in rosbag
"""

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class DepthTrajectoryTFNode(Node):
    def __init__(self):
        super().__init__('depth_trajectory_recorder_node')

        # Parameters
        self.declare_parameter('depth_diff_threshold_m', 0.12)
        self.declare_parameter('min_depth_m', 0.25)   # Ignore robot belt body
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('min_cluster_pixels', 20)
        self.declare_parameter('max_cluster_pixels', 8000)
        self.declare_parameter('max_cloth_size_m', 0.65)  # Super cloth is ~0.4x0.6m; reject opponent robot (>0.7m)
        self.declare_parameter('camera_pitch_deg', 20.0)
        self.declare_parameter('camera_height_offset_m', 0.35)
        self.declare_parameter('camera_x_offset_m', 0.25)

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('trajectory_history_sec', 3.0)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera Intrinsic
        self.fx = self.fy = self.cx = self.cy = None

        # Background Depth Map
        self.bg_depth_map = None
        self.bg_frames = []
        self.is_collecting_bg = True
        self.bg_target_frames = 20

        # Visualization Publishers for Experiment Evaluation
        self.pub_point = self.create_publisher(PointStamped, '/cloth/position', 10)
        self.pub_path = self.create_publisher(Path, '/cloth/measured_path', 10)
        self.pub_marker = self.create_publisher(Marker, '/cloth/trajectory_marker', 10)

        self.trajectory_points = []  # list of (timestamp_sec, Point)

        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_depth = self.create_subscription(
            Image, depth_topic, self.depth_callback, qos
        )
        self.sub_info = self.create_subscription(
            CameraInfo, camera_info_topic, self.info_callback, 10
        )

        self.get_logger().info(f'Lightweight Cloth TF Broadcaster started. Listening on {depth_topic}...')

    def info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'Camera intrinsics loaded: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def depth_callback(self, msg: Image):
        if self.fx is None:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return

        depth_m = depth_image.astype(np.float32) * 0.001  # Convert mm to meters

        # Phase 1: Background Learning (First 20 frames)
        if self.is_collecting_bg:
            valid_mask = (depth_m > 0.1) & (depth_m < self.get_parameter('max_depth_m').value)
            self.bg_frames.append(np.where(valid_mask, depth_m, np.nan))

            if len(self.bg_frames) >= self.bg_target_frames:
                stack = np.stack(self.bg_frames, axis=0)
                self.bg_depth_map = np.nanmedian(stack, axis=0)
                self.bg_depth_map = np.where(np.isnan(self.bg_depth_map), 10.0, self.bg_depth_map)
                self.is_collecting_bg = False
                self.bg_frames.clear()
                self.get_logger().info('Background calibrated! Ready to broadcast flying_cloth TF.')
            return

        # Phase 2: Ultra-fast Foreground Extraction
        diff_thresh = self.get_parameter('depth_diff_threshold_m').value
        min_depth = self.get_parameter('min_depth_m').value
        max_depth = self.get_parameter('max_depth_m').value

        fg_mask = (depth_m > min_depth) & (depth_m < (self.bg_depth_map - diff_thresh)) & (depth_m < max_depth)
        fg_u8 = (fg_mask * 255).astype(np.uint8)

        # Morphological open to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        fg_clean = cv2.morphologyEx(fg_u8, cv2.MORPH_OPEN, kernel)

        # Find cloth blob
        contours, _ = cv2.findContours(fg_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_pix = self.get_parameter('min_cluster_pixels').value
        max_pix = self.get_parameter('max_cluster_pixels').value

        best_cnt = None
        best_area = 0
        max_cloth_size = self.get_parameter('max_cloth_size_m').value

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_pix < area < max_pix:
                # Lightweight 3D bounding size check (reject opponent robot)
                bx, by, bw, bh = cv2.boundingRect(cnt)
                mid_u = min(depth_m.shape[1] - 1, max(0, bx + bw // 2))
                mid_v = min(depth_m.shape[0] - 1, max(0, by + bh // 2))
                z_est = float(depth_m[mid_v, mid_u])

                if z_est > min_depth and z_est < max_depth:
                    w_3d = (bw * z_est) / self.fx
                    h_3d = (bh * z_est) / self.fy
                    if w_3d > max_cloth_size or h_3d > max_cloth_size:
                        continue  # Too large -> opponent robot! Skip immediately.

                if area > best_area:
                    best_area = area
                    best_cnt = cnt

        if best_cnt is not None:
            # Centroid
            m = cv2.moments(best_cnt)
            if m['m00'] > 0:
                u = int(m['m10'] / m['m00'])
                v = int(m['m01'] / m['m00'])

                # Sample local median depth around centroid
                u_min, u_max = max(0, u - 5), min(depth_m.shape[1], u + 6)
                v_min, v_max = max(0, v - 5), min(depth_m.shape[0], v + 6)
                local_depth = depth_m[v_min:v_max, u_min:u_max]
                valid_depths = local_depth[local_depth > min_depth]

                if len(valid_depths) > 0:
                    z_cam = float(np.median(valid_depths))
                    x_cam = float((u - self.cx) * z_cam / self.fx)
                    y_cam = float((v - self.cy) * z_cam / self.fy)

                    # Transform to base_link (Robot: X forward, Y left, Z up)
                    pitch_rad = np.deg2rad(self.get_parameter('camera_pitch_deg').value)
                    x_off = self.get_parameter('camera_x_offset_m').value
                    h_off = self.get_parameter('camera_height_offset_m').value

                    # Camera optical frame -> base_link
                    x_robot = x_off + (z_cam * np.cos(pitch_rad) - y_cam * np.sin(pitch_rad))
                    y_robot = -x_cam  # lateral offset
                    z_robot = h_off + (z_cam * np.sin(pitch_rad) + y_cam * np.cos(pitch_rad))

                    # Broadcast TF: base_link -> flying_cloth
                    now = msg.header.stamp
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = now
                    tf_msg.header.frame_id = 'base_link'
                    tf_msg.child_frame_id = 'flying_cloth'
                    tf_msg.transform.translation.x = float(x_robot)
                    tf_msg.transform.translation.y = float(y_robot)
                    tf_msg.transform.translation.z = float(z_robot)
                    tf_msg.transform.rotation.w = 1.0

                    self.tf_broadcaster.sendTransform(tf_msg)

                    # Optional point
                    pt = PointStamped()
                    pt.header.stamp = now
                    pt.header.frame_id = 'base_link'
                    pt.point.x = float(x_robot)
                    pt.point.y = float(y_robot)
                    pt.point.z = float(z_robot)
                    self.pub_point.publish(pt)

                    # Manage trajectory history for visual evaluation in RViz
                    now_sec = self.get_clock().now().nanoseconds * 1e-9
                    keep_duration = self.get_parameter('trajectory_history_sec').value

                    # If there's a big time gap (>1.5s), reset history to start a new trajectory
                    if len(self.trajectory_points) > 0:
                        last_t, _ = self.trajectory_points[-1]
                        if (now_sec - last_t) > 1.5:
                            self.trajectory_points.clear()

                    p_geom = Point()
                    p_geom.x = float(x_robot)
                    p_geom.y = float(y_robot)
                    p_geom.z = float(z_robot)
                    self.trajectory_points.append((now_sec, p_geom))

                    # Remove old points outside time window
                    self.trajectory_points = [
                        (t, p) for (t, p) in self.trajectory_points
                        if (now_sec - t) <= keep_duration
                    ]

                    # Publish Marker (Yellow line strip showing experimental flight arc)
                    marker = Marker()
                    marker.header.stamp = now
                    marker.header.frame_id = 'base_link'
                    marker.ns = 'cloth_experiment'
                    marker.id = 0
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.scale.x = 0.035  # Line thickness (m)
                    marker.color.r = 1.0
                    marker.color.g = 0.85
                    marker.color.b = 0.0
                    marker.color.a = 0.95
                    marker.points = [p for (_, p) in self.trajectory_points]
                    self.pub_marker.publish(marker)

                    # Also publish Path
                    path_msg = Path()
                    path_msg.header.stamp = now
                    path_msg.header.frame_id = 'base_link'
                    for _, p in self.trajectory_points:
                        pose = PoseStamped()
                        pose.header.stamp = now
                        pose.header.frame_id = 'base_link'
                        pose.pose.position.x = p.x
                        pose.pose.position.y = p.y
                        pose.pose.position.z = p.z
                        pose.pose.orientation.w = 1.0
                        path_msg.poses.append(pose)
                    self.pub_path.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthTrajectoryTFNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
