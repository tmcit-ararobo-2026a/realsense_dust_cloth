#!/usr/bin/env python3
"""
High-performance ROS 2 Node for tracking flying cloth using RealSense PointCloud2.
- Input: PointCloud2 (/camera/camera/depth/color/points or /camera/depth/color/points)
- Processing:
    1. Voxel / downsample for low Jetson load
    2. PassThrough filter in robot base_link frame (crop belt, floor, ceiling)
    3. Fast Euclidean clustering (DBSCAN / connected components)
    4. Size and spread filtering (Normal cloth & Super cloth allowed; Opponent robot rejected)
- Output:
    - TF (base_link -> flying_cloth)
    - RViz Marker (Yellow flight arc)
    - Cloth PointCloud (/cloth/cluster_points)
    - Path (/cloth/measured_path)
"""

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped, PointStamped, Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


class PointCloudClothTrackerNode(Node):
    def __init__(self):
        super().__init__('pointcloud_cloth_tracker_node')

        # 3D ROI Parameters (in camera / robot frame)
        self.declare_parameter('points_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('camera_pitch_deg', 45.0)
        self.declare_parameter('camera_height_offset_m', 0.35)
        self.declare_parameter('camera_x_offset_m', 0.25)

        # Spatial Filtering bounds (base_link frame: X forward, Y left, Z up)
        self.declare_parameter('min_x_m', 0.30)   # Ignore launcher belt body (<0.3m)
        self.declare_parameter('max_x_m', 6.0)    # Forward max range
        self.declare_parameter('min_y_m', -1.5)   # Left/right limit
        self.declare_parameter('max_y_m', 1.5)
        self.declare_parameter('min_z_m', 0.05)   # Floor cutoff
        self.declare_parameter('max_z_m', 3.5)    # Ceiling cutoff

        # Cloth Clustering Parameters
        self.declare_parameter('voxel_size_m', 0.03)        # 3cm downsampling for low Jetson CPU usage
        self.declare_parameter('cluster_eps_m', 0.15)       # DBSCAN neighbor distance
        self.declare_parameter('min_cluster_points', 10)     # Minimum points for cloth
        self.declare_parameter('max_cluster_points', 1500)   # Maximum points
        self.declare_parameter('max_cloth_size_m', 0.65)    # Super cloth is ~0.4x0.6m; reject opponent robot (>0.7m)
        self.declare_parameter('trajectory_history_sec', 3.0)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.pub_point = self.create_publisher(PointStamped, '/cloth/position', 10)
        self.pub_cluster_cloud = self.create_publisher(PointCloud2, '/cloth/cluster_points', 10)
        self.pub_marker = self.create_publisher(Marker, '/cloth/trajectory_marker', 10)
        self.pub_path = self.create_publisher(Path, '/cloth/measured_path', 10)

        self.trajectory_points = []

        # Subscription
        points_topic = self.get_parameter('points_topic').value
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_points = self.create_subscription(
            PointCloud2, points_topic, self.pointcloud_callback, qos
        )

        self.get_logger().info(f'PointCloud Cloth Tracker started. Listening to PointCloud2 on: {points_topic}')

    def pointcloud_callback(self, msg: PointCloud2):
        now = msg.header.stamp

        # Read xyz points from PointCloud2
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.fromiter(gen, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        if len(pts) == 0:
            return

        x_cam = pts['x']
        y_cam = pts['y']
        z_cam = pts['z']

        # Transform to base_link frame:
        # Camera is mounted vertically (90 deg roll) and tilted up by pitch_deg
        pitch_rad = np.deg2rad(self.get_parameter('camera_pitch_deg').value)
        x_off = self.get_parameter('camera_x_offset_m').value
        h_off = self.get_parameter('camera_height_offset_m').value

        # Transform formula from camera frame to base_link
        x_rob = x_off + (z_cam * np.cos(pitch_rad) - y_cam * np.sin(pitch_rad))
        y_rob = -x_cam
        z_rob = h_off + (z_cam * np.sin(pitch_rad) + y_cam * np.cos(pitch_rad))

        # Spatial PassThrough Filter in base_link
        min_x = self.get_parameter('min_x_m').value
        max_x = self.get_parameter('max_x_m').value
        min_y = self.get_parameter('min_y_m').value
        max_y = self.get_parameter('max_y_m').value
        min_z = self.get_parameter('min_z_m').value
        max_z = self.get_parameter('max_z_m').value

        mask = (x_rob > min_x) & (x_rob < max_x) & \
               (y_rob > min_y) & (y_rob < max_y) & \
               (z_rob > min_z) & (z_rob < max_z)

        valid_x = x_rob[mask]
        valid_y = y_rob[mask]
        valid_z = z_rob[mask]

        if len(valid_x) < self.get_parameter('min_cluster_points').value:
            return

        points_3d = np.stack([valid_x, valid_y, valid_z], axis=1)

        # Voxel grid downsampling for ultra-low latency on Jetson
        voxel_size = self.get_parameter('voxel_size_m').value
        quantized = np.floor(points_3d / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(quantized, axis=0, return_index=True)
        downsampled_pts = points_3d[unique_idx]

        min_pts = self.get_parameter('min_cluster_points').value
        max_pts = self.get_parameter('max_cluster_points').value

        if len(downsampled_pts) < min_pts:
            return

        # Fast Euclidean Clustering via DBSCAN
        eps = self.get_parameter('cluster_eps_m').value
        clustering = DBSCAN(eps=eps, min_samples=min_pts).fit(downsampled_pts)
        labels = clustering.labels_

        unique_labels = set(labels) - {-1}  # Ignore noise label -1
        if not unique_labels:
            return

        best_cluster = None
        best_count = 0
        max_cloth_size = self.get_parameter('max_cloth_size_m').value

        for lbl in unique_labels:
            cluster_pts = downsampled_pts[labels == lbl]
            c_len = len(cluster_pts)
            if c_len < min_pts or c_len > max_pts:
                continue

            # Bounding box size check in 3D (Reject opponent robot > 0.65m)
            bbox_min = np.min(cluster_pts, axis=0)
            bbox_max = np.max(cluster_pts, axis=0)
            bbox_dim = bbox_max - bbox_min
            
            # Super cloth (0.4x0.6m) passes; Opponent robot (>0.7m) is rejected immediately
            if np.any(bbox_dim > max_cloth_size):
                continue

            if c_len > best_count:
                best_count = c_len
                best_cluster = cluster_pts

        if best_cluster is None:
            return

        # Centroid calculation
        centroid = np.mean(best_cluster, axis=0)
        cx, cy, cz = float(centroid[0]), float(centroid[1]), float(centroid[2])

        # 1. Broadcast TF: base_link -> flying_cloth
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'base_link'
        tf_msg.child_frame_id = 'flying_cloth'
        tf_msg.transform.translation.x = cx
        tf_msg.transform.translation.y = cy
        tf_msg.transform.translation.z = cz
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

        # 2. Publish Centroid Point
        pt = PointStamped()
        pt.header.stamp = now
        pt.header.frame_id = 'base_link'
        pt.point.x = cx
        pt.point.y = cy
        pt.point.z = cz
        self.pub_point.publish(pt)

        # 3. Publish Segmented Cloth PointCloud
        header = msg.header
        header.frame_id = 'base_link'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloth_cloud_msg = pc2.create_cloud(header, fields, best_cluster)
        self.pub_cluster_cloud.publish(cloth_cloud_msg)

        # 4. Trajectory History & Arc Marker for RViz
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        keep_sec = self.get_parameter('trajectory_history_sec').value

        if len(self.trajectory_points) > 0:
            last_t, _ = self.trajectory_points[-1]
            if (now_sec - last_t) > 1.5:
                self.trajectory_points.clear()

        p_geom = Point(x=cx, y=cy, z=cz)
        self.trajectory_points.append((now_sec, p_geom))
        self.trajectory_points = [
            (t, p) for (t, p) in self.trajectory_points
            if (now_sec - t) <= keep_sec
        ]

        # Publish Flight Arc Marker
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'base_link'
        marker.ns = 'cloth_pointcloud_arc'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.points = [p for (_, p) in self.trajectory_points]
        self.pub_marker.publish(marker)

        # Publish Path
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
    node = PointCloudClothTrackerNode()
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
