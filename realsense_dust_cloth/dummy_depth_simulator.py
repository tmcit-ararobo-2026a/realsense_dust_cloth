#!/usr/bin/env python3
"""
Dummy Depth Stream Simulator for testing realsense_dust_cloth without physical RealSense camera.
Simulates a static background wall and a projectile flying in a parabolic trajectory.
"""

import time
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class DummyDepthSimulator(Node):
    def __init__(self):
        super().__init__('dummy_depth_simulator')
        self.pub_depth = self.create_publisher(Image, '/camera/camera/depth/image_rect_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera/depth/camera_info', 10)
        self.bridge = CvBridge()

        # Image specs: 848x480 (typical D435 resolution)
        self.width = 848
        self.height = 480
        self.fx = 425.0
        self.fy = 425.0
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Background: a wall at 4.0 meters
        self.bg_depth = np.full((self.height, self.width), 4000, dtype=np.uint16)

        # Simulation states
        self.frame_count = 0
        self.shot_active = False
        self.t_shot = 0.0
        self.dt = 1.0 / 30.0  # 30 fps
        
        # Parabolic motion parameters (camera optical frame)
        # In optical frame: X is right, Y is down, Z is forward
        self.v0 = 8.5
        self.angle_deg = 50.0
        self.angle_rad = math.radians(self.angle_deg)
        self.g = 9.81

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info("Dummy Depth Simulator started. Generating background...")

    def timer_callback(self):
        self.frame_count += 1
        now = self.get_clock().now()

        # Publish CameraInfo
        info_msg = CameraInfo()
        info_msg.header.stamp = now.to_msg()
        info_msg.header.frame_id = "camera_depth_optical_frame"
        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self.pub_info.publish(info_msg)

        # Depth frame base (background)
        depth_frame = self.bg_depth.copy()

        # Trigger shot after background collection (at frame 40)
        if self.frame_count == 45:
            self.get_logger().info(">>> Triggering projectile shot simulation! <<<")
            self.shot_active = True
            self.t_shot = 0.0

        if self.shot_active:
            # Trajectory in optical frame:
            # Z is forward depth
            # Y is upward in world (downward in camera: -Y)
            z_m = 0.4 + (self.v0 * math.cos(self.angle_rad)) * self.t_shot
            y_world_m = 0.3 + (self.v0 * math.sin(self.angle_rad)) * self.t_shot - 0.5 * self.g * (self.t_shot ** 2)
            x_m = 0.05 * math.sin(self.t_shot * 2.0)  # slight drift

            # Camera optical frame: Y_cam = -Y_world
            y_cam_m = -y_world_m + 0.4

            # Project to pixel coordinates
            if z_m > 0.3 and y_world_m >= 0.0:
                u = int(self.cx + (x_m * self.fx / z_m))
                v = int(self.cy + (y_cam_m * self.fy / z_m))
                depth_val_mm = int(z_m * 1000.0)

                # Draw a blob (representing the dust cloth) with depth
                radius_px = max(10, int(60.0 / z_m))
                cv2.circle(depth_frame, (u, v), radius_px, depth_val_mm, -1)
                
                self.t_shot += self.dt
            else:
                self.shot_active = False
                self.get_logger().info("Projectile landed. Shot ended.")

        # Publish Depth Image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
        depth_msg.header.stamp = now.to_msg()
        depth_msg.header.frame_id = "camera_depth_optical_frame"
        self.pub_depth.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyDepthSimulator()
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
