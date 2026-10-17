#!/usr/bin/env python3
"""
Simulate flying cloth shooting experiment for visual testing in RViz2.
Shoots a cloth repeatedly in an arc parabola so you can watch the CUDA tracker in action!
"""

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ClothFlightSimulator(Node):
    def __init__(self):
        super().__init__('cloth_flight_simulator')
        self.pub_image = self.create_publisher(Image, '/camera/camera/depth/image_rect_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera/depth/camera_info', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish) # 30 fps
        self.bridge = CvBridge()

        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_pitch_deg', 20.0)
        self.declare_parameter('camera_roll_deg', 90.0)
        self.declare_parameter('camera_x_offset_m', 0.50)
        self.declare_parameter('camera_height_offset_m', 1.00)

        self.width = self.get_parameter('camera_width').value
        self.height = self.get_parameter('camera_height').value
        self.pitch_rad = math.radians(float(self.get_parameter('camera_pitch_deg').value))
        self.roll_rad = math.radians(float(self.get_parameter('camera_roll_deg').value))
        self.x_off = float(self.get_parameter('camera_x_offset_m').value)
        self.h_off = float(self.get_parameter('camera_height_offset_m').value)

        self.start_time = time.time()
        self.fx, self.fy = 385.0, 385.0
        self.cx, self.cy = self.width * 0.5, self.height * 0.5

        self.get_logger().info("Cloth Flight Simulator started! Launching cloth parabolas every 2.5 seconds...")

    def update_and_publish(self):
        now_msg = self.get_clock().now().to_msg()

        # 1. CameraInfo
        info = CameraInfo()
        info.header.stamp = now_msg
        info.header.frame_id = 'camera_depth_optical_frame'
        info.width = self.width
        info.height = self.height
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)

        # 2. Physics of flying cloth (Cycle: 2.5 seconds, wait 1.2s initially for background calibration)
        elapsed = time.time() - self.start_time
        t_cycle = (elapsed - 1.2) % 2.5 if elapsed >= 1.2 else 2.0

        depth_img = np.zeros((self.height, self.width), dtype=np.uint16)

        if t_cycle < 1.0: # Flight duration ~1.0 second
            t = t_cycle
            # Parabolic trajectory in robot frame (base_link) originating inside launch gate
            x_rob = self.x_off + 0.20 + 2.8 * t
            y_rob = 0.04 * math.sin(t * 3.0)
            z_rob = self.h_off + 0.10 + 3.2 * t - 0.5 * 9.81 * (t ** 2)

            if z_rob > 0.15: # Above landing threshold
                # Inverse transform: base_link -> camera_depth_optical_frame
                dx = x_rob - self.x_off
                dy = y_rob
                dz = z_rob - self.h_off

                cos_p = math.cos(self.pitch_rad)
                sin_p = math.sin(self.pitch_rad)
                cos_r = math.cos(self.roll_rad)
                sin_r = math.sin(self.roll_rad)

                z_cam = dx * cos_p + dz * sin_p
                y_roll = dy
                z_roll = -dx * sin_p + dz * cos_p

                x_cam = -y_roll * cos_r - z_roll * sin_r
                y_cam = y_roll * sin_r - z_roll * cos_r

                if z_cam > 0.3:
                    u = int(self.cx + (x_cam * self.fx / z_cam))
                    v = int(self.cy + (y_cam * self.fy / z_cam))

                    # Cloth size in pixels (~0.24m x ~0.24m)
                    half_w = int(max(5, (0.12 * self.fx) / z_cam))
                    half_h = int(max(5, (0.12 * self.fy) / z_cam))

                    u_min, u_max = max(0, u - half_w), min(self.width, u + half_w)
                    v_min, v_max = max(0, v - half_h), min(self.height, v + half_h)

                    depth_mm = int(z_cam * 1000.0)
                    depth_img[v_min:v_max, u_min:u_max] = depth_mm

        img_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        img_msg.header.stamp = now_msg
        img_msg.header.frame_id = 'camera_depth_optical_frame'
        self.pub_image.publish(img_msg)

def main():
    rclpy.init()
    node = ClothFlightSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

