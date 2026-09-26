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

        self.start_time = time.time()
        self.fx, self.fy = 425.0, 425.0
        self.cx, self.cy = 424.0, 240.0
        self.pitch_rad = math.radians(20.0) # 20 deg pitch
        self.x_off = 0.25
        self.h_off = 0.35

        self.get_logger().info("Cloth Flight Simulator started! Launching cloth parabolas every 2.5 seconds...")

    def update_and_publish(self):
        now_msg = self.get_clock().now().to_msg()

        # 1. CameraInfo
        info = CameraInfo()
        info.header.stamp = now_msg
        info.header.frame_id = 'camera_depth_optical_frame'
        info.width = 848
        info.height = 480
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)

        # 2. Physics of flying cloth (Cycle: 2.5 seconds)
        t_cycle = (time.time() - self.start_time) % 2.5

        depth_img = np.zeros((480, 848), dtype=np.uint16)

        if t_cycle < 1.0: # Flight duration ~1.0 second
            t = t_cycle
            # Parabolic trajectory in robot frame (base_link)
            # Forward speed: ~3.5 m/s, Initial vertical speed: ~3.2 m/s
            x_rob = self.x_off + 0.15 + 3.5 * t
            y_rob = 0.05 * math.sin(t * 3.0) # slight lateral wobble
            z_rob = self.h_off + 0.1 + 3.2 * t - 0.5 * 9.8 * (t ** 2)

            if z_rob > 0.1: # Above floor
                # Transform base_link -> Camera frame
                # x_rob = x_off + (z_cam*cos(p) - y_cam*sin(p))
                # y_rob = -x_cam
                # z_rob = h_off + (z_cam*sin(p) + y_cam*cos(p))
                dx = x_rob - self.x_off
                dz = z_rob - self.h_off

                cos_p = math.cos(self.pitch_rad)
                sin_p = math.sin(self.pitch_rad)

                z_cam = dx * cos_p + dz * sin_p
                y_cam = -dx * sin_p + dz * cos_p
                x_cam = -y_rob

                if z_cam > 0.3:
                    u = int(self.cx + (x_cam * self.fx / z_cam))
                    v = int(self.cy + (y_cam * self.fy / z_cam))

                    # Cloth size in pixels (~0.25m x ~0.35m)
                    half_w = int(max(4, (0.12 * self.fx) / z_cam))
                    half_h = int(max(4, (0.17 * self.fy) / z_cam))

                    u_min, u_max = max(0, u - half_w), min(848, u + half_w)
                    v_min, v_max = max(0, v - half_h), min(480, v + half_h)

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

