#!/usr/bin/env python3
"""
Lightweight relay node to decompress /camera/camera/color/image_raw/compressed
into uncompressed /cloth/camera_view for RViz2 playback.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DecompressRelayNode(Node):
    def __init__(self):
        super().__init__('decompress_relay_node')
        self.bridge = CvBridge()
        self.pub_ = self.create_publisher(Image, '/cloth/camera_view', 10)
        self.sub_ = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.image_callback,
            10
        )
        self.get_logger().info('Decompress Relay started: /camera/camera/color/image_raw/compressed -> /cloth/camera_view')

    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is not None:
                img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                img_msg.header = msg.header
                self.pub_.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to decompress image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DecompressRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

