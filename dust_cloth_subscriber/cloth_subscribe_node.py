import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ClothSubscribeNode(Node):
    def __init__(self):
        super().__init__('cloth_subscribe_node')

        # --- パラメータ設定 (ここを調整して精度を追い込みます) ---
        self.declare_parameter('hsv_low', [0, 0, 150])      
        self.declare_parameter('hsv_high', [180, 60, 255])
        self.declare_parameter('min_area', 15000.0)      # 最小面積
        self.declare_parameter('min_solidity', 0.75)     # 埋没度（0.7〜0.9推奨）
        self.declare_parameter('max_aspect_ratio', 1.8)  # 縦横比（2.0以下なら正方形に近い）

        self.armdata = 0.0
        self.latest_depth_frame = None

        # --- カメラパラメータ (D435用) ---
        self.fx, self.fy = 600.0, 600.0
        self.cx, self.cy = 320.0, 240.0

        # --- 通信設定 ---
        self.sub_rgb = self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)

        # パブリッシャー
        self.publisher_pos = self.create_publisher(Float32MultiArray, 'cloth/pos_xyz', 10)
        self.publisher_angle = self.create_publisher(Float32, 'cloth/angle', 10)
        self.publisher_is_spread = self.create_publisher(Bool, 'cloth/is_spread', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Mop 3D Analyzer (High Precision) started.")

    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError:
            pass

    def rgb_callback(self, rgb_msg):
        if self.latest_depth_frame is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError:
            return

        # 1. 前処理（マスク作成とノイズ除去）
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_low').value)
        upper = np.array(self.get_parameter('hsv_high').value)
        mask = cv2.inRange(hsv, lower, upper)
        
        # 小さなノイズを消して形を整える
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) 

        # 2. 輪郭抽出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            target_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(target_cnt)
            
            # 3. 形状解析
            rect = cv2.minAreaRect(target_cnt)
            (u, v), (w, h), angle = rect
            
            long_side = max(w, h)
            short_side = min(w, h)
            aspect_ratio = long_side / short_side if short_side > 0 else 0
            solidity = area / (w * h) if (w * h) > 0 else 0

            # --- 厳格な判定条件 ---
            is_valid_mop = (
                area > self.get_parameter('min_area').value and
                solidity > self.get_parameter('min_solidity').value and
                aspect_ratio < self.get_parameter('max_aspect_ratio').value
            )

            if is_valid_mop:
                # 4. 3D座標計算
                z_mm = self.latest_depth_frame[int(v), int(u)]
                z_m = float(z_mm) / 1000.0

                if z_m > 0:
                    real_x = (u - self.cx) * z_m / self.fx
                    real_y = (v - self.cy) * z_m / self.fy
                    actual_angle = angle if w < h else angle + 90.0

                    # ログ出力
                    self.get_logger().info(f"VALID: X:{real_x:.2f} Y:{real_y:.2f} Z:{z_m:.2f} Ang:{actual_angle:.1f}")

                    # トピック配信
                    self.publisher_pos.publish(Float32MultiArray(data=[real_x, real_y, z_m]))
                    self.publisher_angle.publish(Float32(data=float(actual_angle)))
                    self.publisher_is_spread.publish(Bool(data=True))

                    # 描画（成功時：緑）
                    box = cv2.boxPoints(rect).astype(np.int32)
                    cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                    cv2.putText(frame, f"XYZ: {real_x:.2f},{real_y:.2f},{z_m:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, "Depth Error", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                # 判定に落ちた場合（失敗時：赤）
                cv2.putText(frame, "Searching Mop...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                self.publisher_is_spread.publish(Bool(data=False))

        cv2.imshow("Mop 3D Analysis", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ClothSubscribeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()