import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ClothSubscribeNode(Node):
    def __init__(self):
        super().__init__('cloth_subscribe_node')

        # --- パラメータ設定 ---
        self.declare_parameter('hsv_low', [0, 0, 150])      # 白雑巾用：Vを150以上に設定
        self.declare_parameter('hsv_high', [180, 60, 255])
        
        # 雑巾が「開いている」とみなす閾値
        self.declare_parameter('min_area_spread', 15000.0)  # 開いている時の最小面積
        self.declare_parameter('min_solidity_spread', 0.75) # 詰まり具合（0.0~1.0）
        self.declare_parameter('max_aspect_ratio', 2.0)     # 縦横比（2倍以内なら正方形に近い）

        # --- 通信設定 ---
        self.subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # 抽出したデータの配信
        self.publisher_angle = self.create_publisher(Float32, 'cloth/angle', 10)
        self.publisher_is_spread = self.create_publisher(Bool, 'cloth/is_spread', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Mop Analyzer Node (Angle & State) started.")

    def image_callback(self, msg):
        try:
            # ROS 2 Image を OpenCV 形式に変換
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # 1. 前処理（HSV変換とマスク作成）
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_low').value)
        upper = np.array(self.get_parameter('hsv_high').value)
        mask = cv2.inRange(hsv, lower, upper)

        # ノイズ除去
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 2. 輪郭抽出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 最大の物体を選択
            target_cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(target_cnt)

            # 3. 形状解析（回転外接矩形）
            rect = cv2.minAreaRect(target_cnt)
            (x, y), (w, h), angle = rect
            
            # 長辺と短辺の特定
            long_side = max(w, h)
            short_side = min(w, h)
            aspect_ratio = long_side / short_side if short_side > 0 else 0
            
            # 矩形度（詰まり具合）の計算
            solidity = area / (w * h) if (w * h) > 0 else 0

            # 4. 「開いているか」の判定（bool）
            # 面積が大きく、かつ四角形として形が整っている場合のみTrue
            is_spread = (
                area > self.get_parameter('min_area_spread').value and
                solidity > self.get_parameter('min_solidity_spread').value and
                aspect_ratio < self.get_parameter('max_aspect_ratio').value
            )

            # 5. 長辺の角度計算（float）
            # OpenCVのangleは定義が特殊なため、長辺が常に基準になるよう補正
            actual_angle = angle if w < h else angle + 90.0

            # 6. トピック配信
            msg_angle = Float32()
            msg_angle.data = float(actual_angle)
            self.publisher_angle.publish(msg_angle)

            msg_bool = Bool()
            msg_bool.data = bool(is_spread)
            self.publisher_is_spread.publish(msg_bool)

            # --- 視覚化（デバッグ画面） ---
            box = np.int0(cv2.boxPoints(rect))
            color = (0, 255, 0) if is_spread else (0, 0, 255) # 開いていれば緑、それ以外は赤
            
            cv2.drawContours(frame, [box], 0, color, 2)
            status_txt = "SPREAD" if is_spread else "NOT SPREAD"
            cv2.putText(frame, f"{status_txt} | Deg:{actual_angle:.1f}", (int(x), int(y)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # ログ出力（コンソール確認用）
            if is_spread:
                self.get_logger().info(f"MOP OPEN: {actual_angle:.1f} deg")

        # 画面表示
        cv2.imshow("Mop Analysis", frame)
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