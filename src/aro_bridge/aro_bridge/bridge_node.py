import numpy as np
import traceback
import cv2
import os, threading
import rclpy, roslibpy
import base64
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage, Image 
from cv_bridge import CvBridge, CvBridgeError

DEFAULT_PI_IP = os.getenv('PI_IP', '100.64.163.104')
R_TOPIC = "/camera/image_jpeg"
L_TOPIC = "/remote_pi/camera/image_raw" 

class Bridge(Node):
    def __init__(self):
        super().__init__('roslibpy_bridge')
        self.declare_parameter("pi_ip", DEFAULT_PI_IP)
        pi_ip = self.get_parameter("pi_ip").get_parameter_value().string_value
        self.get_logger().info(f"Connecting to Pi rosbridge @ {pi_ip}:9090")

        # local publisher
        self.pub = self.create_publisher(Image, L_TOPIC, 10)
        self.bridge = CvBridge()

        # roslibpy client
        self.client = roslibpy.Ros(pi_ip, 9090)
        self.client.on_ready(self._on_ready, run_in_thread=True)


    def _on_ready(self):
        self.get_logger().info("roslibpy ready!!!")
        sub = roslibpy.Topic(
            self.client, R_TOPIC,
            'sensor_msgs/msg/CompressedImage',
            queue_length=1,
            throttle_rate=0,
            compression='none'
        )
        sub.subscribe(self._forward_compressed)
        self.get_logger().info(f"Subscribed to remote {R_TOPIC} (type: CompressedImage)")

    def _forward_compressed(self, msg_dict):
        try:
            compressed_data_base64 = msg_dict.get("data")
            if not compressed_data_base64:
                self.get_logger().warn("No 'data' in compressed image message.")
                return
            
            img_format = msg_dict.get("format", "jpeg")
            compressed_bytes = base64.b64decode(compressed_data_base64)

            np_arr = np.frombuffer(compressed_bytes, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn(f"cv2.imdecode failed for format '{img_format}'.")
                return

            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "remote_camera_compressed"

            self.pub.publish(ros_image_msg)
            self.get_logger().info(
                f"forward compressed ▶ {ros_image_msg.width}×{ros_image_msg.height} (format: {img_format}, original bytes: {len(compressed_bytes)})"
            )

        except Exception as e:
            self.get_logger().error(f"Compressed image forward error: {e}\n{traceback.format_exc()}")

            """
            img = Image()
            img.header = Header()
            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = "remote_camera"

            img.height       = msg_dict["height"]
            img.width        = msg_dict["width"]
            img.encoding     = msg_dict["encoding"]
            img.is_bigendian = msg_dict["is_bigendian"]
            img.step         = msg_dict["step"]

            # --- data 변환 (Base-64 → bytes) ---
            raw = msg_dict["data"]
            if isinstance(raw, str):        # 표준 rosbridge 형식
                img.data = base64.b64decode(raw)
            else:                           # list[int] fallback
                img.data = bytes(raw)

            self.pub.publish(img)
            self.get_logger().info(
                f"forward ▶ {img.width}×{img.height} ({len(img.data)} B)"
            )

        except Exception as e:
            self.get_logger().error(f"Image forward error: {e}")

        """

def main():
    rclpy.init()
    node = Bridge()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    try:
        node.client.run_forever()
    finally:
        node.client.terminate()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

