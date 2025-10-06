import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class RTSPBridge(Node):
    def __init__(self):
        super().__init__('rtsp_bridge_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos_profile)
        
        self.bridge = CvBridge()
        
        rpi_ip_address = "100.64.163.104" 
        
        rtsp_url = f"rtsp://{rpi_ip_address}:8554/mystream"
        
        self.get_logger().info(f"Attempting to open RTSP stream: {rtsp_url}")
        self.cap = cv2.VideoCapture(rtsp_url)

        if not self.cap.isOpened():
            self.get_logger().fatal(f"FATAL: Failed to open RTSP stream. Shutting down.")
            raise ConnectionError(f"Failed to open RTSP stream: {rtsp_url}")

        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info(f"RTSP bridge started, publishing to /image_raw")

    def timer_callback(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn("Failed to read frame from RTSP stream.")
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down node and releasing camera.")
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RTSPBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ConnectionError) as e:
        if isinstance(e, ConnectionError):
            print(f"Node startup failed: {e}", file=sys.stderr)
        else:
            print("Node stopped cleanly by user.", file=sys.stderr)
    except Exception as e:
        print(f"An unhandled exception occurred: {e}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
