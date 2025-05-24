#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage # CompressedImage 메시지 타입 추가
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor_node')
        self.get_logger().info('Image Compressor Node has been started.')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.image_callback,
            10) 

        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/camera/image_jpeg', 
            10) 

        self.get_logger().info('Subscribed to /camera/image_raw, publishing to /camera/image_jpeg')

        
        self.jpeg_quality = self.declare_parameter('jpeg_quality', 80).get_parameter_value().integer_value
        self.get_logger().info(f"Using JPEG quality: {self.jpeg_quality}")


    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)

            if not result:
                self.get_logger().warn('cv2.imencode failed')
                return

            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = msg.header.stamp 
            compressed_msg.header.frame_id = msg.header.frame_id 
            compressed_msg.format = "jpeg"
            compressed_msg.data = encimg.tobytes() 

            self.publisher_.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to compress and publish image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageCompressorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in ImageCompressorNode: {e}")
        else:
            print(f"Exception before node creation: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()