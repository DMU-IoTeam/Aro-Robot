import os, threading
import rclpy, roslibpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_PI_IP = os.getenv('PI_IP', '100.64.163.104')
R_TOPIC = '/test_string'
L_TOPIC = '/remote_pi/test_string'

class Bridge(Node):
    def __init__(self):
        super().__init__('roslibpy_bridge')
        self.declare_parameter("pi_ip", DEFAULT_PI_IP)
        pi_ip = self.get_parameter("pi_ip").get_parameter_value().string_value
        self.get_logger().info(f"Connecting to Pi rosbridge @ {pi_ip}:9090")

        # local publisher
        self.pub = self.create_publisher(String, L_TOPIC, 10)

        # roslibpy client
        self.client = roslibpy.Ros(pi_ip, 9090)
        self.client.on_ready(self._on_ready, run_in_thread=True)


    def _on_ready(self):
        sub = roslibpy.Topic(self.client, R_TOPIC, 'std_msgs/msg/String')
        sub.subscribe(lambda m: self._forward(m['data']))
        self.get_logger().info(f"Subscribed to remote {R_TOPIC}")

    def _forward(self, text):
        msg = String(); msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'forward ▶ {text}')

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

