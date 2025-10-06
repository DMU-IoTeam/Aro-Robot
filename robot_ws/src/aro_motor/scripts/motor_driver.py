#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # 시리얼 포트 설정
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 9600
        self.ser = None

        # 아두이노 연결 시도
        self.connect_to_arduino()

        # /cmd_vel 토픽을 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('Motor driver node has been started.')

    def connect_to_arduino(self):
        try:
            self.get_logger().info(f"Connecting to Arduino on port {self.arduino_port}...")
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            time.sleep(2) # 아두이노가 리셋될 시간을 줍니다.
            self.get_logger().info("Arduino connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        command = 's' # 기본값은 정지

        if linear_x > 0:
            command = 'f' # 전진
        elif linear_x < 0:
            command = 'b' # 후진
        elif angular_z > 0:
            command = 'l' # 좌회전
        elif angular_z < 0:
            command = 'r' # 우회전
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command.encode())
                self.get_logger().info(f"Sent command to Arduino: {command}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriver()
    try:
        rclpy.spin(motor_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
