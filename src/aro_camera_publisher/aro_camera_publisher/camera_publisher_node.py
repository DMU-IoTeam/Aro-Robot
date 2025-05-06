#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class CameraPublisherNode(Node):
    """
    웹캠에서 이미지를 캡처하여 ROS 2 토픽으로 발행하는 노드
    """
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.get_logger().info('Camera Publisher Node has been started.')

        # ROS 2 Pubisher
        # Image 메시지 타입을 /camera/image_raw 토픽으로 발행
        # QoS 설정: depth=10
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # 초당 발행 빈도
        timer_period = 1.0 / 30.0  # 30 FPS
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        # OpenCV 비디오 캡처 객체
        # 일단 기본 시스템 카메라 0으로 설정, 돌려보고 다른 카메라면 1, 2 등으로 변경해야 함
        self.cap_ = cv2.VideoCapture(0)
        if not self.cap_.isOpened():
            self.get_logger().error('Could not open video capture device!')
            # 에러 처리 또는 노드 종료 로직 추가해야 함
            rclpy.shutdown() # 일단 카메라 못 열면 노드 종료하게 함
            return

        self.get_logger().info(f'Opened camera with index 0. Frame width: {self.cap_.get(cv2.CAP_PROP_FRAME_WIDTH)}, height: {self.cap_.get(cv2.CAP_PROP_FRAME_HEIGHT)}')

        # CvBridge 객체
        # OpenCV <-> ROS 이미지 변환
        self.bridge_ = CvBridge()

    def timer_callback(self):
        """
        타이머 주기에 맞춰 호출되어 카메라 프레임을 읽고 ROS 토픽으로 발행
        """
        ret, frame = self.cap_.read()  # 카메라에서 프레임 읽기

        if ret:
            # 프레임 읽기 성공 시
            # OpenCV 이미지를 ROS Image 메시지로 변환
            # encoding='bgr8': OpenCV 기본 색상 순서(BGR)와 8비트 3채널
            img_msg = self.bridge_.cv2_to_imgmsg(frame, encoding="bgr8")

            # 메시지 헤더에 타임스탬프 추가
            img_msg.header.stamp = self.get_clock().now().to_msg()
            # 이미지 좌표계 ID 설정
            img_msg.header.frame_id = 'camera_optical_frame' # 예시 이름

            # 메시지 발행
            self.publisher_.publish(img_msg)
            # self.get_logger().info('Publishing video frame') # 너무 자주 출력
        else:
            # 프레임 읽기 실패 시
            self.get_logger().warn('Could not read frame from camera!')

    def destroy_node(self):
        """ 노드 종료 시 카메라 리소스 안전하게 해제 """
        self.get_logger().info('Shutting down Camera Publisher Node.')
        if self.cap_.isOpened():
            self.cap_.release() # 비디오 캡처 객체 해제
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args) # rclpy 초기화
    try:
        camera_publisher = CameraPublisherNode() # 노드 클래스 인스턴스 생성
        rclpy.spin(camera_publisher) # 노드를 계속 실행하며 콜백 함수 호출 대기
    except KeyboardInterrupt:
        # Ctrl+C 로 종료
        pass
    except Exception as e:
        # 기타 예외 발생 시
        if 'camera_publisher' in locals(): # 노드가 생성된 후 에러 발생 시
             camera_publisher.get_logger().error(f"Exception in node: {e}")
        else: # 노드 생성 전 에러 발생 시
             print(f"Exception before node creation: {e}")
    finally:
        # 노드 종료 처리
        if 'camera_publisher' in locals() and rclpy.ok():
            camera_publisher.destroy_node()
        # rclpy 종료
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()