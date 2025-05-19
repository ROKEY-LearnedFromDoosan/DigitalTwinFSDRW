import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')

        # 이미지 수신 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 카메라 정보 구독
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # 마킹된 이미지 발행
        self.image_pub = self.create_publisher(Image, '/camera/image_marked', 10)

        self.bridge = CvBridge()
        self.saved_camera_info = False  # 중복 저장 방지

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 간단히 이미지를 복사해서 발행 (여기서 체스보드 마킹 등 추가 가능)
            marked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(marked_msg)

            self.get_logger().info('Published marked image.')

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def camera_info_callback(self, msg):
        if self.saved_camera_info:
            return  # 이미 저장했으면 패스

        # CameraInfo 메시지를 dict로 변환
        camera_info_dict = {
            'image_width': msg.width,
            'image_height': msg.height,
            'camera_name': msg.header.frame_id,
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': list(msg.k)
            },
            'distortion_model': msg.distortion_model,
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(msg.d),
                'data': list(msg.d)
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': list(msg.r)
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': list(msg.p)
            }
        }

        # YAML 파일로 저장
        yaml_filename = os.path.join(os.getcwd(), 'camera_info.yaml')
        with open(yaml_filename, 'w') as file:
            yaml.dump(camera_info_dict, file)

        self.get_logger().info(f'Camera info saved to {yaml_filename}')
        self.saved_camera_info = True  # 저장 상태 기록

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

