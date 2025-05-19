from camera_info_manager import CameraInfoManager
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'image_raw/camera_info', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.publish_image)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Load calibration from YAML file
        self.camera_info_manager = CameraInfoManager(self, 'camera', 'file:///home/rokey-jw/rokeypj_ws/src/aruco_yolo/resource/asdf_calibration.yaml')

        if not self.camera_info_manager.loadCameraInfo():
            self.get_logger().warn("Failed to load camera calibration file.")

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            now = self.get_clock().now().to_msg()

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = "camera"

            # Get pre-loaded CameraInfo
            cam_info_msg = self.camera_info_manager.getCameraInfo()
            cam_info_msg.header = img_msg.header

            self.image_pub.publish(img_msg)
            self.camera_info_pub.publish(cam_info_msg)
            self.get_logger().info("Published image and camera info.")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

