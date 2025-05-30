#!/usr/bin/env python3
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
from aruco_msgs.msg import Marker, MarkerArray
from aruco_dist_msg.msg import ArucoDistance
from cv_bridge import CvBridge
import time

def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(image)
    detect_data = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        
        if rvecs is not None and tvecs is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw, pitch, roll = rotationMatrixToEulerAngles(rot_mat)
                marker_pos = np.dot(-rot_mat.T, tvec).flatten()
                distance = np.linalg.norm(tvec)
                detect_data.append([marker_id, marker_pos, (yaw, pitch, roll), distance])
    return image, detect_data

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs, []

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)

def load_camera_parameters(yaml_file):
    package_share_directory = get_package_share_directory('aruco_yolo')
    calibration_file = os.path.join(package_share_directory, 'config', yaml_file)

    with open(calibration_file, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
        
    
    return camera_matrix, dist_coeffs
    


class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)

        self.marker_publisher = self.create_publisher(MarkerArray, 'detected_markers', 10)
        self.marker_distance_publisher = self.create_publisher(ArucoDistance, 'detected_marker_distance', 10)
        self.img_publisher = self.create_publisher(CompressedImage, 'aruco/compressed', 1)
        
        self.bridge = CvBridge()
        self.marker_size = 0.04
        self.camera_matrix, self.dist_coeffs = load_camera_parameters('calibration_params.yaml')
        self.last_pub_time = time.time()
        # self.camera_matrix  = []
        # self.dist_coeffs = []
    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # width, height, channels = frame.shape
        # self.camera_matrix  = np.array([[360, 0, width/2], [0, 360, height/2], [0, 0, 1]])
        # self.dist_coeffs =  np.array([[0, 0, 0, 0, 0]])
        
        frame, detect_data = detect_markers(frame, self.camera_matrix, self.dist_coeffs, self.marker_size)
        if len(detect_data) == 0:
            self.get_logger().info("No markers detected")
        else:
            closest_marker = min(detect_data, key=lambda x: x[3])
            self.get_logger().info(f"Closest Marker ID: {closest_marker[0]}, Distance: {closest_marker[3]:.2f}m")

            marker_array_msg = ArucoDistance()
            id_list = []
            distance_list = []
            for marker in detect_data:
                marker_array_msg.id.append(int(marker[0]))
                marker_array_msg.distance.append(marker[3])

                # marker_array_msg.markers.append(marker_msg)
                # marker_dist_array_msg.append(marker_dist_msg)
            # self.marker_publisher.publish(marker_array_msg)
            self.marker_distance_publisher.publish(marker_array_msg)
            print(marker_array_msg)

        self.publish_img(frame)
        # cv2.imshow('Detected Markers', frame)
        # cv2.waitKey(1)
    def listener_callback_org(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # width, height, channels = frame.shape
        # self.camera_matrix  = np.array([[360, 0, width/2], [0, 360, height/2], [0, 0, 1]])
        # self.dist_coeffs =  np.array([[0, 0, 0, 0, 0]])
        
        frame, detect_data = detect_markers(frame, self.camera_matrix, self.dist_coeffs, self.marker_size)
        if len(detect_data) == 0:
            self.get_logger().info("No markers detected")
        else:
            closest_marker = min(detect_data, key=lambda x: x[3])
            self.get_logger().info(f"Closest Marker ID: {closest_marker[0]}, Distance: {closest_marker[3]:.2f}m")

            marker_array_msg = MarkerArray()
            marker_dist_array_msg = Float32MultiArray()
            for marker in detect_data:
                marker_msg = Marker()
                marker_msg.id = int(marker[0])
                marker_msg.pose.pose.position.x = marker[1][0]
                marker_msg.pose.pose.position.y = marker[1][1]
                marker_msg.pose.pose.position.z = marker[1][2]
                marker_msg.pose.pose.orientation.x = marker[2][2]
                marker_msg.pose.pose.orientation.y = marker[2][1]
                marker_msg.pose.pose.orientation.z = marker[2][0]

                marker_dist_msg = Float32()
                marker_dist_msg = marker[3]

                marker_array_msg.markers.append(marker_msg)
                # marker_dist_array_msg.append(marker_dist_msg)
            self.marker_publisher.publish(marker_array_msg)
            self.marker_distance_publisher.publish(marker_dist_array_msg)

        self.publish_img(frame)
        # cv2.imshow('Detected Markers', frame)
        # cv2.waitKey(1)

    def publish_img(self, frame):
        time_cur = time.time()

        if time_cur - 0.2 < self.last_pub_time:
            return

        self.last_pub_time = time_cur

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
        _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

        # 압축된 이미지를 CompressedImage 메시지로 변환
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        msg.header.frame_id = "camera"  # 프레임 ID 설정
        msg.format = "jpeg"  # 압축 형식 설정
        msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

        # CompressedImage 퍼블리시
        self.img_publisher.publish(msg)
        # self.get_logger().info('Publishing compressed image...')

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detector = ArucoMarkerDetector()
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Detect ArUco markers.')
    parser.add_argument('--marker_size', type=float, default=0.04,
                        help='Size of the ArUco markers in meters.')
    args = parser.parse_args()
    ArucoMarkerDetector.marker_size = args.marker_size
    main()
