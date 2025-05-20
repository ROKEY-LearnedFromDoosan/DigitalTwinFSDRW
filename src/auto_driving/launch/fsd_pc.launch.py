import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    calibration_mode_arg = DeclareLaunchArgument(
        # calibration_mode 인자 생성
        # calibration_mode:=True -> /camera/image_calib 토픽 발행
        'calibration_mode',
        default_value='True',
        description='calibration mode type [True, False]')
    calibration_mode = LaunchConfiguration('calibration_mode')

    # compensation 파라미터의 위치
    compensation_param = os.path.join(
        get_package_share_directory('auto_driving'),
        'calibration',
        'extrinsic_calibration',
        'compensation.yaml'
        )
    
    detect_param = os.path.join(
        get_package_share_directory('auto_driving'),
        'param',
        'lane',
        'lane.yaml'
        )

    # projection 파라미터의 위치
    projection_param = os.path.join(
        get_package_share_directory('auto_driving'),
        'calibration',
        'extrinsic_calibration',
        'projection.yaml'
        )

    # 실제 노드 정의1
    # 노드 이름 : /camera/image_projection
    image_projection_node = Node(
        package='auto_driving',
        executable='image_projection',
        namespace='camera',
        name='image_projection',
        output='screen',
        parameters=[
            projection_param,
            {'is_extrinsic_camera_calibration_mode': calibration_mode}
        ],
        # 토픽 이름 리매핑(파이썬 스크립트, rqt) 
        remappings=[
            ('/camera/image_input', '/camera/image_rect_color'),
            ('/camera/image_input/compressed', '/image_raw/compressed'),  # 구독: 압축 이미지
            ('/camera/image_output', '/camera/image_projected'),                        # 퍼블리시: 비압축 이미지
            ('/camera/image_output/compressed', '/camera/image_projected/compressed'),
            ('/camera/image_calib', '/camera/image_extrinsic_calib'),                   # 퍼블리시: 비압축 이미지
            ('/camera/image_calib/compressed', '/camera/image_extrinsic_calib/compressed')
        ],

    )

    # 실제 노드 정의2
    # 노드 이름 : /camera/image_compensation
    image_compensation_node = Node(
        package='auto_driving',
        executable='image_compensation',
        namespace='camera',
        name='image_compensation',
        output='screen',
        parameters=[{
            'is_extrinsic_camera_calibration_mode': calibration_mode
            },
            compensation_param
        ],
        remappings=[
            ('/camera/image_input', '/camera/image_rect_color'),
            ('/camera/image_input/compressed', '/camera/image_rect_color/compressed'),  # 구독: 압축 이미지
            ('/camera/image_output', '/camera/image_compensated'),                      # 퍼블리시: 비압축 이미지
            ('/camera/image_output/compressed', '/camera/image_compensated/compressed')
        ]
    )

    detect_lane_node = Node(
        package='auto_driving',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[
            {'is_detection_calibration_mode': calibration_mode, '_image_transport':'compressed'},
            detect_param
        ],
        remappings=[
            ('/detect/image_input', '/camera/image_projected'),
            ('/detect/image_input/compressed', '/camera/image_projected/compressed'),
            ('/detect/image_output', '/detect/image_lane'),
            ('/detect/image_output/compressed', '/detect/image_lane/compressed'),
            ('/detect/image_output_sub1', '/detect/image_white_lane_marker'),
            ('/detect/image_output_sub1/compressed', '/detect/image_white_lane_marker/compressed'),
            ('/detect/image_output_sub2', '/detect/image_yellow_lane_marker'),
            ('/detect/image_output_sub2/compressed', '/detect/image_yellow_lane_marker/compressed')
        ]
    )

    control_lane_node = Node(
        package='auto_driving',
        executable='lane_control',
        name='control_lane',
        output='screen'
    )

    return LaunchDescription([
        calibration_mode_arg,
        image_projection_node,
        image_compensation_node,
        calibration_mode_arg,
        detect_lane_node,
        # control_lane_node
    ])

""" !/usr/bin/env python3

Copyright 2025 ROBOTIS CO., LTD.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Author: Hyungyu Kim """
