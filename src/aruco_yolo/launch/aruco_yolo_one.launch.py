from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_yolo',
            executable='raw_image_with_yaml',
            name='raw_image_with_yaml',
            output='screen'
        )
    ])
