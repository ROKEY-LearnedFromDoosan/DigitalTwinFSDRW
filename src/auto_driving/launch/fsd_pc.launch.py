import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='False',
        description='calibration mode type [True, False]'
    )
    calibration_mode = LaunchConfiguration('calibration_mode')

    pkg_auto_driving = get_package_share_directory('auto_driving')

    # Path to parameter files
    projection_param = os.path.join(pkg_auto_driving, 'calibration', 'extrinsic_calibration', 'projection.yaml')
    compensation_param = os.path.join(pkg_auto_driving, 'calibration', 'extrinsic_calibration', 'compensation.yaml')
    lane_param = os.path.join(pkg_auto_driving, 'param', 'lane.yaml')

    # 1. Image Projection Node
    image_projection_node = Node(
        package='auto_driving',
        executable='image_projection',
        namespace='camera',
        name='image_projection',
        output='screen',
        parameters=[projection_param, {'is_extrinsic_camera_calibration_mode': calibration_mode}],
        remappings=[
            ('/camera/image_input', '/camera/image_rect_color'),
            ('/camera/image_input/compressed', '/image_raw/compressed'),
            ('/camera/image_output', '/camera/image_projected'),
            ('/camera/image_output/compressed', '/camera/image_projected/compressed'),
            ('/camera/image_calib', '/camera/image_extrinsic_calib'),
            ('/camera/image_calib/compressed', '/camera/image_extrinsic_calib/compressed')
        ]
    )

    # 2. Image Compensation Node
    image_compensation_node = Node(
        package='auto_driving',
        executable='image_compensation',
        namespace='camera',
        name='image_compensation',
        output='screen',
        parameters=[compensation_param, {'is_extrinsic_camera_calibration_mode': calibration_mode}],
        remappings=[
            ('/camera/image_input', '/camera/image_rect_color'),
            ('/camera/image_input/compressed', '/camera/image_rect_color/compressed'),
            ('/camera/image_output', '/camera/image_compensated'),
            ('/camera/image_output/compressed', '/camera/image_compensated/compressed')
        ]
    )

    # 3. Lane Detection Node
    detect_lane_node = Node(
        package='auto_driving',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[lane_param, {'is_extrinsic_camera_calibration_mode': calibration_mode}],
        remappings=[
            ('/camera/image_input', '/camera/image_projected')
        ]
    )

    # 4. Lane Control Node (Python Node)
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
        detect_lane_node,
        control_lane_node
    ])