from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    use_calibration = LaunchConfiguration('use_calibration')

    declare_use_calibration_arg = DeclareLaunchArgument(
        'use_calibration',
        default_value='false',
        description='Use camera calibration file (true/false)'
    )

    # Calibration parameters based on toggle
    calibration_params = [
        {
            'queue_size': 20,
            'camera_info_url': 'file:///home/rokey/camera_calibration.yaml'
        }
    ]

    no_calibration_params = [
        {
            'queue_size': 20
        }
    ]

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace='camera',
            parameters=[calibration_params if use_calibration.perform(None) == 'true' else no_calibration_params]
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace='camera',
            remappings=[
                ('image_raw', 'image_rect'),
                ('image_color/compressed', 'image_rect_color/compressed')
            ]
        )
    ]

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),
            ('out', '/camera/image')
        ]
    )

    image_proc_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes
    )

    return LaunchDescription([
        declare_use_calibration_arg,
        republish_node,
        image_proc_container
    ])
