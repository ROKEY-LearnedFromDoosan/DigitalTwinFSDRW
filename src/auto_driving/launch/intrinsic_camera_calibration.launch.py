from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    use_calibration = LaunchConfiguration('use_calibration').perform(context) == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    params = {'queue_size': 20, "use_sim_time":use_sim_time}
    
    if use_calibration:
        params['camera_info_url'] = 'file:///home/rokey/DigitalTwinFSD/src/auto_driving/resource/asdf_calibration.yaml'

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace='camera',
            parameters=[params],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace='camera',
            parameters=[params],
            remappings=[
                ('image_raw', 'image_rect'),
                ('image_color', 'image_rect_color')
            ]
        )
    ]

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish',
        output='screen',
        arguments=['compressed', 'raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
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

    return [republish_node, image_proc_container]

def generate_launch_description():
    declare_use_calibration_arg = DeclareLaunchArgument(
        'use_calibration',
        default_value='false',
        description='Use camera calibration file (true/false)'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        declare_use_calibration_arg,
        declare_use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])

