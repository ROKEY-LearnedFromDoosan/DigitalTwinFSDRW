from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer  # 여러 노드를 컨테이너에 묶어 실행
from launch_ros.actions import Node  # 독립 실행 노드
from launch_ros.descriptions import ComposableNode  # 컨테이너에 넣을 노드 설명

def generate_launch_description():
    composable_nodes = [
        ComposableNode(  # 왜곡 제거 노드
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace='camera',
            parameters=[{'queue_size': 20}]  # 처리 대기열 크기 설정
        ),

        ComposableNode(  # 센서 Raw 이미지 → 컬러 이미지 변환 노드
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace='camera',
            remappings=[
                ('image_raw', 'image_rect'),  # 입력: 왜곡 제거된 이미지
                ('image_color/compressed', 'image_rect_color/compressed')  # 압축 이미지 리맵핑
            ]
        )
    ]

    republish_node = Node(  # 압축 이미지를 raw 이미지로 변환하는 노드
        package='image_transport',
        executable='republish',
        name='republish',
        output='screen',
        arguments=[
            'compressed',  # 입력 포맷
            'raw'          # 출력 포맷
        ],
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),  # 입력 토픽
            ('out', '/camera/image')  # 출력 토픽
        ]
    )

    image_proc_container = ComposableNodeContainer(  # composable 노드들을 하나로 묶어 실행
        name='image_proc_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,  # 위에서 정의한 composable 노드들
    )

    return LaunchDescription([
        republish_node,        # 독립 노드 실행
        image_proc_container   # 컨테이너 노드 실행
    ])
