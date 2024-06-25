from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    for i in range(8):
        node = Node(
            package='video_receiver',
            executable='video_receiver',
            name=f'rtp_video_receiver_node{i}',
            parameters=[
            {'ip': '10.42.0.1'},
            {'port': 8420 + i},
            {'topic': f'cam{i}'},
            {'width': 1920},
            {'height': 1280},
            {'timestamp_config': 0}
            ]
        )
        nodes.append(node)

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node'
        )
    )

    return LaunchDescription(nodes)
    