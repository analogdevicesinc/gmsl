from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_receiver',
            executable='video_receiver',
            name='rtp_video_receiver_node',
            parameters=[
                {'ip': '127.0.0.1'},
                {'port': 5004},
                {'topic': 'cam0'},
                {'width': 1280},
                {'height': 720}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node'
        )
    ])
    