from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_detector_3d',
            executable='frontier_detector_node',
            name='frontier_detector_3d',
            parameters=[
                {'octomap_topic': '/octomap_binary'},
                {'frontier_topic': '/exploration/frontiers'},
                {'min_frontier_size': 10},
                {'max_frontiers': 100},
                {'connectivity': 6},
            ],
            output='screen',
        ),
    ])
