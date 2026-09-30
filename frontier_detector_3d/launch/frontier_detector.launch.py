import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('frontier_detector_3d')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time')

    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    node = Node(
        package='frontier_detector_3d',
        executable='frontier_detector_node',
        name='frontier_detector_3d',
        parameters=[
            sim_time,
            os.path.join(pkg_share, 'config', 'frontier_detector.yaml'),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        node,
    ])
