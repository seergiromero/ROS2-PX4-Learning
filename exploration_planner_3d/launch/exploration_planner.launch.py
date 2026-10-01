import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package = 'exploration_planner_3d'
    pkg_share = get_package_share_directory(package)

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation time')

    node = Node(
        package=package,
        executable='exploration_planner_node',
        name='exploration_planner_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            os.path.join(pkg_share, 'config', 'exploration_planner.yaml'),
        ],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        node,
    ])
