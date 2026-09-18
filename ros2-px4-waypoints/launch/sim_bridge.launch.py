import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description() -> LaunchDescription:
    package = 'ros2-px4-waypoints'
    pkg_share = get_package_share_directory(package)

    def pkg(*parts):
        return os.path.join(pkg_share, *parts)

    declare_start_gz = DeclareLaunchArgument(
        'start_gz', default_value='true',
        description='Launch Gazebo with the indoor_room world')
    declare_with_offboard = DeclareLaunchArgument(
        'with_offboard', default_value='false',
        description='Also run the offboard waypoints node')
    declare_with_takeoff = DeclareLaunchArgument(
        'with_takeoff', default_value='false',
        description='Run the automatic offboard takeoff node')
    declare_takeoff_height = DeclareLaunchArgument(
        'takeoff_height_m', default_value='2.0',
        description='Takeoff height above the captured local origin in metres')

    actions = []

    # ros_gz_bridge for the sensors (config/bridge.yaml)
    actions.append(RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=pkg('config', 'bridge.yaml')))

    # Static TF from the drone body to the sensors.
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'lidar3d_link'],
        output='screen'))
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0.03', '0.002', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'))

    # RViz with the sensor displays and base_link as fixed frame.
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', pkg('rviz', 'indoor_room.rviz')],
        output='screen'))

    # Offboard waypoints node (optional). MicroXRCEAgent must already be
    # running in another terminal before enabling this argument.
    actions.append(Node(
        package=package,
        executable='offboard_waypoints_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_offboard'))))

    actions.append(Node(
        package=package,
        executable='offboard_takeoff_node',
        parameters=[{
            'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_takeoff'))))

    return LaunchDescription([
        declare_with_offboard,
        declare_with_takeoff,
        declare_takeoff_height,
        *actions,
    ])
