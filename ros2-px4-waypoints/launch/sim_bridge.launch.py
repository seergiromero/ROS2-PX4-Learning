import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description() -> LaunchDescription:
    package = 'ros2-px4-waypoints'
    pkg_share = get_package_share_directory(package)

    def pkg(*parts):
        return os.path.join(pkg_share, *parts)

    declare_with_offboard = DeclareLaunchArgument(
        'with_offboard', default_value='false',
        description='Also run the offboard waypoints node')
    declare_with_takeoff = DeclareLaunchArgument(
        'with_takeoff', default_value='false',
        description='Run the automatic offboard takeoff node')
    declare_takeoff_height = DeclareLaunchArgument(
        'takeoff_height_m', default_value='2.0',
        description='Takeoff height above the captured local origin in metres')
    declare_with_fast_lio = DeclareLaunchArgument(
        'with_fast_lio', default_value='true',
        description='Run Fast-LIO mapping')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation time')

    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

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
        parameters=[sim_time],
        output='screen'))
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0.03', '0.002', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[sim_time],
        output='screen'))

    # Dynamic odom -> base_link transform from PX4 vehicle odometry.
    actions.append(Node(
        package=package,
        executable='px4_odometry_tf_node',
        parameters=[sim_time],
        output='screen'))

    # Dynamic map -> odom transform computed from FAST-LIO (map -> body) and
    # PX4 (odom -> base_link). This aligns the two independent origins so that
    # the depth camera and the FAST-LIO laser map coincide in RViz.
    actions.append(Node(
        package=package,
        executable='map_odom_tf_bridge_node',
        parameters=[sim_time],
        output='screen'))

    # Include Fast-LIO but keep one RViz instance: the project's RViz.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fast_lio'),
            'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_file': 'avia.yaml',
            # This FAST-LIO fork uses its own wall-clock processing timers.
            # Its published messages retain the sensor (Gazebo) timestamps.
            'use_sim_time': 'false',
            'rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_fast_lio'))))

    # RViz uses the Fast-LIO map as the common fixed frame. The use_sim_time
    # parameter must be passed as a CLI argument: rviz2 ignores the parameters
    # block of the Node action.
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', pkg('rviz', 'indoor_room.rviz'),
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        output='screen'))

    # Offboard waypoints node (optional). MicroXRCEAgent must already be
    # running in another terminal before enabling this argument.
    actions.append(Node(
        package=package,
        executable='offboard_waypoints_node',
        parameters=[sim_time],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_offboard'))))

    actions.append(Node(
        package=package,
        executable='offboard_takeoff_node',
        parameters=[{
            'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_takeoff'))))

    return LaunchDescription([
        declare_with_offboard,
        declare_with_takeoff,
        declare_takeoff_height,
        declare_with_fast_lio,
        declare_use_sim_time,
        *actions,
    ])
