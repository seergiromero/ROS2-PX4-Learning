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
    declare_with_path_follower = DeclareLaunchArgument(
        'with_path_follower', default_value='false',
        description='Run the A* path follower (single owner of PX4 motion)')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation time')

    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Read the drone URDF once; robot_state_publisher loads it as its
    # robot_description parameter.
    with open(pkg('urdf', 'x500.urdf')) as f:
        robot_description = f.read()

    actions = []

    # ros_gz_bridge for the sensors (config/bridge.yaml)
    actions.append(RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=pkg('config', 'bridge.yaml')))

    # Publish the drone URDF. The fixed joints (base_link -> lidar3d_link,
    # base_link -> camera_link, and the body visual links) come from the URDF,
    # so the old static_transform_publisher nodes are no longer needed.
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[sim_time, {'robot_description': robot_description}],
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

    # Frontier detector (Batinovic et al., RA-L 2021). It owns the OctoMap: it
    # subscribes to the raw LiDAR cloud, updates an in-memory octree and runs
    # the multi-resolution frontier detection + mean-shift clustering on that
    # same tree. The algorithm fixes the 26-neighbourhood, the
    # free+unknown-no-occupied test and the incremental update, so the only
    # knobs are the octree resolution, the detection level, the mean-shift
    # bandwidth and the exploration box.
    actions.append(Node(
        package='frontier_detector_3d',
        executable='frontier_detector_node',
        name='frontier_detector_3d',
        parameters=[
            sim_time,
            {
                'cloud_topic': '/lidar_3d/points',
                'frontier_topic': '/exploration/frontiers',
                'map_frame': 'map',
                'use_latest_transform': False,
                'publish_map': True,
                'process_rate_hz': 0.5,
                'resolution': 0.3,
                'point_subsample': 3,
                'sensor_model.max_range': 10.0,
                # Multi-resolution level: octomap depth 16 is the finest leaf,
                # 15 makes one detection cell 2 x resolution (0.6 m here).
                'exploration_depth': 15,
                # Mean-shift bandwidth: higher merges more cells into one
                # frontier (must be >= the parent cell size).
                'clustering.kernel_bandwidth': 1.0,
                # Drop single-cell clusters (LiDAR FOV rim noise).
                'min_frontier_size': 6,
                # Cylindrical pose filter around the drone: ignore every
                # frontier within 1 m in XY, and everything above/below it
                # (|z - z_drone| > 0.6 m).
                'min_frontier_radius': 3.5,
                'max_frontier_height': 4.0,
                # Best frontier: information-gain goal selection.
                'publish_best_frontier': True,
                'box_length': 5.0,
                'k_gain': 100.0,
                'lambda': 0.1386,
                'pose_filter_marker_topic': 'frontier_pose_filter_marker',
                'bounds_enabled': False,
                'min_x': -6.0,
                'max_x': 8.0,
                'min_y': -4.0,
                'max_y': 8.0,
                'min_z': 0.3,
                'max_z': 3.7,
                'publish_voxels': True,
                'publish_centroids': True,
            },
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_fast_lio'))))

    # Global planner: A* over the frontier detector's OctoMap. It subscribes to
    # /octomap_binary and /exploration/goal, plans from the vehicle pose and
    # publishes /exploration/path (plus a marker) for RViz and, later, the path
    # follower. Its own config file is the source of truth for the A* settings.
    actions.append(Node(
        package='exploration_planner_3d',
        executable='exploration_planner_node',
        name='exploration_planner_node',
        parameters=[
            sim_time,
            os.path.join(
                get_package_share_directory('exploration_planner_3d'),
                'config', 'exploration_planner.yaml'),
        ],
        output='screen',
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

    # Path follower: follows /exploration/path and streams PX4 offboard
    # setpoints. It is the SINGLE owner of /fmu/in/trajectory_setpoint, so do
    # not enable with_offboard or with_takeoff at the same time. MicroXRCEAgent
    # must already be running in another terminal.
    actions.append(Node(
        package=package,
        executable='exploration_path_follower_node',
        name='exploration_path_follower',
        parameters=[
            sim_time,
            {
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'path_topic': '/exploration/path',
                'octomap_topic': '/octomap_binary',
                'lookahead_distance_m': 0.8,
                'goal_tolerance_m': 0.5,
                'collision_check': True,
                'check_resolution_m': 0.2,
                'path_timeout_s': 2.0,
                'takeoff_height_m': LaunchConfiguration('takeoff_height_m'),
                'setpoint_rate_hz': 50.0,
                'stream_count': 20,
            },
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_path_follower'))))

    return LaunchDescription([
        declare_with_offboard,
        declare_with_takeoff,
        declare_takeoff_height,
        declare_with_fast_lio,
        declare_with_path_follower,
        declare_use_sim_time,
        *actions,
    ])
