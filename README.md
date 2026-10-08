# PX4 3D Autonomous Exploration

ROS 2 project for experimenting with autonomous 3D exploration of an aerial
robot in a simulated indoor environment. The system combines LiDAR-inertial
odometry, 3D occupancy mapping, frontier-based exploration and PX4 offboard
control.

This repository is presented as a technical demonstration of the implemented
pipeline. It is an evolving research and learning project, not a packaged
framework or a plug-and-play installation guide.

## Demonstrations

The final demonstrations will show two complementary results:

### Frontier-based exploration in RViz

The drone explores an indoor room while RViz displays:

- The vehicle model and sensor frames.
- The LiDAR data and the incrementally built OctoMap.
- Detected 3D frontiers.
- The selected information-gain goal.
- The A* route generated toward the next unexplored region.
- The inflated collision-checking footprint used by the planner.

> Frontier exploration video: to be added.

### 3D reconstruction with FAST-LIO

The LiDAR and IMU data are fused by FAST-LIO to estimate motion and build an
incremental point-cloud reconstruction of the environment. The resulting PCD
map can be inspected independently from the occupancy map used for planning.

> FAST-LIO reconstruction video or screenshot: to be added.

The two outputs are intentionally shown separately:

- **OctoMap**: an occupancy representation used for frontier detection and
  collision-aware planning. It is saved as `octomap.bt` when the mapping node
  shuts down.
- **FAST-LIO map**: an accumulated 3D point cloud used for reconstruction and
  visualization. It is saved as `scans.pcd` when the mapping process shuts
  down.

## What The System Does

The pipeline is designed around a simulated PX4 X500-style quadrotor equipped
with a 3D LiDAR and an RGB/depth camera. Starting from an initially unknown
indoor environment, it:

1. Receives LiDAR point clouds and IMU measurements.
2. Estimates the vehicle motion with FAST-LIO.
3. Integrates LiDAR observations into an in-memory OctoMap.
4. Detects free-space cells adjacent to unknown space as exploration frontiers.
5. Groups frontier cells and selects the next goal using an information-gain
   criterion.
6. Plans a collision-aware 3D route with A*.
7. Converts the route into PX4-compatible offboard setpoints.
8. Repeats the process as new parts of the environment become observable.

## Architecture

```text
Gazebo Sim
  |  LiDAR + IMU + PX4 odometry
  v
FAST-LIO ----------------------> 3D point-cloud reconstruction (.pcd)
  |
  | estimated map -> body motion
  v
frontier_detector_3d ----------> OctoMap (.bt)
  |                         \
  | frontiers and goal        \ occupancy map
  v                            v
exploration_planner_3d ------> 3D A* path
  |
  v
exploration_path_follower ----> PX4 Offboard / TrajectorySetpoint
```

### `frontier_detector_3d`

This node owns the in-memory OctoMap built from `/lidar_3d/points`. Frontier
cells are identified using a 26-neighbourhood test: a free cell is considered
a frontier when it borders unknown space without an occupied neighbour.

The implementation follows the multi-resolution frontier exploration approach
described by Batinovic et al. It projects frontier cells to a coarser Octree
level, clusters them with a Gaussian mean-shift implementation and selects a
goal according to local information gain and distance.

It publishes the occupancy map, frontier markers, the selected exploration
goal and visualization markers for RViz.

### `exploration_planner_3d`

This node runs a global 3D A* search over the OctoMap. It accounts for the
vehicle footprint, allows controlled traversal of unknown cells when needed to
reach a frontier and publishes both a `nav_msgs/Path` and RViz markers.

Goals that are outside the usable map or occupied are projected toward a valid
known-free cell when possible.

### `ros2-px4-waypoints`

This package connects perception and planning to PX4. It provides:

- Gazebo-to-ROS 2 sensor bridging.
- PX4 odometry conversion from NED to ROS ENU.
- The `map -> odom -> base_link` TF structure.
- PX4 offboard, arming and takeoff logic.
- Path following with map-to-NED coordinate conversion.
- RViz configuration for the indoor exploration scene.

Only one node should own PX4 trajectory setpoints during a mission. The
waypoint, takeoff and exploration path-follower nodes are alternative control
experiments, not nodes intended to run concurrently.

## Technology

- ROS 2 Jazzy
- PX4 SITL and PX4 offboard control
- Gazebo Sim
- FAST-LIO 2 for LiDAR-inertial odometry
- OctoMap for 3D occupancy mapping
- 3D frontier detection and mean-shift clustering
- Global 3D A* planning
- RViz2 visualization
- Micro XRCE-DDS Agent for PX4/ROS 2 communication
- C++ and `ament_cmake`

## Environment

The demonstration scene is a self-contained indoor room with walls, boxes and
a column. It uses a custom `x500_test` simulation model containing:

- A 360-degree 3D LiDAR with 16 vertical beams.
- An Oak-D-Lite RGB/depth camera.

The current simulated sensor topics include:

| Topic | Purpose |
| --- | --- |
| `/lidar_3d/points` | 3D LiDAR point cloud |
| `/imu/data` | IMU measurements for FAST-LIO |
| `/depth_camera/image` | Depth image |
| `/depth_camera/points` | Depth point cloud |
| `/odom` | PX4 odometry converted to ROS conventions |

## Outputs

The project produces two different 3D representations:

| Output | Representation | Main use |
| --- | --- | --- |
| `octomap.bt` | Binary occupancy tree | Frontier detection and planning |
| `scans.pcd` | Accumulated LiDAR point cloud | 3D reconstruction and visualization |

The OctoMap is saved by `frontier_detector_3d` on shutdown. FAST-LIO saves its
accumulated point cloud on shutdown when PCD saving is enabled. The output
locations are currently tied to the source/workspace configuration and may be
refined as the project evolves.

## Project Status

Implemented:

- PX4 SITL indoor simulation.
- Gazebo sensor bridge and custom vehicle model.
- PX4 odometry to ROS frame conversion.
- FAST-LIO integration with the simulated LiDAR and IMU.
- Incremental OctoMap construction.
- 3D frontier detection and goal selection.
- Global 3D A* planning.
- PX4 offboard path following.
- Automatic saving of the OctoMap and FAST-LIO point cloud.

In progress:

- Recording and publishing the final frontier-exploration demonstration.
- Recording the FAST-LIO reconstruction demonstration.
- Improving mission completion, landing and shutdown behavior.
- Further validation of planning and frame conversions in more environments.

## Disclaimer

This repository documents an experimental autonomous-drone exploration
pipeline developed for simulation, learning and research. It has not been
presented as a production-ready autonomy stack and should not be deployed on
real hardware without independent validation of estimation, transforms,
planning, command arbitration and failsafe behavior.
