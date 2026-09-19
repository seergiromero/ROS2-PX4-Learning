# ROS2-PX4-Learning

ROS 2 and PX4 offboard-control learning workspace using Gazebo Sim and a
custom `x500_test` model.

The world at
`ros2-px4-waypoints/worlds/indoor_room.sdf` is a small indoor test room. It
contains walls, two boxes and a column, and does not depend on external world
assets.

The custom model is located at
`~/PX4-Autopilot/Tools/simulation/gz/models/x500_test`. It includes:

- A 3D lidar with 360 horizontal samples and 16 vertical beams.
- An Oak-D-Lite RGB/depth camera.

## Requirements

- PX4-Autopilot built with the custom `22000_gz_x500_test` airframe.
- ROS 2 Jazzy.
- Gazebo Sim.
- `MicroXRCEAgent` built and available after sourcing the workspace.

Build the ROS 2 workspace before the first run:

```bash
cd ~/px4_ros2_rust_offboard_ws
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Launch Procedure

Use one terminal for each step. Start the terminals in the order shown below.

### 1. Start Gazebo

In terminal 1:

```bash
cd ~/px4_ros2_rust_offboard_ws
source /opt/ros/jazzy/setup.zsh
export GZ_SIM_RESOURCE_PATH="$PWD/src/ROS2-PX4-Learning/ros2-px4-waypoints/worlds:${GZ_SIM_RESOURCE_PATH:-}"
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
  "$PWD/src/ROS2-PX4-Learning/ros2-px4-waypoints/worlds/indoor_room.sdf"
```

Keep this terminal running.

### 2. Start Micro XRCE-DDS Agent

In terminal 2:

```bash
cd ~/px4_ros2_rust_offboard_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
MicroXRCEAgent udp4 -p 8888
```

This agent transports PX4 topics such as `/fmu/in/*` and `/fmu/out/*` to and
from ROS 2. Keep this terminal running before starting PX4.

### 3. Start PX4 SITL

In terminal 3:

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=indoor_room \
PX4_SYS_AUTOSTART=22000 \
PX4_SIM_MODEL=x500_test \
PX4_GZ_MODEL_POSE=0,-2,1.5 \
make px4_sitl gz_x500_test
```

PX4 connects to the agent on UDP port `8888` and spawns `x500_test_0` in the
running Gazebo world. Keep this terminal running.

### 4. Start the ROS 2 launch

In terminal 4:

```bash
cd ~/px4_ros2_rust_offboard_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch ros2-px4-waypoints sim_bridge.launch.py
```

This launch starts:

- The Gazebo-to-ROS 2 sensor bridge.
- The dynamic `odom -> base_link` transform from PX4 vehicle odometry.
- Static TF from `base_link` to `lidar3d_link`.
- Static TF from `base_link` to `camera_link`.
- RViz with the 3D lidar and depth-image displays.

The launch does not start Gazebo, PX4 or `MicroXRCEAgent`.

## Automatic Takeoff

To make the vehicle enter Offboard mode, arm and climb to a specified height:

```bash
ros2 launch ros2-px4-waypoints sim_bridge.launch.py \
  with_takeoff:=true \
  takeoff_height_m:=2.0
```

The takeoff node captures the current local XY position, climbs vertically to
the requested height and holds that position. PX4 uses NED coordinates, so a
positive height such as `2.0` is internally sent as `z = -2.0`.

Do not run `with_takeoff:=true` and `with_offboard:=true` at the same time. Both
nodes would publish competing Offboard setpoints.

## Waypoint Mission

To run the existing waypoint node instead of the takeoff node:

```bash
ros2 launch ros2-px4-waypoints sim_bridge.launch.py \
  with_offboard:=true
```

The waypoint node takes off and then follows the waypoints defined in
`src/offboard_waypoints_node.cpp`.

## ROS 2 Sensor Topics

The launch configures these Gazebo-to-ROS 2 mappings in
`config/bridge.yaml`:

| ROS 2 topic | Message type | Sensor |
| --- | --- | --- |
| `/lidar_3d/scan` | `sensor_msgs/msg/LaserScan` | 3D lidar scan |
| `/lidar_3d/points` | `sensor_msgs/msg/PointCloud2` | 3D lidar point cloud |
| `/depth_camera/image` | `sensor_msgs/msg/Image` | Depth image |
| `/depth_camera/points` | `sensor_msgs/msg/PointCloud2` | Depth point cloud |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU (accelerometer + gyroscope) |

Check the topics after launching:

```bash
ros2 topic list | grep -E "lidar|depth|fmu"
```

Check the TF tree:

```bash
ros2 run tf2_ros tf2_echo base_link lidar3d_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

RViz uses `base_link` as its fixed frame.

The current odometry node publishes `/odom` and the `odom -> base_link` TF.
This is PX4 state odometry, not SLAM yet; a future lidar-SLAM node will use it
alongside `/lidar_3d/points` to build and save a 3D map.

## Stopping

Stop the ROS 2 launch with `Ctrl+C`, then stop PX4, the Micro XRCE-DDS Agent
and Gazebo in their respective terminals. If a previous simulation remains
running, stop it before starting another one to avoid duplicate Gazebo models
or DDS connections.
