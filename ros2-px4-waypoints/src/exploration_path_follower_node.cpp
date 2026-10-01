#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/path.hpp>

#include "ros2-px4-waypoints/path_follower.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace ros2_px4_waypoints
{

/// Follows the global A* path and streams PX4 offboard setpoints.
///
/// This node is the single owner of the PX4 motion commands: it streams
/// `OffboardControlMode`, engages offboard, arms, takes off, and then follows
/// `/exploration/path` with a lookahead point converted from the map frame to
/// the PX4 local NED frame. It holds position when the path is missing/expired
/// or when the next segment is blocked by occupied space, letting the global
/// planner produce a new path.
///
/// Do not run it together with `offboard_waypoints_node` or
/// `offboard_takeoff_node`: all of them publish to
/// `/fmu/in/trajectory_setpoint`.
class ExplorationPathFollower : public rclcpp::Node
{
public:
  ExplorationPathFollower()
  : Node("exploration_path_follower")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    const std::string path_topic =
      declare_parameter<std::string>("path_topic", "/exploration/path");
    const std::string octomap_topic =
      declare_parameter<std::string>("octomap_topic", "/octomap_binary");

    lookahead_m_ = declare_parameter<double>("lookahead_distance_m", 0.8);
    goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m", 0.5);
    collision_check_ = declare_parameter<bool>("collision_check", true);
    check_resolution_m_ = declare_parameter<double>("check_resolution_m", 0.2);
    path_timeout_s_ = declare_parameter<double>("path_timeout_s", 2.0);
    takeoff_height_m_ = declare_parameter<double>("takeoff_height_m", 2.0);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.25);
    const double setpoint_rate_hz = declare_parameter<double>("setpoint_rate_hz", 50.0);
    const int stream_count = declare_parameter<int>("stream_count", 20);

    stream_count_target_ = static_cast<uint32_t>(stream_count > 0 ? stream_count : 20);

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    trajectory_pub_ = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    local_position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleLocalPosition::SharedPtr msg) {local_position_ = *msg;});

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", rclcpp::SensorDataQoS(),
      [this](const VehicleStatus::SharedPtr msg) {status_ = *msg;});

    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack_v1", rclcpp::SensorDataQoS(),
      [this](const VehicleCommandAck::SharedPtr msg) {
        if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
        msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
        {
          RCLCPP_INFO(get_logger(), "PX4 ACK command=%u result=%u", msg->command, msg->result);
        }
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&ExplorationPathFollower::pathCallback, this, std::placeholders::_1));

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&ExplorationPathFollower::octomapCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / std::max(setpoint_rate_hz, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ExplorationPathFollower::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Path follower: path '%s', map '%s' -> PX4 local NED "
      "(lookahead=%.2f m, takeoff=%.2f m, collision_check=%d)",
      path_topic.c_str(), map_frame_.c_str(), lookahead_m_, takeoff_height_m_,
      static_cast<int>(collision_check_));
  }

private:
  enum class State
  {
    STREAMING,
    WAIT_OFFBOARD,
    WAIT_ARMED,
    TAKEOFF,
    FOLLOW,
    HOLD
  };

  void tick()
  {
    publishOffboardMode();

    switch (state_) {
      case State::STREAMING:
        publishHold();
        if (++stream_count_ >= stream_count_target_) {
          publishCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          RCLCPP_INFO(get_logger(), "Offboard mode requested");
          state_ = State::WAIT_OFFBOARD;
        }
        break;

      case State::WAIT_OFFBOARD:
        publishHold();
        if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          publishCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
          RCLCPP_INFO(get_logger(), "Offboard accepted; arm requested");
          state_ = State::WAIT_ARMED;
        }
        break;

      case State::WAIT_ARMED:
        publishHold();
        if (status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(get_logger(), "Armed; starting mission");
          state_ = takeoff_height_m_ > 0.0 ? State::TAKEOFF : State::FOLLOW;
        }
        break;

      case State::TAKEOFF:
        publishTakeoff();
        if (takeoffReached()) {
          RCLCPP_INFO(get_logger(), "Takeoff complete; following the path");
          state_ = State::FOLLOW;
        }
        break;

      case State::FOLLOW:
        followStep();
        break;

      case State::HOLD:
        publishHold();
        if (hasUsablePath()) {
          state_ = State::FOLLOW;
        }
        break;
    }
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<octomap::point3d> path;
    path.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      path.emplace_back(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = std::move(path);
    path_stamp_ = now();
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract(
      msg->binary ? octomap_msgs::binaryMsgToMap(*msg) : octomap_msgs::fullMsgToMap(*msg));
    if (!abstract) {
      return;
    }
    auto * octree = dynamic_cast<octomap::OcTree *>(abstract.get());
    if (octree == nullptr) {
      return;
    }
    abstract.release();
    std::lock_guard<std::mutex> lock(mutex_);
    tree_.reset(octree);
  }

  bool hasUsablePath() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path_.empty()) {
      return false;
    }
    return (now() - path_stamp_).seconds() <= path_timeout_s_;
  }

  void followStep()
  {
    octomap::point3d current;
    if (!currentMapPosition(current)) {
      publishHold();
      return;
    }

    std::vector<octomap::point3d> path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path = path_;
    }
    if (path.empty() || (now() - path_stamp_).seconds() > path_timeout_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No usable path; holding position");
      state_ = State::HOLD;
      publishHold();
      return;
    }

    // Goal reached: the frontier detector will publish a new goal soon, so hold
    // until a fresh path arrives.
    if ((path.back() - current).norm() < goal_tolerance_m_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Goal reached; holding for a new goal");
      state_ = State::HOLD;
      publishHold();
      return;
    }

    octomap::point3d target;
    if (!path_follower::selectLookaheadPoint(path, current, lookahead_m_, target)) {
      state_ = State::HOLD;
      publishHold();
      return;
    }

    if (segmentBlocked(current, target)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Next path segment is blocked; holding and waiting for a new plan");
      state_ = State::HOLD;
      publishHold();
      return;
    }

    std::array<float, 3> ned{};
    if (!toPx4Ned(target, ned)) {
      publishHold();
      return;
    }
    publishSetpoint(ned[0], ned[1], ned[2], 0.0f);
  }

  bool currentMapPosition(octomap::point3d & point) const
  {
    try {
      const auto base_to_map = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, rclcpp::Time(0));
      const auto & t = base_to_map.transform.translation;
      point = octomap::point3d(t.x, t.y, t.z);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No transform %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  /// Converts a map-frame point to the PX4 local NED setpoint frame.
  ///
  /// `odom` is ENU (see px4_odometry_tf_node) and is related to `map` by the
  /// map_odom_tf_bridge transform, so `p_odom = odom_T_map * p_map` and then
  /// `ned = (p_odom.y, p_odom.x, -p_odom.z)`.
  bool toPx4Ned(const octomap::point3d & map_point, std::array<float, 3> & ned) const
  {
    geometry_msgs::msg::TransformStamped odom_to_map;
    try {
      odom_to_map = tf_buffer_->lookupTransform(odom_frame_, map_frame_, rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No transform %s -> %s: %s",
        odom_frame_.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
    const auto & t = odom_to_map.transform.translation;
    const auto & q = odom_to_map.transform.rotation;
    const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    const tf2::Vector3 p_map(map_point.x(), map_point.y(), map_point.z());
    const tf2::Vector3 p_odom =
      tf2::quatRotate(rotation, p_map) + tf2::Vector3(t.x, t.y, t.z);
    ned = {
      static_cast<float>(p_odom.y()),
      static_cast<float>(p_odom.x()),
      static_cast<float>(-p_odom.z())};
    return true;
  }

  bool segmentBlocked(const octomap::point3d & a, const octomap::point3d & b) const
  {
    if (!collision_check_) {
      return false;
    }
    std::shared_ptr<octomap::OcTree> tree;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tree = tree_;
    }
    if (!tree) {
      return false;
    }
    const double distance = (b - a).norm();
    const int steps = std::max(
      1, static_cast<int>(std::ceil(distance / std::max(check_resolution_m_, 0.05))));
    for (int i = 0; i <= steps; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(steps);
      const octomap::point3d p = a + (b - a) * t;
      const octomap::OcTreeNode * node = tree->search(p);
      if (node != nullptr && tree->isNodeOccupied(node)) {
        return true;
      }
    }
    return false;
  }

  bool takeoffReached() const
  {
    if (!local_position_.z_valid) {
      return false;
    }
    return std::abs(local_position_.z - (-takeoff_height_m_)) < position_tolerance_m_;
  }

  void publishTakeoff()
  {
    if (!local_position_.xy_valid) {
      return;
    }
    publishSetpoint(local_position_.x, local_position_.y, -takeoff_height_m_, 0.0f);
  }

  void publishHold()
  {
    if (local_position_.xy_valid && local_position_.z_valid) {
      publishSetpoint(local_position_.x, local_position_.y, local_position_.z, 0.0f);
    } else {
      // No local position yet: stream a neutral setpoint so PX4 keeps receiving
      // proof-of-life plus a setpoint before it engages offboard.
      publishSetpoint(0.0f, 0.0f, static_cast<float>(-takeoff_height_m_), 0.0f);
    }
  }

  void publishOffboardMode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = timestampNow();
    offboard_mode_pub_->publish(msg);
  }

  void publishSetpoint(float x, float y, float z, float yaw)
  {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.velocity = {nan(), nan(), nan()};
    msg.acceleration = {nan(), nan(), nan()};
    msg.yaw = yaw;
    msg.yawspeed = nan();
    msg.timestamp = timestampNow();
    trajectory_pub_->publish(msg);
  }

  void publishCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = timestampNow();
    command_pub_->publish(msg);
  }

  static float nan()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  uint64_t timestampNow() const
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;

  double lookahead_m_{0.8};
  double goal_tolerance_m_{0.5};
  double check_resolution_m_{0.2};
  double path_timeout_s_{2.0};
  double takeoff_height_m_{2.0};
  double position_tolerance_m_{0.25};
  uint32_t stream_count_target_{20};
  bool collision_check_{true};

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::vector<octomap::point3d> path_;
  rclcpp::Time path_stamp_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<octomap::OcTree> tree_;

  VehicleLocalPosition local_position_{};
  VehicleStatus status_{};
  State state_{State::STREAMING};
  uint32_t stream_count_{0};
};

}  // namespace ros2_px4_waypoints

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_px4_waypoints::ExplorationPathFollower>());
  rclcpp::shutdown();
  return 0;
}
