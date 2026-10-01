#include "exploration_planner_3d/exploration_planner_node.hpp"

#include <octomap_msgs/conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace exploration_planner_3d
{

ExplorationPlannerNode::ExplorationPlannerNode()
: Node("exploration_planner_node")
{
  using namespace std::chrono_literals;

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  const std::string octomap_topic =
    declare_parameter<std::string>("octomap_topic", "/octomap_binary");
  const std::string goal_topic =
    declare_parameter<std::string>("goal_topic", "/exploration/goal");
  const std::string path_topic =
    declare_parameter<std::string>("path_topic", "/exploration/path");
  const std::string path_marker_topic =
    declare_parameter<std::string>("path_marker_topic", "/exploration/path_marker");
  const double planning_rate_hz = declare_parameter<double>("planning_rate_hz", 1.0);

  AStar3D::Config config;
  config.resolution = declare_parameter<double>("resolution", 0.3);
  config.inflation_radius = declare_parameter<double>("inflation_radius", 0.3);
  config.allow_unknown = declare_parameter<bool>("allow_unknown", true);
  config.unknown_cost = declare_parameter<double>("unknown_cost", 3.0);
  config.max_nodes = static_cast<std::size_t>(declare_parameter<int>("max_nodes", 500000));
  config.heuristic_weight = declare_parameter<double>("heuristic_weight", 1.0);
  config.simplify = declare_parameter<bool>("simplify", true);

  bounds_.enabled = declare_parameter<bool>("bounds_enabled", false);
  bounds_.min_x = declare_parameter<double>("min_x", -6.0);
  bounds_.max_x = declare_parameter<double>("max_x", 6.0);
  bounds_.min_y = declare_parameter<double>("min_y", -4.0);
  bounds_.max_y = declare_parameter<double>("max_y", 4.0);
  bounds_.min_z = declare_parameter<double>("min_z", 0.3);
  bounds_.max_z = declare_parameter<double>("max_z", 3.7);

  try {
    planner_ = std::make_unique<AStar3D>(config);
  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(get_logger(), "Invalid planner configuration: %s", e.what());
    throw;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // The frontier detector publishes the map transient-local, so matching that
  // QoS makes the planner receive the current map immediately at start-up.
  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic, rclcpp::QoS(1).transient_local(),
    std::bind(&ExplorationPlannerNode::octomapCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, 10,
    std::bind(&ExplorationPlannerNode::goalCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    path_topic, rclcpp::QoS(1).transient_local());
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    path_marker_topic, rclcpp::QoS(1).transient_local());

  const auto period = std::chrono::duration<double>(1.0 / std::max(planning_rate_hz, 0.1));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&ExplorationPlannerNode::planCycle, this));

  RCLCPP_INFO(
    get_logger(),
    "Exploration planner: map '%s', goal '%s', path '%s' "
    "(resolution=%.2f, inflation=%.2f, allow_unknown=%d)",
    octomap_topic.c_str(), goal_topic.c_str(), path_topic.c_str(),
    config.resolution, config.inflation_radius, static_cast<int>(config.allow_unknown));
}

void ExplorationPlannerNode::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::unique_ptr<octomap::AbstractOcTree> abstract(
    msg->binary ? octomap_msgs::binaryMsgToMap(*msg) : octomap_msgs::fullMsgToMap(*msg));
  if (!abstract) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to convert the OctoMap message");
    return;
  }
  auto * octree = dynamic_cast<octomap::OcTree *>(abstract.get());
  if (octree == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "OctoMap message is not an OcTree");
    return;
  }
  abstract.release();
  std::lock_guard<std::mutex> lock(mutex_);
  tree_.reset(octree);
}

void ExplorationPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goal_ = *msg;
  RCLCPP_INFO(
    get_logger(), "New exploration goal (%.2f, %.2f, %.2f) in frame '%s'",
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    msg->header.frame_id.c_str());
}

void ExplorationPlannerNode::planCycle()
{
  std::shared_ptr<octomap::OcTree> tree;
  std::optional<geometry_msgs::msg::PoseStamped> goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tree = tree_;
    goal = goal_;
  }
  if (!tree) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "No OctoMap received yet, waiting...");
    return;
  }
  if (!goal) {
    return;
  }

  geometry_msgs::msg::TransformStamped base_to_map;
  try {
    base_to_map = tf_buffer_->lookupTransform(map_frame_, base_frame_, rclcpp::Time(0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "No transform %s -> %s: %s",
      map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return;
  }
  const auto & translation = base_to_map.transform.translation;
  const octomap::point3d start(translation.x, translation.y, translation.z);

  octomap::point3d goal_point;
  const std::string goal_frame =
    goal->header.frame_id.empty() ? map_frame_ : goal->header.frame_id;
  if (goal_frame == map_frame_) {
    goal_point = octomap::point3d(
      goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
  } else {
    geometry_msgs::msg::TransformStamped goal_to_map;
    try {
      goal_to_map = tf_buffer_->lookupTransform(map_frame_, goal_frame, rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform goal from %s to %s: %s",
        goal_frame.c_str(), map_frame_.c_str(), ex.what());
      return;
    }
    tf2::Transform transform;
    tf2::fromMsg(goal_to_map.transform, transform);
    const tf2::Vector3 point = transform * tf2::Vector3(
      goal->pose.position.x, goal->pose.position.y, goal->pose.position.z);
    goal_point = octomap::point3d(point.x(), point.y(), point.z());
  }

  PlanStatus status = PlanStatus::kSuccess;
  const std::vector<octomap::point3d> path =
    planner_->plan(*tree, start, goal_point, bounds_, &status);
  const rclcpp::Time stamp = now();
  publishPath(path, stamp);

  if (path.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No path [%s] from (%.2f, %.2f, %.2f) to goal (%.2f, %.2f, %.2f)",
      toString(status), start.x(), start.y(), start.z(),
      goal_point.x(), goal_point.y(), goal_point.z());
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Path [%s] with %zu waypoints to goal (%.2f, %.2f, %.2f)",
      toString(status), path.size(), goal_point.x(), goal_point.y(), goal_point.z());
  }
}

void ExplorationPlannerNode::publishPath(
  const std::vector<octomap::point3d> & path, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = map_frame_;
  path_msg.header.stamp = stamp;

  visualization_msgs::msg::Marker marker;
  marker.header = path_msg.header;
  marker.ns = "exploration_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.08;
  marker.color.r = 0.1f;
  marker.color.g = 1.0f;
  marker.color.b = 0.1f;
  marker.color.a = 1.0f;

  if (path.empty()) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
      marker.points.push_back(pose.pose.position);
    }
  }

  path_pub_->publish(path_msg);
  marker_pub_->publish(marker);
}

}  // namespace exploration_planner_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<exploration_planner_3d::ExplorationPlannerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("exploration_planner_3d"), "Failed to start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
