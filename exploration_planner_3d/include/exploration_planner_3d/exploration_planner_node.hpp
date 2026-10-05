#ifndef EXPLORATION_PLANNER_3D__EXPLORATION_PLANNER_NODE_HPP_
#define EXPLORATION_PLANNER_3D__EXPLORATION_PLANNER_NODE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "exploration_planner_3d/astar_3d.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace exploration_planner_3d
{

/// Global planner node.
///
/// Subscribes to the occupancy map published by the frontier detector and to
/// the exploration goal, looks up the vehicle pose, and runs a 3D A* over the
/// map, publishing the resulting route as `nav_msgs/Path` plus a RViz line
/// marker.
class ExplorationPlannerNode : public rclcpp::Node
{
public:
  ExplorationPlannerNode();

private:
  /// Stores the latest occupancy map.
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  /// Stores the latest exploration goal.
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// Plans from the current vehicle pose to the stored goal.
  void planCycle();

  /// Publishes the route as a `nav_msgs/Path` and a line-strip marker.
  void publishPath(const std::vector<octomap::point3d> & path, const rclcpp::Time & stamp);

  /// Publishes the footprint-inflated clearance envelope as a marker.
  void publishFootprint(const rclcpp::Time & stamp, const octomap::OcTree & tree);

  std::string map_frame_;
  std::string base_frame_;

  std::unique_ptr<AStar3D> planner_;
  Bounds3D bounds_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool publish_footprint_ {true};

  std::mutex mutex_;
  std::shared_ptr<octomap::OcTree> tree_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
};

}  // namespace exploration_planner_3d

#endif  // EXPLORATION_PLANNER_3D__EXPLORATION_PLANNER_NODE_HPP_
