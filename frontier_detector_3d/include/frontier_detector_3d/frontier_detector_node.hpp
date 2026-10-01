#ifndef FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_NODE_HPP_
#define FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_NODE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "frontier_detector_3d/best_frontier.hpp"
#include "frontier_detector_3d/frontier_detector.hpp"
#include "frontier_detector_3d/frontier_pipeline.hpp"
#include "frontier_detector_3d/frontier_visualizer.hpp"
#include "frontier_detector_3d/octomap_mapper.hpp"
#include "frontier_detector_3d/sensor_transform.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace frontier_detector_3d
{

/// ROS 2 node that owns the map and publishes frontiers.
///
/// This node replaces the old two-node design (octomap_server + a subscriber
/// node that deserialized the map). It:
///
/// 1. subscribes to the raw LiDAR cloud (`/lidar_3d/points` by default),
/// 2. feeds the cloud + the LATEST available sensor->map transform to
///    `FrontierPipeline` (the sim mixes clocks between gazebo stamps and the
///    TF tree, so matching exact message timestamps drops everything),
/// 3. publishes the (optional) binary OctoMap and the frontier markers.
///
/// A low-rate timer (`process_rate_hz`) drains the *latest* cloud only, so the
/// heavy map update + detection work never runs more often than configured and
/// the subscription callback stays non-blocking.
class FrontierDetectorNode : public rclcpp::Node
{
public:
  FrontierDetectorNode();

private:
  /// Callback of the cloud subscription: stores the latest cloud for the timer.
  ///
  /// \param[in] cloud Newly arrived cloud.
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  /// Periodic cycle: look up the latest TF, run the pipeline and publish.
  void processCycle();

  /// Publishes the current octree as an `octomap_msgs/Octomap` binary message.
  ///
  /// \param[in] stamp Timestamp attached to the message.
  void publishOctomap(const rclcpp::Time & stamp);

  /// Selects the best frontier among the clusters and publishes it as a goal
  /// (`geometry_msgs/PoseStamped`) plus a marker.
  ///
  /// \param[in] frontiers Clusters detected this cycle.
  /// \param[in] stamp Timestamp attached to the goal and marker.
  /// \param[in] current_position Vehicle position in the map frame.
  void publishBestFrontier(
    const std::vector<Frontier> & frontiers, const rclcpp::Time & stamp,
    const octomap::point3d & current_position);

  /// Publishes the cylindrical pose-filter exclusion zone in RViz.
  void publishPoseFilterMarker(
    const rclcpp::Time & stamp, const octomap::point3d & current_position);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  std::unique_ptr<OctomapMapper> mapper_;
  std::unique_ptr<FrontierPipeline> pipeline_;
  std::unique_ptr<FrontierVisualizer> visualizer_;
  std::unique_ptr<BestFrontier> best_frontier_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_filter_marker_pub_;

  std::mutex latest_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;

  std::string map_frame_;
  std::string base_frame_;
  bool publish_map_;
  bool publish_best_frontier_;
  bool use_latest_transform_;
  double min_frontier_radius_ {0.0};
  double max_frontier_height_ {-1.0};
  double frontier_cell_size_ {0.0};
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_NODE_HPP_
