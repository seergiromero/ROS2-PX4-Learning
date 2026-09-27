#ifndef FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_NODE_HPP_
#define FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_NODE_HPP_

#include "frontier_detector_3d/frontier_detector.hpp"

#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace frontier_detector_3d
{

/// ROS wrapper that feeds deserialized OctoMaps into `FrontierDetector`.
///
/// The node is deliberately read-only with respect to the vehicle: it never
/// commands motion. It consumes one message source (`/octomap_binary` by
/// default) and produces one visualization topic (`/exploration/frontiers`).
/// All frontier detection and clustering logic lives in `FrontierDetector`,
/// which is kept independent from ROS so it can be unit-tested in isolation.
///
/// Parameters
/// ==========
/// - `octomap_topic` (string, default `/octomap_binary`): input map topic.
/// - `frontier_topic` (string, default `/exploration/frontiers`): output
///   marker topic.
/// - `min_frontier_size` (int, default 10): minimum cluster size in voxels.
/// - `max_frontiers` (int, default 100): maximum number of clusters to publish.
/// - `connectivity` (int, default 6): neighbourhood topology used for
///   clustering, one of 6, 18 or 26.
/// - `max_dist_to_occupied` (double, default 0.5): maximum distance from a
///   frontier voxel to the nearest occupied voxel for the voxel to be kept.
///   A non-positive value disables the filter and reports every free voxel
///   bordering unknown space, including sensor artifacts in empty air.
/// - `cluster_size_xy` (double, default 3.0): maximum horizontal extent of a
///   cluster before it is split along its first principal component.
/// - `cluster_size_z` (double, default 2.0): maximum vertical extent of a
///   cluster before it is split.
/// - `ground_z` (double, default 0.4): frontier voxels below this height are
///   discarded.
class FrontierDetectorNode : public rclcpp::Node
{
public:
  /// Creates the node, declares ROS parameters, and sets up subscriptions and
  /// publishers. Does not perform any map processing until the first octomap
  /// message arrives.
  FrontierDetectorNode();

private:
  /// Deserializes the incoming OctoMap and runs the full detection pipeline.
  ///
  /// \param[in] msg The octomap message, typically published by octomap_server.
  ///            Can be binary or full probability format; both are handled
  ///            through octomap_msgs::msgToMap.
  ///
  /// \note Non-OcTree map types (e.g. ColorOcTree) are rejected. Because this
  ///       message can arrive at a high rate, deserialization is performed on
  ///       a fresh tree each callback rather than mutating a shared instance.
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  /// Publishes each frontier cluster as a single CUBE_LIST marker in a
  /// MarkerArray, so RViz can color each cluster differently.
  ///
  /// A DELETEALL marker is always emitted first so that stale markers from a
  /// previous map update never persist on screen.
  ///
  /// \param[in] frame_id The TF frame the markers are expressed in (usually "map").
  /// \param[in] frontiers The cluster list produced by FrontierDetector.
  /// \param[in] resolution The octree leaf size in metres, used as the marker
  ///            cube scale so voxels are drawn at their true size.
  void publishMarkers(
    const std::string & frame_id,
    const std::vector<Frontier> & frontiers,
    double resolution);

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;

  std::string octomap_topic_;
  std::string frontier_topic_;
  int min_frontier_size_;
  int max_frontiers_;
  Connectivity connectivity_;
  double max_dist_to_occupied_;
  double cluster_size_xy_;
  double cluster_size_z_;
  double ground_z_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_NODE_HPP_
