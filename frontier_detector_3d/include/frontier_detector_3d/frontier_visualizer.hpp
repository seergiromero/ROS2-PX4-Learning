#ifndef FRONTIER_DETECTOR_3D__FRONTIER_VISUALIZER_HPP_
#define FRONTIER_DETECTOR_3D__FRONTIER_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_detector_3d/frontier_detector.hpp"

#include <string>
#include <vector>

namespace frontier_detector_3d
{

/// Publishes frontier clusters as RViz markers.
///
/// Keeps every RViz concern out of the detector/pipeline classes. Two output
/// modes are supported:
///
/// - `publish_centroids` (default): one CUBE per cluster placed at its
///   centroid. Cheap for RViz and enough to steer a vehicle.
/// - `publish_voxels`: every frontier voxel of every cluster as a CUBE_LIST.
///   Debug-only; can be very heavy for large maps.
class FrontierVisualizer
{
public:
  /// Creates the visualizer.
  ///
  /// \param[in] node ROS node used to create the publisher and read the clock.
  /// \param[in] topic Marker topic to publish on.
  /// \param[in] publish_voxels Emit per-voxel CUBE_LIST markers.
  /// \param[in] publish_centroids Emit per-cluster centroid markers.
  FrontierVisualizer(
    rclcpp::Node & node, const std::string & topic, bool publish_voxels,
    bool publish_centroids);

  /// Publishes the frontiers for one pipeline cycle.
  ///
  /// \param[in] frontiers Clusters to visualize.
  /// \param[in] frame_id TF frame of the coordinates.
  /// \param[in] stamp Timestamp for the markers.
  /// \param[in] resolution Octree leaf size, used as the marker cube scale.
  void publish(
    const std::vector<Frontier> & frontiers, const std::string & frame_id,
    const rclcpp::Time & stamp, double resolution);

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  bool publish_voxels_;
  bool publish_centroids_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__FRONTIER_VISUALIZER_HPP_
