#ifndef FRONTIER_DETECTOR_3D__FRONTIER_PIPELINE_HPP_
#define FRONTIER_DETECTOR_3D__FRONTIER_PIPELINE_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "frontier_detector_3d/frontier_detector.hpp"
#include "frontier_detector_3d/octomap_mapper.hpp"
#include "frontier_detector_3d/sensor_transform.hpp"

#include <rclcpp/time.hpp>

#include <vector>

namespace frontier_detector_3d
{

/// Coherent output of one pipeline cycle.
///
/// `frontiers` were detected from exactly the tree produced by the `update()`
/// of the same cycle, so the map and the frontiers can never drift apart.
struct PipelineResult
{
  /// Mean-shift frontier clusters detected on the freshly updated tree.
  std::vector<Frontier> frontiers;

  /// TF frame the tree and the frontier coordinates are expressed in.
  std::string frame_id;

  /// Timestamp of the input cloud that produced this result.
  rclcpp::Time stamp;
};

/// Chains `OctomapMapper` and `FrontierDetector`.
///
/// `process()` first inserts the incoming cloud into the shared octree and
/// then runs the detector on that same tree. This is the "map and frontiers
/// updated together" behaviour requested: there is exactly one octree, it is
/// owned by the mapper, and detection always runs on the freshest version of
/// it without any serialization round-trip.
class FrontierPipeline
{
public:
  /// Creates the pipeline.
  ///
  /// \param[in] mapper Mapper that owns the octree. The pipeline keeps a
  ///            reference; the mapper must outlive the pipeline.
  /// \param[in] detector Detector used to find frontiers. Kept by value so the
  ///            pipeline is self-contained.
  FrontierPipeline(OctomapMapper & mapper, FrontierDetector detector);

  /// Inserts the cloud into the map and detects frontiers on the same tree.
  ///
  /// \param[in] cloud Cloud to insert, expressed in the sensor frame.
  /// \param[in] sensor_to_map Transform of the sensor frame into the map
  ///            frame. Its translation becomes the ray-casting origin.
  /// \param[in] frame_id Map frame used for the output metadata.
  /// \param[in] stamp Timestamp propagated to the output metadata.
  /// \param[in] current_position Optional vehicle position (map frame) for the
  ///            detector's cylindrical pose filter. Pass nullptr to disable.
  /// \return Frontiers detected on the updated tree plus context metadata.
  PipelineResult process(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const SensorTransform & sensor_to_map,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const octomap::point3d * current_position = nullptr);

  /// Converts a `geometry_msgs/TransformStamped` into the minimal
  /// `SensorTransform` used by the pure algorithm code.
  ///
  /// \param[in] transform The TF transform (sensor frame -> map frame).
  /// \return The equivalent rotation + translation.
  static SensorTransform toSensorTransform(
    const geometry_msgs::msg::TransformStamped & transform);

private:
  OctomapMapper & mapper_;
  FrontierDetector detector_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__FRONTIER_PIPELINE_HPP_
