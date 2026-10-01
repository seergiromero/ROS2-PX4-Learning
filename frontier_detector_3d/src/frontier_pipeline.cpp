#include "frontier_detector_3d/frontier_pipeline.hpp"

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <utility>

namespace frontier_detector_3d
{

FrontierPipeline::FrontierPipeline(OctomapMapper & mapper, FrontierDetector detector)
: mapper_(mapper),
  detector_(std::move(detector))
{
}

PipelineResult FrontierPipeline::process(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const SensorTransform & sensor_to_map,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const octomap::point3d * current_position)
{
  // 1) Grow the map from the fresh cloud.
  mapper_.update(cloud, sensor_to_map);

  // 2) Detect frontiers on the SAME tree that was just updated, restricting the
  //    frontier test to the cells this update changed (incremental detection).
  PipelineResult result;
  result.frontiers = detector_.detect(
    mapper_.tree(), mapper_.changedKeys(), current_position);
  result.frame_id = frame_id;
  result.stamp = stamp;
  return result;
}

SensorTransform FrontierPipeline::toSensorTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform tf;
  tf2::fromMsg(transform.transform, tf);

  SensorTransform result;
  // getBasis() returns a tf2::Matrix3x3; convert it to Eigen row-major, then
  // normalise so it is a proper rotation (tf2 already enforces unit norm).
  const auto & basis = tf.getBasis();
  result.rotation <<
    basis[0][0], basis[0][1], basis[0][2],
    basis[1][0], basis[1][1], basis[1][2],
    basis[2][0], basis[2][1], basis[2][2];
  result.translation = Eigen::Vector3d(
    tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z());
  return result;
}

}  // namespace frontier_detector_3d
