#ifndef FRONTIER_DETECTOR_3D__SENSOR_TRANSFORM_HPP_
#define FRONTIER_DETECTOR_3D__SENSOR_TRANSFORM_HPP_

#include <Eigen/Geometry>

#include <cstddef>

namespace frontier_detector_3d
{

/// Rigid transform (rotation + translation) that maps sensor coordinates into
/// the map frame.
///
/// This is the minimal amount of information the mapper needs to insert a
/// point cloud into the OctoMap. Keeping it as a plain struct (instead of
/// depending on `geometry_msgs` or `tf2`) lets the pure algorithm code be
/// unit-tested without a running TF tree.
struct SensorTransform
{
  /// Rotation matrix from the sensor frame to the map frame.
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  /// Translation of the sensor origin expressed in the map frame.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  /// Transforms a point expressed in the sensor frame into the map frame.
  Eigen::Vector3d apply(const Eigen::Vector3d & point) const
  {
    return rotation * point + translation;
  }
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__SENSOR_TRANSFORM_HPP_
