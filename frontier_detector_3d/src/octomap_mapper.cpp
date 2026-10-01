#include "frontier_detector_3d/octomap_mapper.hpp"

#include <octomap/Pointcloud.h>

#include <stdexcept>
#include <utility>

namespace frontier_detector_3d
{

OctomapMapper::OctomapMapper(
  double resolution, double max_range, double prob_hit, double prob_miss,
  double prob_min, double prob_max, bool compress, std::size_t point_subsample)
: max_range_(max_range),
  compress_(compress),
  point_subsample_(std::max<std::size_t>(1u, point_subsample))
{
  if (resolution <= 0.0) {
    throw std::invalid_argument("octomap resolution must be positive");
  }

  tree_ = std::make_unique<octomap::OcTree>(resolution);
  tree_->setProbHit(prob_hit);
  tree_->setProbMiss(prob_miss);
  tree_->setClampingThresMin(prob_min);
  tree_->setClampingThresMax(prob_max);
  // Record every updated leaf key so the frontier detector can run
  // incrementally on the cells that actually changed, as the reference
  // `OctomapServer::trackChanges()` does.
  tree_->enableChangeDetection(true);
}

bool OctomapMapper::update(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const SensorTransform & sensor_to_map)
{
  const octomap::point3d sensor_origin(
    sensor_to_map.translation.x(),
    sensor_to_map.translation.y(),
    sensor_to_map.translation.z());

  // Accumulate every point of the cloud into a flat octomap point cloud so we
  // can loop over it once with a cheap index. We deliberately do NOT convert
  // via pcl::fromROSMsg: that allocates a second cloud plus per-point pcl
  // structs for no benefit here.
  octomap::Pointcloud points;
  points.reserve(cloud.width * cloud.height);

  const float * data = reinterpret_cast<const float *>(cloud.data.data());
  const std::size_t point_step = cloud.point_step;
  const bool has_x = cloud.fields.size() > 0 && cloud.fields[0].name == "x";
  const bool has_y = cloud.fields.size() > 1 && cloud.fields[1].name == "y";
  const bool has_z = cloud.fields.size() > 2 && cloud.fields[2].name == "z";
  const std::size_t x_off =
    has_x ? static_cast<std::size_t>(cloud.fields[0].offset) : 0u;
  const std::size_t y_off =
    has_y ? static_cast<std::size_t>(cloud.fields[1].offset) : 0u;
  const std::size_t z_off =
    has_z ? static_cast<std::size_t>(cloud.fields[2].offset) : 0u;

  for (std::size_t i = 0; i < cloud.width * cloud.height; i += point_subsample_) {
    const float * p = data + (i * point_step) / sizeof(float);
    if (!has_x || !has_y || !has_z) {
      continue;
    }
    const Eigen::Vector3d sensor_pt(p[x_off / sizeof(float)], p[y_off / sizeof(float)],
      p[z_off / sizeof(float)]);
    if (!sensor_pt.allFinite()) {
      continue;  // skip NaNs (gazebo fills the far plane with NaN points)
    }
    const Eigen::Vector3d map_pt = sensor_to_map.apply(sensor_pt);
    points.push_back(map_pt.x(), map_pt.y(), map_pt.z());
  }

  if (points.size() == 0) {
    return false;
  }

  // Ray-cast from the sensor origin to every point. Cells crossed by a ray
  // become free, the endpoint becomes occupied. Points beyond max_range are
  // clamped so they only clear space up to the sensor's reach. A clamped
  // endpoint is NOT an obstacle: it means the sensor returned no hit within
  // range, and marking it occupied creates an artificial frontier shell at
  // max_range.
  for (std::size_t i = 0; i < points.size(); ++i) {
    octomap::point3d point = points[i];
    bool endpoint_occupied = true;
    if (max_range_ > 0.0 && (point - sensor_origin).norm() > max_range_) {
      endpoint_occupied = false;
      const octomap::point3d dir = (point - sensor_origin).normalized();
      point = sensor_origin + dir * max_range_;
    }

    octomap::KeyRay ray;
    if (tree_->computeRayKeys(sensor_origin, point, ray)) {
      for (const auto & key : ray) {
        tree_->updateNode(key, false);  // free along the beam
      }
    }

    octomap::OcTreeKey end_key;
    if (tree_->coordToKeyChecked(point, end_key)) {
      tree_->updateNode(end_key, endpoint_occupied);
    }
  }

  if (compress_) {
    tree_->prune();
  }

  // Snapshot the changed leaves for this update, then start a new change set.
  // OctoMap's change detection is a leaf-key set, exactly what the incremental
  // frontier test consumes.
  changed_keys_.clear();
  for (auto it = tree_->changedKeysBegin(), end = tree_->changedKeysEnd(); it != end; ++it) {
    changed_keys_.push_back(it->first);
  }
  tree_->resetChangeDetection();

  return true;
}

}  // namespace frontier_detector_3d
