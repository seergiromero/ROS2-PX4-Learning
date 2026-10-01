#ifndef FRONTIER_DETECTOR_3D__OCTOMAP_MAPPER_HPP_
#define FRONTIER_DETECTOR_3D__OCTOMAP_MAPPER_HPP_

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "frontier_detector_3d/sensor_transform.hpp"

#include <memory>
#include <vector>

namespace frontier_detector_3d
{

/// Builds and owns a single `octomap::OcTree`, updated incrementally from
/// `PointCloud2` messages.
///
/// The mapper is the single owner of the occupancy tree. Each `update()`
/// call ray-casts the incoming cloud (already transformed to the map frame by
/// the caller) from the sensor origin, marking the cells along each beam as
/// free and the endpoint as occupied, following the standard OctoMap
/// probabilistic update loop.
///
/// Keeping the tree here (instead of round-tripping it through the
/// `octomap_msgs/Octomap` wire format) removes the serialization + DDS +
/// deserialization cost that the old two-node design paid on every message:
/// the frontier detector then runs directly on the in-memory tree produced by
/// this class.
class OctomapMapper
{
public:
  /// Creates the mapper and its octree.
  ///
  /// \param[in] resolution Leaf size of the octree in metres.
  /// \param[in] max_range Maximum ray length in metres. Points further away
  ///            than this from the sensor origin are clamped on the ray, so
  ///            they only clear free space and never create occupied cells.
  ///            A non-positive value disables the range clamp.
  /// \param[in] prob_hit Log-odds update weight for occupied endpoints.
  /// \param[in] prob_miss Log-odds update weight for free cells along a ray.
  /// \param[in] prob_min Clamping threshold for the minimum occupancy.
  /// \param[in] prob_max Clamping threshold for the maximum occupancy.
  /// \param[in] compress When true, `octree_->prune()` is called after every
  ///            update, collapsing homogeneous volumes into coarse leaves.
  /// \param[in] point_subsample Process only every Nth point of each cloud
  ///            (1 = all points). Subsampling cuts ray-casting cost roughly by
  ///            N^2 in a dense grid scan at the price of a sparser map.
  /// \throws std::invalid_argument if the resolution is not positive.
  explicit OctomapMapper(
    double resolution, double max_range = -1.0, double prob_hit = 0.7,
    double prob_miss = 0.4, double prob_min = 0.12, double prob_max = 0.97,
    bool compress = false, std::size_t point_subsample = 1);

  /// Ray-casts one point cloud into the octree.
  ///
  /// \param[in] cloud The cloud to insert. Its points must already be in the
  ///            map frame (the caller is responsible for the transform).
  /// \param[in] sensor_to_map Transform from the sensor frame (in which the
  ///            cloud was captured) to the map frame. Its translation is used
  ///            as the ray-casting origin.
  /// \return True when at least one point was inserted.
  bool update(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const SensorTransform & sensor_to_map);

  /// Returns the latest occupancy tree.
  ///
  /// The tree is owned by this mapper and stays valid for the lifetime of the
  /// mapper. It is mutated by `update()`; call it only from the thread that
  /// owns the mapper.
  const octomap::OcTree & tree() const {return *tree_;}

  /// Non-const accessor, useful to seed the map (e.g. in tests) before the
  /// first cloud arrives.
  octomap::OcTree & mutableTree() {return *tree_;}

  /// Returns the number of nodes currently stored in the octree.
  std::size_t nodeCount() const {return tree_->size();}

  /// Returns the leaf keys touched by the last `update()`.
  ///
  /// Mirrors `OctomapServer::trackChanges()` of the reference implementation:
  /// the set is captured from OctoMap change detection and cleared at the end
  /// of every `update()`, so it always describes exactly the cells modified by
  /// the most recent cloud.
  const std::vector<octomap::OcTreeKey> & changedKeys() const {return changed_keys_;}

  /// Clears the map to its initial empty state.
  void clear()
  {
    tree_->clear();
    changed_keys_.clear();
  }

private:
  std::unique_ptr<octomap::OcTree> tree_;
  double max_range_;
  bool compress_;
  std::size_t point_subsample_;
  std::vector<octomap::OcTreeKey> changed_keys_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__OCTOMAP_MAPPER_HPP_
