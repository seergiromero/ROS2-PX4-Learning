#ifndef FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_HPP_
#define FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_HPP_

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>

#include <cstddef>
#include <set>
#include <vector>

namespace frontier_detector_3d
{

/// Strict weak ordering over octree keys.
///
/// `octomap::OcTreeKey` does not provide an ordering, which prevents it from
/// being used directly in ordered associative containers. This comparator
/// orders keys lexicographically on (x, y, z) so that the same voxel always has
/// a single canonical representation, regardless of insertion order.
struct KeyCompare
{
  /// Compares two octree keys lexicographically on (x, y, z).
  bool operator()(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs) const;
};

/// A connected region of frontier cells in world coordinates.
///
/// `points` are the coarse parent cells (at `exploration_depth`) that the
/// mean-shift clustering assigned to this region. `representative` is the
/// parent cell nearest to the cluster mode, i.e. the value a planner would use
/// as the exploration goal.
struct Frontier
{
  /// World-frame centers of the parent cells that make up the cluster.
  std::vector<octomap::point3d> points;

  /// Parent cell nearest to the cluster mode (world frame).
  octomap::point3d representative;

  /// Number of cells in the cluster.
  std::size_t size() const noexcept {return points.size();}
};

/// Axis-aligned exploration box expressed in the map frame.
///
/// Everything outside this box is ignored, so frontiers are never published in
/// space the vehicle is not expected to visit. A disabled box keeps all
/// frontiers.
struct Bounds3D
{
  bool enabled = false;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;

  /// Returns true when the point lies inside the (inclusive) box.
  bool contains(double x, double y, double z) const
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y &&
           z >= min_z && z <= max_z;
  }
};

/// Multi-resolution frontier detection and clustering over an `octomap::OcTree`.
///
/// This is a direct reimplementation of the algorithm in Batinovic et al.,
/// "A Multi-Resolution Frontier-Based Planner for Autonomous 3D Exploration"
/// (IEEE RA-L 2021) and the `larics/uav_frontier_exploration_3d` reference
/// implementation. The configuration surface is intentionally reduced to the
/// parameters the paper actually uses.
///
/// Frontier definition
/// ===================
/// A frontier cell is a free voxel that, within its 26-neighbourhood:
///   - borders at least one unknown cell (`OcTree::search()` returns nullptr),
///   - borders at least one free cell, and
///   - borders no occupied cell.
/// This is exactly `findFrontier` + `updateGlobalFrontier` from the reference
/// implementation. The joint requirement rejects free voxels floating in mid
/// air and walls that only touch thin unknown slivers, without any extra
/// distance/threshold filters.
///
/// Incremental update
/// ==================
/// Detection is driven by the octree change set produced by the map update
/// (`OctomapMapper::changedKeys()`), mirroring `OctomapServer::trackChanges()`.
/// The detector keeps the global set of frontier cells between calls: newly
/// changed free cells are tested and added, and every stored cell is re-tested
/// so cells that gained an occupied neighbour (or lost their unknown frontier)
/// are dropped. Calling `detect(tree)` without a change set performs a full
/// scan instead and rebuilds the global set from scratch.
///
/// Multi-resolution
/// ================
/// Each frontier cell is projected onto its parent node at `exploration_depth`
/// (the `exploration.depth` parameter of the paper; octomap depth 16 is the
/// finest leaf level). Detection and clustering therefore happen on cells of
/// size `resolution * 2^(tree_depth - exploration_depth)`, which is the main
/// cost lever and the source of the planner's name.
///
/// Clustering
/// ==========
/// The parent cells are grouped with the vendored Gaussian-kernel mean-shift
/// (`mean_shift.hpp`, the `clustering.kernel_bandwidth` parameter). Each
/// cluster's representative is the parent cell nearest to the converged mode,
/// exactly as in the reference `MSCluster`.
class FrontierDetector
{
public:
  using KeySet = std::set<octomap::OcTreeKey, KeyCompare>;

  /// Creates the detector.
  ///
  /// \param[in] exploration_depth Octree level at which parent frontier cells
  ///            are searched and clustered. 16 is the finest leaf level; the
  ///            coarser the level, the larger the detection cells. Values are
  ///            clamped to `[0, tree_depth]`.
  /// \param[in] kernel_bandwidth Mean-shift bandwidth in metres. Must be
  ///            positive.
  /// \param[in] bounds Optional axis-aligned exploration box. Frontier cells
  ///            outside it are discarded.
  /// \param[in] min_frontier_size Minimum number of parent cells a cluster must
  ///            contain to be kept. `1` keeps every mode (paper behaviour);
  ///            larger values discard small isolated clusters (noise).
  /// \throws std::invalid_argument if `kernel_bandwidth` is not positive or
  ///         `min_frontier_size` is less than 1.
  explicit FrontierDetector(
    unsigned int exploration_depth = 16, double kernel_bandwidth = 1.0,
    Bounds3D bounds = Bounds3D(), int min_frontier_size = 1);

  /// Runs one incremental detection + clustering cycle.
  ///
  /// \param[in] tree The occupancy tree.
  /// \param[in] changed_cells Keys updated by the last map update (empty is
  ///            allowed and simply re-evaluates the stored frontier set).
  /// \return Clusters, one per mean-shift mode.
  std::vector<Frontier> detect(
    const octomap::OcTree & tree,
    const std::vector<octomap::OcTreeKey> & changed_cells);

  /// Runs a full detection + clustering cycle, rebuilding the stored frontier
  /// set from every free leaf. Intended for tests and for the first cycle when
  /// no change set is available.
  ///
  /// \param[in] tree The occupancy tree.
  /// \return Clusters, one per mean-shift mode.
  std::vector<Frontier> detect(const octomap::OcTree & tree);

  /// Updates the stored frontier set from `changed_cells` and returns it.
  ///
  /// \param[in] tree The occupancy tree.
  /// \param[in] changed_cells Keys updated by the last map update.
  /// \return The current global frontier cell set (full resolution).
  KeySet findFrontierKeys(
    const octomap::OcTree & tree,
    const std::vector<octomap::OcTreeKey> & changed_cells);

  /// Rebuilds the stored frontier set from every free leaf (full scan).
  ///
  /// \param[in] tree The occupancy tree.
  /// \return The current global frontier cell set (full resolution).
  KeySet rebuildFrontierKeys(const octomap::OcTree & tree);

  /// Returns the stored global frontier cells.
  const KeySet & globalFrontierKeys() const {return global_frontier_cells_;}

  /// Projects the stored frontier cells onto their parent nodes at
  /// `exploration_depth`.
  ///
  /// \param[in] tree The occupancy tree.
  /// \return Set of parent cell keys at the detection level.
  KeySet parentKeys(const octomap::OcTree & tree) const;

  /// Mean-shift clusters the given parent cells.
  ///
  /// \param[in] tree The occupancy tree (used to convert keys to coordinates).
  /// \param[in] parent_keys Parent cells at `exploration_depth`.
  /// \return Clusters, one per mean-shift mode.
  std::vector<Frontier> clusterParentKeys(
    const octomap::OcTree & tree, const KeySet & parent_keys) const;

  /// Returns the 26 neighbour keys of `key` inside the octree address space.
  ///
  /// Keys that would fall outside `[0, 65535]` are skipped instead of wrapping.
  ///
  /// \param[in] key The center key.
  /// \return The valid neighbour keys (excluding the center itself).
  std::vector<octomap::OcTreeKey> neighbors(const octomap::OcTreeKey & key) const;

  /// Tests whether a full-resolution key is a frontier cell.
  ///
  /// Free cell with at least one unknown neighbour, at least one free
  /// neighbour and no occupied neighbour in the 26-neighbourhood.
  ///
  /// \param[in] tree The occupancy tree.
  /// \param[in] key The full-resolution key to test.
  /// \return True when the cell is a frontier cell.
  static bool isFrontierCell(const octomap::OcTree & tree, const octomap::OcTreeKey & key);

  /// Checks whether a full-resolution key has no node stored in the tree.
  ///
  /// \param[in] tree The occupancy tree to query.
  /// \param[in] key The full-resolution key to test.
  /// \return True if the key is absent from the tree (unknown space).
  static bool isUnknownNeighbor(const octomap::OcTree & tree, const octomap::OcTreeKey & key);

  /// Returns the configured exploration depth.
  unsigned int explorationDepth() const {return exploration_depth_;}

private:
  unsigned int exploration_depth_;
  double kernel_bandwidth_;
  Bounds3D bounds_;
  int min_frontier_size_;
  KeySet global_frontier_cells_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_HPP_
