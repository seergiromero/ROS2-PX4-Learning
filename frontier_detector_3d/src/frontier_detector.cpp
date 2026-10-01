#include "frontier_detector_3d/frontier_detector.hpp"

#include "frontier_detector_3d/mean_shift.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace frontier_detector_3d
{

namespace
{

constexpr int kMaxKey = std::numeric_limits<unsigned short>::max();

/// Returns the 26 neighbours of a full-resolution key, skipping any that would
/// fall outside the octree address space instead of wrapping around.
std::vector<octomap::OcTreeKey> neighborKeys(const octomap::OcTreeKey & key)
{
  std::vector<octomap::OcTreeKey> result;
  result.reserve(26);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const int nx = static_cast<int>(key.k[0]) + dx;
        const int ny = static_cast<int>(key.k[1]) + dy;
        const int nz = static_cast<int>(key.k[2]) + dz;
        if (nx < 0 || ny < 0 || nz < 0 || nx > kMaxKey || ny > kMaxKey || nz > kMaxKey) {
          continue;
        }
        result.emplace_back(
          static_cast<unsigned short>(nx), static_cast<unsigned short>(ny),
          static_cast<unsigned short>(nz));
      }
    }
  }
  return result;
}

/// True when the cell center of `key` lies inside the configured box.
bool inBounds(
  const octomap::OcTree & tree, const octomap::OcTreeKey & key, const Bounds3D & bounds)
{
  if (!bounds.enabled) {
    return true;
  }
  const octomap::point3d coord = tree.keyToCoord(key);
  return bounds.contains(coord.x(), coord.y(), coord.z());
}

}  // namespace

bool KeyCompare::operator()(
  const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs) const
{
  if (lhs.k[0] != rhs.k[0]) {
    return lhs.k[0] < rhs.k[0];
  }
  if (lhs.k[1] != rhs.k[1]) {
    return lhs.k[1] < rhs.k[1];
  }
  return lhs.k[2] < rhs.k[2];
}

FrontierDetector::FrontierDetector(
  unsigned int exploration_depth, double kernel_bandwidth, Bounds3D bounds,
  int min_frontier_size, double min_frontier_radius, double max_frontier_height)
: exploration_depth_(exploration_depth),
  kernel_bandwidth_(kernel_bandwidth),
  bounds_(bounds),
  min_frontier_size_(min_frontier_size),
  min_frontier_radius_(min_frontier_radius),
  max_frontier_height_(max_frontier_height)
{
  if (kernel_bandwidth_ <= 0.0) {
    throw std::invalid_argument("kernel_bandwidth must be positive");
  }
  if (min_frontier_size_ < 1) {
    throw std::invalid_argument("min_frontier_size must be >= 1");
  }
}

std::vector<octomap::OcTreeKey> FrontierDetector::neighbors(
  const octomap::OcTreeKey & key) const
{
  return neighborKeys(key);
}

bool FrontierDetector::isUnknownNeighbor(
  const octomap::OcTree & tree, const octomap::OcTreeKey & key)
{
  return tree.search(key) == nullptr;
}

bool FrontierDetector::isFrontierCell(
  const octomap::OcTree & tree, const octomap::OcTreeKey & key)
{
  const octomap::OcTreeNode * node = tree.search(key);
  if (node == nullptr || tree.isNodeOccupied(node)) {
    return false;
  }

  bool has_unknown = false;
  bool has_free = false;
  bool has_occupied = false;
  for (const auto & neighbor : neighborKeys(key)) {
    const octomap::OcTreeNode * n = tree.search(neighbor);
    if (n == nullptr) {
      has_unknown = true;
    } else if (tree.isNodeOccupied(n)) {
      has_occupied = true;
    } else {
      has_free = true;
    }
  }

  return has_unknown && has_free && !has_occupied;
}

std::vector<Frontier> FrontierDetector::detect(
  const octomap::OcTree & tree,
  const std::vector<octomap::OcTreeKey> & changed_cells,
  const octomap::point3d * current_position)
{
  findFrontierKeys(tree, changed_cells);
  const KeySet frontier_keys = global_frontier_cells_;
  KeySet parents = parentKeys(tree, frontier_keys);
  if (current_position != nullptr) {
    unsigned int depth = exploration_depth_;
    if (depth > tree.getTreeDepth()) {
      depth = tree.getTreeDepth();
    }
    // Filter the parent cells that will actually be published. The persistent
    // full-resolution frontier set remains intact so cells reappear after the
    // vehicle moves away from them.
    parents = filterKeysByPose(tree, parents, depth, *current_position);
  }
  return clusterParentKeys(tree, parents);
}

std::vector<Frontier> FrontierDetector::detect(const octomap::OcTree & tree)
{
  rebuildFrontierKeys(tree);
  const KeySet parents = parentKeys(tree);
  return clusterParentKeys(tree, parents);
}

FrontierDetector::KeySet FrontierDetector::filterKeysByPose(
  const octomap::OcTree & tree, const KeySet & keys, unsigned int depth,
  const octomap::point3d & current_position) const
{
  const bool radius_enabled = min_frontier_radius_ > 0.0;
  const bool height_enabled = max_frontier_height_ >= 0.0;
  if (!radius_enabled && !height_enabled) {
    return keys;
  }

  const double radius_sq = min_frontier_radius_ * min_frontier_radius_;
  KeySet filtered;
  for (const auto & key : keys) {
    const octomap::point3d coord = tree.keyToCoord(key, depth);
    const double dx = coord.x() - current_position.x();
    const double dy = coord.y() - current_position.y();

    if (radius_enabled && (dx * dx + dy * dy) < radius_sq) {
      continue;
    }
    if (height_enabled &&
      std::abs(coord.z() - current_position.z()) > max_frontier_height_)
    {
      continue;
    }
    filtered.insert(key);
  }
  return filtered;
}

FrontierDetector::KeySet FrontierDetector::findFrontierKeys(
  const octomap::OcTree & tree,
  const std::vector<octomap::OcTreeKey> & changed_cells)
{
  // Newly changed free cells that satisfy the frontier test become part of the
  // global set. This mirrors `FrontierServer::findFrontier` in the reference.
  for (const auto & key : changed_cells) {
    if (inBounds(tree, key, bounds_) && isFrontierCell(tree, key)) {
      global_frontier_cells_.insert(key);
    }
  }

  // Re-test every stored cell so frontiers that gained an occupied neighbour or
  // lost their unknown border are removed (the paper's `updateGlobalFrontier`).
  for (auto it = global_frontier_cells_.begin(); it != global_frontier_cells_.end(); ) {
    if (!inBounds(tree, *it, bounds_) || !isFrontierCell(tree, *it)) {
      it = global_frontier_cells_.erase(it);
    } else {
      ++it;
    }
  }

  return global_frontier_cells_;
}

FrontierDetector::KeySet FrontierDetector::rebuildFrontierKeys(
  const octomap::OcTree & tree)
{
  global_frontier_cells_.clear();
  const unsigned int depth = tree.getTreeDepth();
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if (tree.isNodeOccupied(*it)) {
      continue;
    }
    const octomap::point3d coord = it.getCoordinate();
    if (bounds_.enabled && !bounds_.contains(coord.x(), coord.y(), coord.z())) {
      continue;
    }
    octomap::OcTreeKey key;
    if (!tree.coordToKeyChecked(coord, depth, key)) {
      continue;
    }
    if (isFrontierCell(tree, key)) {
      global_frontier_cells_.insert(key);
    }
  }
  return global_frontier_cells_;
}

FrontierDetector::KeySet FrontierDetector::parentKeys(const octomap::OcTree & tree) const
{
  return parentKeys(tree, global_frontier_cells_);
}

FrontierDetector::KeySet FrontierDetector::parentKeys(
  const octomap::OcTree & tree, const KeySet & frontier_keys) const
{
  unsigned int depth = exploration_depth_;
  const unsigned int tree_depth = tree.getTreeDepth();
  if (depth > tree_depth) {
    depth = tree_depth;
  }

  KeySet parents;
  for (const auto & key : frontier_keys) {
    const octomap::point3d coord = tree.keyToCoord(key);
    octomap::OcTreeKey parent;
    if (tree.coordToKeyChecked(coord, depth, parent)) {
      parents.insert(parent);
    }
  }
  return parents;
}

std::vector<Frontier> FrontierDetector::clusterParentKeys(
  const octomap::OcTree & tree, const KeySet & parent_keys) const
{
  unsigned int depth = exploration_depth_;
  const unsigned int tree_depth = tree.getTreeDepth();
  if (depth > tree_depth) {
    depth = tree_depth;
  }
  if (parent_keys.empty()) {
    return {};
  }

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(parent_keys.size());
  for (const auto & key : parent_keys) {
    const octomap::point3d coord = tree.keyToCoord(key, depth);
    geometry_msgs::msg::Point point;
    point.x = coord.x();
    point.y = coord.y();
    point.z = coord.z();
    points.push_back(point);
  }

  // Reference Gaussian-kernel mean-shift (vendored, no ROS 1 / dlib dependency).
  MeanShift mean_shift;
  const auto clusters = mean_shift.cluster(points, kernel_bandwidth_);

  std::vector<Frontier> frontiers;
  frontiers.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    if (cluster.original_points.empty()) {
      continue;
    }
    Frontier frontier;
    frontier.points.reserve(cluster.original_points.size());
    for (const auto & point : cluster.original_points) {
      frontier.points.emplace_back(point.x, point.y, point.z);
    }

    // Representative: original cell nearest to the converged mode, matching
    // `MSCluster::getMeanShiftClusters` of the reference implementation.
    double best_distance = std::numeric_limits<double>::max();
    std::size_t best_index = 0;
    for (std::size_t i = 0; i < cluster.original_points.size(); ++i) {
      const auto & point = cluster.original_points[i];
      const double dx = point.x - cluster.mode.x;
      const double dy = point.y - cluster.mode.y;
      const double dz = point.z - cluster.mode.z;
      const double distance = dx * dx + dy * dy + dz * dz;
      if (distance < best_distance) {
        best_distance = distance;
        best_index = i;
      }
    }
    const auto & representative = cluster.original_points[best_index];
    frontier.representative =
      octomap::point3d(representative.x, representative.y, representative.z);

    // Discard small clusters. This is the noise filter the reference algorithm
    // deliberately lacks; `min_frontier_size = 1` restores its exact behaviour.
    if (static_cast<int>(frontier.points.size()) < min_frontier_size_) {
      continue;
    }
    frontiers.push_back(std::move(frontier));
  }
  return frontiers;
}

}  // namespace frontier_detector_3d
