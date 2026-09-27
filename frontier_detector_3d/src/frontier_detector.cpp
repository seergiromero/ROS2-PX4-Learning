#include "frontier_detector_3d/frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace frontier_detector_3d
{

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
  int min_frontier_size, int max_frontiers, Connectivity connectivity,
  double max_dist_to_occupied, double cluster_size_xy, double cluster_size_z,
  double ground_z)
: min_frontier_size_(min_frontier_size),
  max_frontiers_(max_frontiers),
  connectivity_(connectivity),
  max_dist_to_occupied_(max_dist_to_occupied),
  cluster_size_xy_(cluster_size_xy),
  cluster_size_z_(cluster_size_z),
  ground_z_(ground_z)
{
  if (min_frontier_size_ < 1) {
    throw std::invalid_argument("min_frontier_size must be >= 1");
  }
  if (max_frontiers_ < 1) {
    throw std::invalid_argument("max_frontiers must be >= 1");
  }
  if (connectivity_ != Connectivity::k6 && connectivity_ != Connectivity::k18 &&
    connectivity_ != Connectivity::k26)
  {
    throw std::invalid_argument("connectivity must be one of 6, 18 or 26");
  }
}

std::vector<Frontier> FrontierDetector::detect(const octomap::OcTree & tree) const
{
  return clusterFrontiers(detectFrontierKeys(tree), tree);
}

FrontierDetector::KeySet FrontierDetector::detectFrontierKeys(
  const octomap::OcTree & tree) const
{
  const unsigned int max_depth = tree.getTreeDepth();

  // Optional filter: keep only frontier voxels within `max_dist_to_occupied_`
  // of an occupied voxel. With a ray-cast LiDAR map, free space is only
  // cleared along the beams, so the gaps between beams and the sensor FOV
  // edges produce "frontier" voxels floating in empty air. Such voxels are
  // far from any occupied cell, unlike real frontiers at walls, obstacles and
  // openings, and are filtered out here.
  const bool use_occupancy_filter = max_dist_to_occupied_ > 0.0;
  OccupiedGrid occupied_grid;
  if (use_occupancy_filter) {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    occupied_grid.resolution = max_dist_to_occupied_;
    occupied_grid.origin_x = std::floor(min_x / occupied_grid.resolution) *
      occupied_grid.resolution;
    occupied_grid.origin_y = std::floor(min_y / occupied_grid.resolution) *
      occupied_grid.resolution;
    occupied_grid.origin_z = std::floor(min_z / occupied_grid.resolution) *
      occupied_grid.resolution;
    occupied_grid.nx = std::max(
      1, static_cast<int>(std::ceil((max_x - occupied_grid.origin_x) / occupied_grid.resolution)));
    occupied_grid.ny = std::max(
      1, static_cast<int>(std::ceil((max_y - occupied_grid.origin_y) / occupied_grid.resolution)));
    occupied_grid.nz = std::max(
      1, static_cast<int>(std::ceil((max_z - occupied_grid.origin_z) / occupied_grid.resolution)));
  }

  KeySet candidates;

  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if (tree.isNodeOccupied(*it)) {
      if (use_occupancy_filter) {
        const octomap::point3d center = it.getCoordinate();
        const int cx = static_cast<int>(std::floor(
            (center.x() - occupied_grid.origin_x) / occupied_grid.resolution));
        const int cy = static_cast<int>(std::floor(
            (center.y() - occupied_grid.origin_y) / occupied_grid.resolution));
        const int cz = static_cast<int>(std::floor(
            (center.z() - occupied_grid.origin_z) / occupied_grid.resolution));
        if (cx >= 0 && cx < occupied_grid.nx && cy >= 0 && cy < occupied_grid.ny &&
          cz >= 0 && cz < occupied_grid.nz)
        {
          occupied_grid.cells.insert(occupied_grid.index(cx, cy, cz));
        }
      }
      continue;
    }

    const unsigned int depth = it.getDepth();
    if (depth == 0) {
      // The whole tree collapsed into a single free leaf: the boundary is the
      // entire 16-level address space, which is not meaningful to explore.
      continue;
    }
    const octomap::OcTreeKey node_key = it.getIndexKey();

    // Skip coarse free leaves whose whole neighbourhood is already stored in
    // the tree: they are fully enclosed by known space and can never contain
    // frontier voxels. Expanding their surface would waste time on the large
    // pruned free volumes inside the explored map.
    if (depth < max_depth && !bordersUnknownAtDepth(tree, node_key, depth)) {
      continue;
    }

    // Expand the node's surface shell at full depth. Interior voxels have all
    // their neighbours inside the node (free) and can never be frontiers, so
    // only the shell has to be examined. The shell is generated directly as
    // O(surface) work, never by iterating the whole node volume. For a
    // full-depth leaf the shell collapses to the voxel itself.
    const unsigned int shift = max_depth - depth;
    const unsigned int lo_x = static_cast<unsigned int>(node_key.k[0]) << shift;
    const unsigned int hi_x = ((static_cast<unsigned int>(node_key.k[0]) + 1u) << shift) - 1u;
    const unsigned int lo_y = static_cast<unsigned int>(node_key.k[1]) << shift;
    const unsigned int hi_y = ((static_cast<unsigned int>(node_key.k[1]) + 1u) << shift) - 1u;
    const unsigned int lo_z = static_cast<unsigned int>(node_key.k[2]) << shift;
    const unsigned int hi_z = ((static_cast<unsigned int>(node_key.k[2]) + 1u) << shift) - 1u;

    const auto addIfFrontier = [&](unsigned int x, unsigned int y, unsigned int z) {
        const octomap::OcTreeKey key(
          static_cast<unsigned short>(x),
          static_cast<unsigned short>(y),
          static_cast<unsigned short>(z));
        for (const auto & neighbor : neighbors(key)) {
          if (isUnknownNeighbor(tree, neighbor)) {
            candidates.insert(key);
            return;
          }
        }
      };

    // Face pairs perpendicular to z (always present, covers the whole plane).
    for (unsigned int x = lo_x; x <= hi_x; ++x) {
      for (unsigned int y = lo_y; y <= hi_y; ++y) {
        addIfFrontier(x, y, lo_z);
        if (hi_z != lo_z) {
          addIfFrontier(x, y, hi_z);
        }
      }
    }
    // Face pairs perpendicular to y, over the interior z range.
    if (hi_z > lo_z) {
      for (unsigned int x = lo_x; x <= hi_x; ++x) {
        for (unsigned int z = lo_z + 1; z <= hi_z - 1; ++z) {
          addIfFrontier(x, lo_y, z);
          if (hi_y != lo_y) {
            addIfFrontier(x, hi_y, z);
          }
        }
      }
    }
    // Face pairs perpendicular to x, over the interior y and z ranges.
    if (hi_y > lo_y && hi_z > lo_z) {
      for (unsigned int y = lo_y + 1; y <= hi_y - 1; ++y) {
        for (unsigned int z = lo_z + 1; z <= hi_z - 1; ++z) {
          addIfFrontier(lo_x, y, z);
          addIfFrontier(hi_x, y, z);
        }
      }
    }
  }

  if (!use_occupancy_filter && ground_z_ <= 0.0) {
    return candidates;
  }

  KeySet result;
  for (const auto & key : candidates) {
    const octomap::point3d coord = tree.keyToCoord(key);
    if (ground_z_ > 0.0 && coord.z() < ground_z_) {
      continue;
    }
    if (use_occupancy_filter && !isNearOccupied(tree, key, occupied_grid)) {
      continue;
    }
    result.insert(key);
  }
  return result;
}

bool FrontierDetector::bordersUnknownAtDepth(
  const octomap::OcTree & tree, const octomap::OcTreeKey & node_key,
  unsigned int depth) const
{
  constexpr int max_key = std::numeric_limits<unsigned short>::max();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const int nx = static_cast<int>(node_key.k[0]) + dx;
        const int ny = static_cast<int>(node_key.k[1]) + dy;
        const int nz = static_cast<int>(node_key.k[2]) + dz;
        if (nx < 0 || ny < 0 || nz < 0 || nx > max_key || ny > max_key || nz > max_key) {
          continue;
        }
        const octomap::OcTreeKey neighbor(
          static_cast<unsigned short>(nx),
          static_cast<unsigned short>(ny),
          static_cast<unsigned short>(nz));
        if (tree.search(neighbor, depth) == nullptr) {
          return true;
        }
      }
    }
  }
  return false;
}

std::vector<Frontier> FrontierDetector::clusterFrontiers(
  const KeySet & frontier_keys, const octomap::OcTree & tree) const
{
  std::vector<Frontier> raw_clusters;
  KeySet remaining = frontier_keys;

  while (!remaining.empty()) {
    const auto seed = *remaining.begin();
    remaining.erase(remaining.begin());

    std::queue<octomap::OcTreeKey> queue;
    queue.push(seed);
    Frontier cluster;

    while (!queue.empty()) {
      const auto current = queue.front();
      queue.pop();
      cluster.points.push_back(tree.keyToCoord(current));

      for (const auto & neighbor : neighbors(current)) {
        const auto it = remaining.find(neighbor);
        if (it != remaining.end()) {
          queue.push(*it);
          remaining.erase(it);
        }
      }
    }

    raw_clusters.push_back(std::move(cluster));
  }

  // Split oversized clusters so each published cluster fits the configured
  // size limits, then discard clusters below the minimum size.
  const bool split_enabled = cluster_size_xy_ > 0.0 || cluster_size_z_ > 0.0;
  std::vector<Frontier> clusters;
  for (auto & cluster : raw_clusters) {
    std::vector<Frontier> pieces;
    if (split_enabled) {
      splitCluster(cluster, pieces);
    } else {
      pieces.push_back(std::move(cluster));
    }
    for (auto & piece : pieces) {
      if (static_cast<int>(piece.size()) >= min_frontier_size_) {
        clusters.push_back(std::move(piece));
      }
    }
  }

  std::sort(
    clusters.begin(), clusters.end(),
    [](const Frontier & lhs, const Frontier & rhs) {return lhs.size() > rhs.size();});
  if (static_cast<int>(clusters.size()) > max_frontiers_) {
    clusters.resize(static_cast<size_t>(max_frontiers_));
  }
  return clusters;
}

void FrontierDetector::splitCluster(
  const Frontier & cluster, std::vector<Frontier> & pieces) const
{
  const std::size_t n = cluster.points.size();
  double mean_x = 0.0;
  double mean_y = 0.0;
  double mean_z = 0.0;
  for (const auto & point : cluster.points) {
    mean_x += point.x();
    mean_y += point.y();
    mean_z += point.z();
  }
  mean_x /= static_cast<double>(n);
  mean_y /= static_cast<double>(n);
  mean_z /= static_cast<double>(n);

  // Accept the cluster if it fits the configured size limits.
  bool too_large = false;
  for (const auto & point : cluster.points) {
    const double dx = point.x() - mean_x;
    const double dy = point.y() - mean_y;
    const double dz = point.z() - mean_z;
    if ((cluster_size_xy_ > 0.0 && std::hypot(dx, dy) > cluster_size_xy_) ||
      (cluster_size_z_ > 0.0 && std::abs(dz) > cluster_size_z_))
    {
      too_large = true;
      break;
    }
  }
  if (!too_large) {
    pieces.push_back(cluster);
    return;
  }

  // Covariance of the horizontal coordinates.
  double var_xx = 0.0;
  double var_xy = 0.0;
  double var_yy = 0.0;
  for (const auto & point : cluster.points) {
    const double dx = point.x() - mean_x;
    const double dy = point.y() - mean_y;
    var_xx += dx * dx;
    var_xy += dx * dy;
    var_yy += dy * dy;
  }
  const double trace = var_xx + var_yy;
  const double lambda_max =
    trace * 0.5 + std::sqrt(std::max(0.0,
      trace * trace * 0.25 - var_xx * var_yy + var_xy * var_xy));

  // First principal component of the 2x2 covariance. Axis-aligned when the
  // cross term is zero to keep the direction well defined.
  double pc_x;
  double pc_y;
  if (std::abs(var_xy) > 1e-12) {
    pc_x = lambda_max - var_yy;
    pc_y = var_xy;
  } else if (var_xx >= var_yy) {
    pc_x = 1.0;
    pc_y = 0.0;
  } else {
    pc_x = 0.0;
    pc_y = 1.0;
  }

  Frontier first;
  Frontier second;
  for (const auto & point : cluster.points) {
    const double dx = point.x() - mean_x;
    const double dy = point.y() - mean_y;
    if (dx * pc_x + dy * pc_y >= 0.0) {
      first.points.push_back(point);
    } else {
      second.points.push_back(point);
    }
  }

  // A degenerate split (all cells on one side) cannot reduce the cluster size,
  // so keep the cluster as-is to guarantee termination.
  if (first.points.empty() || second.points.empty()) {
    pieces.push_back(cluster);
    return;
  }

  splitCluster(first, pieces);
  splitCluster(second, pieces);
}

std::vector<octomap::OcTreeKey> FrontierDetector::neighbors(
  const octomap::OcTreeKey & key) const
{
  std::vector<octomap::OcTreeKey> result;
  result.reserve(26);

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
        if (connectivity_ == Connectivity::k6 && manhattan != 1) {
          continue;
        }
        if (connectivity_ == Connectivity::k18 && manhattan > 2) {
          continue;
        }

        const int nx = static_cast<int>(key.k[0]) + dx;
        const int ny = static_cast<int>(key.k[1]) + dy;
        const int nz = static_cast<int>(key.k[2]) + dz;
        if (nx < 0 || ny < 0 || nz < 0) {
          continue;
        }
        constexpr int max_key = std::numeric_limits<unsigned short>::max();
        if (nx > max_key || ny > max_key || nz > max_key) {
          continue;
        }
        result.emplace_back(
          static_cast<unsigned short>(nx),
          static_cast<unsigned short>(ny),
          static_cast<unsigned short>(nz));
      }
    }
  }
  return result;
}

bool FrontierDetector::isUnknownNeighbor(
  const octomap::OcTree & tree, const octomap::OcTreeKey & key)
{
  return tree.search(key) == nullptr;
}

bool FrontierDetector::isNearOccupied(
  const octomap::OcTree & tree, const octomap::OcTreeKey & key,
  const OccupiedGrid & grid)
{
  const octomap::point3d coord = tree.keyToCoord(key);
  const int cx = static_cast<int>(std::floor((coord.x() - grid.origin_x) / grid.resolution));
  const int cy = static_cast<int>(std::floor((coord.y() - grid.origin_y) / grid.resolution));
  const int cz = static_cast<int>(std::floor((coord.z() - grid.origin_z) / grid.resolution));

  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int ncx = cx + dx;
        const int ncy = cy + dy;
        const int ncz = cz + dz;
        if (ncx < 0 || ncy < 0 || ncz < 0 ||
          ncx >= grid.nx || ncy >= grid.ny || ncz >= grid.nz)
        {
          continue;
        }
        if (grid.cells.count(grid.index(ncx, ncy, ncz)) > 0) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace frontier_detector_3d
