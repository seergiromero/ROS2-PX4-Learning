#include "exploration_planner_3d/astar_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace exploration_planner_3d
{

namespace
{

constexpr uint8_t kFree = 0;
constexpr uint8_t kUnknown = 1;
constexpr uint8_t kOccupied = 2;
constexpr uint8_t kInflated = 3;

/// Rasterised occupancy grid used internally by the planner.
struct Grid
{
  double resolution = 0.3;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_z = 0.0;
  int nx = 0;
  int ny = 0;
  int nz = 0;
  bool allow_unknown = true;
  double unknown_cost = 3.0;
  std::vector<uint8_t> state;

  std::size_t index(int x, int y, int z) const
  {
    return (static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) +
           static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) +
           static_cast<std::size_t>(z);
  }

  bool inside(int x, int y, int z) const
  {
    return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
  }

  octomap::point3d center(int x, int y, int z) const
  {
    return octomap::point3d(
      static_cast<float>(origin_x + (x + 0.5) * resolution),
      static_cast<float>(origin_y + (y + 0.5) * resolution),
      static_cast<float>(origin_z + (z + 0.5) * resolution));
  }

  bool worldToCell(const octomap::point3d & p, int & x, int & y, int & z) const
  {
    x = static_cast<int>(std::floor((p.x() - origin_x) / resolution));
    y = static_cast<int>(std::floor((p.y() - origin_y) / resolution));
    z = static_cast<int>(std::floor((p.z() - origin_z) / resolution));
    return inside(x, y, z);
  }

  bool traversable(int x, int y, int z) const
  {
    if (!inside(x, y, z)) {
      return false;
    }
    const uint8_t cell = state[index(x, y, z)];
    if (cell == kFree) {
      return true;
    }
    return cell == kUnknown && allow_unknown;
  }

  bool knownFree(int x, int y, int z) const
  {
    return inside(x, y, z) && state[index(x, y, z)] == kFree;
  }

  /// Multiplier applied to the step cost of entering a cell.
  double cellCost(int x, int y, int z) const
  {
    return state[index(x, y, z)] == kUnknown ? unknown_cost : 1.0;
  }
};

Grid buildGrid(
  const octomap::OcTree & tree, const AStar3D::Config & config,
  const Bounds3D & bounds)
{
  const double res = config.resolution;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  if (bounds.enabled) {
    min_x = bounds.min_x;
    min_y = bounds.min_y;
    min_z = bounds.min_z;
    max_x = bounds.max_x;
    max_y = bounds.max_y;
    max_z = bounds.max_z;
  } else {
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
  }
  // One-cell margin so a path can hug the map edge.
  min_x -= res;
  min_y -= res;
  min_z -= res;
  max_x += res;
  max_y += res;
  max_z += res;

  Grid grid;
  grid.resolution = res;
  grid.origin_x = min_x;
  grid.origin_y = min_y;
  grid.origin_z = min_z;
  grid.allow_unknown = config.allow_unknown;
  grid.unknown_cost = std::max(1.0, config.unknown_cost);
  grid.nx = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / res)));
  grid.ny = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / res)));
  grid.nz = std::max(1, static_cast<int>(std::ceil((max_z - min_z) / res)));
  grid.state.assign(
    static_cast<std::size_t>(grid.nx) * static_cast<std::size_t>(grid.ny) *
    static_cast<std::size_t>(grid.nz), kFree);

  for (int x = 0; x < grid.nx; ++x) {
    for (int y = 0; y < grid.ny; ++y) {
      for (int z = 0; z < grid.nz; ++z) {
        // Unknown stays unknown; whether it is traversable is decided by
        // `allow_unknown` so its cost can be penalised rather than lost.
        const octomap::OcTreeNode * node = tree.search(grid.center(x, y, z));
        uint8_t cell;
        if (node == nullptr) {
          cell = kUnknown;
        } else if (tree.isNodeOccupied(node)) {
          cell = kOccupied;
        } else {
          cell = kFree;
        }
        grid.state[grid.index(x, y, z)] = cell;
      }
    }
  }

  // Cylindrical footprint: inflate every occupied cell by `footprint_radius` in
  // XY and `footprint_height` in Z (plus half a map cell so the discretisation
  // itself is covered). This keeps the route at least one body radius away from
  // obstacles horizontally while still allowing flight above/below them.
  const double radius = config.footprint_radius;
  const double height = config.footprint_height;
  if (radius > 0.0 || height > 0.0) {
    const double effective_radius = radius + 0.5 * res;
    const double effective_height = height + 0.5 * res;
    const int radius_cells =
      static_cast<int>(std::ceil(effective_radius / res));
    const int height_cells =
      static_cast<int>(std::ceil(effective_height / res));
    std::vector<uint8_t> inflated = grid.state;
    for (int x = 0; x < grid.nx; ++x) {
      for (int y = 0; y < grid.ny; ++y) {
        for (int z = 0; z < grid.nz; ++z) {
          if (grid.state[grid.index(x, y, z)] != kOccupied) {
            continue;
          }
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
              const double horizontal =
                std::sqrt(static_cast<double>(dx * dx + dy * dy)) * res;
              if (horizontal > effective_radius) {
                continue;
              }
              for (int dz = -height_cells; dz <= height_cells; ++dz) {
                if (std::abs(dz) * res > effective_height) {
                  continue;
                }
                const int cx = x + dx;
                const int cy = y + dy;
                const int cz = z + dz;
                if (!grid.inside(cx, cy, cz)) {
                  continue;
                }
                // Inflate both free and unknown cells so the route keeps
                // clearance even through unmapped space. They are tagged
                // kInflated so they can be told apart from real obstacles when
                // visualising the footprint.
                uint8_t & cell = inflated[grid.index(cx, cy, cz)];
                if (cell == kFree || cell == kUnknown) {
                  cell = kInflated;
                }
              }
            }
          }
        }
      }
    }
    grid.state.swap(inflated);
  }

  return grid;
}

/// Moves (x, y, z) to the nearest traversable cell within `max_radius` cells.
bool snapToTraversable(const Grid & grid, int & x, int & y, int & z, int max_radius)
{
  if (grid.traversable(x, y, z)) {
    return true;
  }
  for (int r = 1; r <= max_radius; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != r) {
            continue;
          }
          if (grid.traversable(x + dx, y + dy, z + dz)) {
            x += dx;
            y += dy;
            z += dz;
            return true;
          }
        }
      }
    }
  }
  return false;
}

/// Finds the nearest known-free cell to a world point across the whole search
/// grid. This is used for frontier goals that land on an occupied/unknown cell
/// or just outside the configured search box.
bool nearestKnownFreeCell(
  const Grid & grid, const octomap::point3d & target, int & x, int & y, int & z)
{
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int ix = 0; ix < grid.nx; ++ix) {
    for (int iy = 0; iy < grid.ny; ++iy) {
      for (int iz = 0; iz < grid.nz; ++iz) {
        if (!grid.knownFree(ix, iy, iz)) {
          continue;
        }
        const double distance = (grid.center(ix, iy, iz) - target).norm();
        if (distance < best_distance) {
          best_distance = distance;
          x = ix;
          y = iy;
          z = iz;
          found = true;
        }
      }
    }
  }
  return found;
}

/// Fallback when no known-free cell exists but unknown traversal is enabled.
bool nearestTraversableCell(
  const Grid & grid, const octomap::point3d & target, int & x, int & y, int & z)
{
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int ix = 0; ix < grid.nx; ++ix) {
    for (int iy = 0; iy < grid.ny; ++iy) {
      for (int iz = 0; iz < grid.nz; ++iz) {
        if (!grid.traversable(ix, iy, iz)) {
          continue;
        }
        const double distance = (grid.center(ix, iy, iz) - target).norm();
        if (distance < best_distance) {
          best_distance = distance;
          x = ix;
          y = iy;
          z = iz;
          found = true;
        }
      }
    }
  }
  return found;
}

/// True when the straight segment between two points stays on free cells.
bool segmentClear(const Grid & grid, const octomap::point3d & a, const octomap::point3d & b)
{
  const double distance = (b - a).norm();
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / (grid.resolution * 0.5))));
  for (int i = 0; i <= steps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(steps);
    const octomap::point3d p = a + (b - a) * t;
    int x, y, z;
    if (!grid.worldToCell(p, x, y, z) || !grid.traversable(x, y, z)) {
      return false;
    }
  }
  return true;
}

}  // namespace

AStar3D::AStar3D()
: AStar3D(Config{})
{
}

AStar3D::AStar3D(Config config)
: config_(config)
{
  if (config_.resolution <= 0.0) {
    throw std::invalid_argument("AStar3D resolution must be positive");
  }
}

std::vector<octomap::point3d> AStar3D::plan(
  const octomap::OcTree & tree, const octomap::point3d & start,
  const octomap::point3d & goal, const Bounds3D & bounds, PlanStatus * status) const
{
  const auto set_status = [status](PlanStatus value) {
      if (status != nullptr) {
        *status = value;
      }
    };

  if (tree.size() == 0 && !bounds.enabled) {
    set_status(PlanStatus::kEmptyMap);
    return {};
  }

  const Grid grid = buildGrid(tree, config_, bounds);

  int sx, sy, sz;
  int gx, gy, gz;
  if (!grid.worldToCell(start, sx, sy, sz)) {
    set_status(PlanStatus::kStartOutOfBounds);
    return {};
  }

  bool start_snapped = false;
  if (!grid.traversable(sx, sy, sz)) {
    if (!snapToTraversable(grid, sx, sy, sz, 3)) {
      set_status(PlanStatus::kStartBlocked);
      return {};
    }
    start_snapped = true;
  }

  // A frontier goal can be just outside the configured planning box or can
  // land on an occupied/unknown raster cell due to the different resolutions.
  // Prefer the nearest known-free cell, and only fall back to an unknown cell
  // when unknown traversal is explicitly enabled.
  bool goal_snapped = false;
  bool goal_projected = false;
  if (!grid.worldToCell(goal, gx, gy, gz) || !grid.knownFree(gx, gy, gz)) {
    if (!nearestKnownFreeCell(grid, goal, gx, gy, gz)) {
      if (!config_.allow_unknown ||
        !nearestTraversableCell(grid, goal, gx, gy, gz))
      {
        set_status(
          grid.worldToCell(goal, gx, gy, gz) ? PlanStatus::kGoalBlocked :
          PlanStatus::kGoalOutOfBounds);
        return {};
      }
    }
    goal_snapped = true;
    goal_projected = true;
  }

  const std::size_t n = grid.state.size();
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(n, infinity);
  std::vector<int64_t> parent(n, -1);
  std::vector<uint8_t> closed(n, 0);

  const std::size_t start_index = grid.index(sx, sy, sz);
  const std::size_t goal_index = grid.index(gx, gy, gz);

  const auto heuristic = [&](int x, int y, int z) {
      const double dx = (x - gx) * config_.resolution;
      const double dy = (y - gy) * config_.resolution;
      const double dz = (z - gz) * config_.resolution;
      return config_.heuristic_weight * std::sqrt(dx * dx + dy * dy + dz * dz);
    };

  using QueueItem = std::pair<double, std::size_t>;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  g_score[start_index] = 0.0;
  open.push({heuristic(sx, sy, sz), start_index});

  std::size_t expanded = 0;
  bool found = false;
  while (!open.empty()) {
    const QueueItem top = open.top();
    open.pop();
    const std::size_t current = top.second;
    if (closed[current] != 0) {
      continue;
    }
    closed[current] = 1;

    if (current == goal_index) {
      found = true;
      break;
    }
    if (++expanded > config_.max_nodes) {
      break;
    }

    const int cz = static_cast<int>(current % static_cast<std::size_t>(grid.nz));
    const std::size_t rest = current / static_cast<std::size_t>(grid.nz);
    const int cy = static_cast<int>(rest % static_cast<std::size_t>(grid.ny));
    const int cx = static_cast<int>(rest / static_cast<std::size_t>(grid.ny));

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const int nx = cx + dx;
          const int ny = cy + dy;
          const int nz = cz + dz;
          if (!grid.traversable(nx, ny, nz)) {
            continue;
          }
          const std::size_t neighbor = grid.index(nx, ny, nz);
          if (closed[neighbor] != 0) {
            continue;
          }
          const double step =
            std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) *
            config_.resolution;
          const double tentative =
            g_score[current] + step * grid.cellCost(nx, ny, nz);
          if (tentative < g_score[neighbor]) {
            g_score[neighbor] = tentative;
            parent[neighbor] = static_cast<int64_t>(current);
            open.push({tentative + heuristic(nx, ny, nz), neighbor});
          }
        }
      }
    }
  }

  if (!found) {
    set_status(PlanStatus::kNoPath);
    return {};
  }

  std::vector<octomap::point3d> path;
  for (int64_t index = static_cast<int64_t>(goal_index); index >= 0;
    index = parent[static_cast<std::size_t>(index)])
  {
    const int z = static_cast<int>(index % grid.nz);
    const int64_t rest = index / grid.nz;
    const int y = static_cast<int>(rest % grid.ny);
    const int x = static_cast<int>(rest / grid.ny);
    path.push_back(grid.center(x, y, z));
    if (static_cast<std::size_t>(index) == start_index) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());

  if (config_.simplify && path.size() > 2) {
    std::vector<octomap::point3d> simplified;
    simplified.push_back(path.front());
    std::size_t i = 0;
    while (i + 1 < path.size()) {
      std::size_t j = path.size() - 1;
      for (; j > i + 1; --j) {
        if (segmentClear(grid, path[i], path[j])) {
          break;
        }
      }
      simplified.push_back(path[j]);
      i = j;
    }
    path.swap(simplified);
  }

  // Anchor the path exactly on the requested start/goal only when they already
  // fell in a traversable cell; a snapped endpoint stays at its safe cell so
  // the published route never starts or ends inside an obstacle.
  if (!path.empty()) {
    if (!start_snapped) {
      path.front() = start;
    }
    if (!goal_snapped) {
      path.back() = goal;
    }
  }
  set_status(goal_projected ? PlanStatus::kGoalProjected : PlanStatus::kSuccess);
  return path;
}

std::vector<octomap::point3d> AStar3D::inflatedCells(
  const octomap::OcTree & tree, const Bounds3D & bounds) const
{
  const Grid grid = buildGrid(tree, config_, bounds);
  std::vector<octomap::point3d> cells;
  for (int x = 0; x < grid.nx; ++x) {
    for (int y = 0; y < grid.ny; ++y) {
      for (int z = 0; z < grid.nz; ++z) {
        if (grid.state[grid.index(x, y, z)] == kInflated) {
          cells.push_back(grid.center(x, y, z));
        }
      }
    }
  }
  return cells;
}

const char * toString(PlanStatus status)
{
  switch (status) {
    case PlanStatus::kSuccess:
      return "success";
    case PlanStatus::kEmptyMap:
      return "empty_map";
    case PlanStatus::kStartOutOfBounds:
      return "start_out_of_bounds";
    case PlanStatus::kGoalOutOfBounds:
      return "goal_out_of_bounds";
    case PlanStatus::kStartBlocked:
      return "start_blocked";
    case PlanStatus::kGoalBlocked:
      return "goal_blocked";
    case PlanStatus::kGoalProjected:
      return "goal_projected";
    case PlanStatus::kNoPath:
      return "no_path";
  }
  return "unknown";
}

}  // namespace exploration_planner_3d
