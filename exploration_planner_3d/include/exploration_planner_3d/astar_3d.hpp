#ifndef EXPLORATION_PLANNER_3D__ASTAR_3D_HPP_
#define EXPLORATION_PLANNER_3D__ASTAR_3D_HPP_

#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>

#include <cstddef>
#include <vector>

namespace exploration_planner_3d
{

/// Axis-aligned search box in the map frame.
///
/// When disabled the grid bounds are taken from the octree's own metric extent
/// (plus a one-cell margin); when enabled the search is restricted to the box.
struct Bounds3D
{
  bool enabled = false;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;

  bool contains(double x, double y, double z) const
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y &&
           z >= min_z && z <= max_z;
  }
};

/// Outcome of a plan request, so callers can report why it failed.
enum class PlanStatus
{
  kSuccess,
  kEmptyMap,
  kStartOutOfBounds,
  kGoalOutOfBounds,
  kStartBlocked,
  kGoalBlocked,
  kGoalProjected,
  kNoPath,
};

/// Human-readable name of a `PlanStatus`.
const char * toString(PlanStatus status);

/// A* path planner over an `octomap::OcTree`.
///
/// The planner rasterises the octree into a private occupancy grid at
/// `resolution`, classifies each cell as free / unknown / occupied, optionally
/// inflates occupied cells by `inflation_radius`, and runs a 26-connected A*
/// from the start to the goal. It is stateless: every `plan()` call rebuilds
/// the grid from the tree it is given.
class AStar3D
{
public:
  struct Config
  {
    /// Grid cell size in metres.
    double resolution = 0.3;
    /// Occupied cells are dilated by this radius (metres) before planning.
    /// 0 disables inflation.
    double inflation_radius = 0.0;
    /// When true, unknown cells are traversable; when false they are blocked.
    bool allow_unknown = true;
    /// Cost multiplier applied when a path enters an unknown cell. Values > 1
    /// bias the route toward known free space while still allowing it to cross
    /// unmapped gaps to reach a frontier. Only used when `allow_unknown`.
    double unknown_cost = 3.0;
    /// Safety cap on the number of expanded nodes.
    std::size_t max_nodes = 500000;
    /// Heuristic scale (>= 1 speeds the search up at the cost of optimality).
    double heuristic_weight = 1.0;
    /// When true, the raw grid path is shortcut to a line-of-sight path.
    bool simplify = true;
  };

  AStar3D();
  explicit AStar3D(Config config);

  /// Plans a collision-free path.
  ///
  /// The start cell and the goal cell are snapped to the nearest traversable
  /// cell if they are blocked (e.g. the vehicle sits inside an inflated cell or
  /// the goal sits next to a wall), within a small search radius.
  ///
  /// \param[in] tree Occupancy tree to plan on.
  /// \param[in] start Start position in the map frame.
  /// \param[in] goal Goal position in the map frame.
  /// \param[in] bounds Optional search box.
  /// \param[out] status Optional outcome, set to the reason for success/failure.
  /// \return Waypoints from start to goal (both included) in the map frame, or
  ///         an empty vector when no path exists.
  std::vector<octomap::point3d> plan(
    const octomap::OcTree & tree, const octomap::point3d & start,
    const octomap::point3d & goal, const Bounds3D & bounds = Bounds3D(),
    PlanStatus * status = nullptr) const;

  const Config & config() const {return config_;}

private:
  Config config_;
};

}  // namespace exploration_planner_3d

#endif  // EXPLORATION_PLANNER_3D__ASTAR_3D_HPP_
