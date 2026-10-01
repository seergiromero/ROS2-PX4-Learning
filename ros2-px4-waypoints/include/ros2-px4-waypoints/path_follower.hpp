#ifndef ROS2_PX4_WAYPOINTS__PATH_FOLLOWER_HPP_
#define ROS2_PX4_WAYPOINTS__PATH_FOLLOWER_HPP_

#include <octomap/octomap_types.h>

#include <cstddef>
#include <limits>
#include <vector>

namespace ros2_px4_waypoints
{
namespace path_follower
{

/// Selects the point on `path` that lies `lookahead_distance` metres ahead of
/// the projection of `current` onto the path.
///
/// The nearest path vertex to `current` is used as the starting point, then the
/// path is walked forward accumulating segment lengths until the lookahead
/// distance is reached (interpolating within the segment). If the remaining
/// path is shorter, the last point (goal) is returned.
///
/// \param[in] path Path waypoints in the map frame.
/// \param[in] current Current vehicle position in the map frame.
/// \param[in] lookahead_distance Lookahead distance in metres.
/// \param[out] target Selected target point in the map frame.
/// \return False when `path` is empty.
inline bool selectLookaheadPoint(
  const std::vector<octomap::point3d> & path, const octomap::point3d & current,
  double lookahead_distance, octomap::point3d & target)
{
  if (path.empty()) {
    return false;
  }
  if (path.size() == 1) {
    target = path.front();
    return true;
  }

  std::size_t nearest = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double distance = (path[i] - current).norm();
    if (distance < best_distance) {
      best_distance = distance;
      nearest = i;
    }
  }

  double accumulated = 0.0;
  for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
    const double segment = (path[i + 1] - path[i]).norm();
    if (accumulated + segment >= lookahead_distance) {
      const double remaining = lookahead_distance - accumulated;
      const float t = segment > 1e-9 ?
        static_cast<float>(remaining / segment) : 0.0f;
      target = path[i] + (path[i + 1] - path[i]) * t;
      return true;
    }
    accumulated += segment;
  }

  target = path.back();
  return true;
}

}  // namespace path_follower
}  // namespace ros2_px4_waypoints

#endif  // ROS2_PX4_WAYPOINTS__PATH_FOLLOWER_HPP_
