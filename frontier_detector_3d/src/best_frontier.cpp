#include "frontier_detector_3d/best_frontier.hpp"

#include <cmath>
#include <limits>

namespace frontier_detector_3d
{

BestFrontier::BestFrontier(double box_length, double k_gain, double lambda)
: box_length_(box_length),
  k_gain_(k_gain),
  lambda_(lambda)
{
}

double BestFrontier::unknownRatio(
  const octomap::OcTree & tree, const octomap::point3d & center) const
{
  const double resolution = tree.getResolution();
  if (resolution <= 0.0 || box_length_ <= 0.0) {
    return 0.0;
  }

  const double half = box_length_ * 0.5;
  const double min_x = center.x() - half;
  const double min_y = center.y() - half;
  const double min_z = center.z() - half;
  const double max_x = center.x() + half;
  const double max_y = center.y() + half;
  const double max_z = center.z() + half;

  long total = 0;
  long unknown = 0;
  for (double ix = min_x; ix < max_x; ix += resolution) {
    for (double iy = min_y; iy < max_y; iy += resolution) {
      for (double iz = min_z; iz < max_z; iz += resolution) {
        ++total;
        if (!tree.search(ix, iy, iz)) {
          ++unknown;
        }
      }
    }
  }

  if (total == 0) {
    return 0.0;
  }
  return static_cast<double>(unknown) / static_cast<double>(total);
}

double BestFrontier::informationGain(
  const octomap::OcTree & tree, const octomap::point3d & current_position,
  const octomap::point3d & candidate) const
{
  const double unknown_volume = unknownRatio(tree, candidate);
  const double distance = (candidate - current_position).norm();
  return k_gain_ * unknown_volume * std::exp(-lambda_ * distance);
}

BestFrontierResult BestFrontier::select(
  const octomap::OcTree & tree, const octomap::point3d & current_position,
  const std::vector<octomap::point3d> & candidates) const
{
  BestFrontierResult result;
  double best_gain = -std::numeric_limits<double>::max();
  for (const auto & candidate : candidates) {
    const double gain = informationGain(tree, current_position, candidate);
    if (gain > best_gain) {
      best_gain = gain;
      result.point = candidate;
      result.valid = true;
    }
  }
  if (result.valid) {
    result.gain = best_gain;
  }
  return result;
}

}  // namespace frontier_detector_3d
