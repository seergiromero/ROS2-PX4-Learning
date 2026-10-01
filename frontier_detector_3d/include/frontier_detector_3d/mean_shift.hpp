#ifndef FRONTIER_DETECTOR_3D__MEAN_SHIFT_HPP_
#define FRONTIER_DETECTOR_3D__MEAN_SHIFT_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <vector>

namespace frontier_detector_3d
{

/// One mean-shift cluster, mirroring the reference `mean_shift_clustering`
/// package used by larics/uav_frontier_exploration_3d.
struct Cluster
{
  geometry_msgs::msg::Point mode;
  std::vector<geometry_msgs::msg::Point> original_points;
  std::vector<geometry_msgs::msg::Point> shifted_points;
};

/// Gaussian-kernel mean-shift clustering.
///
/// This is the algorithm from https://github.com/larics/mean_shift_clustering
/// (used by the reference frontier planner), vendored here so the package needs
/// no ROS 1 / dlib dependency. Only the `ros::WallTime` timing statements of the
/// original were removed; the kernel, convergence and mode-merging behaviour are
/// unchanged.
class MeanShift
{
public:
  using Point = geometry_msgs::msg::Point;

  MeanShift();
  explicit MeanShift(double (*kernel_func)(double, double));

  /// Shifts every point to its kernel-weighted mean until convergence.
  ///
  /// \param[in] points Input points.
  /// \param[in] kernel_bandwidth Kernel bandwidth in metres.
  /// \param[in] epsilon Convergence tolerance in metres (squared internally).
  /// \return The converged (shifted) points, one per input point.
  std::vector<Point> meanshift(
    const std::vector<Point> & points, double kernel_bandwidth, double epsilon = 0.3);

  /// Runs mean-shift and groups the converged modes.
  ///
  /// \param[in] points Input points.
  /// \param[in] kernel_bandwidth Kernel bandwidth in metres.
  /// \return One cluster per distinct mode.
  std::vector<Cluster> cluster(const std::vector<Point> & points, double kernel_bandwidth);

private:
  void set_kernel(double (*kernel_func)(double, double));
  void shift_point(
    const Point & point, const std::vector<Point> & points, double kernel_bandwidth,
    Point & shifted_point) const;
  std::vector<Cluster> cluster(
    const std::vector<Point> & points, const std::vector<Point> & shifted_points) const;

  double (*kernel_func_)(double, double);
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__MEAN_SHIFT_HPP_
