#include "frontier_detector_3d/mean_shift.hpp"

#include <cmath>

namespace frontier_detector_3d
{

namespace
{
// Fixed mode-merge threshold of the reference package (metres).
constexpr double kClusterEpsilon = 2.0;

double euclideanDistance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::sqrt(
    std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

double euclideanDistanceSqr(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
}

double gaussianKernel(double distance, double kernel_bandwidth)
{
  return std::exp(-0.5 * (distance * distance) / (kernel_bandwidth * kernel_bandwidth));
}
}  // namespace

MeanShift::MeanShift()
{
  set_kernel(nullptr);
}

MeanShift::MeanShift(double (*kernel_func)(double, double))
{
  set_kernel(kernel_func);
}

void MeanShift::set_kernel(double (*kernel_func)(double, double))
{
  if (!kernel_func) {
    kernel_func_ = gaussianKernel;
  } else {
    kernel_func_ = kernel_func;
  }
}

void MeanShift::shift_point(
  const Point & point, const std::vector<Point> & points, double kernel_bandwidth,
  Point & shifted_point) const
{
  shifted_point.x = 0;
  shifted_point.y = 0;
  shifted_point.z = 0;
  double total_weight = 0;
  for (std::size_t i = 0; i < points.size(); i++) {
    const Point & temp_point = points[i];
    const double distance = euclideanDistance(point, temp_point);
    const double weight = kernel_func_(distance, kernel_bandwidth);
    shifted_point.x += temp_point.x * weight;
    shifted_point.y += temp_point.y * weight;
    shifted_point.z += temp_point.z * weight;
    total_weight += weight;
  }
  const double total_weight_inv = 1.0 / total_weight;
  shifted_point.x *= total_weight_inv;
  shifted_point.y *= total_weight_inv;
  shifted_point.z *= total_weight_inv;
}

std::vector<MeanShift::Point> MeanShift::meanshift(
  const std::vector<Point> & points, double kernel_bandwidth, double epsilon)
{
  const double EPSILON_SQR = epsilon * epsilon;
  std::vector<bool> stop_moving(points.size(), false);
  std::vector<Point> shifted_points = points;
  double max_shift_distance;
  Point point_new;
  do {
    max_shift_distance = 0;
    for (std::size_t i = 0; i < points.size(); i++) {
      if (!stop_moving[i]) {
        shift_point(shifted_points[i], points, kernel_bandwidth, point_new);
        const double shift_distance_sqr = euclideanDistanceSqr(point_new, shifted_points[i]);
        if (shift_distance_sqr > max_shift_distance) {
          max_shift_distance = shift_distance_sqr;
        }
        if (shift_distance_sqr <= EPSILON_SQR) {
          stop_moving[i] = true;
        }
        shifted_points[i] = point_new;
      }
    }
  } while (max_shift_distance > EPSILON_SQR);
  return shifted_points;
}

std::vector<Cluster> MeanShift::cluster(
  const std::vector<Point> & points, const std::vector<Point> & shifted_points) const
{
  std::vector<Cluster> clusters;
  for (std::size_t i = 0; i < shifted_points.size(); i++) {
    std::size_t c = 0;
    for (; c < clusters.size(); c++) {
      if (euclideanDistance(shifted_points[i], clusters[c].mode) <= kClusterEpsilon) {
        break;
      }
    }
    if (c == clusters.size()) {
      Cluster clus;
      clus.mode = shifted_points[i];
      clusters.push_back(clus);
    }
    clusters[c].original_points.push_back(points[i]);
    clusters[c].shifted_points.push_back(shifted_points[i]);
  }
  return clusters;
}

std::vector<Cluster> MeanShift::cluster(
  const std::vector<Point> & points, double kernel_bandwidth)
{
  const std::vector<Point> shifted_points = meanshift(points, kernel_bandwidth);
  return cluster(points, shifted_points);
}

}  // namespace frontier_detector_3d
