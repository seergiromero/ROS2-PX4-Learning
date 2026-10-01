#include "frontier_detector_3d/frontier_visualizer.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <utility>

namespace frontier_detector_3d
{

FrontierVisualizer::FrontierVisualizer(
  rclcpp::Node & node, const std::string & topic, bool publish_voxels,
  bool publish_centroids)
: node_(node),
  publish_voxels_(publish_voxels),
  publish_centroids_(publish_centroids)
{
  pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
    topic, rclcpp::QoS(1).transient_local());
}

void FrontierVisualizer::publish(
  const std::vector<Frontier> & frontiers, const std::string & frame_id,
  const rclcpp::Time & stamp, double cell_size)
{
  visualization_msgs::msg::MarkerArray markers;

  // Clear stale markers from previous cycles before adding the new ones.
  // RViz2's DELETEALL clears only the namespace carried by the marker, so the
  // namespace is left EMPTY here to clear the whole topic (voxels AND
  // centroids). Using ns="frontiers" would leave old centroids on screen.
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.ns = "";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(std::move(clear));

  std::size_t id = 1;
  for (const auto & cluster : frontiers) {
    if (cluster.points.empty()) {
      continue;
    }

    // Deterministic per-cluster hue so each cluster is distinguishable.
    const float hue = static_cast<float>((id * 0.37) - std::floor(id * 0.37));

    if (publish_voxels_) {
      visualization_msgs::msg::Marker voxels;
      voxels.header.frame_id = frame_id;
      voxels.header.stamp = stamp;
      voxels.ns = "frontiers";
      voxels.id = static_cast<int32_t>(id);
      voxels.type = visualization_msgs::msg::Marker::CUBE_LIST;
      voxels.action = visualization_msgs::msg::Marker::ADD;
      voxels.scale.x = cell_size;
      voxels.scale.y = cell_size;
      voxels.scale.z = cell_size;
      voxels.color.r = hue;
      voxels.color.g = 0.8f;
      voxels.color.b = 1.0f - hue;
      voxels.color.a = 1.0f;

      for (const auto & point : cluster.points) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.x();
        marker_point.y = point.y();
        marker_point.z = point.z();
        voxels.points.push_back(marker_point);
      }
      markers.markers.push_back(std::move(voxels));
    }

    if (publish_centroids_) {
      // Geometric center (mean) of this cluster's voxels, so the sphere sits in
      // the middle of the same-colored cube group.
      double cx = 0.0;
      double cy = 0.0;
      double cz = 0.0;
      for (const auto & point : cluster.points) {
        cx += point.x();
        cy += point.y();
        cz += point.z();
      }
      const double inv = 1.0 / static_cast<double>(cluster.points.size());
      cx *= inv;
      cy *= inv;
      cz *= inv;

      visualization_msgs::msg::Marker centroid;
      centroid.header.frame_id = frame_id;
      centroid.header.stamp = stamp;
      centroid.ns = "frontier_centroids";
      centroid.id = static_cast<int32_t>(id);
      centroid.type = visualization_msgs::msg::Marker::SPHERE;
      centroid.action = visualization_msgs::msg::Marker::ADD;
      const double scale = std::max(0.2, 2.0 * cell_size);
      centroid.scale.x = scale;
      centroid.scale.y = scale;
      centroid.scale.z = scale;
      // Same color as the cluster's voxels.
      centroid.color.r = hue;
      centroid.color.g = 0.8f;
      centroid.color.b = 1.0f - hue;
      centroid.color.a = 1.0f;
      centroid.pose.position.x = cx;
      centroid.pose.position.y = cy;
      centroid.pose.position.z = cz;
      centroid.pose.orientation.w = 1.0;
      markers.markers.push_back(std::move(centroid));
    }

    ++id;
  }

  pub_->publish(markers);
}

}  // namespace frontier_detector_3d
