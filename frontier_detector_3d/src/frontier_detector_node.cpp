#include "frontier_detector_3d/frontier_detector_node.hpp"

#include <octomap_msgs/octomap_msgs/conversions.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace frontier_detector_3d
{

FrontierDetectorNode::FrontierDetectorNode()
: Node("frontier_detector_3d")
{
  octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_binary");
  frontier_topic_ = declare_parameter<std::string>(
    "frontier_topic", "/exploration/frontiers");
  min_frontier_size_ = declare_parameter<int>("min_frontier_size", 10);
  max_frontiers_ = declare_parameter<int>("max_frontiers", 100);
  connectivity_ = static_cast<Connectivity>(declare_parameter<int>("connectivity", 6));
  max_dist_to_occupied_ = declare_parameter<double>("max_dist_to_occupied", 0.5);
  cluster_size_xy_ = declare_parameter<double>("cluster_size_xy", 3.0);
  cluster_size_z_ = declare_parameter<double>("cluster_size_z", 2.0);
  ground_z_ = declare_parameter<double>("ground_z", 0.4);

  // Fail fast on an invalid configuration instead of emitting empty markers.
  try {
    (void)FrontierDetector(
      min_frontier_size_, max_frontiers_, connectivity_, max_dist_to_occupied_,
      cluster_size_xy_, cluster_size_z_, ground_z_);
  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(get_logger(), "Invalid configuration: %s", e.what());
    throw;
  }

  frontier_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    frontier_topic_, rclcpp::QoS(1).transient_local());
  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::QoS(1).transient_local(),
    std::bind(&FrontierDetectorNode::octomapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Listening to %s and publishing frontiers on %s "
    "(min_frontier_size=%d, max_frontiers=%d, connectivity=%d, max_dist_to_occupied=%f, "
    "cluster_size_xy=%f, cluster_size_z=%f, ground_z=%f)",
    octomap_topic_.c_str(), frontier_topic_.c_str(), min_frontier_size_, max_frontiers_,
    static_cast<int>(connectivity_), max_dist_to_occupied_, cluster_size_xy_, cluster_size_z_,
    ground_z_);
}

void FrontierDetectorNode::octomapCallback(
  const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree{octomap_msgs::msgToMap(*msg)};
  if (!abstract_tree) {
    RCLCPP_WARN(get_logger(), "Could not deserialize the OctoMap message");
    return;
  }

  auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
  if (!tree) {
    RCLCPP_WARN(get_logger(), "Received unsupported OctoMap type: %s", msg->id.c_str());
    return;
  }

  const FrontierDetector detector(
    min_frontier_size_, max_frontiers_, connectivity_, max_dist_to_occupied_,
    cluster_size_xy_, cluster_size_z_, ground_z_);
  const auto frontier_keys = detector.detectFrontierKeys(*tree);
  const auto frontiers = detector.clusterFrontiers(frontier_keys, *tree);
  publishMarkers(msg->header.frame_id, frontiers, tree->getResolution());

  RCLCPP_DEBUG(
    get_logger(), "Detected %zu frontier clusters from %zu frontier voxels",
    frontiers.size(), frontier_keys.size());
}

void FrontierDetectorNode::publishMarkers(
  const std::string & frame_id,
  const std::vector<Frontier> & frontiers,
  double resolution)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = now();
  clear.ns = "frontiers";
  clear.id = -1;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (size_t index = 0; index < frontiers.size(); ++index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.ns = "frontiers";
    marker.id = static_cast<int32_t>(index);
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = static_cast<float>((index * 0.37) - std::floor(index * 0.37));
    marker.color.g = 0.8f;
    marker.color.b = 1.0f - marker.color.r;
    marker.color.a = 1.0f;

    for (const auto & point : frontiers[index].points) {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.x();
      marker_point.y = point.y();
      marker_point.z = point.z();
      marker.points.push_back(marker_point);
    }
    markers.markers.push_back(std::move(marker));
  }

  frontier_pub_->publish(markers);
}

}  // namespace frontier_detector_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<frontier_detector_3d::FrontierDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("frontier_detector_3d"), "Failed to start node: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
