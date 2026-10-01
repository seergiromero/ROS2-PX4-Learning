#include "frontier_detector_3d/frontier_detector_node.hpp"

#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace frontier_detector_3d
{

FrontierDetectorNode::FrontierDetectorNode()
: Node("frontier_detector_3d")
{
  using namespace std::chrono_literals;

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  const std::string cloud_topic = declare_parameter<std::string>(
    "cloud_topic", "/lidar_3d/points");
  const std::string frontier_topic = declare_parameter<std::string>(
    "frontier_topic", "/exploration/frontiers");
  const double process_rate_hz = declare_parameter<double>("process_rate_hz", 2.0);
  use_latest_transform_ = declare_parameter<bool>("use_latest_transform", false);

  // --- OctoMap model (standard OctoMap parameters) -----------------------
  const double resolution = declare_parameter<double>("resolution", 0.5);
  const double max_range = declare_parameter<double>("sensor_model.max_range", 10.0);
  const bool compress_map = declare_parameter<bool>("compress_map", true);
  const int point_subsample = declare_parameter<int>("point_subsample", 1);
  publish_map_ = declare_parameter<bool>("publish_map", true);

  // --- Frontier detection (Batinovic et al., RA-L 2021) -------------------
  // The paper only exposes the multi-resolution octree level, the mean-shift
  // bandwidth and the exploration bounding box. Everything else is fixed by the
  // algorithm (26-neighbourhood, free+unknown-no-occupied test, incremental
  // update), so it is deliberately not configurable here.
  const int exploration_depth = declare_parameter<int>("exploration_depth", 16);
  const double kernel_bandwidth =
    declare_parameter<double>("clustering.kernel_bandwidth", 1.0);
  // Noise filter (not in the paper): clusters smaller than this many parent
  // cells are dropped. 1 disables it and restores the reference behaviour.
  const int min_frontier_size = declare_parameter<int>("min_frontier_size", 1);
  const unsigned int detection_depth =
    static_cast<unsigned int>(std::max(exploration_depth, 0));

  // Exploration box. Disabled unless explicitly requested, so the detector can
  // still be used in unbounded outdoor scenarios.
  Bounds3D bounds;
  bounds.enabled = declare_parameter<bool>("bounds_enabled", false);
  bounds.min_x = declare_parameter<double>("min_x", -6.0);
  bounds.max_x = declare_parameter<double>("max_x", 6.0);
  bounds.min_y = declare_parameter<double>("min_y", -4.0);
  bounds.max_y = declare_parameter<double>("max_y", 4.0);
  bounds.min_z = declare_parameter<double>("min_z", 0.3);
  bounds.max_z = declare_parameter<double>("max_z", 3.5);

  // --- Visualization -------------------------------------------------------
  const bool publish_voxels = declare_parameter<bool>("publish_voxels", false);
  const bool publish_centroids = declare_parameter<bool>("publish_centroids", true);

  // Fail fast on an invalid configuration instead of emitting empty markers.
  try {
    (void)FrontierDetector(detection_depth, kernel_bandwidth, bounds, min_frontier_size);
    mapper_ = std::make_unique<OctomapMapper>(
      resolution, max_range, 0.7, 0.4, 0.12, 0.97, compress_map,
      static_cast<std::size_t>(point_subsample));
  } catch (const std::invalid_argument & e) {
    RCLCPP_FATAL(get_logger(), "Invalid configuration: %s", e.what());
    throw;
  }

  pipeline_ = std::make_unique<FrontierPipeline>(
    *mapper_, FrontierDetector(detection_depth, kernel_bandwidth, bounds, min_frontier_size));

  // Physical size of one detection cell: the octomap leaf doubled once per
  // level between the leaf and `exploration_depth`.
  const int shift = std::max(
    0, static_cast<int>(mapper_->tree().getTreeDepth()) - static_cast<int>(detection_depth));
  frontier_cell_size_ = resolution * std::pow(2.0, static_cast<double>(shift));
  visualizer_ = std::make_unique<FrontierVisualizer>(
    *this, frontier_topic, publish_voxels, publish_centroids);

  // OctoMap output. transient_local lets late subscribers (e.g. RViz started
  // after this node) still receive the current map.
  octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
    "octomap_binary", rclcpp::QoS(1).transient_local());

  // TF plumbing for the cloud.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribe to the configured cloud topic (best effort, sensor data).
  // `subscribe()` keeps the topic configurable at runtime via `cloud_topic`.
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS(),
    std::bind(&FrontierDetectorNode::cloudCallback, this, std::placeholders::_1));

  // Throttled processing: at most `process_rate_hz` full map+frontier cycles.
  const auto period = std::chrono::duration<double>(1.0 / std::max(process_rate_hz, 0.1));
  process_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&FrontierDetectorNode::processCycle, this));

  RCLCPP_INFO(
    get_logger(),
    "Listening to %s (frame %s), publishing frontiers on %s "
    "(process_rate_hz=%.1f, resolution=%.2f, point_subsample=%d, "
    "exploration_depth=%u, kernel_bandwidth=%.2f, min_frontier_size=%d, "
    "bounds_enabled=%d, publish_map=%s)",
    cloud_topic.c_str(), map_frame_.c_str(), frontier_topic.c_str(), process_rate_hz,
    resolution, point_subsample, detection_depth, kernel_bandwidth, min_frontier_size,
    static_cast<int>(bounds.enabled), publish_map_ ? "true" : "false");
}

void FrontierDetectorNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "cloud received: %zux%zu, %zu bytes, frame=%s, is_dense=%d",
    static_cast<std::size_t>(cloud->width), static_cast<std::size_t>(cloud->height),
    cloud->data.size(), cloud->header.frame_id.c_str(), static_cast<int>(cloud->is_dense));
  std::lock_guard<std::mutex> lock(latest_mutex_);
  latest_cloud_ = cloud;
}

void FrontierDetectorNode::processCycle()
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    cloud = latest_cloud_;
  }
  if (!cloud) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "no cloud available yet, waiting...");
    return;
  }

  // Prefer the transform at the cloud timestamp. Applying the latest pose to
  // an old cloud smears the map when the vehicle moves and makes frontiers
  // appear to jump. `use_latest_transform` is an explicit compatibility
  // escape hatch for systems whose sensor and TF clocks cannot be aligned.
  geometry_msgs::msg::TransformStamped sensor_to_map;
  try {
    const rclcpp::Time query_time = use_latest_transform_ ?
      rclcpp::Time(0) : rclcpp::Time(cloud->header.stamp);
    sensor_to_map = tf_buffer_->lookupTransform(
      map_frame_, cloud->header.frame_id, query_time,
      rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No transform %s -> %s at cloud time yet, skipping cycle: %s",
      map_frame_.c_str(), cloud->header.frame_id.c_str(), ex.what());
    return;
  }

  const auto result = pipeline_->process(
    *cloud, FrontierPipeline::toSensorTransform(sensor_to_map),
    map_frame_, cloud->header.stamp);

  if (publish_map_) {
    publishOctomap(cloud->header.stamp);
  }
  visualizer_->publish(
    result.frontiers, result.frame_id, result.stamp, frontier_cell_size_);

  // One-line per cycle so the operator can see exactly where the pipeline is.
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "cloud %zux%zu (%s) -> %zu octomap nodes, %zu frontier clusters",
    static_cast<std::size_t>(cloud->width), static_cast<std::size_t>(cloud->height),
    cloud->header.frame_id.c_str(), mapper_->nodeCount(), result.frontiers.size());
}

void FrontierDetectorNode::publishOctomap(const rclcpp::Time & stamp)
{
  octomap_msgs::msg::Octomap map;
  map.header.frame_id = map_frame_;
  map.header.stamp = stamp;
  if (!octomap_msgs::binaryMapToMsg(mapper_->tree(), map)) {
    RCLCPP_ERROR(get_logger(), "Failed to serialize the OctoMap");
    return;
  }
  octomap_pub_->publish(map);
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
