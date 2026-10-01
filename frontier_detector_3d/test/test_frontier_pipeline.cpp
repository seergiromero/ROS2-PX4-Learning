#include <gtest/gtest.h>

#include <octomap/OcTreeKey.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "frontier_detector_3d/frontier_detector.hpp"
#include "frontier_detector_3d/frontier_pipeline.hpp"
#include "frontier_detector_3d/octomap_mapper.hpp"
#include "frontier_detector_3d/sensor_transform.hpp"

namespace
{
using frontier_detector_3d::FrontierDetector;
using frontier_detector_3d::FrontierPipeline;
using frontier_detector_3d::OctomapMapper;
using frontier_detector_3d::SensorTransform;

/// Builds a PointCloud2 holding one XYZ point per entry in `points`.
sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;

  sensor_msgs::msg::PointField fx;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fx.count = 1;
  sensor_msgs::msg::PointField fy = fx;
  fy.name = "y";
  fy.offset = 4;
  sensor_msgs::msg::PointField fz = fx;
  fz.name = "z";
  fz.offset = 8;
  cloud.fields = {fx, fy, fz};

  cloud.data.resize(points.size() * cloud.point_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    float * p = reinterpret_cast<float *>(cloud.data.data() + i * cloud.point_step);
    p[0] = points[i][0];
    p[1] = points[i][1];
    p[2] = points[i][2];
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeEmptyCloud()
{
  return makeCloud({});
}

}  // namespace

TEST(FrontierPipeline, DetectsFrontiersOnTheJustUpdatedTree)
{
  OctomapMapper mapper(0.5);
  FrontierDetector detector(15, 1.0);
  FrontierPipeline pipeline(mapper, detector);

  // A single ray clears free space up to an occupied endpoint. The free cells
  // along the beam border unknown space, so the pipeline must report a cluster
  // built from the cells this very update changed.
  const auto cloud = makeCloud({{5.0f, 0.0f, 0.0f}});

  const rclcpp::Time stamp(123, 0);
  const auto result = pipeline.process(cloud, SensorTransform{}, "map", stamp);

  EXPECT_EQ(result.frame_id, "map");
  EXPECT_EQ(result.stamp, stamp);
  EXPECT_FALSE(result.frontiers.empty());
}

TEST(FrontierPipeline, ReportsEmptyWhenTreeIsEmpty)
{
  OctomapMapper mapper(0.5);
  FrontierDetector detector(15, 1.0);
  FrontierPipeline pipeline(mapper, detector);

  const auto result = pipeline.process(
    makeEmptyCloud(), SensorTransform{}, "map", rclcpp::Time(0));
  EXPECT_TRUE(result.frontiers.empty());
  EXPECT_EQ(result.frame_id, "map");
}

TEST(FrontierPipeline, ConverterRoundTripsTransformStamped)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = 1.0;
  msg.transform.translation.y = 2.0;
  msg.transform.translation.z = 3.0;
  msg.transform.rotation.w = 1.0;  // identity quaternion

  const auto st = FrontierPipeline::toSensorTransform(msg);
  EXPECT_NEAR(st.translation.x(), 1.0, 1e-9);
  EXPECT_NEAR(st.translation.y(), 2.0, 1e-9);
  EXPECT_NEAR(st.translation.z(), 3.0, 1e-9);

  const Eigen::Vector3d p = st.apply(Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_NEAR(p.x(), 1.0, 1e-9);
  EXPECT_NEAR(p.y(), 2.0, 1e-9);
  EXPECT_NEAR(p.z(), 3.0, 1e-9);
}
