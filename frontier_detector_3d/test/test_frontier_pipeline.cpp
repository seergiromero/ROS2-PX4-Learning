#include <gtest/gtest.h>

#include <octomap/OcTreeKey.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "frontier_detector_3d/frontier_detector.hpp"
#include "frontier_detector_3d/frontier_pipeline.hpp"
#include "frontier_detector_3d/octomap_mapper.hpp"
#include "frontier_detector_3d/sensor_transform.hpp"

namespace
{
using frontier_detector_3d::Connectivity;
using frontier_detector_3d::FrontierDetector;
using frontier_detector_3d::FrontierPipeline;
using frontier_detector_3d::OctomapMapper;
using frontier_detector_3d::SensorTransform;

/// Inserts a solid free block at full depth (keys min..max inclusive).
void insertFreeBlock(
  octomap::OcTree & tree, const octomap::OcTreeKey & min_key,
  const octomap::OcTreeKey & max_key)
{
  for (uint16_t x = min_key.k[0]; x <= max_key.k[0]; ++x) {
    for (uint16_t y = min_key.k[1]; y <= max_key.k[1]; ++y) {
      for (uint16_t z = min_key.k[2]; z <= max_key.k[2]; ++z) {
        tree.updateNode(octomap::OcTreeKey(x, y, z), false);
      }
    }
  }
}

sensor_msgs::msg::PointCloud2 makeEmptyCloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = 0;
  cloud.point_step = 12;
  return cloud;
}

}  // namespace

TEST(FrontierPipeline, DetectsFrontiersOnTheJustUpdatedTree)
{
  OctomapMapper mapper(0.1);
  FrontierDetector detector(1, 10, Connectivity::k26);
  FrontierPipeline pipeline(mapper, detector);

  // A free block fully surrounded by unknown space: every free voxel on its
  // surface borders unknown voxels, so the pipeline must report a cluster.
  insertFreeBlock(mapper.mutableTree(), octomap::OcTreeKey(40, 40, 40),
    octomap::OcTreeKey(43, 43, 43));

  const rclcpp::Time stamp(123, 0);
  const auto result = pipeline.process(makeEmptyCloud(), SensorTransform{}, "map", stamp);

  EXPECT_EQ(result.frame_id, "map");
  EXPECT_EQ(result.stamp, stamp);
  EXPECT_FALSE(result.frontiers.empty());

  // The cluster consists of the shell of the 4x4x4 block: 4^3 - 2^3 = 56
  // voxels, matching the detector's pruned-free-leaf test.
  ASSERT_EQ(result.frontiers.size(), 1u);
  EXPECT_EQ(result.frontiers[0].size(), 56u);
}

TEST(FrontierPipeline, ReportsEmptyWhenTreeIsEmpty)
{
  OctomapMapper mapper(0.1);
  FrontierDetector detector(1, 10, Connectivity::k26);
  FrontierPipeline pipeline(mapper, detector);

  const auto result = pipeline.process(makeEmptyCloud(), SensorTransform{}, "map", rclcpp::Time(0));
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
