#include <gtest/gtest.h>

#include <octomap/OcTree.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstdint>
#include <array>
#include <stdexcept>
#include <vector>

#include "frontier_detector_3d/octomap_mapper.hpp"
#include "frontier_detector_3d/sensor_transform.hpp"

namespace
{
using frontier_detector_3d::OctomapMapper;
using frontier_detector_3d::SensorTransform;

/// Builds a PointCloud2 holding one XYZ point per entry in `points`.
/// Fields follow the standard x,y,z layout with a 12-byte point step.
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

bool isOccupied(const octomap::OcTree & tree, double x, double y, double z)
{
  const octomap::OcTreeNode * node = tree.search(octomap::point3d(x, y, z));
  return node != nullptr && tree.isNodeOccupied(node);
}

}  // namespace

TEST(OctomapMapper, RejectsNonPositiveResolution)
{
  EXPECT_THROW(OctomapMapper(0.0), std::invalid_argument);
  EXPECT_THROW(OctomapMapper(-0.1), std::invalid_argument);
  EXPECT_NO_THROW(OctomapMapper(0.1));
}

TEST(OctomapMapper, InsertsOccupiedEndpointsAlongRays)
{
  OctomapMapper mapper(0.1);
  const auto cloud = makeCloud({
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.5, 0.0, 0.0},
  });

  SensorTransform identity;  // sensor at the map origin, no rotation.
  ASSERT_TRUE(mapper.update(cloud, identity));
  ASSERT_GT(mapper.nodeCount(), 0u);

  // Endpoints become occupied.
  EXPECT_TRUE(isOccupied(mapper.tree(), 1.0, 0.0, 0.0));
  EXPECT_TRUE(isOccupied(mapper.tree(), 0.0, 1.0, 0.0));
  EXPECT_TRUE(isOccupied(mapper.tree(), 0.0, 0.0, 1.0));
  EXPECT_TRUE(isOccupied(mapper.tree(), 0.5, 0.0, 0.0));
}

TEST(OctomapMapper, IgnoresEmptyAndAllNaNClouds)
{
  OctomapMapper mapper(0.1);

  const auto empty = makeCloud({});
  EXPECT_FALSE(mapper.update(empty, SensorTransform{}));
  EXPECT_EQ(mapper.nodeCount(), 0u);

  // NaNs (as gazebo produces in the far plane) must be skipped, not inserted.
  const auto nan_cloud = makeCloud({{NAN, 0.0, 0.0}, {0.0, NAN, 0.0}});
  EXPECT_FALSE(mapper.update(nan_cloud, SensorTransform{}));
  EXPECT_EQ(mapper.nodeCount(), 0u);
}

TEST(OctomapMapper, HonorsMaxRange)
{
  OctomapMapper mapper(0.1, /*max_range=*/0.5);
  const auto cloud = makeCloud({{5.0, 0.0, 0.0}});
  ASSERT_TRUE(mapper.update(cloud, SensorTransform{}));

  // The endpoint is clamped to 0.5 m, so nothing is marked occupied at 5 m
  // and the clamped endpoint remains free: no-hit rays must not manufacture
  // an obstacle at the sensor's maximum range.
  EXPECT_FALSE(isOccupied(mapper.tree(), 5.0, 0.0, 0.0));
  const auto * clamped = mapper.tree().search(octomap::point3d(0.5, 0.0, 0.0));
  ASSERT_NE(clamped, nullptr);
  EXPECT_FALSE(mapper.tree().isNodeOccupied(clamped));
}

TEST(OctomapMapper, ReportsChangedKeysPerUpdate)
{
  OctomapMapper mapper(0.1);

  ASSERT_TRUE(mapper.update(makeCloud({{1.0, 0.0, 0.0}}), SensorTransform{}));
  EXPECT_FALSE(mapper.changedKeys().empty());

  // A second update starts a fresh change set.
  ASSERT_TRUE(mapper.update(makeCloud({{0.0, 0.0, 1.0}}), SensorTransform{}));
  EXPECT_FALSE(mapper.changedKeys().empty());
}

TEST(OctomapMapper, AppliesTranslation)
{
  OctomapMapper mapper(0.1);
  // Sensor is at (1, 2, 3) in the map frame. A point 1 m ahead of the sensor
  // along +x therefore lands at (2, 2, 3).
  SensorTransform t;
  t.translation = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto cloud = makeCloud({{1.0, 0.0, 0.0}});
  ASSERT_TRUE(mapper.update(cloud, t));
  EXPECT_TRUE(isOccupied(mapper.tree(), 2.0, 2.0, 3.0));
}
