#include <gtest/gtest.h>

#include <octomap/octomap_types.h>

#include <vector>

#include "ros2-px4-waypoints/path_follower.hpp"

using ros2_px4_waypoints::path_follower::selectLookaheadPoint;

TEST(PathFollower, EmptyPathReturnsFalse)
{
  const std::vector<octomap::point3d> path;
  octomap::point3d target;
  EXPECT_FALSE(selectLookaheadPoint(path, octomap::point3d(0.0, 0.0, 0.0), 1.0, target));
}

TEST(PathFollower, SinglePointReturnsThatPoint)
{
  const std::vector<octomap::point3d> path = {octomap::point3d(1.0, 2.0, 3.0)};
  octomap::point3d target;
  ASSERT_TRUE(selectLookaheadPoint(path, octomap::point3d(0.0, 0.0, 0.0), 1.0, target));
  EXPECT_NEAR(target.x(), 1.0, 1e-4);
  EXPECT_NEAR(target.y(), 2.0, 1e-4);
  EXPECT_NEAR(target.z(), 3.0, 1e-4);
}

TEST(PathFollower, InterpolatesAlongSegmentAtLookahead)
{
  const std::vector<octomap::point3d> path = {
    octomap::point3d(0.0, 0.0, 0.0),
    octomap::point3d(10.0, 0.0, 0.0),
  };
  octomap::point3d target;
  ASSERT_TRUE(selectLookaheadPoint(path, octomap::point3d(0.0, 0.0, 0.0), 2.0, target));
  EXPECT_NEAR(target.x(), 2.0, 1e-3);
  EXPECT_NEAR(target.y(), 0.0, 1e-6);
  EXPECT_NEAR(target.z(), 0.0, 1e-6);
}

TEST(PathFollower, ClampsToGoalWhenLookaheadBeyondEnd)
{
  const std::vector<octomap::point3d> path = {
    octomap::point3d(0.0, 0.0, 0.0),
    octomap::point3d(1.0, 0.0, 0.0),
  };
  octomap::point3d target;
  ASSERT_TRUE(selectLookaheadPoint(path, octomap::point3d(0.0, 0.0, 0.0), 5.0, target));
  EXPECT_NEAR(target.x(), 1.0, 1e-4);
}

TEST(PathFollower, StartsFromNearestVertex)
{
  const std::vector<octomap::point3d> path = {
    octomap::point3d(0.0, 0.0, 0.0),
    octomap::point3d(5.0, 0.0, 0.0),
    octomap::point3d(10.0, 0.0, 0.0),
  };
  octomap::point3d target;
  ASSERT_TRUE(selectLookaheadPoint(path, octomap::point3d(5.1, 0.0, 0.0), 2.0, target));
  EXPECT_NEAR(target.x(), 7.0, 1e-3);
}
