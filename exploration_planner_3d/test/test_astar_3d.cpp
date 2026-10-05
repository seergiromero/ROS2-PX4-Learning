#include <gtest/gtest.h>

#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "exploration_planner_3d/astar_3d.hpp"

namespace
{
using exploration_planner_3d::AStar3D;
using exploration_planner_3d::Bounds3D;
using exploration_planner_3d::PlanStatus;

Bounds3D testBounds()
{
  Bounds3D bounds;
  bounds.enabled = true;
  bounds.min_x = -1.0;
  bounds.max_x = 5.0;
  bounds.min_y = -1.0;
  bounds.max_y = 3.0;
  bounds.min_z = -1.0;
  bounds.max_z = 3.0;
  return bounds;
}
}  // namespace

TEST(AStar3D, RejectsNonPositiveResolution)
{
  AStar3D::Config config;
  config.resolution = 0.0;
  EXPECT_THROW(AStar3D{config}, std::invalid_argument);
}

TEST(AStar3D, PlansThroughUnknownWhenAllowed)
{
  octomap::OcTree tree(0.25);

  AStar3D::Config config;
  config.resolution = 0.25;
  config.allow_unknown = true;
  config.footprint_radius = 0.0;
  config.footprint_height = 0.0;
  AStar3D planner(config);

  const octomap::point3d start(0.0, 1.0, 1.0);
  const octomap::point3d goal(4.0, 1.0, 1.0);
  PlanStatus status = PlanStatus::kNoPath;
  const auto path = planner.plan(tree, start, goal, testBounds(), &status);

  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(status == PlanStatus::kSuccess || status == PlanStatus::kGoalProjected);
  EXPECT_NEAR(path.front().x(), start.x(), 1e-4);
  EXPECT_NEAR(path.front().y(), start.y(), 1e-4);
  EXPECT_NEAR(path.back().x(), goal.x(), 0.25);
  EXPECT_NEAR(path.back().y(), goal.y(), 0.25);
}

TEST(AStar3D, FailsWhenUnknownBlocked)
{
  octomap::OcTree tree(0.25);

  AStar3D::Config config;
  config.resolution = 0.25;
  config.allow_unknown = false;
  config.footprint_radius = 0.0;
  config.footprint_height = 0.0;
  AStar3D planner(config);

  PlanStatus status = PlanStatus::kSuccess;
  const auto path = planner.plan(
    tree, octomap::point3d(0.0, 1.0, 1.0), octomap::point3d(4.0, 1.0, 1.0), testBounds(),
    &status);
  EXPECT_TRUE(path.empty());
  EXPECT_EQ(status, PlanStatus::kStartBlocked);
}

TEST(AStar3D, GoalInUnknownIsReachableWhenAllowed)
{
  octomap::OcTree tree(0.25);
  // Free cells around the start so the start cell is known free, goal unknown.
  for (double y = 0.0; y <= 2.0 + 1e-9; y += 0.25) {
    for (double z = 0.0; z <= 2.0 + 1e-9; z += 0.25) {
      tree.updateNode(octomap::point3d(0.0, y, z), false, true);
    }
  }

  AStar3D::Config config;
  config.resolution = 0.25;
  config.allow_unknown = true;
  config.unknown_cost = 3.0;
  config.footprint_radius = 0.0;
  config.footprint_height = 0.0;
  AStar3D planner(config);

  PlanStatus status = PlanStatus::kNoPath;
  const auto path = planner.plan(
    tree, octomap::point3d(0.0, 1.0, 1.0), octomap::point3d(4.0, 1.0, 1.0), testBounds(),
    &status);
  EXPECT_FALSE(path.empty());
  EXPECT_TRUE(status == PlanStatus::kSuccess || status == PlanStatus::kGoalProjected);
}

TEST(AStar3D, RoutesAroundObstacleAndStaysOffOccupiedCells)
{
  octomap::OcTree tree(0.25);
  // Wall at x in [1.5, 2.5], y in [-1, 2], with a gap at y in (2, 3].
  for (double x = 1.5; x <= 2.5 + 1e-9; x += 0.25) {
    for (double y = -1.0; y <= 2.0 + 1e-9; y += 0.25) {
      for (double z = -1.0; z <= 3.0 + 1e-9; z += 0.25) {
        tree.updateNode(octomap::point3d(x, y, z), true, true);
      }
    }
  }

  AStar3D::Config config;
  config.resolution = 0.25;
  config.allow_unknown = true;
  config.footprint_radius = 0.0;
  config.footprint_height = 0.0;
  config.simplify = false;
  AStar3D planner(config);

  const auto path = planner.plan(
    tree, octomap::point3d(0.0, 1.0, 1.0), octomap::point3d(4.0, 1.0, 1.0), testBounds());

  ASSERT_FALSE(path.empty());

  // The path must never enter an occupied cell (unknown cells are allowed).
  double max_y = -100.0;
  for (const auto & point : path) {
    const octomap::OcTreeNode * node = tree.search(point);
    if (node != nullptr) {
      EXPECT_FALSE(tree.isNodeOccupied(node));
    }
    max_y = std::max(max_y, static_cast<double>(point.y()));
  }
  // It must detour through the gap (y > 2).
  EXPECT_GT(max_y, 2.0);
}

TEST(AStar3D, FootprintBlocksNarrowGap)
{
  octomap::OcTree tree(0.25);
  // Wall at x = 0 spanning the whole search box in z, with a 1 m gap in y
  // between 0.75 and 1.75.
  Bounds3D bounds;
  bounds.enabled = true;
  bounds.min_x = -3.0;
  bounds.max_x = 3.0;
  bounds.min_y = -1.0;
  bounds.max_y = 3.0;
  bounds.min_z = -1.0;
  bounds.max_z = 3.0;

  for (double x = -0.5; x <= 0.5 + 1e-9; x += 0.25) {
    for (double y = -1.0; y <= 0.75 + 1e-9; y += 0.25) {
      for (double z = -1.0; z <= 3.0 + 1e-9; z += 0.25) {
        tree.updateNode(octomap::point3d(x, y, z), true, true);
      }
    }
    for (double y = 1.75; y <= 3.0 + 1e-9; y += 0.25) {
      for (double z = -1.0; z <= 3.0 + 1e-9; z += 0.25) {
        tree.updateNode(octomap::point3d(x, y, z), true, true);
      }
    }
  }

  const octomap::point3d start(-2.0, 1.25, 1.25);
  const octomap::point3d goal(2.0, 1.25, 1.25);

  // Without a footprint the 1 m gap is passable.
  AStar3D::Config thin;
  thin.resolution = 0.25;
  thin.allow_unknown = true;
  thin.footprint_radius = 0.0;
  thin.footprint_height = 0.0;
  AStar3D thin_planner(thin);
  EXPECT_FALSE(thin_planner.plan(tree, start, goal, bounds).empty());

  // A 0.6 m body needs a wider clearance than the gap offers, so the route is
  // rejected instead of squeezing through it.
  AStar3D::Config thick;
  thick.resolution = 0.25;
  thick.allow_unknown = true;
  thick.footprint_radius = 0.6;
  thick.footprint_height = 0.4;
  AStar3D thick_planner(thick);
  EXPECT_TRUE(thick_planner.plan(tree, start, goal, bounds).empty());
}

TEST(AStar3D, InflatedCellsExposeFootprintEnvelope)
{
  octomap::OcTree tree(0.25);
  tree.updateNode(octomap::point3d(0.0, 0.0, 0.0), true, true);

  Bounds3D bounds;
  bounds.enabled = true;
  bounds.min_x = -1.0;
  bounds.max_x = 1.0;
  bounds.min_y = -1.0;
  bounds.max_y = 1.0;
  bounds.min_z = -1.0;
  bounds.max_z = 1.0;

  AStar3D::Config thin;
  thin.resolution = 0.25;
  thin.footprint_radius = 0.0;
  thin.footprint_height = 0.0;
  AStar3D thin_planner(thin);
  EXPECT_TRUE(thin_planner.inflatedCells(tree, bounds).empty());

  AStar3D::Config thick;
  thick.resolution = 0.25;
  thick.footprint_radius = 0.5;
  thick.footprint_height = 0.3;
  AStar3D thick_planner(thick);
  EXPECT_FALSE(thick_planner.inflatedCells(tree, bounds).empty());
}
