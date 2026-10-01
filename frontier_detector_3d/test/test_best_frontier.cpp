#include <gtest/gtest.h>

#include <octomap/OcTree.h>

#include <vector>

#include "frontier_detector_3d/best_frontier.hpp"

namespace
{
using frontier_detector_3d::BestFrontier;

constexpr double kResolution = 0.1;

/// Inserts a solid free block centred on `center` with half-extent `half`.
void insertFreeCube(octomap::OcTree & tree, double center, double half)
{
  for (double x = center - half; x <= center + half; x += kResolution) {
    for (double y = -half; y <= half; y += kResolution) {
      for (double z = -half; z <= half; z += kResolution) {
        tree.updateNode(octomap::point3d(x, y, z), false, true);
      }
    }
  }
}

}  // namespace

TEST(BestFrontier, UnknownRatioIsOneOnEmptyTree)
{
  octomap::OcTree tree(kResolution);
  const BestFrontier best(0.5, 100.0, 0.1386);
  EXPECT_NEAR(best.unknownRatio(tree, octomap::point3d(0.0, 0.0, 0.0)), 1.0, 1e-9);
}

TEST(BestFrontier, UnknownRatioDropsWhenBoxIsFilledWithFreeCells)
{
  octomap::OcTree tree(kResolution);
  insertFreeCube(tree, 0.0, 0.35);

  const BestFrontier best(0.5, 100.0, 0.1386);
  EXPECT_NEAR(best.unknownRatio(tree, octomap::point3d(0.0, 0.0, 0.0)), 0.0, 1e-9);
}

TEST(BestFrontier, SelectPrefersCloserCandidateWithEqualUnknown)
{
  octomap::OcTree tree(kResolution);  // empty: everything is unknown
  const BestFrontier best(0.5, 100.0, 0.1386);

  const std::vector<octomap::point3d> candidates = {
    octomap::point3d(1.0, 0.0, 0.0),
    octomap::point3d(5.0, 0.0, 0.0),
  };

  const auto result = best.select(tree, octomap::point3d(0.0, 0.0, 0.0), candidates);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.point.x(), 1.0, 1e-6);
}

TEST(BestFrontier, SelectPrefersCandidateWithMoreUnknown)
{
  octomap::OcTree tree(kResolution);
  // Fill the cube around the +x candidate with free cells; the -x candidate
  // stays in unknown space. Both are 2 m from the vehicle.
  insertFreeCube(tree, 2.0, 0.35);

  const BestFrontier best(0.5, 100.0, 0.1386);
  const std::vector<octomap::point3d> candidates = {
    octomap::point3d(2.0, 0.0, 0.0),   // surrounded by free cells
    octomap::point3d(-2.0, 0.0, 0.0),  // surrounded by unknown
  };

  const auto result = best.select(tree, octomap::point3d(0.0, 0.0, 0.0), candidates);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.point.x(), 0.0);
}

TEST(BestFrontier, SelectReportsInvalidWithoutCandidates)
{
  octomap::OcTree tree(kResolution);
  const BestFrontier best(0.5, 100.0, 0.1386);
  EXPECT_FALSE(best.select(tree, octomap::point3d(0.0, 0.0, 0.0), {}).valid);
}
