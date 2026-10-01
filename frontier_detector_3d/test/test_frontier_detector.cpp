#include <gtest/gtest.h>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "frontier_detector_3d/frontier_detector.hpp"
#include "frontier_detector_3d/mean_shift.hpp"

namespace
{
using frontier_detector_3d::Bounds3D;
using frontier_detector_3d::FrontierDetector;
using frontier_detector_3d::MeanShift;

geometry_msgs::msg::Point makePoint(float x, float y, float z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

constexpr double kResolution = 0.1;

octomap::OcTree makeTree(double resolution = kResolution)
{
  return octomap::OcTree(resolution);
}

/// Inserts a free voxel block spanning `min_key`..`max_key` (inclusive) at full
/// depth. `lazy_eval = true` skips OctoMap's automatic pruning so the tree keeps
/// one leaf per voxel, which is what the full-scan tests assume.
void insertFreeBlock(
  octomap::OcTree & tree, const octomap::OcTreeKey & min_key,
  const octomap::OcTreeKey & max_key)
{
  for (uint16_t x = min_key.k[0]; x <= max_key.k[0]; ++x) {
    for (uint16_t y = min_key.k[1]; y <= max_key.k[1]; ++y) {
      for (uint16_t z = min_key.k[2]; z <= max_key.k[2]; ++z) {
        tree.updateNode(octomap::OcTreeKey(x, y, z), false, true);
      }
    }
  }
}

bool containsKey(
  const FrontierDetector::KeySet & keys, const octomap::OcTreeKey & target)
{
  return keys.find(target) != keys.end();
}

}  // namespace

TEST(KeyCompare, OrdersLexicographically)
{
  const frontier_detector_3d::KeyCompare compare;
  const octomap::OcTreeKey a(0, 1, 0);
  const octomap::OcTreeKey b(1, 0, 0);
  const octomap::OcTreeKey c(1, 0, 0);

  EXPECT_TRUE(compare(a, b));      // x dominates the ordering.
  EXPECT_FALSE(compare(b, a));     // Strict weak ordering.
  EXPECT_FALSE(compare(c, c));     // A key is never "less" than itself.
  EXPECT_FALSE(compare(b, c));     // Equivalent keys compare equal both ways.
  EXPECT_FALSE(compare(c, b));
}

TEST(Neighbors, ReturnsFull26Neighbourhood)
{
  const octomap::OcTreeKey center(100, 100, 100);
  const FrontierDetector detector;

  EXPECT_EQ(detector.neighbors(center).size(), 26u);
}

TEST(Neighbors, NeverWrapsAroundAtAddressBoundaries)
{
  const FrontierDetector detector;

  // At the origin only the non-negative offsets survive: 2^3 - 1 = 7.
  EXPECT_EQ(detector.neighbors(octomap::OcTreeKey(0, 0, 0)).size(), 7u);
  // Same at the upper corner.
  EXPECT_EQ(detector.neighbors(octomap::OcTreeKey(65535, 65535, 65535)).size(), 7u);
}

TEST(IsUnknownNeighbor, DistinguishesUnknownFromKnown)
{
  octomap::OcTree tree = makeTree();
  const octomap::OcTreeKey known(5, 5, 5);
  const octomap::OcTreeKey unknown(6, 6, 6);
  tree.updateNode(known, false);

  EXPECT_FALSE(FrontierDetector::isUnknownNeighbor(tree, known));
  EXPECT_TRUE(FrontierDetector::isUnknownNeighbor(tree, unknown));
}

TEST(FrontierCell, ShellOfFreeBlockAreFrontiers)
{
  octomap::OcTree tree = makeTree();
  // 4x4x4 free block: 4^3 - 2^3 = 56 shell voxels border unknown space, the 8
  // interior voxels are fully surrounded by free space.
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(43, 43, 43));

  FrontierDetector detector;
  const auto keys = detector.rebuildFrontierKeys(tree);
  EXPECT_EQ(keys.size(), 56u);
  EXPECT_FALSE(containsKey(keys, octomap::OcTreeKey(41, 41, 41)));
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(40, 41, 41)));
}

TEST(FrontierCell, TouchingAnObstacleDisqualifiesTheCell)
{
  octomap::OcTree tree = makeTree();
  // A small free pair, one of which touches an occupied voxel.
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(6, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(7, 5, 5), true, true);

  // (5,5,5) is free, borders unknown and free space and no obstacle.
  EXPECT_TRUE(FrontierDetector::isFrontierCell(tree, octomap::OcTreeKey(5, 5, 5)));
  // (6,5,5) borders the occupied (7,5,5), so it is dropped.
  EXPECT_FALSE(FrontierDetector::isFrontierCell(tree, octomap::OcTreeKey(6, 5, 5)));
}

TEST(FrontierCell, OccupiedCellIsNeverAFrontier)
{
  octomap::OcTree tree = makeTree();
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), true, true);
  EXPECT_FALSE(FrontierDetector::isFrontierCell(tree, octomap::OcTreeKey(5, 5, 5)));
}

TEST(Incremental, KeepsAndPrunesStoredFrontierCells)
{
  octomap::OcTree tree = makeTree();
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(6, 5, 5), false, true);

  FrontierDetector detector;
  const std::vector<octomap::OcTreeKey> changed = {
    octomap::OcTreeKey(5, 5, 5), octomap::OcTreeKey(6, 5, 5)};
  auto keys = detector.findFrontierKeys(tree, changed);
  EXPECT_EQ(keys.size(), 2u);

  // (7,5,5) becomes occupied, which disqualifies its neighbour (6,5,5).
  tree.updateNode(octomap::OcTreeKey(7, 5, 5), true, true);
  keys = detector.findFrontierKeys(tree, {});
  EXPECT_FALSE(containsKey(keys, octomap::OcTreeKey(6, 5, 5)));
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(5, 5, 5)));
}

TEST(ParentKeys, ProjectionCollapsesFineCells)
{
  octomap::OcTree tree = makeTree();
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(43, 43, 43));

  FrontierDetector fine(16, 1.0);   // finest level: one parent per frontier cell.
  fine.rebuildFrontierKeys(tree);
  EXPECT_EQ(fine.parentKeys(tree).size(), 56u);

  // One level coarser merges groups of 2x2x2 frontier cells into one parent.
  FrontierDetector coarse(15, 1.0);
  coarse.rebuildFrontierKeys(tree);
  const auto coarse_parents = coarse.parentKeys(tree);
  EXPECT_GT(coarse_parents.size(), 0u);
  EXPECT_LT(coarse_parents.size(), 56u);
}

TEST(MeanShift, SeparatesPointsFartherThanBandwidth)
{
  const std::vector<MeanShift::Point> points = {
    makePoint(0.0f, 0.0f, 0.0f),
    makePoint(0.1f, 0.0f, 0.0f),
    makePoint(10.0f, 0.0f, 0.0f),
    makePoint(10.1f, 0.0f, 0.0f),
  };

  MeanShift mean_shift;
  const auto clusters = mean_shift.cluster(points, 1.0);
  EXPECT_EQ(clusters.size(), 2u);
  std::size_t total = 0;
  for (const auto & cluster : clusters) {
    total += cluster.original_points.size();
  }
  EXPECT_EQ(total, points.size());
}

TEST(MeanShift, MergesPointsWithinBandwidth)
{
  const std::vector<MeanShift::Point> points = {
    makePoint(0.0f, 0.0f, 0.0f),
    makePoint(0.2f, 0.0f, 0.0f),
    makePoint(0.4f, 0.0f, 0.0f),
  };

  MeanShift mean_shift;
  const auto clusters = mean_shift.cluster(points, 1.0);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].original_points.size(), 3u);
}

TEST(Detect, FullScanProducesClustersWithRepresentative)
{
  octomap::OcTree tree = makeTree();
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(43, 43, 43));

  FrontierDetector detector(16, 0.5);
  const auto clusters = detector.detect(tree);
  ASSERT_FALSE(clusters.empty());

  std::size_t total = 0;
  for (const auto & cluster : clusters) {
    total += cluster.size();
    EXPECT_FALSE(cluster.points.empty());
    // The representative is one of the cluster cells.
    bool found = false;
    for (const auto & point : cluster.points) {
      if ((point - cluster.representative).norm() < 1e-6) {
        found = true;
      }
    }
    EXPECT_TRUE(found);
  }
  EXPECT_EQ(total, 56u);
}

TEST(MinFrontierSize, DropsSmallClusters)
{
  octomap::OcTree tree = makeTree();
  // A 3-cell frontier region and a 2-cell one, ~5 m apart so mean-shift keeps
  // them as two separate modes.
  tree.updateNode(octomap::OcTreeKey(0, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(1, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(2, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(50, 5, 5), false, true);
  tree.updateNode(octomap::OcTreeKey(51, 5, 5), false, true);

  FrontierDetector keep_all(16, 0.5, Bounds3D(), 1);
  EXPECT_EQ(keep_all.detect(tree).size(), 2u);

  FrontierDetector min_three(16, 0.5, Bounds3D(), 3);
  const auto clusters = min_three.detect(tree);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 3u);
}

TEST(Bounds, RemovesFrontiersOutsideBox)
{
  octomap::OcTree tree = makeTree();
  // Two free pairs ~1 m and ~5 m along x; both border unknown space.
  tree.updateNode(octomap::point3d(1.0, 0.0, 0.5), false, true);
  tree.updateNode(octomap::point3d(1.1, 0.0, 0.5), false, true);
  tree.updateNode(octomap::point3d(5.0, 0.0, 0.5), false, true);
  tree.updateNode(octomap::point3d(5.1, 0.0, 0.5), false, true);

  Bounds3D bounds;
  bounds.enabled = true;
  bounds.min_x = 0.0;
  bounds.max_x = 2.0;
  bounds.min_y = -1.0;
  bounds.max_y = 1.0;
  bounds.min_z = 0.0;
  bounds.max_z = 2.0;

  FrontierDetector detector(16, 0.5, bounds);
  const auto clusters = detector.detect(tree);
  ASSERT_EQ(clusters.size(), 1u);
  // Only the frontier near x ~= 1 m survives the box.
  EXPECT_NEAR(clusters[0].representative.x(), 1.0, 0.3);
}

TEST(PoseFilter, DropsCellsNearVehicleAndOffAltitude)
{
  octomap::OcTree tree = makeTree();
  // Near the vehicle in XY (dropped by the radius).
  tree.updateNode(octomap::OcTreeKey(0, 0, 0), false, true);
  tree.updateNode(octomap::OcTreeKey(1, 0, 0), false, true);
  // Far in XY, at flight altitude (kept).
  tree.updateNode(octomap::OcTreeKey(50, 0, 0), false, true);
  tree.updateNode(octomap::OcTreeKey(51, 0, 0), false, true);
  // Far in XY but high above (dropped by the height band).
  tree.updateNode(octomap::OcTreeKey(50, 0, 50), false, true);
  tree.updateNode(octomap::OcTreeKey(51, 0, 50), false, true);

  FrontierDetector detector(16, 0.5, Bounds3D(), 1, 1.0, 0.6);
  const std::vector<octomap::OcTreeKey> changed = {
    octomap::OcTreeKey(0, 0, 0), octomap::OcTreeKey(1, 0, 0),
    octomap::OcTreeKey(50, 0, 0), octomap::OcTreeKey(51, 0, 0),
    octomap::OcTreeKey(50, 0, 50), octomap::OcTreeKey(51, 0, 50)};

  // The vehicle sits at the world coordinate of key (0,0,0); octree keys map to
  // coordinates centred on the tree's address space, so this is where the near
  // pair is.
  const octomap::point3d vehicle = tree.keyToCoord(octomap::OcTreeKey(0, 0, 0));
  const auto clusters = detector.detect(tree, changed, &vehicle);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 2u);

  const auto & chosen = clusters[0].representative;
  const double dx = chosen.x() - vehicle.x();
  const double dy = chosen.y() - vehicle.y();
  const double dz = chosen.z() - vehicle.z();
  EXPECT_GT(std::hypot(dx, dy), 1.0);   // beyond the exclusion radius
  EXPECT_LT(std::abs(dz), 0.6);         // inside the altitude band

  // The pose filter is a temporary view: after the vehicle moves to the
  // previously visible pair, that pair must reappear without a new map change.
  const octomap::point3d moved_vehicle =
    tree.keyToCoord(octomap::OcTreeKey(50, 0, 0));
  const auto moved_clusters = detector.detect(tree, {}, &moved_vehicle);
  ASSERT_EQ(moved_clusters.size(), 1u);
  EXPECT_EQ(moved_clusters[0].size(), 2u);
  const double moved_dx = moved_clusters[0].representative.x() - moved_vehicle.x();
  EXPECT_GT(std::abs(moved_dx), 1.0);
}

TEST(Constructor, RejectsNonPositiveBandwidth)
{
  EXPECT_THROW(FrontierDetector(16, 0.0), std::invalid_argument);
  EXPECT_THROW(FrontierDetector(16, -1.0), std::invalid_argument);
  EXPECT_THROW(FrontierDetector(16, 1.0, Bounds3D(), 0), std::invalid_argument);
  EXPECT_NO_THROW(FrontierDetector(15, 1.0));
}
