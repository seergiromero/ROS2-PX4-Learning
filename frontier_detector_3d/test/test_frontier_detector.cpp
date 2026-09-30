#include <gtest/gtest.h>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>

#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <vector>

#include "frontier_detector_3d/frontier_detector.hpp"

namespace
{
using frontier_detector_3d::Connectivity;
using frontier_detector_3d::Bounds3D;
using frontier_detector_3d::FrontierDetector;

constexpr double kResolution = 0.1;

octomap::OcTree makeTree(double resolution = kResolution)
{
  return octomap::OcTree(resolution);
}

/// Inserts a free voxel block spanning `min_key`..`max_key` (inclusive) at full depth.
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

bool containsKey(
  const frontier_detector_3d::FrontierDetector::KeySet & keys,
  const octomap::OcTreeKey & target)
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

TEST(Neighbors, ReturnsCorrectCountPerConnectivity)
{
  const octomap::OcTreeKey center(100, 100, 100);

  FrontierDetector six(1, 10, Connectivity::k6);
  EXPECT_EQ(six.neighbors(center).size(), 6u);

  FrontierDetector eighteen(1, 10, Connectivity::k18);
  EXPECT_EQ(eighteen.neighbors(center).size(), 18u);

  FrontierDetector twenty_six(1, 10, Connectivity::k26);
  EXPECT_EQ(twenty_six.neighbors(center).size(), 26u);
}

TEST(Neighbors, NeverWrapsAroundAtLowerAddressBoundary)
{
  const octomap::OcTreeKey origin(0, 0, 0);

  // At the origin only the non-negative offsets survive: 2^3 - 1 = 7.
  FrontierDetector twenty_six(1, 10, Connectivity::k26);
  const auto neighbors = twenty_six.neighbors(origin);
  ASSERT_EQ(neighbors.size(), 7u);
  for (const auto & key : neighbors) {
    // A wrap-around would produce key value 65535; all valid keys must be <= 1.
    EXPECT_LE(key.k[0], 1u);
    EXPECT_LE(key.k[1], 1u);
    EXPECT_LE(key.k[2], 1u);
  }

  FrontierDetector six(1, 10, Connectivity::k6);
  EXPECT_EQ(six.neighbors(origin).size(), 3u);

  FrontierDetector eighteen(1, 10, Connectivity::k18);
  EXPECT_EQ(eighteen.neighbors(origin).size(), 6u);
}

TEST(Neighbors, NeverWrapsAroundAtUpperAddressBoundary)
{
  const octomap::OcTreeKey max_key(65535, 65535, 65535);

  FrontierDetector six(1, 10, Connectivity::k6);
  const auto neighbors = six.neighbors(max_key);
  ASSERT_EQ(neighbors.size(), 3u);
  for (const auto & key : neighbors) {
    EXPECT_NE(key, max_key);
    // Each neighbour decreases exactly one axis by 1; a wrap-around would
    // produce value 0 in that axis.
    const int decreased_axes =
      (key.k[0] == static_cast<uint16_t>(65534) ? 1 : 0) +
      (key.k[1] == static_cast<uint16_t>(65534) ? 1 : 0) +
      (key.k[2] == static_cast<uint16_t>(65534) ? 1 : 0);
    EXPECT_EQ(decreased_axes, 1) << "neighbour must decrease exactly one axis";
    EXPECT_NE(key.k[0], 0u);
    EXPECT_NE(key.k[1], 0u);
    EXPECT_NE(key.k[2], 0u);
  }
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

TEST(DetectFrontierKeys, FindsAllFreeVoxelsSurroundedByUnknown)
{
  octomap::OcTree tree = makeTree();
  insertFreeBlock(tree, octomap::OcTreeKey(5, 5, 5), octomap::OcTreeKey(6, 6, 6));

  FrontierDetector detector(1, 10, Connectivity::k26);
  const auto keys = detector.detectFrontierKeys(tree);
  EXPECT_EQ(keys.size(), 8u);
}

TEST(DetectFrontierKeys, ExpandsPrunedFreeLeafToFullDepthShell)
{
  // A 4x4x4 free block that, once pruned, collapses into a single coarse leaf.
  octomap::OcTree tree = makeTree();
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(43, 43, 43));
  tree.prune();

  // Sanity: the block really is stored as a single coarse free leaf now.
  unsigned int leaf_count = 0;
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    ++leaf_count;
    EXPECT_FALSE(tree.isNodeOccupied(*it));
    EXPECT_LT(it.getDepth(), tree.getTreeDepth());
  }
  ASSERT_EQ(leaf_count, 1u);

  // Only the 4^3 - 2^3 = 56 shell voxels border unknown space; the 8 interior
  // voxels are fully surrounded by free space. A detector that projects the
  // leaf center onto a single voxel would report exactly 1 key here.
  FrontierDetector detector(1, 10, Connectivity::k26);
  const auto keys = detector.detectFrontierKeys(tree);
  EXPECT_EQ(keys.size(), 56u);
}

TEST(DetectFrontierKeys, ExpandsLargerPrunedLeafWithoutIteratingTheVolume)
{
  // An 8x8x8 free block pruned into a single depth-13 leaf (shell of 8^3 - 6^3
  // = 296 voxels). Exercises the O(surface) shell generator on a larger node.
  octomap::OcTree tree = makeTree();
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(47, 47, 47));
  tree.prune();

  unsigned int leaf_count = 0;
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    ++leaf_count;
  }
  ASSERT_EQ(leaf_count, 1u);

  FrontierDetector detector(1, 10, Connectivity::k26);
  const auto keys = detector.detectFrontierKeys(tree);
  EXPECT_EQ(keys.size(), 296u);
}

namespace
{
/// A wall in the x=0 plane with a two-voxel doorway gap at y in [4, 5], a free
/// corridor hugging the wall at z=2, and an isolated free voxel far from any
/// surface ("air").
octomap::OcTree makeWallWithDoorwayTree()
{
  octomap::OcTree tree = makeTree();

  for (int y = 0; y < 10; ++y) {
    if (y == 4 || y == 5) {
      continue;
    }
    for (int z = 0; z <= 4; ++z) {
      tree.updateNode(octomap::OcTreeKey(0, static_cast<uint16_t>(y), static_cast<uint16_t>(z)),
        true);
    }
  }

  for (int y = 0; y < 10; ++y) {
    tree.updateNode(octomap::OcTreeKey(1, static_cast<uint16_t>(y), 2), false);
  }
  tree.updateNode(octomap::OcTreeKey(20, 20, 20), false);

  return tree;
}
}  // namespace

TEST(DetectFrontierKeys, AirVoxelIsReportedWithoutOccupancyFilter)
{
  auto tree = makeWallWithDoorwayTree();

  FrontierDetector detector(1, 10, Connectivity::k26);
  const auto keys = detector.detectFrontierKeys(tree);

  // The doorway voxels and the far-away air voxel are all free and border
  // unknown space, so without the occupancy filter they all count.
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(1, 4, 2)));
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(1, 5, 2)));
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(20, 20, 20)));
}

TEST(DetectFrontierKeys, OccupancyFilterRemovesAirVoxel)
{
  auto tree = makeWallWithDoorwayTree();

  // Frontier voxels must lie within 0.5 m of an occupied voxel.
  FrontierDetector detector(1, 10, Connectivity::k26, 0.5);
  const auto keys = detector.detectFrontierKeys(tree);

  // Doorway voxels hug the wall frame and survive...
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(1, 4, 2)));
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(1, 5, 2)));
  // ...while the air voxel ~1.5 m from the nearest surface is dropped.
  EXPECT_FALSE(containsKey(keys, octomap::OcTreeKey(20, 20, 20)));
}

TEST(DetectFrontierKeys, OccupancyFilterKeepsVoxelsTouchingWalls)
{
  auto tree = makeWallWithDoorwayTree();

  FrontierDetector detector(1, 10, Connectivity::k26, 0.5);
  const auto keys = detector.detectFrontierKeys(tree);

  // Corridor voxels directly against the wall are frontier candidates (their
  // z=1 / z=3 neighbours are unknown) and lie next to occupied cells.
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(1, 0, 2)));
  EXPECT_TRUE(containsKey(keys, octomap::OcTreeKey(1, 9, 2)));
}

TEST(DetectFrontierKeys, GroundFilterRemovesLowFrontiers)
{
  octomap::OcTree tree = makeTree();
  tree.updateNode(octomap::point3d(0.0, 0.0, 0.25), false);   // near the ground
  tree.updateNode(octomap::point3d(0.0, 0.0, 1.05), false);   // at flight altitude

  FrontierDetector without_filter(1, 10, Connectivity::k6);
  const auto all = without_filter.detectFrontierKeys(tree);
  ASSERT_EQ(all.size(), 2u);

  FrontierDetector with_filter(1, 10, Connectivity::k6, -1.0, 0.0, 0.0, 0.4);
  const auto filtered = with_filter.detectFrontierKeys(tree);
  ASSERT_EQ(filtered.size(), 1u);
  bool has_low = false;
  bool has_high = false;
  for (const auto & key : filtered) {
    const double z = tree.keyToCoord(key).z();
    if (std::abs(z - 0.25) < 1e-3) {
      has_low = true;
    }
    if (std::abs(z - 1.05) < 1e-3) {
      has_high = true;
    }
  }
  EXPECT_FALSE(has_low);
  EXPECT_TRUE(has_high);
}

TEST(ClusterFrontiers, SplitsOversizedClustersAlongPrincipalComponent)
{
  octomap::OcTree tree = makeTree();
  // A straight line of 10 frontier voxels along x, placed at cell centres so
  // each one maps to a distinct adjacent cell.
  for (int x = 0; x < 10; ++x) {
    tree.updateNode(octomap::point3d(kResolution * x + 0.5 * kResolution, 1.0, 0.5), false);
  }

  FrontierDetector no_split(1, 10, Connectivity::k6);
  const auto clusters = no_split.detect(tree);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 10u);

  // The farthest cell is 0.45 m from the line's mean, so a 0.3 m size limit
  // forces a split into two 5-cell pieces, each within the limit.
  FrontierDetector split(1, 10, Connectivity::k6, -1.0, 0.3, 10.0);
  const auto pieces = split.detect(tree);
  ASSERT_EQ(pieces.size(), 2u);
  EXPECT_EQ(pieces[0].size(), 5u);
  EXPECT_EQ(pieces[1].size(), 5u);
}

TEST(ClusterFrontiers, CornerAdjacentVoxelsMergeOnlyWithConnectivity26)
{
  octomap::OcTree tree = makeTree();
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), false);
  tree.updateNode(octomap::OcTreeKey(6, 6, 6), false);

  FrontierDetector twenty_six(1, 10, Connectivity::k26);
  const auto clusters_26 = twenty_six.detect(tree);
  ASSERT_EQ(clusters_26.size(), 1u);
  EXPECT_EQ(clusters_26[0].size(), 2u);

  FrontierDetector six(1, 10, Connectivity::k6);
  const auto clusters_6 = six.detect(tree);
  ASSERT_EQ(clusters_6.size(), 2u);
  EXPECT_EQ(clusters_6[0].size(), 1u);
  EXPECT_EQ(clusters_6[1].size(), 1u);

  FrontierDetector eighteen(1, 10, Connectivity::k18);
  const auto clusters_18 = eighteen.detect(tree);
  ASSERT_EQ(clusters_18.size(), 2u);
}

TEST(ClusterFrontiers, ReportsWorldCoordinates)
{
  octomap::OcTree tree = makeTree();
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), false);
  tree.updateNode(octomap::OcTreeKey(6, 6, 6), false);

  FrontierDetector twenty_six(1, 10, Connectivity::k26);
  const auto clusters = twenty_six.detect(tree);
  ASSERT_EQ(clusters.size(), 1u);
  ASSERT_EQ(clusters[0].points.size(), 2u);

  const double expected = std::sqrt(3.0) * kResolution;
  const double dx = clusters[0].points[1].x() - clusters[0].points[0].x();
  const double dy = clusters[0].points[1].y() - clusters[0].points[0].y();
  const double dz = clusters[0].points[1].z() - clusters[0].points[0].z();
  // octomap::point3d stores coordinates as float; with map extents of ~3277 m
  // a tolerance of 1e-3 absorbs the float rounding of the voxel centers.
  EXPECT_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), expected, 1e-3);
}

TEST(ClusterFrontiers, FiltersClustersByMinimumSize)
{
  octomap::OcTree tree = makeTree();
  // Connected blob of 3 voxels.
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), false);
  tree.updateNode(octomap::OcTreeKey(5, 6, 5), false);
  tree.updateNode(octomap::OcTreeKey(5, 5, 6), false);
  // Isolated single voxel.
  tree.updateNode(octomap::OcTreeKey(8, 8, 8), false);

  FrontierDetector strict(4, 10, Connectivity::k6);
  EXPECT_TRUE(strict.detect(tree).empty());

  FrontierDetector loose(2, 10, Connectivity::k6);
  const auto clusters = loose.detect(tree);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].size(), 3u);
}

TEST(ClusterFrontiers, SortsBySizeAndTruncatesToMaxFrontiers)
{
  octomap::OcTree tree = makeTree();
  // Blob of 3.
  tree.updateNode(octomap::OcTreeKey(5, 5, 5), false);
  tree.updateNode(octomap::OcTreeKey(5, 6, 5), false);
  tree.updateNode(octomap::OcTreeKey(5, 5, 6), false);
  // Blob of 2.
  tree.updateNode(octomap::OcTreeKey(10, 10, 10), false);
  tree.updateNode(octomap::OcTreeKey(10, 11, 10), false);
  // Blob of 1.
  tree.updateNode(octomap::OcTreeKey(8, 8, 8), false);

  FrontierDetector detector(1, 2, Connectivity::k6);
  const auto clusters = detector.detect(tree);
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].size(), 3u);
  EXPECT_EQ(clusters[1].size(), 2u);
}

TEST(Constructor, RejectsInvalidParameters)
{
  EXPECT_THROW(FrontierDetector(0, 10, Connectivity::k6), std::invalid_argument);
  EXPECT_THROW(FrontierDetector(10, 0, Connectivity::k6), std::invalid_argument);
  EXPECT_THROW(
    FrontierDetector(10, 10, static_cast<Connectivity>(7)), std::invalid_argument);
  EXPECT_NO_THROW(FrontierDetector(10, 10, Connectivity::k26));
}

TEST(Bounds, RemovesFrontiersOutsideBox)
{
  octomap::OcTree tree = makeTree();
  // Two isolated free voxels ~1 m and ~5 m along x; both border unknown space.
  tree.updateNode(octomap::point3d(1.0, 0.0, 0.5), false);
  tree.updateNode(octomap::point3d(5.0, 0.0, 0.5), false);

  Bounds3D bounds;
  bounds.enabled = true;
  bounds.min_x = 0.0;
  bounds.max_x = 2.0;
  bounds.min_y = -1.0;
  bounds.max_y = 1.0;
  bounds.min_z = 0.0;
  bounds.max_z = 2.0;

  FrontierDetector detector(
    1, 10, Connectivity::k6, -1.0, 0.0, 0.0, 0.0, bounds);
  const auto clusters = detector.detect(tree);
  ASSERT_EQ(clusters.size(), 1u);
  // Only the frontier at x ~= 1 m survives the box.
  EXPECT_NEAR(clusters[0].points[0].x(), 1.0, 0.2);
}

TEST(FrontierShape, MinFreeNeighborsRejectsIsolatedFreeSlivers)
{
  octomap::OcTree tree = makeTree();
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(43, 43, 43));
  // An isolated free voxel floating in unknown space.
  tree.updateNode(octomap::OcTreeKey(20, 20, 20), false);

  // Default: the isolated voxel borders unknown on all sides, so it is kept.
  FrontierDetector loose(1, 100, Connectivity::k26);
  const auto loose_keys = loose.detectFrontierKeys(tree);
  EXPECT_TRUE(containsKey(loose_keys, octomap::OcTreeKey(20, 20, 20)));

  // Requiring at least 2 free neighbours rejects the isolated sliver (0 free
  // neighbours) while keeping the block's frontiers (many free neighbours).
  FrontierDetector strict(
    1, 100, Connectivity::k26, -1.0, 0.0, 0.0, 0.0, Bounds3D(), 0.0, 2, 1);
  const auto strict_keys = strict.detectFrontierKeys(tree);
  EXPECT_FALSE(containsKey(strict_keys, octomap::OcTreeKey(20, 20, 20)));
  EXPECT_GT(strict_keys.size(), 0u);
}

TEST(MultiResolution, CoarserDetectionProducesFewerCells)
{
  octomap::OcTree tree = makeTree();
  // A 4x4x4 free block (0.4 m wide at 0.1 m resolution).
  insertFreeBlock(tree, octomap::OcTreeKey(40, 40, 40), octomap::OcTreeKey(43, 43, 43));

  FrontierDetector fine(1, 100, Connectivity::k26);
  const auto fine_keys = fine.detectFrontierKeys(tree);
  ASSERT_EQ(fine_keys.size(), 56u);

  // Detecting on a 0.8 m grid collapses the whole block into far fewer cells.
  FrontierDetector coarse(
    1, 100, Connectivity::k26, -1.0, 0.0, 0.0, 0.0, Bounds3D(), 0.8);
  const auto coarse_keys = coarse.detectFrontierKeys(tree);
  EXPECT_GT(coarse_keys.size(), 0u);
  EXPECT_LT(coarse_keys.size(), fine_keys.size());
}
