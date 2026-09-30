#ifndef FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_HPP_
#define FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_HPP_

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>

#include <cstddef>
#include <set>
#include <vector>

namespace frontier_detector_3d
{

/// Neighbourhood topology used to decide whether two frontier voxels belong to
/// the same cluster.
///
/// Higher connectivity merges more aggressively: voxels that only touch along
/// a corner or an edge are considered connected with 26- and 18-neighbourhoods
/// respectively, which tends to fuse distinct frontier regions.
enum class Connectivity : int
{
  k6 = 6,      ///< Face-adjacent voxels only.
  k18 = 18,    ///< Face- and edge-adjacent voxels.
  k26 = 26,    ///< Face-, edge- and corner-adjacent voxels (full 3x3x3 minus center).
};

/// Strict weak ordering over octree keys.
///
/// `octomap::OcTreeKey` does not provide an ordering, which prevents it from
/// being used directly in ordered associative containers. This comparator
/// orders keys lexicographically on (x, y, z) so that the same voxel always has
/// a single canonical representation, regardless of insertion order.
struct KeyCompare
{
  /// Compares two octree keys lexicographically on (x, y, z).
  bool operator()(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs) const;
};

/// A connected region of frontier voxels in world coordinates.
///
/// Each entry holds the voxel centers (in the map frame) that make up the
/// cluster. A cluster is a candidate boundary between explored and unexplored
/// space and is the input to a later viewpoint-selection stage.
struct Frontier
{
  /// World-frame centers of the voxels that make up the cluster.
  std::vector<octomap::point3d> points;

  /// Number of voxels in the cluster.
  std::size_t size() const noexcept {return points.size();}
};

/// Axis-aligned exploration box expressed in the map frame.
///
/// Everything outside this box is ignored, so frontiers are never published in
/// space the vehicle is not expected to visit. A disabled box keeps all
/// frontiers.
struct Bounds3D
{
  bool enabled = false;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;

  /// Returns true when the point lies inside the (inclusive) box.
  bool contains(double x, double y, double z) const
  {
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y &&
           z >= min_z && z <= max_z;
  }
};

/// Pure 3D frontier detection and clustering over an `octomap::OcTree`.
///
/// The detector is stateless: it never mutates the tree it is given and it
/// owns no ROS resources, which keeps it trivially unit-testable. The ROS node
/// is a thin wrapper that feeds deserialized trees into `detect()`.
///
/// Frontier definition
/// ===================
/// A frontier voxel is a free voxel that is adjacent to at least one unknown
/// voxel (a key for which `OcTree::search()` returns nullptr, i.e. space that
/// has never been observed). Connected frontier voxels are grouped into
/// clusters using the configured neighbourhood topology.
///
/// Multi-resolution handling
/// =========================
/// OctoMap may store a large homogeneous region as a single coarse leaf node
/// (this happens in particular when `prune()` is called after insertion, as
/// `octomap_server` does with `compress_map: true`). Only the surface shell of
/// such a node can border unknown space, so coarse free leaves are expanded to
/// full depth *along their boundary*. Interior voxels have all neighbours
/// inside the same (fully free) node and can never be frontiers, so expanding
/// them would only waste time and memory.
///
/// Without this expansion, a coarse free leaf would be collapsed to a single
/// voxel (its center), losing most of the frontier surface and producing
/// clusters whose size has no physical meaning.
///
/// Detection resolution
/// ====================
/// Detecting and clustering at the finest octree level produces a huge number
/// of cells and dominates the cost on large maps. `detection_resolution`
/// collapses the tree to coarser cells (one octree level is a factor of two)
/// before the frontier/unknown test and before clustering. This is the
/// multi-resolution strategy of Batinovic et al. (RAL 2021): a coarser
/// detection grid yields far fewer cells while keeping the same exploration
/// frontiers. The published cluster centroids then live on the coarse grid,
/// which is exactly what a waypoint-level planner needs.
///
/// Occupancy filter
/// ================
/// A free voxel bordering unknown space is not necessarily an exploration
/// frontier: with a ray-cast LiDAR map, free space is only cleared along the
/// sensor beams, so the gaps between beams and the edges of the sensor's field
/// of view produce "frontier" voxels floating in empty air that are pure
/// sensor artifacts. Setting `max_dist_to_occupied` to a positive value keeps
/// only frontier voxels that lie within that distance of an occupied voxel
/// (a real surface). Frontiers against walls, obstacles and openings survive;
/// frontiers in open air do not.
///
/// Ground filter
/// =============
/// A flying vehicle rarely needs to explore the ground plane, and the region
/// directly below the sensor is often a blind spot. Setting `ground_z` to a
/// positive value discards frontier voxels below that height.
///
/// Cluster splitting
/// =================
/// Frontiers can be large flat sheets (a whole wall, the boundary of the
/// scanned region) whose centroid is a poor navigation goal. When
/// `cluster_size_xy` and `cluster_size_z` are positive, clusters exceeding
/// those extents are recursively split along their first principal component,
/// so every published cluster stays within the size limits and its centroid
/// is a locally meaningful viewpoint.
class FrontierDetector
{
public:
  using KeySet = std::set<octomap::OcTreeKey, KeyCompare>;

  /// Creates the detector.
  ///
  /// \param[in] min_frontier_size Minimum number of voxels a cluster must have
  ///            to be kept. Smaller clusters are discarded as noise.
  /// \param[in] max_frontiers Maximum number of clusters to return, ordered by
  ///            descending size. Additional clusters are discarded.
  /// \param[in] connectivity Neighbourhood topology used for clustering.
  /// \param[in] max_dist_to_occupied Maximum distance in metres from a frontier
  ///            voxel to the nearest occupied voxel for the voxel to be kept,
  ///            or a non-positive value to disable the filter.
  /// \param[in] cluster_size_xy Maximum horizontal extent of a cluster in
  ///            metres before it is split along its first principal component,
  ///            or a non-positive value to disable splitting.
  /// \param[in] cluster_size_z Maximum vertical extent of a cluster in metres
  ///            before it is split, or a non-positive value to disable it.
  /// \param[in] ground_z Frontier voxels below this height are discarded, or a
  ///            non-positive value to disable the filter.
  /// \param[in] bounds Optional axis-aligned exploration box. Frontiers whose
  ///            representative cell lies outside are discarded.
  /// \param[in] detection_resolution Resolution in metres at which frontier
  ///            cells are detected. A non-positive value (or one not coarser
  ///            than the octree resolution) detects at the finest octree
  ///            level. Coarser values collapse the octree into larger cells
  ///            before detection and clustering, which is the main cost lever
  ///            for large maps.
  /// \param[in] min_free_neighbors Minimum number of free cells (out of the 26
  ///            neighbours) a frontier cell must have. This removes thin free
  ///            slivers in mid air (isolated laser beams) that border unknown
  ///            space but are not part of a solid observed region.
  /// \param[in] min_unknown_neighbors Minimum number of unknown cells (out of
  ///            the 26 neighbours) a frontier cell must border. This removes
  ///            free cells that only touch thin unknown slivers between laser
  ///            beams (the artefacts that carpet walls), while keeping real
  ///            frontiers that face a large unexplored region.
  /// \throws std::invalid_argument if any parameter is out of range.
  FrontierDetector(
    int min_frontier_size, int max_frontiers, Connectivity connectivity,
    double max_dist_to_occupied = -1.0, double cluster_size_xy = 0.0,
    double cluster_size_z = 0.0, double ground_z = 0.0,
    Bounds3D bounds = Bounds3D(), double detection_resolution = 0.0,
    int min_free_neighbors = 0, int min_unknown_neighbors = 1);

  /// Runs the full pipeline (frontier-key extraction + clustering) on a tree.
  ///
  /// \param[in] tree The occupancy tree to analyze (e.g. freshly deserialized
  ///            from an `octomap_msgs::msg::Octomap`).
  /// \return Clusters ordered by descending size, truncated to `max_frontiers`.
  std::vector<Frontier> detect(const octomap::OcTree & tree) const;

  /// Extracts the set of full-depth frontier voxels from the tree.
  ///
  /// A voxel is a frontier candidate when it is free (not occupied) and at
  /// least one of its neighbours (according to the configured connectivity) is
  /// unknown. Coarse free leaves are expanded along their surface shell to full
  /// depth so that cluster sizes reflect the actual boundary area.
  ///
  /// \param[in] tree The occupancy tree to analyze.
  /// \return Set of full-depth keys of free voxels adjacent to unknown space.
  KeySet detectFrontierKeys(const octomap::OcTree & tree) const;

  /// Groups connected frontier voxels into clusters via breadth-first search.
  ///
  /// Connectivity uses the configured neighbourhood topology. Visited voxels
  /// are erased from the working set so that the visited-tracking bookkeeping
  /// stays O(1) per voxel.
  ///
  /// \param[in] frontier_keys The candidate frontier keys from
  ///            `detectFrontierKeys`.
  /// \param[in] tree The tree used to convert keys back to world coordinates.
  /// \return Clusters ordered by descending size, truncated to `max_frontiers`.
  std::vector<Frontier> clusterFrontiers(
    const KeySet & frontier_keys, const octomap::OcTree & tree) const;

  /// Returns the neighbour keys of a full-depth key, honouring the configured
  /// connectivity and the octree address-space bounds.
  ///
  /// Keys that would fall outside the valid key range `[0, 65535]` are skipped
  /// instead of wrapping around.
  ///
  /// \param[in] key The center key.
  /// \return The valid neighbour keys (excluding the center itself).
  std::vector<octomap::OcTreeKey> neighbors(const octomap::OcTreeKey & key) const;

  /// Checks whether a full-depth key has no node stored in the tree.
  ///
  /// In OctoMap, unknown space is not stored explicitly: it is exactly the set
  /// of keys for which `search()` returns nullptr.
  ///
  /// \param[in] tree The occupancy tree to query.
  /// \param[in] key The full-depth key to test.
  /// \return True if the key is absent from the tree (unknown space).
  static bool isUnknownNeighbor(const octomap::OcTree & tree, const octomap::OcTreeKey & key);

private:
  /// Coarse occupancy grid used to filter frontier voxels that lie in empty
  /// air rather than against an observed surface.
  struct OccupiedGrid
  {
    double resolution = 1.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_z = 0.0;
    int nx = 1;
    int ny = 1;
    int nz = 1;
    std::set<std::size_t> cells;

    /// Encodes a coarse cell index (row-major on y then z).
    std::size_t index(int cx, int cy, int cz) const
    {
      return (static_cast<std::size_t>(cx) * static_cast<std::size_t>(ny) +
             static_cast<std::size_t>(cy)) * static_cast<std::size_t>(nz) + cz;
    }
  };

  /// Returns true if an occupied voxel lies within `grid.resolution` of a
  /// frontier cell.
  ///
  /// \param[in] coord World coordinates of the candidate frontier cell.
  /// \param[in] grid The coarse occupancy grid built from the tree's occupied
  ///            leaves.
  /// \return True if the coarse cell of `coord` or any of its 26 neighbours
  ///         contains an occupied voxel.
  static bool isNearOccupied(
    const octomap::point3d & coord, const OccupiedGrid & grid);

  /// Recursively splits `cluster` into pieces that fit the configured size
  /// limits, appending them to `pieces`.
  ///
  /// A cluster that exceeds `cluster_size_xy_` or `cluster_size_z_` is split
  /// into two groups along the first principal component of its horizontal
  /// coordinates. Degenerate splits (no progress along the principal axis) are
  /// accepted as-is to guarantee termination.
  ///
  /// \param[in] cluster The cluster to split.
  /// \param[out] pieces The resulting pieces, appended in place.
  void splitCluster(const Frontier & cluster, std::vector<Frontier> & pieces) const;

  /// Checks whether a coarse free leaf borders unknown space at its own depth.
  ///
  /// A leaf is a frontier candidate only if at least one of the 26 coarse
  /// neighbour cells (at the same tree depth) is absent from the tree. Nodes
  /// fully enclosed by stored space can never contain frontier voxels and are
  /// skipped, which avoids expanding the surface of large pruned free volumes
  /// that are entirely interior to the known map.
  ///
  /// \param[in] tree The occupancy tree to query.
  /// \param[in] node_key Key of the coarse node at its own depth.
  /// \param[in] depth Depth of the coarse node.
  /// \return True if at least one same-depth neighbour is unknown.
  bool bordersUnknownAtDepth(
    const octomap::OcTree & tree, const octomap::OcTreeKey & node_key,
    unsigned int depth) const;

  /// Returns the octree level shift that maps the tree depth to the configured
  /// detection depth (`0` when detection happens at the finest level).
  ///
  /// \param[in] tree The tree whose resolution/depth is used.
  unsigned int detectionShift(const octomap::OcTree & tree) const;

  /// Returns the neighbour keys of a key, stepping by `stride` cells.
  ///
  /// For a full-depth key (the public `neighbors`) the stride is 1. For a
  /// detection cell at a coarser level the stride is the number of finest
  /// voxels per axis spanned by one detection cell.
  ///
  /// \param[in] key The center key.
  /// \param[in] stride Key step between adjacent cells.
  std::vector<octomap::OcTreeKey> neighborsAtStride(
    const octomap::OcTreeKey & key, unsigned int stride) const;

  /// Checks whether a detection-cell key is a valid frontier cell at `depth`.
  ///
  /// A cell is a frontier when it borders at least `min_unknown_neighbors_`
  /// unknown cells and is part of at least `min_free_neighbors_` free cells
  /// (counting the 26 neighbours). The two thresholds reject thin unknown
  /// slivers between laser beams and isolated free slivers in mid air.
  ///
  /// \param[in] tree The occupancy tree to query.
  /// \param[in] cell_key A key centred at the detection level.
  /// \param[in] depth The detection depth.
  bool cellBordersUnknownAtDepth(
    const octomap::OcTree & tree, const octomap::OcTreeKey & cell_key,
    unsigned int depth) const;

  int min_frontier_size_;
  int max_frontiers_;
  Connectivity connectivity_;
  double max_dist_to_occupied_;
  double cluster_size_xy_;
  double cluster_size_z_;
  double ground_z_;
  Bounds3D bounds_;
  double detection_resolution_;
  int min_free_neighbors_;
  int min_unknown_neighbors_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__FRONTIER_DETECTOR_HPP_
