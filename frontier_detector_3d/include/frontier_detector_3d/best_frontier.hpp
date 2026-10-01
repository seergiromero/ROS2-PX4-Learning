#ifndef FRONTIER_DETECTOR_3D__BEST_FRONTIER_HPP_
#define FRONTIER_DETECTOR_3D__BEST_FRONTIER_HPP_

#include <octomap/OcTree.h>

#include <vector>

namespace frontier_detector_3d
{

/// Result of a best-frontier query.
struct BestFrontierResult
{
  /// False when there were no candidates to evaluate.
  bool valid = false;

  /// Best candidate (world frame).
  octomap::point3d point;

  /// Information gain of the best candidate.
  double gain = 0.0;
};

/// Selects the next exploration goal among frontier candidates.
///
/// This is a direct reimplementation of `BestFrontier::bestFrontierInfGain`
/// from larics/uav_frontier_exploration_3d (Batinovic et al., IEEE RA-L 2021).
/// For every candidate it estimates the amount of unknown space in a cube of
/// side `box_length` centred on the candidate and trades it against the travel
/// distance:
///
///     gain = k_gain * unknown_ratio * exp(-lambda * distance)
///
/// The candidate with the highest gain is returned. `k_gain` is a constant
/// scale factor (it does not change the argmax) and is kept for parity with the
/// reference implementation.
class BestFrontier
{
public:
  /// Creates the selector.
  ///
  /// \param[in] box_length Side of the cube (metres) used to measure unknown
  ///            space around each candidate.
  /// \param[in] k_gain Constant gain scale.
  /// \param[in] lambda Distance-decay rate (small lambda favours information
  ///            gain, large lambda favours nearby candidates).
  explicit BestFrontier(
    double box_length = 5.0, double k_gain = 100.0, double lambda = 0.1386);

  /// Picks the best candidate.
  ///
  /// \param[in] tree Occupancy tree used to estimate unknown space.
  /// \param[in] current_position Current vehicle position (world frame).
  /// \param[in] candidates Frontier candidate positions (world frame).
  /// \return The best candidate and its gain, or an invalid result when
  ///         `candidates` is empty.
  BestFrontierResult select(
    const octomap::OcTree & tree, const octomap::point3d & current_position,
    const std::vector<octomap::point3d> & candidates) const;

  /// Computes the information gain of a single candidate.
  ///
  /// \param[in] tree Occupancy tree.
  /// \param[in] current_position Current vehicle position (world frame).
  /// \param[in] candidate Candidate position (world frame).
  /// \return `k_gain * unknown_ratio * exp(-lambda * distance)`.
  double informationGain(
    const octomap::OcTree & tree, const octomap::point3d & current_position,
    const octomap::point3d & candidate) const;

  /// Estimates the fraction of unknown cells in a `box_length` cube centred on
  /// `center`, sampling on the octree's own resolution.
  ///
  /// \param[in] tree Occupancy tree.
  /// \param[in] center Cube centre (world frame).
  /// \return Unknown fraction in `[0, 1]` (0 when no sample could be taken).
  double unknownRatio(
    const octomap::OcTree & tree, const octomap::point3d & center) const;

  double boxLength() const {return box_length_;}
  double kGain() const {return k_gain_;}
  double lambda() const {return lambda_;}

private:
  double box_length_;
  double k_gain_;
  double lambda_;
};

}  // namespace frontier_detector_3d

#endif  // FRONTIER_DETECTOR_3D__BEST_FRONTIER_HPP_
