#pragma once
#include "motion2d/estimation/scan_matcher.hpp"

namespace motion2d
{
/** @brief Conservative local place verification; distances m, angles rad. */
struct LoopConfig
{
  MatchConfig match;
  double min_overlap = .55;
  double max_translation_correction = .5, max_yaw_correction = .25;
  double cycle_translation = .03, cycle_yaw = .02;
  LoopConfig()
  {
    match.metric = MatchMetric::PointToLine;
    match.association_distance = .35; match.max_rmse = .05;
  }
};

/** @brief Explicit loop decision; a close odometry pose alone is never accepted. */
struct LoopResult
{
  bool accepted = false;
  std::string status;
  Pose2D relative; ///< T_reference_current obtained from geometry, not odometry.
  double overlap = 0;
  MatchResult forward;
};

/** @brief Bidirectional point-to-line verification with overlap and cycle checks.
 * @param current Current local-frame cloud and normals.
 * @param reference Candidate old local-frame cloud and normals.
 * @param initial Odometry-based T_reference_current; local search only.
 * @details Reject weak overlap, degeneracy, large corrections, residuals and
 * inconsistent forward/backward matching. Repeated geometry can still fool it;
 * this is NOT global relocalization or proof of place identity.
 */
LoopResult verifyLoop(const MatchTarget & current, const MatchTarget & reference,
  const Pose2D & initial, const LoopConfig & config = {});
}  // namespace motion2d
