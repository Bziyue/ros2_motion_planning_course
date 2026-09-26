#include "motion2d/estimation/loop_closure.hpp"
#include <stdexcept>
#include <cmath>

namespace motion2d
{
LoopResult verifyLoop(const MatchTarget & current, const MatchTarget & reference,
  const Pose2D & initial, const LoopConfig & config)
{
  if (!std::isfinite(config.min_overlap) || config.min_overlap <= 0 || config.min_overlap > 1 ||
    !std::isfinite(config.max_translation_correction) || config.max_translation_correction <= 0 ||
    !std::isfinite(config.max_yaw_correction) || config.max_yaw_correction <= 0 ||
    !std::isfinite(config.cycle_translation) || config.cycle_translation <= 0 ||
    !std::isfinite(config.cycle_yaw) || config.cycle_yaw <= 0) {
    throw std::invalid_argument("invalid geometric loop gates");
  }
  LoopResult result;
  result.forward = matchClouds(current.points, reference, initial, config.match);
  result.status = matchStatusName(result.forward.status);
  if (!result.forward.accepted()) {return result;}
  result.relative = result.forward.pose;
  result.overlap = static_cast<double>(result.forward.pairs) / current.points.size();
  if (result.overlap < config.min_overlap) {result.status = "low_overlap"; return result;}
  if ((result.relative.position-initial.position).norm() > config.max_translation_correction ||
    std::abs(wrapAngle(result.relative.yaw-initial.yaw)) > config.max_yaw_correction) {
    result.status = "large_correction"; return result;
  }
  // loop_cycle_begin
  const auto backward = matchClouds(reference.points, current, inverse(result.relative), config.match);
  if (!backward.accepted() ||
    static_cast<double>(backward.pairs) / reference.points.size() < config.min_overlap) {
    result.status = "reverse_rejected"; return result;
  }
  const auto cycle = compose(result.relative, backward.pose);
  if (cycle.position.norm() > config.cycle_translation || std::abs(cycle.yaw) > config.cycle_yaw) {
    result.status = "inconsistent_cycle"; return result;
  }
  result.accepted = true; result.status = "verified";
  // loop_cycle_end
  return result;
}
}  // namespace motion2d
