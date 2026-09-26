#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Convert a map-frame point (m) back to the robot frame. */
inline Eigen::Vector2d inversePoint(
  const motion2d::Pose2D & map_from_body, const Eigen::Vector2d & point_map)
{
  // EXERCISE(ch02-1): First undo translation, then undo rotation.
  (void)map_from_body;
  return point_map;
}
