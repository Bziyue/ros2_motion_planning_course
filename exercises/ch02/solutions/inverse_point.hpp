#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Solve p_map = R p_body + t for p_body; units are metres. */
inline Eigen::Vector2d inversePoint(
  const motion2d::Pose2D & map_from_body, const Eigen::Vector2d & point_map)
{
  return motion2d::rotation(map_from_body.yaw).transpose() *
         (point_map - map_from_body.position);
}
