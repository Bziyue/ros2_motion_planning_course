#pragma once
#include "motion2d/geometry/obstacles.hpp"

/** @brief Capsule and circle are disjoint only when their centreline gap is positive. */
inline bool studentSweepCircle(const Eigen::Vector2d & from, const Eigen::Vector2d & to,
  double robot_radius, const motion2d::Circle & circle)
{
  return motion2d::pointSegmentDistance(circle.center, from, to) >
         circle.radius + robot_radius;
}
