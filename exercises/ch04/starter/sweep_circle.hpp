#pragma once
#include "motion2d/geometry/obstacles.hpp"

/** @brief Check a moving disk against one circle; contact counts as collision. */
inline bool studentSweepCircle(const Eigen::Vector2d & from, const Eigen::Vector2d & to,
  double robot_radius, const motion2d::Circle & circle)
{
  // EXERCISE(ch04-1): endpoints alone miss obstacles between them.
  return motion2d::signedDistance(circle, from) > robot_radius &&
         motion2d::signedDistance(circle, to) > robot_radius;
}
