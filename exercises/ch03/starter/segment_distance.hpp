#pragma once
#include <Eigen/Core>

/** @brief Distance in metres from p to the closed segment [a,b]. */
inline double studentSegmentDistance(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  // EXERCISE(ch03-1): Project onto the segment and clamp the projection to [0,1].
  (void)b;
  return (p - a).norm();
}
