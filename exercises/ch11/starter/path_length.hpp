#pragma once
#include <Eigen/Core>
#include <vector>
/** @brief Total Euclidean length (m) of a polyline, including its actual endpoints. */
inline double pathLength(const std::vector<Eigen::Vector2d> & points)
{
  // EXERCISE(ch11-2): sum adjacent segment lengths; empty and singleton paths have length zero.
  (void)points;
  return 0.;
}
