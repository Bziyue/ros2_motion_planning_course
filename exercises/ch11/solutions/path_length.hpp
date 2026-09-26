#pragma once
#include <Eigen/Core>
#include <vector>
/** @brief Total Euclidean length (m) of a polyline, including its actual endpoints. */
inline double pathLength(const std::vector<Eigen::Vector2d> & points)
{
  double length = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {length += (points[i]-points[i-1]).norm();}
  return length;
}
