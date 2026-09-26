#pragma once
#include <algorithm>
#include <Eigen/Core>

/** @brief Closest point is a + clamp(t,0,1)*(b-a); degenerate segments are points. */
inline double studentSegmentDistance(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d edge = b - a;
  if (edge.squaredNorm() == 0.0) {return (p - a).norm();}
  const double t = std::clamp((p - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
  return (p - a - t * edge).norm();
}
