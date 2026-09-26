#pragma once
#include <Eigen/Core>
/** @brief Unit outward normal and offset n*p<=b for a CCW polygon edge (m). */
inline Eigen::Vector3d edgeHalfspace(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  // EXERCISE(ch13-1): orient the normal outward, normalize, compute its offset.
  const Eigen::Vector2d edge = b-a;
  const Eigen::Vector2d normal(-edge.y(),edge.x());
  return {normal.x(),normal.y(),normal.dot(a)};
}
