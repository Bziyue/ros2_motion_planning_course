#pragma once
#include <Eigen/Core>
/** @brief Unit outward normal and offset n*p<=b; valid nonzero CCW polygon edge. */
inline Eigen::Vector3d edgeHalfspace(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d edge = b-a;
  const Eigen::Vector2d normal = Eigen::Vector2d(edge.y(),-edge.x()).normalized();
  return {normal.x(),normal.y(),normal.dot(a)};
}
