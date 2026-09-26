#pragma once
#include <Eigen/Core>
/** @brief Derivative w.r.t. additive [tx,ty,yaw]; rotated=R(yaw)*p, in metres. */
inline Eigen::Matrix<double, 2, 3> exerciseJacobian(const Eigen::Vector2d & rotated)
{
  Eigen::Matrix<double, 2, 3> j;
  j << 1, 0, -rotated.y(), 0, 1, rotated.x();
  return j;
}
