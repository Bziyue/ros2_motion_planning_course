#pragma once
#include <Eigen/Core>
/** @brief Derivative w.r.t. additive [tx,ty,yaw]; rotated=R(yaw)*p, in metres. */
inline Eigen::Matrix<double, 2, 3> exerciseJacobian(const Eigen::Vector2d & rotated)
{
  // EXERCISE(ch08-1): translation gives the identity; differentiate the rotation.
  (void)rotated;
  return Eigen::Matrix<double, 2, 3>::Zero();
}
