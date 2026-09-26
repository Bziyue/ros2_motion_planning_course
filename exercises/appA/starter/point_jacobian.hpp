#pragma once
#include <Eigen/Core>
/** @brief EXERCISE(appA-1): d(R(yaw)*point+translation)/d(tx,ty,yaw).
 * @details First two columns are dimensionless; last column is metres/radian.
 */
inline Eigen::Matrix<double,2,3> studentPointJacobian(double yaw,const Eigen::Vector2d & point) {
  (void)yaw;(void)point;return Eigen::Matrix<double,2,3>::Zero();
}
