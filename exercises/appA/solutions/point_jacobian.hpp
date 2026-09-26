#pragma once
#include <Eigen/Core>
#include <cmath>
/** @brief Analytic additive world-translation/yaw perturbation, not a body SE(2) perturbation. */
inline Eigen::Matrix<double,2,3> studentPointJacobian(double yaw,const Eigen::Vector2d & point) {
  Eigen::Matrix<double,2,3> jacobian;
  jacobian<<1.,0.,-std::sin(yaw)*point.x()-std::cos(yaw)*point.y(),
            0.,1., std::cos(yaw)*point.x()-std::sin(yaw)*point.y();
  return jacobian;
}
