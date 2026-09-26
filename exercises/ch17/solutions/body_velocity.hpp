#pragma once
#include <Eigen/Core>
#include <cmath>
/** @brief Body velocity to odom, m/s; yaw in rad. */
inline Eigen::Vector2d bodyVelocity(const Eigen::Vector2d & body,double yaw) {
  const double c=std::cos(yaw),s=std::sin(yaw);
  return {c*body.x()-s*body.y(),s*body.x()+c*body.y()};
}
