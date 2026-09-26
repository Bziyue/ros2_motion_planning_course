#pragma once
#include <Eigen/Core>
#include <cmath>
/** @brief Body velocity to odom, m/s; yaw in rad. */
inline Eigen::Vector2d bodyVelocity(const Eigen::Vector2d & body,double yaw) {
  // EXERCISE(ch17-2): rotate the body-frame velocity by yaw.
  return body;
}
