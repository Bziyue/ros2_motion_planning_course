#pragma once
#include <Eigen/Core>
#include <cmath>

/** @brief Reference body-frame specific force for horizontal motion. */
inline Eigen::Vector3d studentSpecificForce(
  const Eigen::Vector2d & acceleration, double yaw, double gravity)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * acceleration.x() + s * acceleration.y(),
    -s * acceleration.x() + c * acceleration.y(), gravity};
}
