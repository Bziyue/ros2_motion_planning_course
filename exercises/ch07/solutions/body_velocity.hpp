#pragma once
#include <Eigen/Core>
#include <cmath>

/** @brief Reference world-to-body velocity rotation; translation is irrelevant. */
inline Eigen::Vector2d studentBodyVelocity(const Eigen::Vector2d & velocity, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * velocity.x() + s * velocity.y(), -s * velocity.x() + c * velocity.y()};
}
