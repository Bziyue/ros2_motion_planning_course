#pragma once
#include <Eigen/Core>
#include <cmath>
/** @brief Reference: T_odom_laser applied to a finite polar echo. */
inline Eigen::Vector2d studentScanPoint(double range, double angle,
  const Eigen::Vector2d & translation, double yaw)
{
  return translation + range * Eigen::Vector2d(std::cos(angle + yaw), std::sin(angle + yaw));
}
