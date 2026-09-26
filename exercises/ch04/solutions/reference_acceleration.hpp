#pragma once
#include <cmath>
#include <Eigen/Core>

/** @brief Chain rule adds a second factor of omega; centripetal acceleration. */
inline Eigen::Vector2d studentCircleAcceleration(double radius, double omega, double theta)
{
  return radius * omega * omega * Eigen::Vector2d{-std::sin(theta), std::cos(theta)};
}
