#pragma once
#include <Eigen/Core>

/** @brief Circle-reference acceleration in the initial-heading axes, in m/s^2. */
inline Eigen::Vector2d studentCircleAcceleration(double radius, double omega, double theta)
{
  // EXERCISE(ch04-4): differentiate r*omega*(cos(theta),sin(theta)), theta=omega*t.
  (void)radius;
  (void)omega;
  (void)theta;
  return Eigen::Vector2d::Zero();
}
