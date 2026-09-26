#pragma once
#include <Eigen/Core>
/** @brief Return d(0.5*|m*a+c*v|^2)/da, in consistent SI units. */
inline Eigen::Vector2d forceGradient(double mass,double drag,
  const Eigen::Vector2d & velocity,const Eigen::Vector2d & acceleration)
{
  // EXERCISE(ch04-5): chain the force map with the squared-norm cost.
  (void)mass;(void)drag;(void)velocity;(void)acceleration;
  return Eigen::Vector2d::Zero();
}
