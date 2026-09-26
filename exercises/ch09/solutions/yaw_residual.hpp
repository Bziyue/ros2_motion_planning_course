#pragma once
#include <cmath>
/** @brief Measured minus predicted yaw, normalized to [-pi,pi), in rad. */
inline double yawResidual(double measured, double predicted)
{
  const double pi = std::acos(-1.);
  double value = std::remainder(measured - predicted, 2 * pi);
  return value >= pi ? value - 2 * pi : value;
}
