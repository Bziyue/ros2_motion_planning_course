#pragma once
/** @brief Measured minus predicted yaw, normalized to [-pi,pi), in rad. */
inline double yawResidual(double measured, double predicted)
{
  // EXERCISE(ch09-1): normalize the innovation, not just the two input angles.
  return measured - predicted;
}
