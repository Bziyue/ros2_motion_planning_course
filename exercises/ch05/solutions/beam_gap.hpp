#pragma once
#include <cmath>

/** @brief Exact same-range chord; small-angle approximation is distance*angle_step. */
inline double studentBeamGap(double distance, double angle_step)
{
  return 2 * distance * std::sin(angle_step / 2);
}
