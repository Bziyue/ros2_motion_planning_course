#pragma once
#include <cmath>

/** @brief Uniform angle with the correct full-turn/partial-FOV endpoint convention. */
inline double studentBeamAngle(int i, int beams, double angle_min, double fov)
{
  const double two_pi = 2 * std::acos(-1.0);
  const double step = std::abs(fov - two_pi) <= 1e-12 ? two_pi / beams : fov / (beams - 1);
  return angle_min + i * step;
}
