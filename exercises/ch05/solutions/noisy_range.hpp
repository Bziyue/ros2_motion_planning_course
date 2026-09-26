#pragma once
#include <cmath>
#include <limits>

/** @brief Preserve absent/invalid returns; noisy finite returns outside limits become NaN. */
inline float studentNoisyRange(float range, double error, double minimum, double maximum)
{
  if (!std::isfinite(range)) {return range;}
  const double measured = range + error;
  return measured < minimum || measured > maximum ?
         std::numeric_limits<float>::quiet_NaN() : static_cast<float>(measured);
}
