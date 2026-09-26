#pragma once
#include <cmath>
#include <limits>

/** @brief Select the nearest nonnegative root; negative discriminant means a miss. */
inline double studentForwardRoot(double h, double discriminant)
{
  const double miss = std::numeric_limits<double>::infinity();
  if (discriminant < 0) {return miss;}
  const double root = std::sqrt(discriminant);
  if (h - root >= 0) {return h - root;}
  return h + root >= 0 ? h + root : miss;
}
