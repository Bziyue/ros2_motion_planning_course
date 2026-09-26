#pragma once
#include <array>
#include <stdexcept>

/** @brief Row-major diagonal covariance from valid nonnegative x/y/z sigmas. */
inline std::array<double, 9> studentCovariance(const std::array<double, 3> & sigma)
{
  // EXERCISE(ch06-2): squared units, zero off-diagonal elements.
  (void)sigma;
  throw std::logic_error("Complete EXERCISE(ch06-2)");
}
