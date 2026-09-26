#pragma once
#include <array>

/** @brief Reference diagonal covariance; no cross-axis noise correlation. */
inline std::array<double, 9> studentCovariance(const std::array<double, 3> & sigma)
{
  std::array<double, 9> covariance{};
  for (int i = 0; i < 3; ++i) {covariance[4 * i] = sigma[i] * sigma[i];}
  return covariance;
}
