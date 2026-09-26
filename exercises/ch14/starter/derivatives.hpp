#pragma once
#include <array>
/** @brief Return velocity (m/s), acceleration (m/s²) from ascending coefficients. */
inline std::array<double,2> derivatives(const std::array<double,6> & c,double t)
{
  // EXERCISE(ch14-1): derivative powers decrease, factors k and k*(k-1) appear.
  (void)c; (void)t;
  return {0,0};
}
