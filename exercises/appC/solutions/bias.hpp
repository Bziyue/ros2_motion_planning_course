#pragma once
#include <cmath>
/** @brief Brownian bias increment, with one independent standard-normal draw. */
inline double biasIncrement(double density,double dt,double unit_noise) {
  return density*std::sqrt(dt)*unit_noise;
}
