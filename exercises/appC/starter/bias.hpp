#pragma once
#include <cmath>
/** @brief One scalar bias increment; density has bias-unit/sqrt(s), dt>=0 seconds. */
inline double biasIncrement(double density,double dt,double unit_noise) {
  // EXERCISE(appC-1): choose the scaling that preserves variance at fixed elapsed time.
  (void)density;(void)dt;(void)unit_noise;return 0;
}
