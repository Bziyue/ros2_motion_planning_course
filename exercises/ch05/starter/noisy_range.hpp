#pragma once

/** @brief Add a supplied noise error (m), preserve special values, invalidate outside limits.
 * @pre Finite min/max with 0<=min<max, finite error; valid encoded input.
 */
inline float studentNoisyRange(float range, double error, double minimum, double maximum)
{
  // EXERCISE(ch05-3): never turn an invalid/no-return beam into a made-up obstacle.
  (void)error;
  (void)minimum;
  (void)maximum;
  return range;
}
