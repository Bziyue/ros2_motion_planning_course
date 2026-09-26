#pragma once
#include <limits>

/** @brief Nearest nonnegative circle root h +/- sqrt(discriminant), in metres.
 * @param h Centre projection on a unit ray, in m.
 * @param discriminant R^2 - perpendicular_distance^2, in m^2.
 * @return +infinity for a miss or two backward roots. Inputs must be finite.
 */
inline double studentForwardRoot(double h, double discriminant)
{
  // EXERCISE(ch05-1): handle a miss, a tangent, two forward roots and an inside origin.
  (void)h;
  (void)discriminant;
  return std::numeric_limits<double>::infinity();
}
