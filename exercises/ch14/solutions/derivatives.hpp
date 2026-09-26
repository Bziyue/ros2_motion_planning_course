#pragma once
#include <array>
/** @brief Evaluate scalar quintic velocity and acceleration at local time t (s).
 * @pre Coefficients ascend in powers of seconds; inputs are finite.
 * @return {velocity (m/s), acceleration (m/s²)}.
 */
inline std::array<double,2> derivatives(const std::array<double,6> & c,double t)
{
  return {c[1]+t*(2*c[2]+t*(3*c[3]+t*(4*c[4]+t*5*c[5]))),
          2*c[2]+t*(6*c[3]+t*(12*c[4]+t*20*c[5]))};
}
