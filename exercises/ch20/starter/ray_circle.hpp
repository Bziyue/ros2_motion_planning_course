#pragma once
#include <cmath>
#include <limits>
/** @brief EXERCISE(ch20-1): nearest nonnegative ray-circle root (host form of GPU algebra).
 * @pre d normalized, r>0. Return +inf for miss or both roots behind origin.
 */
inline double studentCircleHit(double ox,double oy,double dx,double dy,double cx,double cy,double r) {
  (void)ox;(void)oy;(void)dx;(void)dy;(void)cx;(void)cy;(void)r;
  return std::numeric_limits<double>::infinity();
}
