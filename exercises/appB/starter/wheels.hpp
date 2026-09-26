#pragma once
#include <array>
/** @brief Return {left,right} rad/s for signed forward speed and CCW yaw rate.
 * @pre radius and track are positive metres; speed m/s, omega rad/s.
 */
inline std::array<double,2> wheelRates(double speed,double omega,double radius,double track) {
  // EXERCISE(appB-1): solve the two ideal wheel-kinematics equations.
  (void)speed;(void)omega;(void)radius;(void)track;
  return {0,0};
}
