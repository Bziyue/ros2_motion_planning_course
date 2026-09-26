#pragma once
#include <array>
/** @brief Ideal inverse wheel kinematics; inputs m/s, rad/s and positive metres. */
inline std::array<double,2> wheelRates(double speed,double omega,double radius,double track) {
  return {(speed-omega*track/2)/radius,(speed+omega*track/2)/radius};
}
