#include <iomanip>
#include <iostream>
#include "motion2d/sim/imu.hpp"

/** @brief Analytic ch06 cases; CSV columns use rad/s and m/s^2. */
int main()
{
  using namespace motion2d;
  std::cout << "case,wx,wy,wz,fx,fy,fz\n" << std::fixed << std::setprecision(6);
  const auto print = [](const char * name, const State2D & state) {
      const auto sample = idealImu(state);
      std::cout << name;
      for (int i = 0; i < 3; ++i) {std::cout << ',' << sample.angular_velocity[i];}
      for (int i = 0; i < 3; ++i) {std::cout << ',' << sample.specific_force[i];}
      std::cout << '\n';
    };
  State2D state;
  print("rest", state);
  state.velocity = {2.0, -1.0};
  print("constant_velocity", state);
  state.pose.yaw = kPi / 2;
  state.acceleration = {2.0, 0.0};
  print("rotated_acceleration", state);
  state = {};
  state.yaw_rate = -.3;
  print("spin_at_centre", state);
  print("circle", sampleCircle({}, 1.0, .4, 1.25));
}
