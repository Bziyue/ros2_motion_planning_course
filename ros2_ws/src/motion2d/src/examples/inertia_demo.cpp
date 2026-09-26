#include <cmath>
#include <iomanip>
#include <iostream>
#include "motion2d/sim/robot_model.hpp"

/** @brief Reproducible SI-unit mass/step/braking experiment, without ROS callbacks. */
int main()
{
  using namespace motion2d;
  std::cout << "mass_kg,dt_s,x_after_1s_m,v_after_1s_mps,brake_time_s,brake_distance_m\n";
  std::cout << std::fixed << std::setprecision(6);
  for (const double mass : {1.0, 2.0}) {
    for (const double dt : {.02, .005}) {
      InertialParameters p;
      p.mass = mass;
      State2D state;
      for (int i = 0; i < std::lround(1.0 / dt); ++i) {
        state = stepInertial(state, {{1, 0}, 0}, p, dt);
      }
      // Compare braking from the SAME speed, rather than different terminal speeds.
      State2D braking;
      braking.velocity.x() = 1.0;
      for (int i = 0; i < std::lround(mass / dt); ++i) {
        braking = stepInertial(braking, {{-1, 0}, 0}, p, dt);
      }
      std::cout << mass << ',' << dt << ',' << state.pose.position.x() << ','
                << state.velocity.x() << ',' << mass << ',' << braking.pose.position.x() << '\n';
    }
  }
}
