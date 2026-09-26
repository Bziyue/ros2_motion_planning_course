#include <cmath>
#include <iostream>
#include "force_step.hpp"

int main()
{
  motion2d::State2D state;
  state.pose = {{1, 2}, .2};
  state.velocity = {.3, -.1};
  state.yaw_rate = .4;
  const auto next = studentForceStep(state, {{1, -.5}, .02}, 2, .05, .2);
  if ((next.pose.position - Eigen::Vector2d{1.07, 1.975}).norm() > 1e-12 ||
    (next.velocity - Eigen::Vector2d{.4, -.15}).norm() > 1e-12 ||
    std::abs(next.pose.yaw - .288) > 1e-12 || std::abs(next.yaw_rate - .48) > 1e-12 ||
    std::abs(next.acceleration.x() - .5) > 1e-12)
  {
    std::cerr << "FAIL: check force/mass, torque/inertia, old velocity and dt^2/2\n";
    return 1;
  }
  std::cout << "PASS: constant force and torque step\n";
}
