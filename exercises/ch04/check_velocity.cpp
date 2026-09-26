#include <iostream>
#include <cmath>
#include "velocity_step.hpp"

int main()
{
  const auto result = studentVelocityStep({{1, 2}, motion2d::kPi - .1}, {-1, 2}, 1.0, .2);
  if ((result.position - Eigen::Vector2d{.8, 2.4}).norm() > 1e-12 ||
    std::abs(result.yaw - (-motion2d::kPi + .1)) > 1e-12)
  {
    std::cerr << "FAIL: check dt units, world-frame velocity and yaw wrapping\n";
    return 1;
  }
  std::cout << "PASS: world-frame velocity integration\n";
}
