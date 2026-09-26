#include <cmath>
#include <iostream>
#include "body_velocity.hpp"

int main()
{
  const double pi = std::acos(-1.);
  if (!studentBodyVelocity({2, 1}, pi / 2).isApprox(Eigen::Vector2d(1, -2)) ||
    !studentBodyVelocity({-1, 3}, pi).isApprox(Eigen::Vector2d(1, -3)) ||
    !studentBodyVelocity({.4, 0}, 0).isApprox(Eigen::Vector2d(.4, 0)))
  {
    std::cerr << "FAIL: use R^T on velocity, without a position offset\n";
    return 1;
  }
  std::cout << "PASS: Odometry body-frame twist\n";
}
