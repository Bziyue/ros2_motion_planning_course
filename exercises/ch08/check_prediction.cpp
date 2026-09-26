#include <cmath>
#include <iostream>
#include "constant_velocity.hpp"
int main()
{
  const double pi = std::acos(-1.);
  const motion2d::Pose2D previous{{1, 2}, pi - .1}, current{{1.4, 1.8}, -pi + .1};
  const auto result = predictPose(previous, current, .2, .3);
  const bool ok = (result.position - Eigen::Vector2d(2, 1.5)).norm() < 1e-12 &&
    std::abs(result.yaw - (-pi + .4)) < 1e-12;
  std::cout << (ok ? "PASS" : "FAIL") << " missing-frame interval and wrap-around prediction\n";
  return ok ? 0 : 1;
}
