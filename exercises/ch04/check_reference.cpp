#include <iostream>
#include <cmath>
#include "reference_acceleration.hpp"

int main()
{
  const auto a = studentCircleAcceleration(2, .5, 0);
  const auto b = studentCircleAcceleration(2, 1, std::acos(-1.0) / 2);
  if ((a - Eigen::Vector2d{0, .5}).norm() > 1e-12 ||
    (b - Eigen::Vector2d{-2, 0}).norm() > 1e-12)
  {
    std::cerr << "FAIL: check direction and omega squared in the chain rule\n";
    return 1;
  }
  std::cout << "PASS: analytic reference acceleration\n";
}
