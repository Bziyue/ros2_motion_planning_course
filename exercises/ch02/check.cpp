#include <iostream>
#include "inverse_point.hpp"

int main()
{
  const auto first = inversePoint({{1.0, 2.0}, motion2d::kPi / 2.0}, {1.0, 4.0});
  const auto second = inversePoint({{-3.0, 1.0}, -motion2d::kPi / 2.0}, {-1.0, 1.0});
  if ((first - Eigen::Vector2d{2.0, 0.0}).norm() > 1e-12 ||
    (second - Eigen::Vector2d{0.0, 2.0}).norm() > 1e-12)
  {
    std::cerr << "FAIL: inversePoint must undo translation BEFORE rotation\n";
    return 1;
  }
  std::cout << "PASS: inverse point transforms\n";
}
