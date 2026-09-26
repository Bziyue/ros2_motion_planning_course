#include "scan_point.hpp"
#include <cassert>
#include <iostream>
int main()
{
  const double pi = std::acos(-1);
  assert((studentScanPoint(2, 0, {3, 4}, pi / 2) - Eigen::Vector2d(3, 6)).norm() < 1e-12);
  assert((studentScanPoint(1, pi / 2, {3, 4}, pi / 2) - Eigen::Vector2d(2, 4)).norm() < 1e-12);
  assert((studentScanPoint(3, -pi / 2, {-1, 2}, 0) - Eigen::Vector2d(-1, -1)).norm() < 1e-12);
  std::cout << "PASS ch07-2 scan projection\n";
}
