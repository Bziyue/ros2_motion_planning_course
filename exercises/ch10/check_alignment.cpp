#include <cmath>
#include <iostream>
#include "map_alignment.hpp"

int main()
{
  const double pi = std::acos(-1.);
  const auto transform = mapAlignment({{10,-2}, pi}, {{2,1}, pi/2});
  // R90*(2,1)=(-1,2), so the translation must be (11,-4).
  if ((transform.position-Eigen::Vector2d(11,-4)).norm() > 1e-12 ||
    std::abs(transform.yaw-pi/2) > 1e-12) {
    std::cerr << "FAIL ch10-2: solve the transform equation, do not subtract positions\n"; return 1;
  }
  std::cout << "PASS ch10-2 map to odom\n";
}
