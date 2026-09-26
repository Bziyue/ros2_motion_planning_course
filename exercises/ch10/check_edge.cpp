#include <cmath>
#include <iostream>
#include "edge_residual.hpp"

int main()
{
  const double pi = std::acos(-1.);
  // Rotated from frame: global (0,2) is local (2,0); measurement is (1,0).
  const auto r = edgeResidual({{1, 2}, pi/2}, {{1, 4}, pi/2}, {{1, 0}, 0});
  const auto yaw = edgeResidual({{}, pi-.01}, {{}, -pi+.02}, {});
  if ((r-Eigen::Vector3d(1, 0, 0)).norm() > 1e-12 || std::abs(yaw.z()-.03) > 1e-12) {
    std::cerr << "FAIL ch10-1: frame order or yaw wrapping\n"; return 1;
  }
  std::cout << "PASS ch10-1 edge residual\n";
}
