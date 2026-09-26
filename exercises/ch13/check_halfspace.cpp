#include <iostream>
#include "halfspace.hpp"
int main()
{
  const auto bottom = edgeHalfspace({1,2},{4,2});
  const auto diagonal = edgeHalfspace({4,2},{7,6});
  const bool pass = (bottom-Eigen::Vector3d(0,-1,-2)).norm()<1e-12 &&
    (diagonal-Eigen::Vector3d(.8,-.6,2)).norm()<1e-12;
  std::cout << (pass ? "PASS" : "FAIL") << '\n'; return pass ? 0 : 1;
}
