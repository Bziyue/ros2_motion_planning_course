#include <cmath>
#include <iostream>
#include "specific_force.hpp"

int main()
{
  const double pi = std::acos(-1.0);
  if (!studentSpecificForce({0, 0}, 0, 9.81).isApprox(Eigen::Vector3d(0, 0, 9.81)) ||
    !studentSpecificForce({2, 0}, pi / 2, 9.81).isApprox(Eigen::Vector3d(0, -2, 9.81)) ||
    !studentSpecificForce({2, 0}, -pi / 2, 1.62).isApprox(Eigen::Vector3d(0, 2, 1.62)) ||
    !studentSpecificForce({1, -3}, pi, 9.81).isApprox(Eigen::Vector3d(-1, 3, 9.81)))
  {
    std::cerr << "FAIL: check rotation direction and gravity sign\n";
    return 1;
  }
  std::cout << "PASS: rest, rotated acceleration, gravity and units\n";
}
