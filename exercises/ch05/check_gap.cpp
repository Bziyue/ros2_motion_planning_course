#include <cmath>
#include <iostream>
#include "beam_gap.hpp"

int main()
{
  const double pi = std::acos(-1.0);
  const auto near = [](double value, double expected) {
      return std::isfinite(value) && std::abs(value - expected) <= 1e-12;
    };
  if (!near(studentBeamGap(1, pi / 3), 1) ||
    !near(studentBeamGap(2, pi), 4) ||
    !near(studentBeamGap(5, 2 * pi / 90), .3489949670250097) ||
    studentBeamGap(0, .1) != 0)
  {
    std::cerr << "FAIL: use radians and the half-angle chord formula\n";
    return 1;
  }
  std::cout << "PASS: angular resolution converted to same-range point spacing\n";
}
