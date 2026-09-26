#include <cmath>
#include <iostream>
#include "covariance.hpp"

int main()
{
  const auto covariance = studentCovariance({.02, .03, .04});
  const std::array<double, 9> expected{.0004, 0, 0, 0, .0009, 0, 0, 0, .0016};
  for (int i = 0; i < 9; ++i) {
    if (std::abs(covariance[i] - expected[i]) > 1e-15 ||
      studentCovariance({0, 0, 0})[i] != 0)
    {
      std::cerr << "FAIL: variance is sigma squared; diagonal indices are 0,4,8\n";
      return 1;
    }
  }
  std::cout << "PASS: row-major diagonal variances\n";
}
