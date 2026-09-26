#include <cmath>
#include <iostream>
#include <limits>
#include "noisy_range.hpp"

int main()
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  if (studentNoisyRange(5, .1, 1, 10) != 5.1F ||
    !std::isnan(studentNoisyRange(1, -.1, 1, 10)) ||
    !std::isnan(studentNoisyRange(10, .1, 1, 10)) ||
    studentNoisyRange(2, -1, 1, 10) != 1 ||
    studentNoisyRange(inf, -100, 1, 10) != inf ||
    !std::isnan(studentNoisyRange(nan, 1, 1, 10)))
  {
    std::cerr << "FAIL: add noise, preserve special values, reject out-of-range measurements\n";
    return 1;
  }
  std::cout << "PASS: noisy range encoding\n";
}
