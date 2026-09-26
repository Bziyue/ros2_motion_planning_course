#include <cmath>
#include <iostream>
#include "segment_distance.hpp"

int main()
{
  const bool correct =
    std::abs(studentSegmentDistance({1, 1}, {0, 0}, {2, 0}) - 1) < 1e-12 &&
    std::abs(studentSegmentDistance({3, 0}, {0, 0}, {2, 0}) - 1) < 1e-12 &&
    std::abs(studentSegmentDistance({0, 2}, {0, 0}, {0, 0}) - 2) < 1e-12 &&
    std::abs(studentSegmentDistance({1, 0}, {0, 0}, {2, 0})) < 1e-12;
  if (!correct) {
    std::cerr << "FAIL: check interior projection, endpoints and a=b\n";
    return 1;
  }
  std::cout << "PASS: point-to-segment distance\n";
}
