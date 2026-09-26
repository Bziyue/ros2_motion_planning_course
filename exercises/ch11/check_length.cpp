#include <cmath>
#include <iostream>
#include "path_length.hpp"
int main()
{
  const bool pass = pathLength({}) == 0 && pathLength({{1, 2}}) == 0 &&
    std::abs(pathLength({{1, 2}, {4, 6}})-5) < 1e-12 &&
    std::abs(pathLength({{1, 2}, {4, 6}, {4, 6}, {4, 8}})-7) < 1e-12;
  std::cout << (pass ? "PASS" : "FAIL") << '\n'; return pass ? 0 : 1;
}
