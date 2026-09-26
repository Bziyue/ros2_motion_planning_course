#include "observed_metrics.hpp"
#include <cassert>
#include <iostream>
int main()
{
  assert((studentConfusion({-1, 50, 70, 70, 20, 20}, {1, 1, 1, 0, 1, 0}) ==
    std::array<int, 4>{1, 1, 1, 1}));
  assert((studentConfusion({-1, -1, 36, 64}, {0, 1, 1, 0}) ==
    std::array<int, 4>{0, 0, 0, 0}));
  assert((studentConfusion({35, 65, 35, 65}, {0, 1, 1, 0}) ==
    std::array<int, 4>{1, 1, 1, 1}));
  std::cout << "PASS ch07-4 observed confusion\n";
}
