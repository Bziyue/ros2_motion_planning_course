#include "grid_traversal.hpp"
#include <cassert>
#include <iostream>
int main()
{
  assert((studentRayCells({.5, .5}, {3.5, .5}, 5) == std::vector<int>{0, 1, 2, 3}));
  assert((studentRayCells({.5, .5}, {.5, 3.5}, 5) == std::vector<int>{0, 5, 10, 15}));
  assert((studentRayCells({.5, .5}, {3.5, 3.5}, 5) == std::vector<int>{0, 6, 12, 18}));
  assert((studentRayCells({3.5, 3.5}, {.5, .5}, 5) == std::vector<int>{18, 12, 6, 0}));
  assert((studentRayCells({.5, 1.5}, {1, 1}, 5) == std::vector<int>{5, 6}));
  assert((studentRayCells({.5, .5}, {.6, .6}, 5) == std::vector<int>{0}));
  std::cout << "PASS ch07-3 grid traversal\n";
}
