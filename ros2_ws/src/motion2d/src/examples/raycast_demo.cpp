#include <iostream>
#include "motion2d/sim/raycast.hpp"

/** @brief Chapter 05: hand-checkable ray distances before building a scanner. */
int main()
{
  using namespace motion2d;
  std::cout << "circle at (3,0), radius 1, ray +x: "
            << rayCircle({0, 0}, {1, 0}, {{3, 0}, 1}) << " m (expected 2)\n";
  std::cout << "wall x=4, ray +x: "
            << raySegment({0, 0}, {1, 0}, {4, -2}, {4, 2}) << " m (expected 4)\n";
  World2D world;
  world.circles.push_back({{3, 0}, 1});
  std::cout << "circle occludes boundary x=10: "
            << raycastWorld(world, {0, 0}, {1, 0}) << " m (expected 2)\n";
}
