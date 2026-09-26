#include <iostream>
#include "sweep_circle.hpp"

int main()
{
  const motion2d::Circle circle{{0, 0}, .5};
  if (studentSweepCircle({-2, 0}, {2, 0}, .25, circle) ||
    studentSweepCircle({-2, .75}, {2, .75}, .25, circle) ||
    !studentSweepCircle({-2, .76}, {2, .76}, .25, circle) ||
    !studentSweepCircle({1, 0}, {1, 0}, .25, circle))
  {
    std::cerr << "FAIL: check the whole sweep, tangent contact, and a stationary disk\n";
    return 1;
  }
  std::cout << "PASS: disk-circle sweep\n";
}
