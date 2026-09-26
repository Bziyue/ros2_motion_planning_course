#include <cmath>
#include <iostream>
#include "marker_scale.hpp"

int main()
{
  if (std::abs(markerDiameter(0.2) - 0.4) > 1e-12 ||
    std::abs(markerDiameter(0.35) - 0.7) > 1e-12)
  {
    std::cerr << "FAIL: expected diameters 0.40 m and 0.70 m\n";
    return 1;
  }
  std::cout << "PASS: disk diameters are correct\n";
}
