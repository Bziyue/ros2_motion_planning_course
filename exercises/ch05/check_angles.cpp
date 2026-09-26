#include <cmath>
#include <iostream>
#include "beam_angle.hpp"

int main()
{
  const double pi = std::acos(-1.0);
  bool correct = true;
  for (int n : {90, 360, 720}) {
    const double last = studentBeamAngle(n - 1, n, -pi, 2 * pi);
    correct = correct && std::abs(last - (pi - 2 * pi / n)) < 1e-12 &&
      studentBeamAngle(0, n, -pi, 2 * pi) == -pi;
  }
  correct = correct && std::abs(studentBeamAngle(2, 3, -pi / 4, pi / 2) - pi / 4) < 1e-12 &&
    std::abs(studentBeamAngle(1, 3, -pi / 4, pi / 2)) < 1e-12;
  if (!correct) {
    std::cerr << "FAIL: check N versus N-1 and partial-FOV endpoints\n";
    return 1;
  }
  std::cout << "PASS: uniform scan angles without a duplicate full-turn endpoint\n";
}
