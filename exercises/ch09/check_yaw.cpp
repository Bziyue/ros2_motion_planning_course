#include <cmath>
#include <iostream>
#include "yaw_residual.hpp"
int main()
{
  const double pi = std::acos(-1.);
  const bool ok = std::abs(yawResidual(-pi + .02, pi - .03) - .05) < 1e-12 &&
    std::abs(yawResidual(pi - .02, -pi + .03) + .05) < 1e-12 &&
    std::abs(yawResidual(.7 + 4*pi, .2) - .5) < 1e-12 &&
    std::abs(yawResidual(pi, 0) + pi) < 1e-12;
  std::cout << (ok ? "PASS" : "FAIL") << " shortest yaw innovation\n";
  return ok ? 0 : 1;
}
