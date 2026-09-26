#include <cmath>
#include <iostream>
#include "forward_root.hpp"

int main()
{
  const bool finite_cases =
    studentForwardRoot(3, 1) == 2 && studentForwardRoot(0, 1) == 1 &&
    studentForwardRoot(3, 0) == 3 && studentForwardRoot(-1, 1) == 0;
  const double behind = studentForwardRoot(-3, 1), miss = studentForwardRoot(3, -1);
  if (!finite_cases || !std::isinf(behind) || behind < 0 || !std::isinf(miss) || miss < 0) {
    std::cerr << "FAIL: require nearest forward root, including zero and inside exits\n";
    return 1;
  }
  std::cout << "PASS: circle forward-root selection\n";
}
