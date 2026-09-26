#include <cmath>
#include <iostream>
#include "held_duration.hpp"

int main()
{
  struct Case {long long start, end, left, right; double seconds;};
  const Case cases[]{{12000000, 17000000, 7000000, 14000000, .002},
    {12000000, 17000000, 14000000, 21000000, .003},
    {12000000, 17000000, 17000000, 21000000, 0},
    {123456789012LL, 123456789017LL, 123456789010LL, 123456789020LL, 5e-9}};
  for (const auto & c : cases) {
    if (std::abs(heldDuration(c.start, c.end, c.left, c.right) - c.seconds) > 1e-15) {
      std::cerr << "FAIL: preserve exact acquisition intervals before unit conversion\n"; return 1;
    }
  }
  std::cout << "PASS ch09-2 interval intersection\n";
}
