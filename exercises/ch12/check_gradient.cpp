#include <cmath>
#include <iostream>
#include "bilinear_gradient.hpp"
int main()
{
  // f(x,y)=2x+3y+xy, sampled at (0,0),(.2,0),(0,.2),(.2,.2).
  const auto g = bilinearGradient({0, .4, .6, 1.04}, .3, .7, .2);
  const bool pass = (g-Eigen::Vector2d(2.14,3.06)).norm() < 1e-12 &&
    bilinearGradient({2,2,2,2}, .5, .1, .01).norm() == 0;
  std::cout << (pass ? "PASS" : "FAIL") << '\n'; return pass ? 0 : 1;
}
