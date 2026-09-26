#include "derivatives.hpp"
#include <cmath>
#include <iostream>
int main()
{
  const auto a=derivatives({1,2,3,4,5,6},0);
  const auto b=derivatives({0,0,0,10,-15,6},.5);
  const auto c=derivatives({0,0,0,10./8,-15./16,6./32},1);
  if(std::abs(a[0]-2)>1e-12 || std::abs(a[1]-6)>1e-12 ||
     std::abs(b[0]-1.875)>1e-12 || std::abs(b[1])>1e-12 ||
     std::abs(c[0]-.9375)>1e-12 || std::abs(c[1])>1e-12) {
    std::cerr<<"FAIL derivative or time scaling\n"; return 1;
  }
  std::cout<<"PASS derivatives and seconds-based scaling\n";
}
