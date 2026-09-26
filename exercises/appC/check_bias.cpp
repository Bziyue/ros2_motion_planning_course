#include "bias.hpp"
#include <iostream>
int main() {
  const double a=biasIncrement(.002,.01,1),b=biasIncrement(.002,.04,1);
  if(std::abs(a-.0002)>1e-15 || std::abs(b-2*a)>1e-15 || biasIncrement(.002,0,5)!=0) {
    std::cerr<<"FAIL: bias increment must scale with sqrt(seconds)\n";return 1;
  }
  std::cout<<"PASS: analytical scale and zero-time check\n";
}
