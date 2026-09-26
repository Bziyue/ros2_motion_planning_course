#include "wheels.hpp"
#include <cmath>
#include <iostream>
/** @brief Hand-computed straight, reverse, spin and arc acceptance examples. */
int main() {
  const double cases[][4]={{.2,0,4,4},{-.2,0,-4,-4},{0,1,-4,4},{.2,.5,2,6}};
  for(const auto & c:cases) {
    const auto w=wheelRates(c[0],c[1],.05,.4);
    if(std::abs(w[0]-c[2])>1e-12 || std::abs(w[1]-c[3])>1e-12) {
      std::cerr<<"FAIL: v="<<c[0]<<", omega="<<c[1]<<'\n';return 1;
    }
  }
  std::cout<<"PASS: straight, reverse, spin, arc\n";
}
