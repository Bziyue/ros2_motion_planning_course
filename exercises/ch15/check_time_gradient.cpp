#include "time_gradient.hpp"
#include <cmath>
#include <iostream>
int main(){for(double lower:{.1,.3}) for(double tau:{-1.,.2,1.}) {
  const double T=lower+std::exp(tau),h=1e-6;
  auto f=[&](double x){double t=lower+std::exp(x);return 72/std::pow(t,5)+t;};
  double exact=timeGradient(1-360/std::pow(T,6),T,lower),fd=(f(tau+h)-f(tau-h))/(2*h);
  if(std::abs(exact-fd)>1e-5*std::max(1.,std::abs(fd))) return 1;
}std::cout<<"ch15-2 PASS\n";}
