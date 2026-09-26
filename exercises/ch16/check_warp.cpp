#include "warp_force.hpp"
#include <cmath>
#include <iostream>
int main() {
  const double T=3.,h=1e-6,m=1.7,c=.3,n=2.,a=.4,d=1.2/T,e=-.6/(T*T);
  const auto force=[&](double t){return m*(a*std::pow(1.2/t,2)+n*(-.6/(t*t)))+c*n*1.2/t;};
  if(std::abs(forceTimeGradient(m,c,n,a,d,e,T)-(force(T+h)-force(T-h))/(2*h))>1e-9)return 1;
  std::cout<<"ch16-3 passed\n";
}
