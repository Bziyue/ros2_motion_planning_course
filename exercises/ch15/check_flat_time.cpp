#include "flat_time.hpp"
#include <cmath>
#include <iostream>
int main() {
  // j(t)=t^2, cost=0.5*j^2; analytic direct time derivative includes snap=2*t.
  const double T=1.7,s=.4,w=.5,h=1e-6,t=s*T;const int K=24;
  auto cost=[&](double time){return w*time/K*.5*std::pow(s*time,4);};
  const double fd=(cost(T+h)-cost(T-h))/(2*h);
  const double actual=movingTimePartial(w,.5*std::pow(t,4),K,T/K,s,0,0,2*std::pow(t,3));
  if(std::abs(actual-fd)>1e-9){std::cerr<<"missing weight or snap chain\n";return 1;}
  std::cout<<"ch15-3 passed\n";
}
