#include "bezier.hpp"
#include <cmath>
#include <iostream>
int main(){double T=2.;for(int j=0;j<=5;++j){if(std::abs(bezierFactor(j,0,T)-1)>1e-12)return 1;
 if(j>0 && std::abs(bezierFactor(j,1,T)-j*T/5)>1e-12)return 1;
 if(j>1 && std::abs(bezierFactor(j,2,T)-j*(j-1)*T*T/20)>1e-12)return 1;}
 if(std::abs(bezierFactor(5,5,T)-32)>1e-12)return 1;std::cout<<"ch16-2 PASS\n";}
