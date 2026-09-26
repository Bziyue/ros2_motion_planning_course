#include "jerk_gram.hpp"
#include <cmath>
#include <iostream>
int main()
{
  const double c[6]={0,0,0,10,-15,6};double energy=0;
  for(int k=0;k<6;++k)for(int l=0;l<6;++l)energy+=c[k]*jerkEntry(k,l,1)*c[l];
  if(std::abs(energy-720)>1e-8 || std::abs(jerkEntry(3,4,2)-288)>1e-9 ||
    std::abs(jerkEntry(5,5,2)-23040)>1e-8 || jerkEntry(2,5,1)!=0){
    std::cerr<<"FAIL jerk factors, integration power or inactive rows\n";return 1;
  }
  std::cout<<"PASS jerk Gram and analytic rest-to-rest energy\n";
}
