#pragma once
#include <cmath>
/** @brief Jerk Gram entry for ascending seconds-based coefficients; T>0, k/l in [0,5]. */
inline double jerkEntry(int k,int l,double T)
{
  if(k<3 || l<3)return 0;
  return k*(k-1)*(k-2)*l*(l-1)*(l-2)*std::pow(T,k+l-5)/(k+l-5);
}
