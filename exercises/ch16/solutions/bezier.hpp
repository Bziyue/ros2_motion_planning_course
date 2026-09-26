#pragma once
#include <cmath>
/** @brief Binomial ratio, including seconds-to-normalized-time scaling. */
inline double bezierFactor(int j,int k,double T) {
 double ratio=1;for(int l=0;l<k;++l) ratio*=double(j-l)/(5-l);return ratio*std::pow(T,k);
}
