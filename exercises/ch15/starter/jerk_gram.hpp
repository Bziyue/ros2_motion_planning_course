#pragma once
/** @brief Entry (k,l) of the seconds-based quintic jerk Gram matrix.
 * @pre 0<=k,l<=5, T>0. Coefficients are ascending powers.
 */
inline double jerkEntry(int k,int l,double T)
{
  // EXERCISE(ch15-1): differentiate both basis terms three times, then integrate their product.
  (void)k;(void)l;(void)T;
  return 0;
}
