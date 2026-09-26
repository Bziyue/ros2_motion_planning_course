#pragma once
#include <Eigen/Core>
/** @brief D maps U=[u0,...,uN-1] to [u0,u1-u0,...], two axes per input. */
inline Eigen::MatrixXd inputDifference(int n) {
  Eigen::MatrixXd d=Eigen::MatrixXd::Identity(2*n,2*n);
  for(int k=1;k<n;++k) d.block<2,2>(2*k,2*(k-1))=-Eigen::Matrix2d::Identity();
  return d;
}
