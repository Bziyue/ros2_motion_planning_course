#pragma once
#include <Eigen/Core>
/** @brief D maps U=[u0,...,uN-1] to [u0,u1-u0,...], two axes per input. */
inline Eigen::MatrixXd inputDifference(int n) {
  // EXERCISE(ch18-1): add the negative two-by-two subdiagonal blocks.
  return Eigen::MatrixXd::Identity(2*n,2*n);
}
