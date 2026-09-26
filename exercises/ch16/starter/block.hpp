#pragma once
#include <Eigen/Core>
/** @brief EXERCISE(ch16-1): Schur block D_i = H_i - L_i H_{i-1}^{-1} L_i^T. */
inline Eigen::Matrix2d schurBlock(const Eigen::Matrix2d & current,
 const Eigen::Matrix2d & off,const Eigen::Matrix2d & previous_inverse) {
 (void)off;(void)previous_inverse;return current;
}
