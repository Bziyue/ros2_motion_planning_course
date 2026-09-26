#pragma once
#include <Eigen/Core>
/** @brief Eliminate one two-dimensional neighbouring block. */
inline Eigen::Matrix2d schurBlock(const Eigen::Matrix2d & current,
 const Eigen::Matrix2d & off,const Eigen::Matrix2d & previous_inverse) {
 return current-off*previous_inverse*off.transpose();
}
