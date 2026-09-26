#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Residual of measured^-1 * from^-1 * to; translation m, wrapped yaw rad. */
inline Eigen::Vector3d edgeResidual(const motion2d::Pose2D & from,
  const motion2d::Pose2D & to, const motion2d::Pose2D & measured)
{
  // EXERCISE(ch10-1): compose three transforms in the stated order.
  (void)from; (void)to; (void)measured;
  return Eigen::Vector3d::Zero();
}
