#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Translation/yaw coordinates of the SE(2) edge error. */
inline Eigen::Vector3d edgeResidual(const motion2d::Pose2D & from,
  const motion2d::Pose2D & to, const motion2d::Pose2D & measured)
{
  const auto error = motion2d::compose(motion2d::inverse(measured),
    motion2d::compose(motion2d::inverse(from), to));
  return {error.position.x(), error.position.y(), error.yaw};
}
