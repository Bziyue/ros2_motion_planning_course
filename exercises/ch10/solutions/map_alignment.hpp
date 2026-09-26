#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Global correction without overwriting the original local odometry. */
inline motion2d::Pose2D mapAlignment(const motion2d::Pose2D & map_robot,
  const motion2d::Pose2D & odom_robot)
{
  return motion2d::compose(map_robot, motion2d::inverse(odom_robot));
}
