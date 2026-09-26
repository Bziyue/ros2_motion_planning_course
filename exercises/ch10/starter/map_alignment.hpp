#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief T_map_odom from the same robot's map and local odom poses. */
inline motion2d::Pose2D mapAlignment(const motion2d::Pose2D & map_robot,
  const motion2d::Pose2D & odom_robot)
{
  // EXERCISE(ch10-2): solve T_map_robot = T_map_odom * T_odom_robot.
  (void)map_robot; (void)odom_robot;
  return {};
}
