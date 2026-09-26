#pragma once
#include "motion2d/geometry/se2.hpp"
/** @brief Predict from two accepted poses; dt and horizon are positive seconds. */
inline motion2d::Pose2D predictPose(const motion2d::Pose2D & previous,
  const motion2d::Pose2D & current, double dt, double horizon)
{
  return {current.position + horizon / dt * (current.position - previous.position),
    motion2d::wrapAngle(current.yaw + horizon / dt *
    motion2d::wrapAngle(current.yaw - previous.yaw))};
}
