#pragma once
#include "motion2d/geometry/se2.hpp"
/** @brief Predict from two accepted poses; dt and horizon are positive seconds. */
inline motion2d::Pose2D predictPose(const motion2d::Pose2D & previous,
  const motion2d::Pose2D & current, double dt, double horizon)
{
  // EXERCISE(ch08-2): extrapolate odom translation and the shortest yaw increment.
  (void)previous; (void)dt; (void)horizon;
  return current;
}
