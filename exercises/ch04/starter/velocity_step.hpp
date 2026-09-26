#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Integrate a held odom-frame velocity (m/s) and yaw rate (rad/s) for dt seconds. */
inline motion2d::Pose2D studentVelocityStep(motion2d::Pose2D pose,
  const Eigen::Vector2d & velocity, double yaw_rate, double dt)
{
  // EXERCISE(ch04-2): update position and wrap the new yaw.
  (void)velocity;
  (void)yaw_rate;
  (void)dt;
  return pose;
}
