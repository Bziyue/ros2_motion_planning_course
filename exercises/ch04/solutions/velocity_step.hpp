#pragma once
#include "motion2d/geometry/se2.hpp"

/** @brief Constant odom velocity gives p_next=p+v*dt; yaw is normalized. */
inline motion2d::Pose2D studentVelocityStep(motion2d::Pose2D pose,
  const Eigen::Vector2d & velocity, double yaw_rate, double dt)
{
  pose.position += velocity * dt;
  pose.yaw = motion2d::wrapAngle(pose.yaw + yaw_rate * dt);
  return pose;
}
