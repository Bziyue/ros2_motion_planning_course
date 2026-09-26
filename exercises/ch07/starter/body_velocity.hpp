#pragma once
#include <Eigen/Core>
#include <stdexcept>

/** @brief World velocity (m/s) to body velocity, given body yaw (rad). */
inline Eigen::Vector2d studentBodyVelocity(const Eigen::Vector2d & velocity, double yaw)
{
  // EXERCISE(ch07-1): twist is expressed in Odometry.child_frame_id.
  (void)velocity; (void)yaw;
  throw std::logic_error("Complete EXERCISE(ch07-1)");
}
