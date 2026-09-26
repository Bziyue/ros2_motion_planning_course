#pragma once
#include "motion2d/geometry/se2.hpp"

namespace motion2d
{
/** @brief Disk-centre state; translation derivatives are in the world/odom frame.
 *  @details SI units: m, m/s, m/s^2, rad, rad/s, rad/s^2.
 *  Pose jumps have no physical derivatives and must not drive an IMU simulator.
 */
struct State2D
{
  Pose2D pose;
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
  double yaw_rate = 0.0;
  double yaw_acceleration = 0.0;
};

/** @brief Set an ideal pose and clear derivatives; this is not physical motion.
 *  @pre The caller has validated the pose and checked the entire swept disk.
 */
State2D idealPose(const Pose2D & target);
}  // namespace motion2d
