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

/** @brief Held planar velocity (m/s) and yaw rate (rad/s); body_frame selects axes. */
struct VelocityCommand
{
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double yaw_rate = 0.0;
  bool body_frame = false;
};

/** @brief Limit translational norm and absolute yaw rate, preserving direction.
 *  @pre Finite command and positive finite limits, in m/s and rad/s.
 */
VelocityCommand limitVelocity(VelocityCommand command, double speed_max, double yaw_rate_max);

/** @brief Exact constant-command kinematic step, including body-frame circular arcs.
 *  @param dt Positive interval (s).
 *  @details Command transitions can jump velocity; do not use this mode for IMU fusion.
 */
State2D stepVelocity(const State2D & state, const VelocityCommand & command, double dt);

/** @brief Conservative radius padding for a curved kinematic step (m).
 *  @details Linear-interpolation error is bounded by max|a|*dt^2/8.
 *  For a body-frame command max|a|=|yaw_rate|*|velocity|; odom motion is straight.
 */
double velocitySweepPadding(const VelocityCommand & command, double dt);
}  // namespace motion2d
