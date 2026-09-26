#pragma once
#include <motion2d_interfaces/msg/trajectory2_d.hpp>
#include "motion2d/trajectory/execution.hpp"

namespace motion2d
{
/** @brief Decode and validate odom frame, nonnegative simulation stamps, yaw and C2 pieces.
 * @throws std::invalid_argument if the message cannot represent a timed curve.
 * @details Does not authorize execution or test start-state matching/collision safety.
 */
TimedTrajectory fromTrajectoryMessage(const motion2d_interfaces::msg::Trajectory2D & message);

/** @brief Encode all local coefficients without resampling; publication time is separate.
 * @pre Nonnegative valid ROS times, finite yaw; frame is always odom.
 */
motion2d_interfaces::msg::Trajectory2D toTrajectoryMessage(const TimedTrajectory & trajectory,
  const builtin_interfaces::msg::Time & published);
}  // namespace motion2d
