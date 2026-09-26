#pragma once
#include <nav_msgs/msg/odometry.hpp>
#include "motion2d/sim/robot_model.hpp"

namespace motion2d
{
/** @brief Atomic simulation truth: world pose and ground_truth_base-frame twist.
 * @param stamp State/acquisition time, not callback arrival time.
 * @details Zero covariance means exact simulated state, not a real sensor claim.
 * No TF is broadcast by this conversion. See ch07 for the explicit truth adapter.
 */
nav_msgs::msg::Odometry truthOdometry(
  const State2D & state, const builtin_interfaces::msg::Time & stamp);
}  // namespace motion2d
