#pragma once
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "motion2d/sim/imu.hpp"

namespace motion2d
{
/** @brief Raw imu_link message with diagonal variances and unavailable orientation.
 * @details All-zero gyro/accel covariance (noise-free lab) means unknown under ROS
 * semantics; do not interpret it as infinite confidence in a downstream filter.
 */
sensor_msgs::msg::Imu toImuMessage(const ImuSample & sample, const ImuNoise & noise,
  const builtin_interfaces::msg::Time & stamp);

/** @brief Display horizontal measured specific force at its acquisition pose.
 * @details The arrow uses 3 metres per m/s^2 for visibility; text reports raw z
 * force and yaw rate. Truth pose is display-only, never estimator input.
 */
visualization_msgs::msg::MarkerArray imuMarkers(
  const sensor_msgs::msg::Imu & message, const Pose2D & acquisition_pose);
}  // namespace motion2d
