#pragma once
#include "motion2d/sim/robot_model.hpp"
#include <random>

namespace motion2d
{
/** @brief Raw IMU measurement in imu_link; no orientation or position estimate. */
struct ImuSample
{
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  ///< rad/s.
  Eigen::Vector3d specific_force = Eigen::Vector3d::Zero();    ///< m/s^2, includes +g at rest.
};

/**
 * @brief Sample an ideal IMU mounted at the disk centre, aligned with base_link.
 * @param state Continuous physical state; acceleration is expressed in odom.
 * @param gravity Positive gravitational acceleration magnitude in m/s^2.
 * @pre Finite state and gravity > 0; horizontal motion with roll=pitch=0.
 * @details Computes \f$f=R^\top(a-g)\f$, with \f$g=(0,0,-gravity)\f$.
 * Yaw is used only inside the simulator and is never returned as a measurement.
 * Velocity jumps/pose teleports do not define a physical IMU input.
 */
ImuSample idealImu(const State2D & state, double gravity = 9.81);

/** @brief Independent per-axis, per-sample standard deviations; bias is zero.
 * @details Units are rad/s and m/s^2, NOT continuous noise densities.
 */
struct ImuNoise
{
  Eigen::Vector3d gyro_stddev{.002, .003, .004};
  Eigen::Vector3d accel_stddev{.04, .05, .06};
};

/** @brief Reject negative/nonfinite sigmas or an unrepresentable variance. */
void validateImuNoise(const ImuNoise & noise);

/** @brief Add six independent zero-mean Gaussian samples, without clipping.
 * @pre Validated noise; finite ideal sample.
 * @details A zero-sigma axis consumes no random number. The caller owns the
 * independent IMU engine; reseeding replays samples on the same standard library.
 * No cached normal-distribution state persists across calls or resets.
 */
ImuSample addImuNoise(ImuSample sample, const ImuNoise & noise, std::mt19937 & random);
}  // namespace motion2d
