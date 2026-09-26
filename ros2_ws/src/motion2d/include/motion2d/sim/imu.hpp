#pragma once
#include "motion2d/sim/robot_model.hpp"

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
}  // namespace motion2d
