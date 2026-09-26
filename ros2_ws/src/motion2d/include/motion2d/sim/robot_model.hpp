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

/** @brief Physical parameters: kg, kg/s, kg*m^2, kg*m^2/s, N and N*m. */
struct InertialParameters
{
  double mass = 1.0;
  double linear_drag = 0.0;
  double inertia_z = 0.02;
  double angular_drag = 0.0;
  double force_max = 2.0;   ///< Separate symmetric bound on each odom force axis.
  double torque_max = 0.2;
};

/** @brief Applied planar force in odom (N) and yaw torque (N*m). */
struct Wrench2D
{
  Eigen::Vector2d force = Eigen::Vector2d::Zero();
  double torque = 0.0;
};

/** @brief Validate external physical parameters once; throw on invalid values. */
void validateInertialParameters(const InertialParameters & parameters);

/** @brief Apply per-axis force and absolute torque limits; no speed clipping.
 *  @pre Valid parameters and a finite input.
 */
Wrench2D limitWrench(Wrench2D command, const InertialParameters & parameters);

/**
 * @brief Exact zero-order-hold step of linear damped translation and yaw dynamics.
 * @param dt Positive duration (s); command and parameters stay constant in this interval.
 * @pre Finite state/command and validated parameters.
 * @details \f$m\dot v=F-c_vv,\ I_z\dot\omega=\tau-c_\omega\omega\f$.
 * Saturation is applied before integration. Acceleration is evaluated at the
 * new state under the applied command, ready for later sensor simulation.
 */
State2D stepInertial(const State2D & state, const Wrench2D & command,
  const InertialParameters & parameters, double dt);

/** @brief Radius padding for the full curved inertial step, in metres.
 *  @details Constant input and nonnegative isotropic drag give
 *  \f$a(t)=a(0)e^{-c_vt/m}\f$; max|a|=|a(0)|, so pad by |a(0)|*dt^2/8.
 */
double inertialSweepPadding(const State2D & state, const Wrench2D & command,
  const InertialParameters & parameters, double dt);

/**
 * @brief Sample an ideal circular reference with mutually consistent p/v/a/yaw.
 * @param initial Pose at time zero; initial velocity is radius*omega along its x axis.
 * @param radius Path-circle radius (m), distinct from the robot's collision radius.
 * @param omega Signed reference angular frequency (rad/s).
 * @param time Time since the trial began (s).
 * @pre Finite inputs, radius > 0, time >= 0.
 * @details This is prescribed ideal motion, not force-limited tracking. In axes
 * aligned with initial yaw, displacement is (r*sin(omega*t), r*(1-cos(omega*t))).
 * Derivatives are analytic; resetting starts a new already-moving trial.
 */
State2D sampleCircle(const Pose2D & initial, double radius, double omega, double time);
}  // namespace motion2d
