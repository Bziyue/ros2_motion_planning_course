#pragma once
#include "motion2d/sim/robot_model.hpp"
namespace motion2d {
/** @brief No-slip bicycle parameters (m, kg, kg/s, N, rad, rad/s).
 * @details The reference point and conservative collision-disk centre are at the
 * rear axle midpoint. This is longitudinal inertia plus ideal lateral rolling,
 * not a tire-slip or full chassis dynamics model. Radius is configured separately.
 */
struct AckermannParameters {
  double wheelbase=.3,mass=1,linear_drag=.15;
  double force_max=2,steering_max=.6,steering_rate_max=1;
};
/** @brief Rear-axle pose, signed forward speed (m/s), front steering angle (rad). */
struct AckermannState {Pose2D pose;double speed=0,steering=0;};
/** @brief Longitudinal force (N) and front steering angular velocity (rad/s). */
struct AckermannInput {double force=0,steering_rate=0;};
/** @brief Validate external constants; steering must lie strictly below pi/2. */
void validateAckermannParameters(const AckermannParameters & parameters);
/** @brief Limit held input, including a rate that keeps the whole interval in angle bounds.
 * @pre Valid finite state/input/parameters, dt>0. This limits the applied input,
 * not a pose or velocity after integration.
 */
AckermannInput limitAckermannInput(const AckermannState & state,AckermannInput input,
  const AckermannParameters & parameters,double dt);
/** @brief Integrate a held input: exact damped speed, linear steering, RK4 pose.
 * @pre Finite state/input, validated parameters, steering within its limit, dt>0.
 * @details The caller uses small ticks and verifies integration convergence.
 * Saturation is applied first. Negative plant speeds are allowed; the flat planner
 * in this course supports only the positive-speed branch.
 */
AckermannState stepAckermann(const AckermannState & state,AckermannInput input,
  const AckermannParameters & parameters,double dt);
/** @brief Analytic instantaneous p/v/a/yaw derivatives for laser/IMU simulation.
 * @pre input is the applied (already limited) force/rate, not an unclipped request.
 */
State2D ackermannKinematics(const AckermannState & state,const AckermannInput & input,
  const AckermannParameters & parameters);
/** @brief Acceleration-based chord padding, m; RK4 truncation is separately tested.
 * @details Uses maximum speed/steering over a held-input interval to bound |a|.
 */
double ackermannSweepPadding(const AckermannState & state,AckermannInput input,
  const AckermannParameters & parameters,double dt);
}
