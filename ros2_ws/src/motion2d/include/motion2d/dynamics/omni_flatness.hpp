#pragma once
#include "motion2d/sim/robot_model.hpp"

namespace motion2d
{
/** @brief Derivatives of flat output (p_x,p_y,yaw), in odom and SI units.
 * @details Position/yaw themselves pass straight through to the state. Yaw must
 * be locally unwrapped; it is independent of the direction of planar velocity.
 */
struct OmniFlatInput
{
  Eigen::Vector2d velocity=Eigen::Vector2d::Zero();
  Eigen::Vector2d acceleration=Eigen::Vector2d::Zero();
  Eigen::Vector2d jerk=Eigen::Vector2d::Zero();
  double yaw_rate=0, yaw_acceleration=0;
};
/** @brief Unsaturated flat-map output, or its cost partials in a reverse call.
 * @details Force (N), force rate (N/s), yaw torque (N*m). Horizontal force is
 * NOT quadrotor collective thrust: this supported planar model has no gravity term.
 */
struct OmniFlatOutput
{
  Eigen::Vector2d force=Eigen::Vector2d::Zero();
  Eigen::Vector2d force_rate=Eigen::Vector2d::Zero();
  double torque=0;
};
/** @brief Recover physical inputs from flat-output derivatives, without clipping.
 * @pre Validated inertial parameters; finite derivatives. See ch04 and ch15.
 * @details \f$F=m\ddot p+c_v\dot p,\ \dot F=m p^{(3)}+c_v\ddot p\f$.
 * Clipping here would hide violations from the trajectory optimizer.
 */
OmniFlatOutput omniForward(const OmniFlatInput & input,const InertialParameters & parameters);
/** @brief Vector-Jacobian product of omniForward, not a numerical difference.
 * @param output_gradient Direct cost partials with respect to force/rate/torque.
 * @return Partials with respect to v/a/jerk/yaw rate/yaw acceleration.
 * @pre Validated fixed parameters. Parameter gradients are not optimized here.
 */
OmniFlatInput omniBackward(const OmniFlatOutput & output_gradient,const InertialParameters & parameters);
}  // namespace motion2d
