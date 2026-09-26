#pragma once
#include "motion2d/sim/robot_model.hpp"
namespace motion2d {
/** @brief Feedback gains: kp in s^-2, kd in s^-1; independent yaw loop. */
struct PdGains {
  Eigen::Vector2d kp{4,4},kd{3,3};
  double yaw_kp=4,yaw_kd=3;
  bool feedforward=true;
};
/** @brief Reject nonfinite or negative gains at the configuration boundary. */
void validatePdGains(const PdGains & gains);
/** @brief Both requested and actuator-limited inputs for experiment diagnostics. */
struct PdCommand {Wrench2D requested,applied;bool saturated=false;};
/** @brief Force/torque feedforward plus PD, all translational quantities in odom.
 * @pre Finite states, validated gains and model parameters.
 * @details F=m*(a_ref+kp*(p_ref-p)+kd*(v_ref-v))+c*v.
 * Physical model parameters here are controller estimates, not secretly read truth.
 */
PdCommand trackPd(const State2D & state,const State2D & reference,
  const PdGains & gains,const InertialParameters & model);
/** @brief Fresh bounded damping command for failure handling; never a stale optimizer output.
 * @param rate Positive damping rate in s^-1; force=-m*rate*v, torque=-I*rate*omega.
 * @details Does not guarantee a collision-free stop or satisfy input slew constraints.
 */
Wrench2D dampingBrake(const State2D & state,const InertialParameters & model,double rate=2);
}
