#pragma once
#include "motion2d/sim/ackermann_model.hpp"
namespace motion2d {
/** @brief Rear-axle flat derivatives in world/odom: m/s, m/s^2, m/s^3. */
struct AckermannFlatInput {
  Eigen::Vector2d velocity=Eigen::Vector2d::Zero(),acceleration=Eigen::Vector2d::Zero(),jerk=Eigen::Vector2d::Zero();
};
/** @brief Forward-branch physical state/input quantities, or their direct cost partials.
 * @details SI units: m/s, rad, rad/s, 1/m, rad, rad/s, N, m/s^2, m/s^2.
 * Longitudinal acceleration is signed; lateral acceleration positive to the left.
 */
struct AckermannFlatOutput {
  double speed=0,yaw=0,yaw_rate=0,curvature=0,steering=0,steering_rate=0,force=0;
  double longitudinal_acceleration=0,lateral_acceleration=0;
};
/** @brief Recover the bicycle state and force/rate inputs from p derivatives.
 * @throws std::domain_error for a nonfinite or near-zero tangent; no epsilon clamp.
 * @pre Validated fixed parameters, minimum_speed>0. Only forward motion is represented.
 * @details At a stop, yaw/steering cannot in general be inferred from this jet.
 * The stop-to-stop path parameterization in ch16 retains a regular geometric tangent.
 */
AckermannFlatOutput ackermannForward(const AckermannFlatInput & input,
  const AckermannParameters & parameters,double minimum_speed=1e-6);
/** @brief Analytic vector-Jacobian product for every field of ackermannForward.
 * @pre Same regular branch as forward; yaw differentiation is local (not across its wrap).
 * @return Direct partials with respect to velocity, acceleration and jerk.
 */
AckermannFlatInput ackermannBackward(const AckermannFlatInput & input,
  const AckermannFlatOutput & output_gradient,const AckermannParameters & parameters,double minimum_speed=1e-6);
}
