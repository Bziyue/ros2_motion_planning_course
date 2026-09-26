#pragma once
#include "motion2d/trajectory/ackermann_trajectory.hpp"
namespace motion2d {
/** @brief Local tracking gains; not a global parking/pose stabilization controller. */
struct AckermannGains {double position=2,speed=3,yaw=2,lateral=1.5,steering=6,soft_speed=.3;};
void validateAckermannGains(const AckermannGains & gains);
/** @brief Flat feedforward plus longitudinal and heading/cross-track feedback.
 * @pre Measured rear pose/signed speed/steering; valid parameters/gains; dt>0.
 * @details Does not invert flatness at the measured speed, so stopping is allowed.
 * Heading and lateral errors must stay small; limits are applied before the plant.
 */
AckermannInput trackAckermann(const AckermannState & state,const AckermannReference & reference,
  const AckermannParameters & parameters,const AckermannGains & gains,double dt);
}
