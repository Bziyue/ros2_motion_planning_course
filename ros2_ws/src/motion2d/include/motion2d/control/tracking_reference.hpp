#pragma once
#include "motion2d/sim/robot_model.hpp"
namespace motion2d {
enum class ReferenceShape {Circle,FigureEight};
/** @brief Analytic reference in one fixed odom frame; scale m, omega rad/s, ramp s. */
struct TrackingReferenceConfig {
  Pose2D origin;
  ReferenceShape shape=ReferenceShape::Circle;
  double scale=1,omega=.5,ramp_seconds=2;
};
/** @brief Validate a stationary-start reference configuration. */
void validateTrackingReference(const TrackingReferenceConfig & config);
/** @brief Circle or figure-eight p/v/a with a smooth phase ramp from zero v/a.
 * @details Phase clock tau=R*(s^3-.5*s^4), s=t/R, during ramp; tau=t-R/2 later.
 * Its first/second derivatives join continuously. Independent yaw=origin+.4*sin(phase/2).
 * @pre Valid configuration; finite time>=0, in seconds since reference start.
 */
State2D sampleTrackingReference(const TrackingReferenceConfig & config,double time);
}
