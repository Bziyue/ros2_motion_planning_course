#pragma once
#include "motion2d/trajectory/polynomial.hpp"

namespace motion2d
{
/** @brief Unique quintic matching p/v/a at both ends over a positive duration.
 * @details Solves in normalized time s=t/T, then stores coefficients in seconds.
 * Inputs must be finite. Extremely ill-scaled data that overflow are rejected.
 */
QuinticPiece interpolateQuintic(const TranslationState & start,
  const TranslationState & finish, double duration);

/** @brief Connect waypoints with zero v/a at every knot; each segment is a straight line.
 * @param nominal_speed Controls T=max(min_duration, distance/nominal_speed), m/s.
 * @param min_duration Positive lower duration, seconds, including repeated points.
 * @details This speed is distance/T, not a peak bound. For rest-to-rest quintics
 * the peak speed is 1.875*distance/T. Stops simplify the first teaching example;
 * chapters 15/16 optimize interior derivatives instead.
 */
PolynomialTrajectory stopAtWaypoints(const std::vector<Eigen::Vector2d> & points,
  double nominal_speed, double min_duration = .2);
}  // namespace motion2d
