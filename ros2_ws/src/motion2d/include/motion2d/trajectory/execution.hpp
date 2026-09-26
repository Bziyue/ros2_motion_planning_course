#pragma once
#include "motion2d/trajectory/polynomial.hpp"
#include "motion2d/sim/robot_model.hpp"
#include <cstdint>

namespace motion2d
{
/** @brief A translation curve, absolute simulation start (ns), and constant yaw (rad). */
struct TimedTrajectory
{
  PolynomialTrajectory curve;
  std::int64_t start_ns = 0;
  double yaw = 0;
};

/** @brief Sample p/v/a in odom; hold position before/after execution with zero derivatives.
 * @pre Start/end v/a are zero for a continuous transition to those holds.
 * @details All sensors and the ideal plant use this same analytic time function.
 */
State2D sampleHeldTrajectory(const TimedTrajectory & trajectory, std::int64_t stamp);

/** @brief Conservative bound on ||acceleration|| over a trajectory-time interval (m/s²).
 * @details For each intersecting piece, sum k(k-1)||c_k|| t_max^(k-2).
 * The maximum over pieces bounds acceleration despite coefficient cancellations.
 * Endpoint holds have zero acceleration; caller guarantees C2 joins including holds.
 */
double trajectoryAccelerationBound(const PolynomialTrajectory & curve, double begin, double end);
}  // namespace motion2d
