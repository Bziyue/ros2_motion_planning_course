#pragma once
#include "motion2d/trajectory/polynomial.hpp"
#include "motion2d/planning/corridor.hpp"
#include "motion2d/mapping/esdf.hpp"
#include <limits>

namespace motion2d
{
/** @brief Soft cost weights and targets; weights carry the units needed to sum terms.
 * @details Speed/acceleration targets are vector-norm bounds, SI units. Corridor
 * margin tightens already inflated halfspaces; it is an optional extra buffer,
 * not another robot radius. ESDF target applies to the UNINFLATED centre-distance
 * interpolant; it is a penalty target, not a clearance certificate.
 */
struct TrajectoryCostConfig
{
  double energy_weight=.1, time_weight=1;
  double speed_max=.8, acceleration_max=.8;
  double speed_weight=10, acceleration_weight=10;
  double corridor_weight=500, corridor_margin=.02;
  double esdf_weight=100, esdf_distance=.4;
  int quadrature_steps=24;
};
/** @brief Validate finite nonnegative weights and positive physical targets. */
void validateCostConfig(const TrajectoryCostConfig & config);

/** @brief Analytic jerk + time + trapezoidal integrated cubic-hinge penalties.
 * @param regions Empty to omit corridor costs, or exactly one verified region per piece.
 * @param field Optional uninflated ESDF; null omits its term.
 * @return Cost and direct coefficient/time partials, before MINCO/Spline elimination.
 * @throws std::runtime_error when an enabled field query is outside its valid domain.
 * @pre Valid config; region geometry has already passed whole-region safety checking.
 * @details A small cost is NOT a proof of continuous feasibility.
 */
CoefficientGradient trajectoryCost(const PolynomialTrajectory & trajectory,
  const TrajectoryCostConfig & config,const std::vector<ConvexRegion> & regions={},const Esdf2D * field=nullptr);

/** @brief Separate physical limits for the denser post-optimization sample check. */
struct TrajectoryLimits
{
  double speed=1, acceleration=1, clearance=.2;
  int samples_per_piece=200;
};
/** @brief Diagnostic only: a finite sample test cannot certify a continuous curve. */
struct SampledFeasibility
{
  bool samples_feasible=false;
  std::string status;
  double peak_speed=0, peak_acceleration=0;
  double max_corridor_residual=-std::numeric_limits<double>::infinity();
  double min_clearance_lower_bound=std::numeric_limits<double>::quiet_NaN();
};
/** @brief Check independent, denser samples against physical limits and raw halfspaces.
 * @details ESDF uses its conservative square-clearance LOWER BOUND, never the raw
 * centre distance. A null field omits that check and reports NaN clearance.
 * This is explicitly labelled sampled_feasible, never continuously_safe.
 */
SampledFeasibility checkTrajectorySamples(const PolynomialTrajectory & trajectory,
  const TrajectoryLimits & limits,const std::vector<ConvexRegion> & regions={},const Esdf2D * field=nullptr);
}  // namespace motion2d
