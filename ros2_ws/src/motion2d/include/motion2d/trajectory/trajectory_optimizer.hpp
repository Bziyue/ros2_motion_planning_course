#pragma once
#include "motion2d/trajectory/minco2d.hpp"
#include "motion2d/trajectory/trajectory_cost.hpp"
#include "motion2d/optimization/bfgs.hpp"
#include <optional>
namespace motion2d {
/** @brief Small teaching optimizer. Dense outer BFGS has quadratic memory. */
struct TrajectoryOptimizationConfig {
  TrajectoryCostConfig cost;
  TrajectoryLimits limits;
  BfgsConfig solver;
  bool bezier_penalties=false;  ///< Additional control-point soft costs (ch16).
  bool optimize_waypoints=true, optimize_times=true;
  double min_duration=.2;
};
/** @brief Last accepted iterate and diagnostics; no execution authorization. */
struct TrajectoryOptimizationResult {
  BfgsResult solver;
  std::optional<PolynomialTrajectory> curve;
  std::vector<Eigen::Vector2d> waypoints;
  std::vector<double> durations;
  SampledFeasibility samples;
};
/** @brief Optimize internal positions and positive durations T=Tmin+exp(tau).
 * @details Fixed p/v/a endpoints, one corridor per piece. Soft constraints and
 * finite sampling cannot certify the continuous curve. Exceptions in trial
 * evaluations cause a rejected line-search step; invalid initial data is reported.
 */
TrajectoryOptimizationResult optimizeMinco(const TranslationState & start,const TranslationState & finish,
  const std::vector<Eigen::Vector2d> & interior,const std::vector<double> & durations,
  const TrajectoryOptimizationConfig & config,const std::vector<ConvexRegion> & regions={},const Esdf2D * field=nullptr);
/** @brief Same variables, cost and solver as optimizeMinco; use the independent Spline2D core. */
TrajectoryOptimizationResult optimizeSpline(const TranslationState & start,const TranslationState & finish,
  const std::vector<Eigen::Vector2d> & interior,const std::vector<double> & durations,
  const TrajectoryOptimizationConfig & config,const std::vector<ConvexRegion> & regions={},const Esdf2D * field=nullptr);
}
