#pragma once
#include "motion2d/planning/astar.hpp"
#include "motion2d/trajectory/trajectory_optimizer.hpp"
#include "motion2d/trajectory/bezier_bounds.hpp"
namespace motion2d {
/** @brief Reachable observed route, truncated by arc length to a local endpoint. */
struct LocalRoute {
  bool success=false,reaches_global=false;
  std::string status;
  std::vector<Eigen::Vector2d> path;
};
/** @brief Select the global goal if reachable, otherwise a progress-making frontier.
 * @param observed Raw occupancy from the SAME snapshot as the inflated grid.
 * @details A frontier candidate is a reachable configuration cell near raw unknown
 * cells. Nearest-to-goal selection is greedy, not a complete exploration algorithm.
 * Unknown space is never traversed. No candidate means a reported local failure.
 */
LocalRoute observedLocalRoute(const PlanningGrid & grid,const std::vector<std::int8_t> & observed,
  const Eigen::Vector2d & start,const Eigen::Vector2d & goal,double max_length=2.);
/** @brief Small local planner settings; constraints still need posterior certification. */
struct ReplanConfig {
  double max_route_length=2.,nominal_speed=.4,min_duration=.8,corridor_extension=.6;
  bool optimize=true;
  TrajectoryOptimizationConfig optimization;
  ReplanConfig();
};
struct ReplanResult {
  bool success=false,optimized=false,reaches_global=false;
  std::string status,optimization_status;
  std::optional<PolynomialTrajectory> curve;
  std::vector<ConvexRegion> regions;
  std::vector<Eigen::Vector2d> path;
  double seconds=0;
};
/** @brief One synchronous local solve against one immutable observed map snapshot.
 * @param start Reference p/v/a at the FUTURE handover, expressed in map.
 * @details Spline optimization requires convergence AND a Bezier certificate.
 * If it fails, try explicit stop-at-waypoint quintics preserving the first p/v/a;
 * the fallback also requires an independent certificate. Otherwise return failure.
 */
ReplanResult replanObserved(const PlanningGrid & grid,const std::vector<std::int8_t> & observed,
  const Esdf2D & field,const TranslationState & start,const Eigen::Vector2d & goal,
  const ReplanConfig & config={});
}
