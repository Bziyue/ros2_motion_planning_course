#pragma once
#include <string>
#include <limits>
#include "motion2d/planning/grid.hpp"

namespace motion2d
{
/** @brief Search output. Grid cost is centre-to-centre distance, before simplification. */
struct PathResult
{
  bool success = false;
  std::string status;
  std::vector<int> cells;
  std::vector<Eigen::Vector2d> path; ///< Includes the exact requested start and goal (m).
  double grid_cost = std::numeric_limits<double>::infinity();
  std::size_t expanded = 0;
};

/** @brief 4/8-connected A*; diagonal steps require both adjacent axial cells free.
 * @details Manhattan or octile heuristic is consistent for the corresponding
 * graph. Failure is invalid_start, invalid_goal or unreachable, with empty path.
 * The input grid must be constructed/validated once at its external boundary.
 */
PathResult astar(const PlanningGrid & grid, const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal, bool diagonal = true);

/** @brief Greedy furthest visible waypoint, preserving exact endpoints.
 * @pre A valid path in the same configuration-space grid; throws if an edge is unsafe.
 * @details Returns a shorter polyline, not a time trajectory or global shortest curve.
 */
std::vector<Eigen::Vector2d> simplifyPath(const PlanningGrid & grid,
  const std::vector<Eigen::Vector2d> & path);
}  // namespace motion2d
