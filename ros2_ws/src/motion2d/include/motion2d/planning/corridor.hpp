#pragma once
#include <string>
#include <Eigen/Core>
#include "motion2d/geometry/obstacles.hpp"
#include "motion2d/planning/grid.hpp"

namespace motion2d
{
/** @brief Convex configuration-space region A*p<=b; unit outward row normals.
 * @details Polygon vertices are strictly convex and CCW; coordinates/b are in m.
 */
struct ConvexRegion
{
  Polygon polygon;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  /** @brief Boundary-inclusive containment; tolerance is a signed distance in m. */
  bool contains(const Eigen::Vector2d & p, double tolerance = 1e-10) const;
};
/** @brief Convert a valid, strictly convex CCW polygon to unit-normal half-spaces. */
ConvexRegion makeRegion(const Polygon & polygon);
/** @brief Closed convex intersection; empty polygon for no positive-area overlap. */
Polygon intersectRegions(const ConvexRegion & a, const ConvexRegion & b);
/** @brief Validate the ENTIRE region against blocked configuration cells/exterior.
 * @details Clips against each candidate blocked square, catching obstacles
 * entirely inside a polygon and edge/vertex contact. Vertex-only checks are unsafe.
 */
bool regionIsFree(const PlanningGrid & grid, const ConvexRegion & region);

struct CorridorResult
{
  bool success = false;
  std::string status;
  std::vector<ConvexRegion> regions;
  std::vector<Eigen::Vector2d> waypoints; ///< One more than regions; segment i is assigned to region i.
};
/** @brief Expand safe rectangles along an A* polyline, optionally merge safe convex hulls.
 * @param path Use the original centre-neighbour path, including actual endpoints.
 * @param merge_convex Greedily merge consecutive rectangles only after full hull validation.
 * @param max_extension Maximum rectangle extension per side from its seed, m.
 * @details Rectangle seeds cover each edge's cell bounding box. Long simplified
 * diagonals may have blocked boxes and fail explicitly. Adjacent regions must
 * overlap with positive area; waypoint joins lie in those overlaps. Boundaries
 * are inset by 1e-6 cell to avoid numerical contact. Not a maximum-volume cover.
 */
CorridorResult buildCorridor(const PlanningGrid & grid,
  const std::vector<Eigen::Vector2d> & path, bool merge_convex = true, double max_extension = 1.0);
}  // namespace motion2d
