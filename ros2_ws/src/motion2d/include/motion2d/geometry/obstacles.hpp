#pragma once
#include <vector>
#include <Eigen/Core>

namespace motion2d
{
/** @brief Circular obstacle in map coordinates, metres. */
struct Circle
{
  Eigen::Vector2d center;
  double radius;
};

/** @brief Strictly convex CCW polygon, without repeating the first vertex. */
struct Polygon
{
  std::vector<Eigen::Vector2d> vertices;
};

/**
 * @brief Euclidean point-to-segment distance, including degenerate segments.
 * @param point Query point in the same frame as a and b (m).
 * @param a First endpoint (m).
 * @param b Second endpoint (m); may coincide with a.
 * @return Nonnegative Euclidean distance (m).
 * @pre All input coordinates are finite.
 * @details With e=b-a, project using
 * \f$t=\mathrm{clamp}((p-a)^\top e/(e^\top e),0,1)\f$.
 * The closest point is a+t*e. See chapter 03.
 */
double pointSegmentDistance(
  const Eigen::Vector2d & point, const Eigen::Vector2d & a, const Eigen::Vector2d & b);

/** @brief Convex hull with CCW vertices; empty for fewer than 3 noncollinear points.
 *  @pre All points are finite and expressed in the same frame.
 */
Polygon convexHull(std::vector<Eigen::Vector2d> points);

/** @brief Check finite vertices, strict convexity and CCW order. */
bool isValid(const Polygon & polygon);

/** @brief Signed obstacle distance (m): positive outside, negative inside.
 *  @pre Circle radius is positive; all inputs are finite.
 *  @details \f$d(p)=\|p-c\|-R\f$, where c and R are the obstacle centre and radius.
 */
double signedDistance(const Circle & circle, const Eigen::Vector2d & point);

/** @brief Signed distance (m); polygon must satisfy isValid(). */
double signedDistance(const Polygon & polygon, const Eigen::Vector2d & point);
}  // namespace motion2d
