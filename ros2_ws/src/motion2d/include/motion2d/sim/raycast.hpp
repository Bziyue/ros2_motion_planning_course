#pragma once
#include "motion2d/sim/world.hpp"

namespace motion2d
{
/**
 * @brief Nearest nonnegative ray/segment intersection, in metres.
 * @param origin Ray origin in the world's frame (m).
 * @param direction Unit direction in that frame.
 * @param a Segment start (m).
 * @param b Segment end (m), possibly equal to a.
 * @return Ray parameter t >= 0, or +infinity on a miss.
 * @pre Finite inputs and unit direction. Course scenes use metre-scale geometry.
 * @details Solves o+t*d=a+u*(b-a), t>=0, 0<=u<=1. Collinear
 * overlap returns its nearest forward point; see chapter 05.
 */
double raySegment(const Eigen::Vector2d & origin, const Eigen::Vector2d & direction,
  const Eigen::Vector2d & a, const Eigen::Vector2d & b);

/** @brief Nearest nonnegative ray/circle surface intersection, in metres.
 * @pre Finite inputs, unit direction and positive circle radius.
 * @return +infinity on a miss; the exit surface if the origin is inside.
 * @details With q=c-o and h=q.dot(d), roots are h +/- sqrt(R^2-cross(d,q)^2).
 * A tangent counts as a hit; a surface origin may return zero.
 */
double rayCircle(const Eigen::Vector2d & origin, const Eigen::Vector2d & direction,
  const Circle & circle);

/** @brief Nearest surface in a world, including its rectangular boundary.
 * @pre Valid world, finite origin in free space, unit direction.
 * @details Exhaustively checks circles and polygon edges. No robot self-hit,
 * range clipping or noise here: those belong to the scanner, not geometry.
 */
double raycastWorld(const World2D & world, const Eigen::Vector2d & origin,
  const Eigen::Vector2d & direction);
}  // namespace motion2d
