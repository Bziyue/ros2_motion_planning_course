#include "motion2d/sim/world.hpp"
#include <algorithm>

namespace motion2d
{
namespace
{
double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return a.x() * b.y() - a.y() * b.x();
}

/** @brief Segment distance, including intersections, contact and zero lengths. */
double segmentDistance(const Eigen::Vector2d & a, const Eigen::Vector2d & b,
  const Eigen::Vector2d & c, const Eigen::Vector2d & d)
{
  const double c_side = cross(b - a, c - a), d_side = cross(b - a, d - a);
  const double a_side = cross(d - c, a - c), b_side = cross(d - c, b - c);
  const bool straddle_ab = (c_side <= 0 && d_side >= 0) || (c_side >= 0 && d_side <= 0);
  const bool straddle_cd = (a_side <= 0 && b_side >= 0) || (a_side >= 0 && b_side <= 0);
  // Collinear but disjoint segments also have zero cross products.
  const bool boxes_overlap =
    (a.cwiseMin(b).array() <= c.cwiseMax(d).array()).all() &&
    (c.cwiseMin(d).array() <= a.cwiseMax(b).array()).all();
  if (straddle_ab && straddle_cd && boxes_overlap) {return 0.0;}
  return std::min({pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d),
      pointSegmentDistance(c, a, b), pointSegmentDistance(d, a, b)});
}
}  // namespace

// sweep_begin
bool sweptDiskIsFree(const World2D & world, const Eigen::Vector2d & from,
  const Eigen::Vector2d & to, double radius)
{
  // The inset rectangle is convex, so valid endpoints imply a valid segment.
  if (!isFree(world, from, radius) || !isFree(world, to, radius)) {return false;}
  for (const auto & circle : world.circles) {
    if (pointSegmentDistance(circle.center, from, to) <= radius + circle.radius) {
      return false;
    }
  }
  for (const auto & polygon : world.polygons) {
    const auto & v = polygon.vertices;
    for (std::size_t i = 0; i < v.size(); ++i) {
      if (segmentDistance(from, to, v[i], v[(i + 1) % v.size()]) <= radius) {
        return false;
      }
    }
  }
  return true;
}
// sweep_end
}  // namespace motion2d
