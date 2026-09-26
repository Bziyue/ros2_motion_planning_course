#include "motion2d/sim/raycast.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace motion2d
{
namespace
{
constexpr double kMiss = std::numeric_limits<double>::infinity();
double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return a.x() * b.y() - a.y() * b.x();
}
}  // namespace

// ray_segment_begin
double raySegment(const Eigen::Vector2d & origin, const Eigen::Vector2d & direction,
  const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d edge = b - a, offset = a - origin;
  const double denominator = cross(direction, edge);
  const double tolerance = 1e-12 * std::max(1.0, edge.norm());
  if (std::abs(denominator) <= tolerance) {
    if (std::abs(cross(offset, direction)) > tolerance) {return kMiss;}
    const double t0 = offset.dot(direction), t1 = (b - origin).dot(direction);
    return std::max(t0, t1) < 0.0 ? kMiss : std::max(0.0, std::min(t0, t1));
  }
  const double t = cross(offset, edge) / denominator;
  const double u = cross(offset, direction) / denominator;
  return t >= 0.0 && u >= 0.0 && u <= 1.0 ? t : kMiss;
}
// ray_segment_end

// ray_circle_begin
double rayCircle(const Eigen::Vector2d & origin, const Eigen::Vector2d & direction,
  const Circle & circle)
{
  const Eigen::Vector2d offset = circle.center - origin;
  const double h = offset.dot(direction);
  const double perpendicular = cross(direction, offset);
  const double discriminant = circle.radius * circle.radius - perpendicular * perpendicular;
  if (discriminant < 0.0) {return kMiss;}
  const double root = std::sqrt(discriminant);
  if (h - root >= 0.0) {return h - root;}
  return h + root >= 0.0 ? h + root : kMiss;
}
// ray_circle_end

double raycastWorld(const World2D & world, const Eigen::Vector2d & origin,
  const Eigen::Vector2d & direction)
{
  double nearest = kMiss;
  for (const auto & circle : world.circles) {
    nearest = std::min(nearest, rayCircle(origin, direction, circle));
  }
  for (const auto & polygon : world.polygons) {
    const auto & vertices = polygon.vertices;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      nearest = std::min(nearest,
        raySegment(origin, direction, vertices[i], vertices[(i + 1) % vertices.size()]));
    }
  }
  const double x = world.width / 2.0, y = world.height / 2.0;
  const std::array<Eigen::Vector2d, 4> corners{{{-x, -y}, {x, -y}, {x, y}, {-x, y}}};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    nearest = std::min(nearest,
      raySegment(origin, direction, corners[i], corners[(i + 1) % corners.size()]));
  }
  return nearest;
}
}  // namespace motion2d
