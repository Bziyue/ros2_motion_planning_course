#include "motion2d/geometry/obstacles.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace motion2d
{
namespace
{
double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return a.x() * b.y() - a.y() * b.x();
}
}  // namespace

// distance_begin
double pointSegmentDistance(
  const Eigen::Vector2d & point, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d edge = b - a;
  if (edge.squaredNorm() == 0.0) {
    return (point - a).norm();
  }
  const double t = std::clamp((point - a).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
  return (point - (a + t * edge)).norm();
}
// distance_end

Polygon convexHull(std::vector<Eigen::Vector2d> points)
{
  std::sort(points.begin(), points.end(), [](const auto & a, const auto & b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  points.erase(std::unique(points.begin(), points.end(), [](const auto & a, const auto & b) {
    return a.x() == b.x() && a.y() == b.y();
  }), points.end());
  if (points.size() < 3) {return {};}
  std::vector<Eigen::Vector2d> lower, upper;
  const auto append = [](auto & chain, const auto & p) {
      while (chain.size() >= 2 &&
        cross(chain.back() - chain[chain.size() - 2], p - chain.back()) <= 0.0)
      {
        chain.pop_back();
      }
      chain.push_back(p);
    };
  for (const auto & p : points) {append(lower, p);}
  for (auto i = points.rbegin(); i != points.rend(); ++i) {append(upper, *i);}
  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  if (lower.size() < 3) {return {};}
  return {lower};
}

bool isValid(const Polygon & polygon)
{
  const auto & v = polygon.vertices;
  if (v.size() < 3) {return false;}
  for (const auto & p : v) {
    if (!p.allFinite()) {return false;}
  }
  // Every other vertex must be strictly on the left of every directed edge.
  // Unlike checking consecutive turns only, this also rejects star polygons.
  for (std::size_t i = 0; i < v.size(); ++i) {
    const std::size_t next = (i + 1) % v.size();
    for (std::size_t j = 0; j < v.size(); ++j) {
      if (j != i && j != next && cross(v[next] - v[i], v[j] - v[i]) <= 1e-12) {
        return false;
      }
    }
  }
  return true;
}

double signedDistance(const Circle & circle, const Eigen::Vector2d & point)
{
  return (point - circle.center).norm() - circle.radius;
}

double signedDistance(const Polygon & polygon, const Eigen::Vector2d & point)
{
  double distance = std::numeric_limits<double>::infinity();
  bool inside = true;
  const auto & v = polygon.vertices;
  for (std::size_t i = 0; i < v.size(); ++i) {
    const auto & a = v[i];
    const auto & b = v[(i + 1) % v.size()];
    distance = std::min(distance, pointSegmentDistance(point, a, b));
    inside = inside && cross(b - a, point - a) >= 0.0;
  }
  return inside ? -distance : distance;
}
}  // namespace motion2d
