#include "motion2d/mapping/scan_projection.hpp"
#include <cmath>
#include <stdexcept>

namespace motion2d
{
void validateScan(const Scan2D & scan)
{
  if (scan.ranges.empty() || scan.ranges.size() > 100000 ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment <= 0.0 || !std::isfinite(scan.range_min) ||
    !std::isfinite(scan.range_max) || scan.range_min < 0.0 ||
    scan.range_max <= scan.range_min)
  {
    throw std::invalid_argument("scan: finite angles, positive increment, 0<=min<max, 1..100000 rays");
  }
}

// scan_projection_begin
std::vector<Eigen::Vector2d> projectScan(const Scan2D & scan)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(scan.ranges.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {continue;}
    const double angle = scan.angle_min + i * scan.angle_increment;
    points.emplace_back(range * std::cos(angle), range * std::sin(angle));
  }
  return points;
}
// scan_projection_end

std::vector<Eigen::Vector2d> registerPoints(
  const std::vector<Eigen::Vector2d> & points, const Pose2D & pose)
{
  std::vector<Eigen::Vector2d> registered;
  registered.reserve(points.size());
  for (const auto & point : points) {registered.push_back(transformPoint(pose, point));}
  return registered;
}
}  // namespace motion2d
