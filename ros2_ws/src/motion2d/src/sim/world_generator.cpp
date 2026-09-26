#include "motion2d/sim/world.hpp"
#include "motion2d/geometry/se2.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

namespace motion2d
{
World2D generateWorld(const WorldConfig & c)
{
  if (!std::isfinite(c.width) || !std::isfinite(c.height) ||
    !std::isfinite(c.size_min) || !std::isfinite(c.size_max) ||
    !std::isfinite(c.robot_radius) || !std::isfinite(c.margin) ||
    c.width <= 0 || c.height <= 0 || c.size_min <= 0 || c.size_max < c.size_min ||
    c.size_max * 2 >= std::min(c.width, c.height) || c.robot_radius < 0 || c.margin < 0 ||
    c.circle_count < 0 || c.polygon_count < 0 || c.polygon_samples < 3 ||
    !c.start.allFinite() || !c.goal.allFinite())
  {
    throw std::invalid_argument("Invalid world dimensions, sizes, counts, radius or endpoints");
  }
  World2D world;
  world.width = c.width;
  world.height = c.height;
  world.start = c.start;
  world.goal = c.goal;
  const double keep_clear = c.robot_radius + c.margin;
  if (!isFree(world, c.start, keep_clear) || !isFree(world, c.goal, keep_clear)) {
    throw std::invalid_argument("Start/goal disk is outside the world boundary");
  }
  std::mt19937 random(c.seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const auto size = [&]() {return c.size_min + (c.size_max - c.size_min) * unit(random);};
  const auto center = [&](double bound) {
      return Eigen::Vector2d{
        -c.width / 2.0 + bound + (c.width - 2.0 * bound) * unit(random),
        -c.height / 2.0 + bound + (c.height - 2.0 * bound) * unit(random)};
    };
  const auto endpoints_clear = [&](const auto & shape) {
      return signedDistance(shape, c.start) > keep_clear &&
             signedDistance(shape, c.goal) > keep_clear;
    };
  for (int i = 0; i < c.circle_count; ++i) {
    bool placed = false;
    for (int attempt = 0; attempt < 1000 && !placed; ++attempt) {
      const double radius = size();
      Circle circle{center(radius), radius};
      if (endpoints_clear(circle)) {
        world.circles.push_back(circle);
        placed = true;
      }
    }
    if (!placed) {throw std::runtime_error("Cannot place circle; seed=" + std::to_string(c.seed));}
  }
  // polygon_begin
  for (int i = 0; i < c.polygon_count; ++i) {
    bool placed = false;
    for (int attempt = 0; attempt < 1000 && !placed; ++attempt) {
      const double bound = size();
      const Eigen::Vector2d origin = center(bound);
      std::vector<Eigen::Vector2d> points;
      for (int j = 0; j < c.polygon_samples; ++j) {
        const double angle = 2.0 * kPi * unit(random);
        const double radius = bound * std::sqrt(unit(random));
        points.push_back(origin + radius * Eigen::Vector2d{std::cos(angle), std::sin(angle)});
      }
      Polygon polygon = convexHull(points);
      if (isValid(polygon) && endpoints_clear(polygon)) {
        world.polygons.push_back(polygon);
        placed = true;
      }
    }
    if (!placed) {throw std::runtime_error("Cannot place polygon; seed=" + std::to_string(c.seed));}
  }
  // polygon_end
  if (c.require_connected && !isReachable(world, keep_clear, c.check_resolution)) {
    throw std::runtime_error(
            "Conservative reachability check failed; seed=" + std::to_string(c.seed) +
            ". Change the scene explicitly or reduce check_resolution.");
  }
  return world;
}
}  // namespace motion2d
