#include "motion2d/sim/world.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace motion2d
{
// clearance_begin
double clearance(const World2D & world, const Eigen::Vector2d & center)
{
  double distance = std::min(
    world.width / 2.0 - std::abs(center.x()),
    world.height / 2.0 - std::abs(center.y()));
  for (const auto & circle : world.circles) {
    distance = std::min(distance, signedDistance(circle, center));
  }
  for (const auto & polygon : world.polygons) {
    distance = std::min(distance, signedDistance(polygon, center));
  }
  return distance;
}

bool isFree(const World2D & world, const Eigen::Vector2d & center, double radius)
{
  return clearance(world, center) > radius;
}
// clearance_end

bool isReachable(const World2D & world, double radius, double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("Reachability resolution must be positive and finite");
  }
  if (!isFree(world, world.start, radius) || !isFree(world, world.goal, radius)) {
    return false;
  }
  const int nx = static_cast<int>(std::ceil(world.width / resolution));
  const int ny = static_cast<int>(std::ceil(world.height / resolution));
  const double dx = world.width / nx, dy = world.height / ny;
  const double padded_radius = radius + 0.5 * std::hypot(dx, dy);
  std::vector<bool> free(nx * ny), visited(nx * ny, false);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      const Eigen::Vector2d p{-world.width / 2.0 + (x + 0.5) * dx,
        -world.height / 2.0 + (y + 0.5) * dy};
      free[y * nx + x] = isFree(world, p, padded_radius);
    }
  }
  const auto index = [&](const auto & p) {
      const int x = std::clamp(static_cast<int>((p.x() + world.width / 2.0) / dx), 0, nx - 1);
      const int y = std::clamp(static_cast<int>((p.y() + world.height / 2.0) / dy), 0, ny - 1);
      return y * nx + x;
    };
  const int start = index(world.start), goal = index(world.goal);
  if (!free[start] || !free[goal]) {return false;}
  std::queue<int> queue;
  queue.push(start);
  visited[start] = true;
  const std::array<std::array<int, 2>, 4> neighbors{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop();
    if (current == goal) {return true;}
    for (const auto & offset : neighbors) {
      const int x = current % nx + offset[0], y = current / nx + offset[1];
      if (x < 0 || y < 0 || x >= nx || y >= ny) {continue;}
      const int next = y * nx + x;
      if (free[next] && !visited[next]) {
        visited[next] = true;
        queue.push(next);
      }
    }
  }
  return false;
}
}  // namespace motion2d
