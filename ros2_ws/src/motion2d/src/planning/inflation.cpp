#include "motion2d/planning/grid.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
std::optional<int> PlanningGrid::cellIndex(const Eigen::Vector2d & point) const
{
  if (!point.allFinite()) {return std::nullopt;}
  const Eigen::Vector2d q = (point-geometry.origin)/geometry.resolution;
  if (q.x() < 0 || q.y() < 0 || q.x() >= geometry.width || q.y() >= geometry.height) {return std::nullopt;}
  return static_cast<int>(std::floor(q.y()))*geometry.width + static_cast<int>(std::floor(q.x()));
}

Eigen::Vector2d PlanningGrid::cellCenter(int index) const
{
  return geometry.origin + geometry.resolution * Eigen::Vector2d(index%geometry.width+.5, index/geometry.width+.5);
}

bool PlanningGrid::freeCell(int x, int y) const
{
  return x >= 0 && x < geometry.width && y >= 0 && y < geometry.height && !blocked[y*geometry.width+x];
}

PlanningGrid inflateGrid(const GridConfig & geometry, const std::vector<std::int8_t> & occupancy,
  const InflationConfig & config)
{
  // Reuse the occupancy geometry boundary check; algorithms operate on validated grids.
  OccupancyGrid2D validate_geometry(geometry);
  if (occupancy.size() != static_cast<std::size_t>(geometry.width)*geometry.height ||
    !std::isfinite(config.radius) || !std::isfinite(config.margin) || config.radius < 0 || config.margin < 0 ||
    !std::isfinite(config.radius+config.margin) || config.free_threshold < 0 || config.free_threshold > 100) {
    throw std::invalid_argument("invalid inflation input");
  }
  PlanningGrid grid{geometry, std::vector<std::uint8_t>(occupancy.size(), 0), config.radius+config.margin};
  std::vector<int> obstacles;
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    const int value = occupancy[i];
    if (value < -1 || value > 100) {throw std::invalid_argument("occupancy must be -1 or 0..100");}
    if (value < 0 ? config.unknown_blocked : value > config.free_threshold) {obstacles.push_back(i);}
  }
  const double r = grid.clearance_radius, resolution = geometry.resolution;
  // Clamp in floating point before integer conversion for a radius larger than the map.
  const int nx = static_cast<int>(std::min<double>(geometry.width, std::ceil(r/resolution)+1));
  const int ny = static_cast<int>(std::min<double>(geometry.height, std::ceil(r/resolution)+1));
  std::vector<Eigen::Vector2i> offsets;
  // inflation_stencil_begin
  for (int y = -ny; y <= ny; ++y) {
    for (int x = -nx; x <= nx; ++x) {
      const double dx = std::max(0, std::abs(x)-1)*resolution;
      const double dy = std::max(0, std::abs(y)-1)*resolution;
      if (std::hypot(dx, dy) <= r + 1e-12) {offsets.emplace_back(x, y);}
    }
  }
  // inflation_stencil_end
  for (int index : obstacles) {
    const int ox = index%geometry.width, oy = index/geometry.width;
    for (const auto & delta : offsets) {
      const int x = ox+delta.x(), y = oy+delta.y();
      if (x >= 0 && x < geometry.width && y >= 0 && y < geometry.height) {grid.blocked[y*geometry.width+x] = 1;}
    }
  }
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const double distance = std::min({x, y, geometry.width-1-x, geometry.height-1-y})*resolution;
      if (distance <= r+1e-12) {grid.blocked[y*geometry.width+x] = 1;}
    }
  }
  return grid;
}
}  // namespace motion2d
