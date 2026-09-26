#include "motion2d/planning/astar.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
namespace
{
/** @brief Slab clipping against a closed square: touching counts as intersection. */
bool intersects(const Eigen::Vector2d & from, const Eigen::Vector2d & to,
  const Eigen::Vector2d & lower, const Eigen::Vector2d & upper)
{
  double enter = 0., leave = 1.;
  for (int axis = 0; axis < 2; ++axis) {
    const double delta = to[axis]-from[axis];
    if (std::abs(delta) < 1e-14) {
      if (from[axis] < lower[axis]-1e-12 || from[axis] > upper[axis]+1e-12) {return false;}
    } else {
      double a = (lower[axis]-from[axis])/delta, b = (upper[axis]-from[axis])/delta;
      if (a > b) {std::swap(a,b);}
      enter = std::max(enter,a); leave = std::min(leave,b);
      if (enter > leave+1e-12) {return false;}
    }
  }
  return true;
}
}

bool segmentIsFree(const PlanningGrid & grid, const Eigen::Vector2d & from,
  const Eigen::Vector2d & to)
{
  if (!grid.cellIndex(from) || !grid.cellIndex(to)) {return false;}
  const auto & config = grid.geometry;
  const Eigen::Vector2d low = (from.cwiseMin(to)-config.origin)/config.resolution;
  const Eigen::Vector2d high = (from.cwiseMax(to)-config.origin)/config.resolution;
  const int x0 = std::max(0, static_cast<int>(std::floor(low.x()))-1);
  const int y0 = std::max(0, static_cast<int>(std::floor(low.y()))-1);
  const int x1 = std::min(config.width-1, static_cast<int>(std::floor(high.x())));
  const int y1 = std::min(config.height-1, static_cast<int>(std::floor(high.y())));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (grid.freeCell(x,y)) {continue;}
      const Eigen::Vector2d lower = config.origin+config.resolution*Eigen::Vector2d(x,y);
      if (intersects(from,to,lower,lower+Eigen::Vector2d::Constant(config.resolution))) {return false;}
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> simplifyPath(const PlanningGrid & grid,
  const std::vector<Eigen::Vector2d> & path)
{
  if (path.empty()) {return {};}
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!segmentIsFree(grid, path[i ? i-1 : 0], path[i])) {throw std::invalid_argument("unsafe input polyline");}
  }
  std::vector<Eigen::Vector2d> result{path.front()};
  std::size_t current = 0;
  while (current+1 < path.size()) {
    std::size_t next = path.size()-1;
    while (next > current+1 && !segmentIsFree(grid,path[current],path[next])) {--next;}
    result.push_back(path[next]); current = next;
  }
  return result;
}
}  // namespace motion2d
