#pragma once
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
/** @brief Traverse cells for an unclipped segment; origin=(0,0), resolution=1 m.
 * @pre Finite start/end inside the grid; width is the positive row stride.
 * @details EXERCISE(ch07-3): complete the DDA loop, including exact corners.
 */
inline std::vector<int> studentRayCells(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end, int width)
{
  int x = static_cast<int>(std::floor(start.x()));
  int y = static_cast<int>(std::floor(start.y()));
  const int ex = static_cast<int>(std::floor(end.x()));
  const int ey = static_cast<int>(std::floor(end.y()));
  const Eigen::Vector2d delta = end - start;
  const int sx = (delta.x() > 0) - (delta.x() < 0);
  const int sy = (delta.y() > 0) - (delta.y() < 0);
  const double inf = std::numeric_limits<double>::infinity();
  const double dx = sx ? 1 / std::abs(delta.x()) : inf;
  const double dy = sy ? 1 / std::abs(delta.y()) : inf;
  double tx = sx ? (x + (sx > 0) - start.x()) / delta.x() : inf;
  double ty = sy ? (y + (sy > 0) - start.y()) / delta.y() : inf;
  std::vector<int> cells{y * width + x};
  while (x != ex || y != ey) {
    if (x == ex) {tx = inf;}
    if (y == ey) {ty = inf;}
    if (tx < ty) {x += sx; tx += dx;}
    else if (ty < tx) {y += sy; ty += dy;}
    else {x += sx; y += sy; tx += dx; ty += dy;}
    cells.push_back(y * width + x);
  }
  return cells;
}
