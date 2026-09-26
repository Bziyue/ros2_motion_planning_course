#include "motion2d/planning/astar.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>

namespace motion2d
{
PathResult astar(const PlanningGrid & grid, const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal, bool diagonal)
{
  PathResult result;
  const auto first = grid.cellIndex(start), last = grid.cellIndex(goal);
  if (!first || grid.blocked[*first] || !segmentIsFree(grid,start,start)) {result.status = "invalid_start"; return result;}
  if (!last || grid.blocked[*last] || !segmentIsFree(grid,goal,goal)) {result.status = "invalid_goal"; return result;}
  const int width = grid.geometry.width;
  auto heuristic = [&](int index) {
    const int dx = std::abs(index%width-*last%width), dy = std::abs(index/width-*last/width);
    return grid.geometry.resolution * (diagonal ? std::max(dx,dy)+(std::sqrt(2.)-1)*std::min(dx,dy) : dx+dy);
  };
  using Entry = std::tuple<double, double, int>; // f, g, index: deterministic ties.
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  std::vector<double> distance(grid.blocked.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(grid.blocked.size(), -1);
  std::vector<bool> closed(grid.blocked.size(), false);
  distance[*first] = 0; open.emplace(heuristic(*first), 0., *first);
  while (!open.empty()) {
    const auto [f, g, current] = open.top(); open.pop(); (void)f;
    if (closed[current] || g > distance[current]) {continue;}
    closed[current] = true; ++result.expanded;
    if (current == *last) {
      result.success = true; result.status = "found"; result.grid_cost = g;
      for (int at = current; at >= 0; at = parent[at]) {result.cells.push_back(at);}
      std::reverse(result.cells.begin(), result.cells.end());
      result.path.push_back(start);
      if (result.cells.size() > 1) {
        for (int cell : result.cells) {
          const auto point = grid.cellCenter(cell);
          if ((point-result.path.back()).norm() > 1e-12) {result.path.push_back(point);}
        }
      }
      if ((goal-result.path.back()).norm() > 1e-12) {result.path.push_back(goal);}
      return result;
    }
    // astar_neighbors_begin
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) || (!diagonal && dx != 0 && dy != 0)) {continue;}
        const int x = current%width, y = current/width;
        if (!grid.freeCell(x+dx, y+dy)) {continue;}
        if (dx != 0 && dy != 0 && (!grid.freeCell(x+dx, y) || !grid.freeCell(x, y+dy))) {continue;}
        const int next = (y+dy)*width+x+dx;
        const double candidate = g + grid.geometry.resolution * std::hypot(dx, dy);
        if (candidate < distance[next]) {
          distance[next] = candidate; parent[next] = current;
          open.emplace(candidate+heuristic(next), candidate, next);
        }
      }
    }
    // astar_neighbors_end
  }
  result.status = "unreachable";
  return result;
}
}  // namespace motion2d
