#include "motion2d/planning/corridor.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
namespace
{
using Points = std::vector<Eigen::Vector2d>;
// Sutherland-Hodgman clipping. Keep degenerate output too: contact must be detected.
Points clip(const Points & input, const Eigen::Vector2d & normal, double offset)
{
  Points result;
  if (input.empty()) {return result;}
  Eigen::Vector2d previous = input.back(); double dp = normal.dot(previous)-offset;
  for (const auto & current : input) {
    const double dc = normal.dot(current)-offset;
    if ((dp <= 0) != (dc <= 0)) {result.push_back(previous+(dp/(dp-dc))*(current-previous));}
    if (dc <= 0) {result.push_back(current);}
    previous = current; dp = dc;
  }
  return result;
}

double area(const Polygon & polygon)
{
  double twice = 0;
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    const auto & a = polygon.vertices[i]; const auto & b = polygon.vertices[(i+1)%polygon.vertices.size()];
    twice += a.x()*b.y()-a.y()*b.x();
  }
  return .5*std::abs(twice);
}

bool freeBox(const PlanningGrid & grid, int left, int right, int bottom, int top)
{
  if (left < 0 || bottom < 0 || right >= grid.geometry.width || top >= grid.geometry.height) {return false;}
  for (int y = bottom; y <= top; ++y) {
    for (int x = left; x <= right; ++x) {if (!grid.freeCell(x,y)) {return false;}}
  }
  return true;
}

std::optional<ConvexRegion> expandRectangle(const PlanningGrid & grid,
  const Eigen::Vector2d & from, const Eigen::Vector2d & to, double max_extension)
{
  const auto & g = grid.geometry;
  const Eigen::Vector2d low = (from.cwiseMin(to)-g.origin)/g.resolution;
  const Eigen::Vector2d high = (from.cwiseMax(to)-g.origin)/g.resolution;
  // Include both sides of an endpoint on a cell boundary before insetting.
  int left = std::floor(low.x()-1e-6), right = std::floor(high.x()+1e-6);
  int bottom = std::floor(low.y()-1e-6), top = std::floor(high.y()+1e-6);
  if (!freeBox(grid,left,right,bottom,top)) {return std::nullopt;}
  const int nx = std::min<double>(g.width, std::floor(max_extension/g.resolution));
  const int ny = std::min<double>(g.height, std::floor(max_extension/g.resolution));
  const int limit_left = std::max(0,left-nx), limit_right = std::min(g.width-1,right+nx);
  const int limit_bottom = std::max(0,bottom-ny), limit_top = std::min(g.height-1,top+ny);
  bool changed = true;
  // corridor_rectangle_begin
  while (changed) {
    changed = false;
    if (left > limit_left && freeBox(grid,left-1,left-1,bottom,top)) {--left; changed = true;}
    if (right < limit_right && freeBox(grid,right+1,right+1,bottom,top)) {++right; changed = true;}
    if (bottom > limit_bottom && freeBox(grid,left,right,bottom-1,bottom-1)) {--bottom; changed = true;}
    if (top < limit_top && freeBox(grid,left,right,top+1,top+1)) {++top; changed = true;}
  }
  // corridor_rectangle_end
  const double inset = 1e-6*g.resolution;
  const Eigen::Vector2d a = g.origin+g.resolution*Eigen::Vector2d(left,bottom)+Eigen::Vector2d::Constant(inset);
  const Eigen::Vector2d b = g.origin+g.resolution*Eigen::Vector2d(right+1,top+1)-Eigen::Vector2d::Constant(inset);
  return makeRegion({{a, {b.x(),a.y()}, b, {a.x(),b.y()}}});
}
}  // namespace

bool ConvexRegion::contains(const Eigen::Vector2d & p, double tolerance) const
{
  return p.allFinite() && (A*p-b).maxCoeff() <= tolerance;
}

ConvexRegion makeRegion(const Polygon & polygon)
{
  if (!isValid(polygon)) {throw std::invalid_argument("Corridor requires a strictly convex CCW polygon");}
  ConvexRegion result; result.polygon = polygon;
  result.A.resize(polygon.vertices.size(), 2); result.b.resize(polygon.vertices.size());
  // corridor_halfspace_begin
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    const auto & a = polygon.vertices[i];
    const Eigen::Vector2d e = polygon.vertices[(i+1)%polygon.vertices.size()]-a;
    const Eigen::Vector2d n = Eigen::Vector2d(e.y(),-e.x()).normalized();
    result.A.row(i) = n.transpose(); result.b[i] = n.dot(a);
  }
  // corridor_halfspace_end
  return result;
}

Polygon intersectRegions(const ConvexRegion & a, const ConvexRegion & b)
{
  auto points = a.polygon.vertices;
  for (int i = 0; i < b.A.rows(); ++i) {points = clip(points,b.A.row(i).transpose(),b.b[i]);}
  return convexHull(points); // Reject a shared line or a shared point as overlap.
}

bool regionIsFree(const PlanningGrid & grid, const ConvexRegion & region)
{
  const auto & g = grid.geometry;
  Eigen::Vector2d lower = region.polygon.vertices[0], upper = lower;
  for (const auto & p : region.polygon.vertices) {lower = lower.cwiseMin(p); upper = upper.cwiseMax(p);}
  const Eigen::Vector2d world_upper = g.origin+g.resolution*Eigen::Vector2d(g.width,g.height);
  if ((lower.array() <= g.origin.array()+1e-12).any() ||
      (upper.array() >= world_upper.array()-1e-12).any()) {return false;}
  const auto lo = ((lower-g.origin)/g.resolution).array().floor().cast<int>().eval();
  const auto hi = ((upper-g.origin)/g.resolution).array().floor().cast<int>().eval();
  for (int y = std::max(0,lo.y()-1); y <= std::min(g.height-1,hi.y()); ++y) {
    for (int x = std::max(0,lo.x()-1); x <= std::min(g.width-1,hi.x()); ++x) {
      if (grid.freeCell(x,y)) {continue;}
      const Eigen::Vector2d a = g.origin+g.resolution*Eigen::Vector2d(x,y);
      const Eigen::Vector2d b = a+Eigen::Vector2d::Constant(g.resolution);
      auto remaining = region.polygon.vertices;
      // Expand the forbidden square by a tiny tolerance; contact is unsafe.
      remaining = clip(remaining, {1,0}, b.x()+1e-12);
      remaining = clip(remaining, {-1,0}, -a.x()+1e-12);
      remaining = clip(remaining, {0,1}, b.y()+1e-12);
      remaining = clip(remaining, {0,-1}, -a.y()+1e-12);
      if (!remaining.empty()) {return false;}
    }
  }
  return true;
}

CorridorResult buildCorridor(const PlanningGrid & grid,
  const std::vector<Eigen::Vector2d> & path, bool merge_convex, double max_extension)
{
  auto failure = [](const std::string & status) {return CorridorResult{false,status,{},{}};};
  if (!std::isfinite(max_extension) || max_extension < 0) {return failure("invalid_extension");}
  if (path.empty()) {return failure("empty_path");}
  std::vector<ConvexRegion> rectangles;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & from = path[i ? i-1 : 0]; const auto & to = path[i];
    if (!segmentIsFree(grid,from,to)) {return failure("unsafe_path");}
    auto rectangle = expandRectangle(grid,from,to,max_extension);
    if (!rectangle || !rectangle->contains(from) || !rectangle->contains(to) || !regionIsFree(grid,*rectangle)) {
      return failure("blocked_seed_box");
    }
    // An existing convex region covers the whole segment if it contains both endpoints.
    if (!rectangles.empty() && rectangles.back().contains(from) && rectangles.back().contains(to)) {continue;}
    rectangles.push_back(*rectangle);
  }
  CorridorResult result; result.status = "found";
  for (const auto & rectangle : rectangles) {
    if (merge_convex && !result.regions.empty()) {
      auto vertices = result.regions.back().polygon.vertices;
      vertices.insert(vertices.end(),rectangle.polygon.vertices.begin(),rectangle.polygon.vertices.end());
      const auto polygon = convexHull(vertices);
      // Floating-point near-collinearity can fail the strict polygon criterion.
      // Merging is optional; keep both valid rectangles instead of accepting it.
      if (isValid(polygon)) {
        auto merged = makeRegion(polygon);
        if (regionIsFree(grid,merged)) {result.regions.back() = std::move(merged); continue;}
      }
    }
    result.regions.push_back(rectangle);
  }
  result.waypoints.push_back(path.front());
  for (std::size_t i = 1; i < result.regions.size(); ++i) {
    const auto overlap = intersectRegions(result.regions[i-1],result.regions[i]);
    if (overlap.vertices.empty() || area(overlap) <= 1e-10*grid.geometry.resolution*grid.geometry.resolution) {
      return failure("no_positive_overlap");
    }
    Eigen::Vector2d join = Eigen::Vector2d::Zero();
    for (const auto & p : overlap.vertices) {join += p;}
    join /= overlap.vertices.size();
    result.waypoints.push_back(join);
  }
  result.waypoints.push_back(path.back());
  result.success = true;
  return result;
}
}  // namespace motion2d
