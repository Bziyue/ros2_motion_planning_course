#include "motion2d/mapping/esdf.hpp"
#include "motion2d/mapping/distance_transform.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion2d
{
BilinearSample bilinear(const std::array<double, 4> & d, double u, double v, double resolution)
{
  // esdf_bilinear_begin
  const double value = (1-v)*((1-u)*d[0]+u*d[1]) + v*((1-u)*d[2]+u*d[3]);
  const double dx = ((1-v)*(d[1]-d[0])+v*(d[3]-d[2]))/resolution;
  const double dy = ((1-u)*(d[2]-d[0])+u*(d[3]-d[1]))/resolution;
  return {value, {dx, dy}};
  // esdf_bilinear_end
}

Esdf2D::Esdf2D(const GridConfig & geometry, const std::vector<std::int8_t> & occupancy,
  int free_threshold, bool unknown_blocked) : geometry_(geometry)
{
  OccupancyGrid2D validate(geometry);
  if (geometry.width < 2 || geometry.height < 2 ||
    occupancy.size() != static_cast<std::size_t>(geometry.width)*geometry.height) {
    throw std::invalid_argument("ESDF requires matching dimensions, at least 2x2");
  }
  blocked_ = obstacleMask(occupancy, free_threshold, unknown_blocked);
  const int width = geometry.width+2, height = geometry.height+2;
  std::vector<std::uint8_t> occupied(width*height, 1), free(width*height, 0);
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const int index = (y+1)*width+x+1;
      occupied[index] = blocked_[y*geometry.width+x]; free[index] = !occupied[index];
    }
  }
  const auto to_obstacle = squaredDistanceTransform(width, height, occupied);
  const auto to_free = squaredDistanceTransform(width, height, free);
  distances_.resize(occupancy.size());
  for (int y = 0; y < geometry.height; ++y) {
    for (int x = 0; x < geometry.width; ++x) {
      const int p = (y+1)*width+x+1, i = y*geometry.width+x;
      distances_[i] = geometry.resolution * (blocked_[i] ? -std::sqrt(to_free[p]) : std::sqrt(to_obstacle[p]));
    }
  }
}

std::optional<EsdfSample> Esdf2D::sample(const Eigen::Vector2d & point) const
{
  if (!point.allFinite()) {return std::nullopt;}
  const auto & g = geometry_;
  const Eigen::Vector2d cell = (point-g.origin)/g.resolution-Eigen::Vector2d::Constant(.5);
  if (cell.x() < 0 || cell.y() < 0 || cell.x() > g.width-1 || cell.y() > g.height-1) {return std::nullopt;}
  const int x = std::min(g.width-2, static_cast<int>(std::floor(cell.x())));
  const int y = std::min(g.height-2, static_cast<int>(std::floor(cell.y())));
  const double u = cell.x()-x, v = cell.y()-y;
  const std::array<int, 4> indices{y*g.width+x, y*g.width+x+1, (y+1)*g.width+x, (y+1)*g.width+x+1};
  std::array<double, 4> d;
  bool all_free = true;
  for (int i = 0; i < 4; ++i) {
    d[i] = distances_[indices[i]];
    if (!std::isfinite(d[i])) {return std::nullopt;}
    all_free = all_free && !blocked_[indices[i]];
  }
  const auto interpolated = bilinear(d, u, v, g.resolution);
  double bound = 0;
  if (all_free) {
    // Unsigned centre distance is 1-Lipschitz. Weighted distances to the
    // interpolation corners bound overestimation, then remove a square's radius.
    const std::array<double, 4> weights{(1-u)*(1-v), u*(1-v), (1-u)*v, u*v};
    double error = 0;
    for (int i = 0; i < 4; ++i) {error += weights[i]*g.resolution*std::hypot(u-i%2, v-i/2);}
    bound = std::max(0., interpolated.value-error-g.resolution/std::sqrt(2.));
  }
  return EsdfSample{interpolated.value, interpolated.gradient, bound};
}
}  // namespace motion2d
