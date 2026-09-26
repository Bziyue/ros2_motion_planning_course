#include "motion2d/mapping/occupancy_grid.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion2d
{
OccupancyGrid2D::OccupancyGrid2D(const GridConfig & config) : config_(config)
{
  if (!std::isfinite(config.resolution) || config.resolution <= 0 ||
    !config.origin.allFinite() || config.width <= 0 || config.height <= 0 ||
    static_cast<std::int64_t>(config.width) * config.height > 4000000 ||
    !std::isfinite(config.resolution * std::max(config.width, config.height)) ||
    !(config.hit_probability > .5 && config.hit_probability < 1) ||
    !(config.miss_probability > 0 && config.miss_probability < .5) ||
    !std::isfinite(config.logodds_limit) || config.logodds_limit <= 0)
  {
    throw std::invalid_argument("grid: positive resolution/size (<=4M cells), finite origin, "
            "0<miss<.5<hit<1, positive finite logodds_limit");
  }
  log_odds_.resize(config.width * config.height, 0.0F);
  observed_.resize(log_odds_.size(), 0);
  hit_increment_ = std::log(config.hit_probability / (1 - config.hit_probability));
  miss_increment_ = std::log(config.miss_probability / (1 - config.miss_probability));
}

std::optional<int> OccupancyGrid2D::cellIndex(const Eigen::Vector2d & p) const
{
  const Eigen::Vector2d u = (p - config_.origin) / config_.resolution;
  if (!(u.x() >= 0 && u.x() < config_.width && u.y() >= 0 && u.y() < config_.height)) {
    return std::nullopt;
  }
  return static_cast<int>(std::floor(u.y())) * config_.width +
         static_cast<int>(std::floor(u.x()));
}

Eigen::Vector2d OccupancyGrid2D::cellCenter(int index) const
{
  return config_.origin + config_.resolution *
         Eigen::Vector2d(index % config_.width + .5, index / config_.width + .5);
}

std::vector<int> OccupancyGrid2D::rayCells(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end) const
{
  const auto first = cellIndex(start);
  if (!first) {return {};}
  const Eigen::Vector2d delta = end - start;
  const Eigen::Vector2d upper = config_.origin + config_.resolution *
    Eigen::Vector2d(config_.width, config_.height);
  double finish = 1.0;
  for (int axis = 0; axis < 2; ++axis) {
    if (delta[axis] > 0) {finish = std::min(finish, (upper[axis] - start[axis]) / delta[axis]);}
    if (delta[axis] < 0) {
      finish = std::min(finish, (config_.origin[axis] - start[axis]) / delta[axis]);
    }
  }
  Eigen::Vector2d clipped = start + finish * delta;
  // Upper bounds are excluded; nextafter also handles exact boundary endpoints.
  for (int axis = 0; axis < 2; ++axis) {
    clipped[axis] = std::clamp(clipped[axis], config_.origin[axis],
      std::nextafter(upper[axis], config_.origin[axis]));
  }
  // Work in grid coordinates so clipping cannot round an upper bound back out.
  const Eigen::Vector2d target = (clipped - config_.origin) / config_.resolution;
  const int end_x = std::min(config_.width - 1, static_cast<int>(std::floor(target.x())));
  const int end_y = std::min(config_.height - 1, static_cast<int>(std::floor(target.y())));
  int x = *first % config_.width, y = *first / config_.width;
  const int step_x = (delta.x() > 0) - (delta.x() < 0);
  const int step_y = (delta.y() > 0) - (delta.y() < 0);
  const double inf = std::numeric_limits<double>::infinity();
  const double dt_x = step_x ? config_.resolution / std::abs(delta.x()) : inf;
  const double dt_y = step_y ? config_.resolution / std::abs(delta.y()) : inf;
  double next_x = step_x ? (config_.origin.x() +
    (x + (step_x > 0)) * config_.resolution - start.x()) / delta.x() : inf;
  double next_y = step_y ? (config_.origin.y() +
    (y + (step_y > 0)) * config_.resolution - start.y()) / delta.y() : inf;
  std::vector<int> cells{*first};
  // grid_dda_begin
  while (x != end_x || y != end_y) {
    // A negative-direction endpoint can lie exactly on a grid line.
    // Do not step an axis past the endpoint's half-open cell.
    if (x == end_x) {next_x = inf;}
    if (y == end_y) {next_y = inf;}
    if (next_x < next_y) {
      x += step_x;
      next_x += dt_x;
    } else if (next_y < next_x) {
      y += step_y;
      next_y += dt_y;
    } else {  // Exact corner: advance both axes.
      x += step_x; y += step_y;
      next_x += dt_x; next_y += dt_y;
    }
    cells.push_back(y * config_.width + x);
  }
  // grid_dda_end
  return cells;
}

bool OccupancyGrid2D::insertScan(const Scan2D & scan, const Pose2D & pose)
{
  if (!cellIndex(pose.position)) {return false;}
  std::vector<std::uint8_t> evidence(log_odds_.size(), 0);
  // grid_rays_begin
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    const bool no_return = std::isinf(range) && range > 0;
    const bool hit = std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
    if (!hit && !no_return) {continue;}
    const double angle = pose.yaw + scan.angle_min + i * scan.angle_increment;
    const double length = no_return ? scan.range_max : range;
    const Eigen::Vector2d end = pose.position +
      length * Eigen::Vector2d(std::cos(angle), std::sin(angle));
    const auto hit_cell = hit ? cellIndex(end) : std::nullopt;
    for (const int cell : rayCells(pose.position, end)) {
      evidence[cell] |= (hit_cell && cell == *hit_cell) ? 2 : 1;
    }
  }
  // grid_rays_end
  // grid_logodds_begin
  for (std::size_t i = 0; i < evidence.size(); ++i) {
    if (evidence[i] == 0) {continue;}
    const double increment = (evidence[i] & 2) ? hit_increment_ : miss_increment_;
    log_odds_[i] = static_cast<float>(std::clamp(log_odds_[i] + increment,
      -config_.logodds_limit, config_.logodds_limit));
    observed_[i] = 1;
  }
  // grid_logodds_end
  return true;
}

std::vector<std::int8_t> OccupancyGrid2D::occupancy() const
{
  std::vector<std::int8_t> values(log_odds_.size(), -1);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (observed_[i]) {
      values[i] = static_cast<std::int8_t>(std::lround(100.0 / (1.0 + std::exp(-log_odds_[i]))));
    }
  }
  return values;
}

void OccupancyGrid2D::clear()
{
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0F);
  std::fill(observed_.begin(), observed_.end(), 0);
}
}  // namespace motion2d
