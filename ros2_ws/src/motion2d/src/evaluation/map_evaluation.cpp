#include "motion2d/evaluation/map_evaluation.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion2d
{
namespace
{
bool polygonTouchesBox(const Polygon & polygon, const Eigen::Vector2d & low,
  const Eigen::Vector2d & high)
{
  const std::array<Eigen::Vector2d, 4> box{
    low, Eigen::Vector2d(high.x(), low.y()), high, Eigen::Vector2d(low.x(), high.y())};
  std::vector<Eigen::Vector2d> axes{{1, 0}, {0, 1}};
  for (std::size_t i = 0; i < polygon.vertices.size(); ++i) {
    const Eigen::Vector2d edge =
      polygon.vertices[(i + 1) % polygon.vertices.size()] - polygon.vertices[i];
    axes.emplace_back(-edge.y(), edge.x());
  }
  for (const auto & axis : axes) {
    double p_min = std::numeric_limits<double>::infinity(), p_max = -p_min;
    double b_min = p_min, b_max = p_max;
    for (const auto & point : polygon.vertices) {
      p_min = std::min(p_min, axis.dot(point));
      p_max = std::max(p_max, axis.dot(point));
    }
    for (const auto & point : box) {
      b_min = std::min(b_min, axis.dot(point));
      b_max = std::max(b_max, axis.dot(point));
    }
    if (p_max < b_min || b_max < p_min) {return false;}
  }
  return true;
}

double ratio(int numerator, int denominator)
{
  return denominator ? static_cast<double>(numerator) / denominator :
         std::numeric_limits<double>::quiet_NaN();
}
}

std::vector<std::uint8_t> rasterizeTruth(const World2D & world, const GridConfig & config)
{
  const OccupancyGrid2D geometry(config);  // Validate and reuse cell coordinates only.
  std::vector<std::uint8_t> truth(config.width * config.height, 0);
  const Eigen::Vector2d half = Eigen::Vector2d::Constant(.5 * config.resolution);
  for (int i = 0; i < static_cast<int>(truth.size()); ++i) {
    const Eigen::Vector2d low = geometry.cellCenter(i) - half;
    const Eigen::Vector2d high = geometry.cellCenter(i) + half;
    bool occupied = low.x() <= -world.width / 2 || high.x() >= world.width / 2 ||
      low.y() <= -world.height / 2 || high.y() >= world.height / 2;
    for (const auto & circle : world.circles) {
      const Eigen::Vector2d closest = circle.center.cwiseMax(low).cwiseMin(high);
      occupied = occupied || (circle.center - closest).squaredNorm() <= circle.radius * circle.radius;
    }
    for (const auto & polygon : world.polygons) {
      occupied = occupied || polygonTouchesBox(polygon, low, high);
    }
    truth[i] = occupied;
  }
  return truth;
}

double MapMetrics::precision() const {return ratio(true_positive, true_positive + false_positive);}
double MapMetrics::recall() const {return ratio(true_positive, true_positive + false_negative);}

MapMetrics evaluateObserved(const std::vector<std::int8_t> & observed,
  const std::vector<std::uint8_t> & truth)
{
  if (observed.size() != truth.size()) {throw std::invalid_argument("grid sizes differ");}
  MapMetrics metrics;
  // observed_metrics_begin
  for (std::size_t i = 0; i < observed.size(); ++i) {
    const int value = observed[i];
    if (value < -1 || value > 100 || truth[i] > 1) {
      throw std::invalid_argument("invalid occupancy encoding");
    }
    if (value == -1) {continue;}  // Unknown is not a correct free prediction.
    ++metrics.observed;
    if (value > 35 && value < 65) {++metrics.uncertain; continue;}
    if (value >= 65) {
      if (truth[i]) {++metrics.true_positive;}
      else {++metrics.false_positive;}
    } else {
      if (truth[i]) {++metrics.false_negative;}
      else {++metrics.true_negative;}
    }
  }
  // observed_metrics_end
  return metrics;
}
}  // namespace motion2d
