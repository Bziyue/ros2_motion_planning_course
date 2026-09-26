#include "motion2d/sim/lidar_cpu.hpp"
#include "motion2d/sim/raycast.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motion2d
{
namespace
{
constexpr double kTwoPi = 2.0 * kPi;
}

void validateLidarConfig(const LidarConfig & c)
{
  if (c.beams < 2 || c.beams > 100000 || !std::isfinite(c.angle_min) ||
    !std::isfinite(c.fov) || c.fov <= 0 || c.fov > kTwoPi + 1e-12 ||
    !std::isfinite(c.range_min) || !std::isfinite(c.range_max) ||
    c.range_min < 0 || c.range_max <= c.range_min ||
    c.range_max > std::numeric_limits<float>::max())
  {
    throw std::invalid_argument("Lidar: beams in [2,100000], FOV in (0,2*pi], 0<=min<max");
  }
}

// scan_cpu_begin
double beamIncrement(const LidarConfig & config)
{
  const bool full_turn = std::abs(config.fov - kTwoPi) <= 1e-12;
  return full_turn ? kTwoPi / config.beams : config.fov / (config.beams - 1);
}

std::vector<float> scanCpu(const World2D & world, const Pose2D & pose,
  const LidarConfig & config)
{
  std::vector<float> ranges(config.beams);
  const double increment = beamIncrement(config);
  for (int i = 0; i < config.beams; ++i) {
    const double angle = pose.yaw + config.angle_min + i * increment;
    const Eigen::Vector2d direction{std::cos(angle), std::sin(angle)};
    const double distance = raycastWorld(world, pose.position, direction);
    if (distance < config.range_min) {
      ranges[i] = std::numeric_limits<float>::quiet_NaN();
    } else if (distance > config.range_max) {
      ranges[i] = std::numeric_limits<float>::infinity();
    } else {
      ranges[i] = static_cast<float>(distance);
    }
  }
  return ranges;
}
// scan_cpu_end

std::int64_t lidarPeriodTicks(double rate, double dt)
{
  if (!std::isfinite(rate) || !std::isfinite(dt) || dt < 1e-6 || dt > 1.0 ||
    rate < 0.1 || rate > 1.0 / dt)
  {
    throw std::invalid_argument("Lidar rate must be between 0.1 Hz and the simulation tick rate");
  }
  const auto ticks = std::llround(1.0 / (rate * dt));
  if (ticks < 1 || std::abs(ticks * dt - 1.0 / rate) > 1e-9) {
    throw std::invalid_argument("Lidar period must be an integer multiple of dt; try 10 Hz");
  }
  return ticks;
}
}  // namespace motion2d
