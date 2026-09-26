#include "motion2d/sim/sensor_scheduler.hpp"
#include <cmath>
#include <stdexcept>

namespace motion2d
{
SensorScheduler::SensorScheduler(double rate) : rate_(rate)
{
  if (!std::isfinite(rate) || rate < .1 || rate > 1e6) {
    throw std::invalid_argument("Sensor rate must be finite, in [0.1, 1e6] Hz");
  }
}

// sample_grid_begin
std::int64_t SensorScheduler::nextStamp() const
{
  return std::llround(static_cast<long double>(index_) * 1000000000.0L / rate_);
}
// sample_grid_end
}  // namespace motion2d
