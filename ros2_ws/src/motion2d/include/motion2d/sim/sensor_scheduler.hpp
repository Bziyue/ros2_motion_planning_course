#pragma once
#include <cstdint>

namespace motion2d
{
/**
 * @brief Sampling grid anchored at trial time zero, independent of wall callbacks.
 * @details \f$t_k=\operatorname{round}(10^9 k/f)\f$ ns. Recompute from the index,
 * not by adding a rounded period; phase quantization is at most half a nanosecond.
 * The simulator consumes due samples only after accepting their motion interval.
 */
class SensorScheduler
{
public:
  /** @brief Validate a finite rate in [0.1, 1e6] Hz. */
  explicit SensorScheduler(double rate);
  /** @brief Next acquisition stamp in ns since reset; starts at zero.
   * @pre Trial duration fits signed 64-bit nanoseconds.
   */
  std::int64_t nextStamp() const;
  /** @brief Mark the next sample consumed exactly once. */
  void advance() {++index_;}
  /** @brief Begin a new trial with its first sample at zero. */
  void reset() {index_ = 0;}
  /** @brief Nominal sample period in seconds; individual ns gaps can differ by 1. */
  double period() const {return 1.0 / rate_;}

private:
  double rate_;
  std::int64_t index_ = 0;
};
}  // namespace motion2d
