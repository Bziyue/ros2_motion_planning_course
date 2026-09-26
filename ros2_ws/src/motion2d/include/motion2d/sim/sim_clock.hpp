#pragma once
#include <chrono>
#include <cstdint>

namespace motion2d
{
/**
 * @brief Fixed-step simulated time independent of wall-clock scheduling.
 * @details Integer ticks avoid repeatedly adding a floating-point dt. Chapter 02.
 */
class SimClock
{
public:
  /** @param step Positive simulation step in nanoseconds. */
  explicit SimClock(std::chrono::nanoseconds step);
  /** @brief Advance once if running; return whether time changed. */
  bool advance();
  /** @brief Advance exactly once only when paused. */
  bool singleStep();
  void setPaused(bool paused) {paused_ = paused;}
  bool paused() const {return paused_;}
  /** @brief Start a new paused run at time zero. */
  void reset() {ticks_ = 0; paused_ = true;}
  std::chrono::nanoseconds stepDuration() const {return step_;}
  std::int64_t nanoseconds() const {return ticks_ * step_.count();}
  double seconds() const {return static_cast<double>(nanoseconds()) * 1e-9;}

private:
  std::chrono::nanoseconds step_;
  std::int64_t ticks_ = 0;
  bool paused_ = false;
};
}  // namespace motion2d
