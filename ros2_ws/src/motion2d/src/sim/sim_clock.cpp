#include "motion2d/sim/sim_clock.hpp"
#include <stdexcept>

namespace motion2d
{
SimClock::SimClock(std::chrono::nanoseconds step) : step_(step)
{
  if (step.count() <= 0) {
    throw std::invalid_argument("Simulation time step must be positive");
  }
}

// clock_begin
bool SimClock::advance()
{
  if (paused_) {
    return false;
  }
  ++ticks_;
  return true;
}

bool SimClock::singleStep()
{
  if (!paused_) {
    return false;
  }
  ++ticks_;
  return true;
}
// clock_end
}  // namespace motion2d
