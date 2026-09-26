#pragma once
#include <cstdint>
#include <stdexcept>

/** @brief Acquisition time in ns, from nonnegative index and valid Hz rate. */
inline std::int64_t studentSampleStamp(std::int64_t index, double rate)
{
  // EXERCISE(ch06-3): recompute the grid; do not repeatedly add a rounded period.
  (void)index; (void)rate;
  throw std::logic_error("Complete EXERCISE(ch06-3)");
}
