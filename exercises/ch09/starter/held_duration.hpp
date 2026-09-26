#pragma once
#include <cstdint>

/** @brief Seconds contributed by a held sample [left,right) to [start,end).
 * @pre Ordered, nonnegative integer nanoseconds in each interval.
 */
inline double heldDuration(std::int64_t start, std::int64_t end,
  std::int64_t left, std::int64_t right)
{
  // EXERCISE(ch09-2): intersect the intervals BEFORE converting nanoseconds to seconds.
  (void)start; (void)end; (void)left; (void)right;
  return 0.;
}
