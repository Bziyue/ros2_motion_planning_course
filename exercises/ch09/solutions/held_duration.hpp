#pragma once
#include <algorithm>
#include <cstdint>

/** @brief Length of the intersection of two time intervals, in seconds. */
inline double heldDuration(std::int64_t start, std::int64_t end,
  std::int64_t left, std::int64_t right)
{
  return std::max<std::int64_t>(0, std::min(end, right) - std::max(start, left)) * 1e-9;
}
