#pragma once
#include <cmath>
#include <cstdint>

/** @brief Reference nearest-nanosecond grid anchored at trial start. */
inline std::int64_t studentSampleStamp(std::int64_t index, double rate)
{
  return std::llround(static_cast<long double>(index) * 1000000000.0L / rate);
}
