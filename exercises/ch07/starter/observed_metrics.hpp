#pragma once
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>
/** @brief Return {TP,FP,FN,TN}, excluding unknown/uncertain; thresholds 35/65.
 * @pre Equal sizes; observed is -1 or [0,100]; truth is 0 or 1.
 */
inline std::array<int, 4> studentConfusion(const std::vector<std::int8_t> & observed,
  const std::vector<std::uint8_t> & truth)
{
  // EXERCISE(ch07-4): mask unknown and uncertain before counting.
  (void)observed; (void)truth;
  throw std::logic_error("Complete EXERCISE(ch07-4)");
}
